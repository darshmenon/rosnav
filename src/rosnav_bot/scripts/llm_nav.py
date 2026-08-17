#!/usr/bin/env python3
"""
LLM Navigation node — plain-English commands → multi-step robot plan.

Pipeline:
  mic → Whisper STT  ──┐
                        ├→ ollama (/api/chat tools) → status / nav / dock / undock
  text topic ──────────┘

Usage
─────
  ros2 run rosnav_bot station_server.py
  ros2 run rosnav_bot llm_nav.py

  ros2 run rosnav_bot llm_nav.py --ros-args \
      -p whisper_model:=small \
      -p ollama_model:=llama3.1 \
      -p record_seconds:=6.0

  ros2 topic pub /llm_nav/command std_msgs/msg/String \
      "data: 'undock, go to room_b, then come back and dock'" --once

Deps (already installed): openai-whisper, sounddevice
ollama must be running: `ollama serve`  (`ollama pull llama3.1`)
"""

import json
import math
import os
import re
import threading
import time
import urllib.error
import urllib.request

import numpy as np
import sounddevice as sd
import whisper

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from rosnav_bot.action import DockToStation, UndockFromStation
from rosnav_bot.srv import GetRobotStatus


# ── helpers ───────────────────────────────────────────────────────────────────

def _package_share() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('rosnav_bot')
    except Exception:
        return os.path.join(os.path.expanduser('~'), 'rosnav', 'src', 'rosnav_bot')


def _load_yaml_section(share_dir: str, filename: str, key: str) -> dict:
    candidates = [
        os.path.join(share_dir, 'config', filename),
        os.path.join(os.path.expanduser('~'), 'rosnav', filename),
    ]
    try:
        import yaml
    except ImportError:
        return {}
    for p in candidates:
        if os.path.isfile(p):
            with open(p) as f:
                data = yaml.safe_load(f) or {}
            return data.get(key, {}) or {}
    return {}


def _yaw_to_quat(yaw_deg: float):
    yaw = math.radians(yaw_deg)
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _extract_json(text: str) -> dict | None:
    """Pull the first {...} block out of LLM output and parse it."""
    match = re.search(r'\{[^{}]*\}', text, re.DOTALL)
    if not match:
        return None
    try:
        return json.loads(match.group())
    except json.JSONDecodeError:
        return None


def _topic(ns: str, name: str) -> str:
    ns = (ns or '').strip().strip('/')
    return f'/{ns}/{name}' if ns else name


def _tool_args(call: dict) -> dict:
    fn = call.get('function') or call
    raw = fn.get('arguments', {})
    if isinstance(raw, str):
        try:
            raw = json.loads(raw) if raw else {}
        except json.JSONDecodeError:
            return {}
    return raw if isinstance(raw, dict) else {}


# ── ollama ────────────────────────────────────────────────────────────────────

def call_ollama(model: str, prompt: str, base_url: str = 'http://localhost:11434') -> str:
    payload = json.dumps({
        'model':  model,
        'prompt': prompt,
        'stream': False,
        'format': 'json',
    }).encode()
    req = urllib.request.Request(
        f'{base_url}/api/generate',
        data=payload,
        headers={'Content-Type': 'application/json'},
        method='POST',
    )
    with urllib.request.urlopen(req, timeout=30) as resp:
        return json.loads(resp.read())['response']


def call_ollama_chat(
    model: str,
    messages: list,
    tools: list,
    base_url: str = 'http://localhost:11434',
    timeout: float = 60.0,
) -> dict:
    payload = json.dumps({
        'model': model,
        'messages': messages,
        'stream': False,
        'tools': tools,
    }).encode()
    req = urllib.request.Request(
        f'{base_url}/api/chat',
        data=payload,
        headers={'Content-Type': 'application/json'},
        method='POST',
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        data = json.loads(resp.read())
    return data.get('message') or {}


_TOOLS = [
    {
        'type': 'function',
        'function': {
            'name': 'get_robot_status',
            'description': 'Query whether the robot is docked, plus pose and battery.',
            'parameters': {'type': 'object', 'properties': {}},
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'navigate_to_pose',
            'description': (
                'Navigate to a named location or map coordinates. '
                'yaw is degrees. Use a name from the known locations list.'
            ),
            'parameters': {
                'type': 'object',
                'properties': {
                    'location': {'type': 'string'},
                    'x': {'type': 'number'},
                    'y': {'type': 'number'},
                    'yaw': {'type': 'number'},
                },
            },
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'dock_to_station',
            'description': 'Navigate to the dock staging pose then visually dock.',
            'parameters': {
                'type': 'object',
                'properties': {
                    'station': {
                        'type': 'string',
                        'description': 'Dock name from the known docks list.',
                    },
                },
            },
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'undock_from_station',
            'description': 'Back away from the dock and spin to the exit heading.',
            'parameters': {'type': 'object', 'properties': {}},
        },
    },
    {
        'type': 'function',
        'function': {
            'name': 'stop',
            'description': 'Cancel navigation, docking, or undocking immediately.',
            'parameters': {'type': 'object', 'properties': {}},
        },
    },
]


_SYSTEM = """\
You are a robot navigation assistant. Use tools to carry out the user's request.

Known named locations: {locations}
Known docks: {docks}

Tools:
- get_robot_status: call this first for any multi-step or dock-related request.
- navigate_to_pose: go to a named location or x/y/yaw (yaw in degrees).
- dock_to_station: final approach onto a charging dock.
- undock_from_station: leave the dock.
- stop: cancel motion.

Rules:
- Location and dock names must match the lists exactly.
- If is_docked is true, you MUST call undock_from_station before navigate_to_pose or dock_to_station.
- Chain tools for multi-step commands (e.g. undock → navigate → navigate → dock).
- When finished, reply with one short sentence and no more tools.
- If the command is unclear, ask a brief question instead of guessing.
"""


_FALLBACK_SYSTEM = """\
You are a robot navigation assistant. Parse the user command and return ONLY a JSON object.

Known named locations: {locations}

Return exactly one of these formats:
  Named location : {{"action":"go","location":"<name>"}}
  Coordinates    : {{"action":"go","x":<float>,"y":<float>,"yaw":<float>}}
  Stop robot     : {{"action":"stop"}}
  Unclear        : {{"action":"unknown","reason":"<brief reason>"}}

Rules:
- Return ONLY the JSON — no extra text, no markdown.
- Location names must be from the list above (exact spelling).
- yaw is in degrees, default 0.0.
- If the user says a location not on the list but gives numeric coords, use coordinates format.

User command: "{command}"

JSON:"""


# ── ROS node ──────────────────────────────────────────────────────────────────

class LLMNavigator(Node):
    def __init__(self):
        super().__init__('llm_navigator')

        self.declare_parameter('whisper_model',   'base')
        self.declare_parameter('ollama_model',    'llama3.1')
        self.declare_parameter('ollama_url',      'http://localhost:11434')
        self.declare_parameter('record_seconds',   5.0)
        self.declare_parameter('sample_rate',    16000)
        self.declare_parameter('nav_action',     'navigate_to_pose')
        self.declare_parameter('frame_id',       'map')
        self.declare_parameter('namespace',      '')
        self.declare_parameter('max_steps',      8)

        g = self.get_parameter
        self._whisper_model_name = g('whisper_model').value
        self._ollama_model       = g('ollama_model').value
        self._ollama_url         = g('ollama_url').value
        self._record_sec         = g('record_seconds').value
        self._sample_rate        = int(g('sample_rate').value)
        self._frame_id           = g('frame_id').value
        self._ns                 = str(g('namespace').value or '').strip().strip('/')
        self._max_steps          = int(g('max_steps').value)

        share = _package_share()
        self._locations = _load_yaml_section(share, 'locations.yaml', 'locations')
        self._docks = _load_yaml_section(share, 'docks.yaml', 'docks')
        self.get_logger().info(
            f'Loaded locations: {list(self._locations.keys())}')
        self.get_logger().info(
            f'Loaded docks: {list(self._docks.keys())}')

        nav_action = g('nav_action').value
        if self._ns and '/' not in str(nav_action):
            nav_action = _topic(self._ns, str(nav_action).lstrip('/'))
        self._nav_client = ActionClient(self, NavigateToPose, nav_action)
        self._dock_client = ActionClient(
            self, DockToStation, _topic(self._ns, 'dock_to_station'))
        self._undock_client = ActionClient(
            self, UndockFromStation, _topic(self._ns, 'undock_from_station'))
        self._status_client = self.create_client(
            GetRobotStatus, _topic(self._ns, 'get_robot_status'))

        self.create_subscription(String, '/llm_nav/command',
                                 self._text_cmd_cb, 10)
        self._reply_pub = self.create_publisher(String, '/llm_nav/reply', 10)

        self._current_pose: tuple[float, float] | None = None
        self._goal_xy: tuple[float, float] | None = None
        self._nav_start_time: float | None = None
        self._recovery_count = 0
        amcl_topic = _topic(self._ns, 'amcl_pose') if self._ns else '/amcl_pose'
        self.create_subscription(
            PoseWithCovarianceStamped, amcl_topic, self._amcl_cb, 10)

        self._whisper = None
        self._whisper_lock = threading.Lock()
        threading.Thread(target=self._load_whisper, daemon=True).start()

        self._busy = False
        self._busy_lock = threading.Lock()
        self._stop_requested = False
        self._active_handle = None

        self.get_logger().info(
            f'LLM nav ready.  ollama={self._ollama_model}  '
            f'whisper={self._whisper_model_name}')
        self.get_logger().info(
            'Press Enter in this terminal to speak, '
            'or publish to /llm_nav/command')

    def _reply(self, text: str):
        self.get_logger().info(text)
        print(f'   {text}', flush=True)
        self._reply_pub.publish(String(data=text))

    # ── whisper ───────────────────────────────────────────────────────────────

    def _load_whisper(self):
        self.get_logger().info('Loading Whisper model (first run may take a moment)…')
        model = whisper.load_model(self._whisper_model_name)
        with self._whisper_lock:
            self._whisper = model
        self.get_logger().info('Whisper ready.')

    def _transcribe(self) -> str | None:
        with self._whisper_lock:
            model = self._whisper
        if model is None:
            self.get_logger().warn('Whisper still loading, try again.')
            return None

        print(f'\nRecording {self._record_sec:.0f}s … (speak now)', flush=True)
        audio = sd.rec(
            int(self._record_sec * self._sample_rate),
            samplerate=self._sample_rate,
            channels=1,
            dtype='float32',
        )
        sd.wait()
        print('   Transcribing…', flush=True)
        audio_np = audio.flatten()
        result = model.transcribe(audio_np, language='en', fp16=False)
        text = result['text'].strip()
        print(f'   Heard: "{text}"', flush=True)
        return text if text else None

    # ── action / service wait ─────────────────────────────────────────────────

    def _await_future(self, future, timeout: float):
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            if self._stop_requested:
                return None
            time.sleep(0.05)
        if not future.done():
            return None
        return future.result()

    def _wait_action(self, client, goal, timeout: float, label: str) -> dict:
        if not client.wait_for_server(timeout_sec=2.0):
            return {'ok': False, 'error': f'{label} server not available'}
        kwargs = {}
        if label == 'navigate_to_pose':
            kwargs['feedback_callback'] = self._feedback_cb
        send_future = client.send_goal_async(goal, **kwargs)
        handle = self._await_future(send_future, 10.0)
        if handle is None or not handle.accepted:
            return {'ok': False, 'error': f'{label} goal rejected'}
        self._active_handle = handle
        result_future = handle.get_result_async()
        deadline = time.time() + timeout
        while not result_future.done():
            if self._stop_requested:
                handle.cancel_goal_async()
                self._active_handle = None
                return {'ok': False, 'error': 'cancelled'}
            if time.time() > deadline:
                handle.cancel_goal_async()
                self._active_handle = None
                return {'ok': False, 'error': f'{label} timed out'}
            time.sleep(0.05)
        self._active_handle = None
        wrapped = result_future.result()
        if wrapped is None:
            return {'ok': False, 'error': f'{label} failed'}
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return {'ok': False, 'error': f'{label} ended with status {wrapped.status}'}
        res = wrapped.result
        message = getattr(res, 'message', 'ok')
        success = getattr(res, 'success', True)
        if not success:
            return {'ok': False, 'error': message or f'{label} failed'}
        return {'ok': True, 'message': message or 'ok'}

    def _query_status(self) -> dict:
        if not self._status_client.wait_for_service(timeout_sec=1.0):
            return {'ok': False, 'error': 'get_robot_status unavailable — is station_server running?'}
        future = self._status_client.call_async(GetRobotStatus.Request())
        result = self._await_future(future, 5.0)
        if result is None:
            return {'ok': False, 'error': 'get_robot_status timed out'}
        return {
            'ok': True,
            'is_docked': bool(result.is_docked),
            'dock_name': result.dock_name,
            'x': None if math.isnan(result.x) else round(float(result.x), 3),
            'y': None if math.isnan(result.y) else round(float(result.y), 3),
            'yaw': None if math.isnan(result.yaw) else round(math.degrees(float(result.yaw)), 1),
            'battery_percent': None if math.isnan(result.battery_percent)
            else round(float(result.battery_percent), 1),
            'nav_busy': bool(result.nav_busy),
            'message': result.message,
        }

    def _is_docked(self) -> bool:
        status = self._query_status()
        return bool(status.get('ok') and status.get('is_docked'))

    def _cancel_all(self):
        self._stop_requested = True
        handle = self._active_handle
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass
        try:
            self._nav_client.cancel_all_goals_async()
        except Exception:
            pass
        try:
            self._dock_client.cancel_all_goals_async()
        except Exception:
            pass
        try:
            self._undock_client.cancel_all_goals_async()
        except Exception:
            pass

    # ── tools ─────────────────────────────────────────────────────────────────

    def _tool_navigate(self, args: dict) -> dict:
        if self._is_docked():
            return {
                'ok': False,
                'error': 'robot is docked — call undock_from_station first',
            }
        goal_xyyaw = self._resolve_nav_args(args)
        if goal_xyyaw is None:
            return {
                'ok': False,
                'error': f'need location or x/y. known locations: {list(self._locations.keys())}',
            }
        x, y, yaw_deg = goal_xyyaw
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            return {'ok': False, 'error': 'NavigateToPose server not available'}

        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        qz, qw = _yaw_to_quat(yaw_deg)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._goal_xy = (x, y)
        self._nav_start_time = time.time()
        self._recovery_count = 0
        self._reply(f'Navigating to ({x:.2f}, {y:.2f}, yaw={yaw_deg:.0f}°)')
        result = self._wait_action(self._nav_client, goal, 120.0, 'navigate_to_pose')
        if result.get('ok'):
            self._log_nav_accuracy()
        return result

    def _resolve_nav_args(self, args: dict) -> tuple[float, float, float] | None:
        if args.get('location'):
            name = str(args['location'])
            if name not in self._locations:
                return None
            coords = self._locations[name]
            x, y = float(coords[0]), float(coords[1])
            yaw = float(coords[2]) if len(coords) > 2 else 0.0
            return x, y, yaw
        if 'x' in args and 'y' in args:
            return float(args['x']), float(args['y']), float(args.get('yaw', 0.0))
        return None

    def _tool_dock(self, args: dict) -> dict:
        if self._is_docked():
            return {
                'ok': False,
                'error': 'already docked — call undock_from_station first',
            }
        station = str(args.get('station') or 'charging_dock')
        if self._docks and station not in self._docks:
            return {
                'ok': False,
                'error': f'unknown dock {station!r}. known: {list(self._docks.keys())}',
            }
        goal = DockToStation.Goal()
        goal.station = station
        self._reply(f'Docking at {station}')
        return self._wait_action(self._dock_client, goal, 180.0, 'dock_to_station')

    def _tool_undock(self, _args: dict) -> dict:
        goal = UndockFromStation.Goal()
        self._reply('Undocking')
        return self._wait_action(self._undock_client, goal, 40.0, 'undock_from_station')

    def _execute_tool(self, name: str, args: dict) -> dict:
        if name == 'stop' or self._stop_requested:
            self._cancel_all()
            return {'ok': True, 'message': 'stopped'}
        if name == 'get_robot_status':
            return self._query_status()
        if name == 'navigate_to_pose':
            return self._tool_navigate(args)
        if name == 'dock_to_station':
            return self._tool_dock(args)
        if name == 'undock_from_station':
            return self._tool_undock(args)
        return {'ok': False, 'error': f'unknown tool {name}'}

    # ── planner ───────────────────────────────────────────────────────────────

    def _run_plan(self, text: str):
        self._stop_requested = False
        location_list = ', '.join(self._locations.keys()) if self._locations else 'none'
        dock_list = ', '.join(self._docks.keys()) if self._docks else 'none'
        messages = [
            {'role': 'system', 'content': _SYSTEM.format(
                locations=location_list, docks=dock_list)},
            {'role': 'user', 'content': text},
        ]
        stopped = False
        try:
            for step in range(self._max_steps):
                if self._stop_requested:
                    self._reply('Stopped.')
                    stopped = True
                    break
                print(f'   Asking {self._ollama_model} (step {step + 1})…', flush=True)
                try:
                    message = call_ollama_chat(
                        self._ollama_model, messages, _TOOLS, self._ollama_url)
                except (urllib.error.URLError, TimeoutError) as e:
                    self.get_logger().error(f'ollama error: {e}')
                    self._reply(f'Ollama error: {e}')
                    return

                tool_calls = message.get('tool_calls') or []
                content = (message.get('content') or '').strip()

                if not tool_calls:
                    if content:
                        parsed = _extract_json(content)
                        if parsed and parsed.get('action') in ('go', 'stop', 'unknown'):
                            self._run_fallback_action(parsed)
                            return
                        self._reply(content)
                    else:
                        self._fallback_generate(text)
                    return

                messages.append({
                    'role': 'assistant',
                    'content': content,
                    'tool_calls': tool_calls,
                })
                for call in tool_calls:
                    fn = call.get('function') or {}
                    name = fn.get('name') or call.get('name') or ''
                    args = _tool_args(call)
                    self.get_logger().info(f'tool {name} {args}')
                    result = self._execute_tool(name, args)
                    tool_msg = {
                        'role': 'tool',
                        'content': json.dumps(result),
                    }
                    if name:
                        tool_msg['name'] = name
                    if call.get('id'):
                        tool_msg['tool_call_id'] = call['id']
                    messages.append(tool_msg)
                    if name == 'stop' or self._stop_requested:
                        self._reply('Stopped.')
                        stopped = True
                        break
                if stopped:
                    break
            else:
                self._reply(f'Reached max_steps ({self._max_steps}) — stopping.')
        finally:
            with self._busy_lock:
                self._busy = False

    def _fallback_generate(self, text: str):
        location_list = ', '.join(self._locations.keys()) if self._locations else 'none'
        prompt = _FALLBACK_SYSTEM.format(locations=location_list, command=text)
        try:
            raw = call_ollama(self._ollama_model, prompt, self._ollama_url)
        except (urllib.error.URLError, TimeoutError) as e:
            self.get_logger().error(f'ollama error: {e}')
            self._reply(f'Ollama error: {e}')
            return
        parsed = _extract_json(raw)
        if parsed is None:
            self.get_logger().error(f'LLM returned unparseable: {raw[:200]}')
            self._reply('Could not parse that command.')
            return
        self._run_fallback_action(parsed)

    def _run_fallback_action(self, parsed: dict):
        self.get_logger().info(f'LLM parsed: {parsed}')
        action = parsed.get('action', '')
        if action == 'stop':
            self._cancel_all()
            self._reply('Stopped.')
            return
        if action == 'unknown':
            self._reply(f"Could not parse: {parsed.get('reason', '?')}")
            return
        if action != 'go':
            self._reply(f'Unknown action: {action}')
            return
        result = self._tool_navigate(parsed)
        if result.get('ok'):
            self._log_nav_accuracy()
            self._reply(result.get('message', 'Goal reached.'))
        else:
            self._reply(result.get('error', 'Navigation failed.'))

    def _log_nav_accuracy(self):
        elapsed = time.time() - self._nav_start_time if self._nav_start_time else 0.0
        if self._goal_xy and self._current_pose:
            gx, gy = self._goal_xy
            rx, ry = self._current_pose
            err = math.hypot(rx - gx, ry - gy)
            self.get_logger().info(
                f'  Accuracy: {err:.3f} m from target  |  '
                f'time: {elapsed:.1f}s  |  recoveries: {self._recovery_count}')

    # ── navigation callbacks ──────────────────────────────────────────────────

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        self._current_pose = (p.x, p.y)

    def _feedback_cb(self, fb):
        dist = fb.feedback.distance_remaining
        self._recovery_count = fb.feedback.number_of_recoveries
        if dist > 0.0:
            self.get_logger().info(
                f'  distance remaining: {dist:.2f}m', throttle_duration_sec=3.0)

    # ── entry points ──────────────────────────────────────────────────────────

    def _text_cmd_cb(self, msg: String):
        self._submit(msg.data.strip())

    def handle_voice(self):
        text = self._transcribe()
        if text:
            self._submit(text)

    def handle_typed(self, text: str):
        self._submit(text)

    def _submit(self, text: str):
        if not text:
            return
        if text.lower() in ('stop', 'cancel', 'halt'):
            self._cancel_all()
            self._reply('Stopped.')
            return
        with self._busy_lock:
            busy = self._busy
            if not busy:
                self._busy = True
        if busy:
            print('Still executing — wait or type stop.', flush=True)
            return
        self.get_logger().info(f'Command: "{text}"')
        threading.Thread(target=self._run_plan, args=(text,), daemon=True).start()


# ── main ──────────────────────────────────────────────────────────────────────

def _ui_loop(node: LLMNavigator):
    print('\n─────────────────────────────────────────', flush=True)
    print(' LLM Navigator  |  ctrl-C to quit', flush=True)
    print(' Press Enter    → record speech', flush=True)
    print(' Type a command → send as text', flush=True)
    print('─────────────────────────────────────────\n', flush=True)
    while rclpy.ok():
        try:
            line = input('> ').strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not line:
            node.handle_voice()
        else:
            node.handle_typed(line)


def main(args=None):
    rclpy.init(args=args)
    node = LLMNavigator()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        _ui_loop(node)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
