#!/usr/bin/env python3
"""
LLM Navigation node — plain-English commands → Nav2 goal.

Pipeline:
  mic → Whisper STT  ──┐
                        ├→ ollama LLM → NavigateToPose action
  text topic ──────────┘

Usage
─────
  # start (in a sourced terminal):
  ros2 run diff_drive_robot llm_nav.py

  # override defaults:
  ros2 run diff_drive_robot llm_nav.py --ros-args \
      -p whisper_model:=small \
      -p ollama_model:=llama2 \
      -p record_seconds:=6.0

  # text-only (no mic):
  ros2 topic pub /llm_nav/command std_msgs/msg/String "data: 'go to room_b'" --once

Deps (already installed): openai-whisper, sounddevice
ollama must be running: `ollama serve`
"""

import json
import math
import os
import re
import sys
import threading
import time
import urllib.request
import urllib.error

import numpy as np
import sounddevice as sd
import whisper

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose


# ── helpers ───────────────────────────────────────────────────────────────────

def _load_locations(share_dir: str) -> dict:
    candidates = [
        os.path.join(share_dir, 'config', 'locations.yaml'),
        os.path.join(os.path.expanduser('~'), 'rosnav', 'locations.yaml'),
    ]
    try:
        import yaml
    except ImportError:
        return {}
    for p in candidates:
        if os.path.isfile(p):
            with open(p) as f:
                data = yaml.safe_load(f) or {}
            return data.get('locations', {})
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


# ── ollama call ───────────────────────────────────────────────────────────────

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


# ── LLM prompt ────────────────────────────────────────────────────────────────

_SYSTEM = """\
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

        # params
        self.declare_parameter('whisper_model',   'base')
        self.declare_parameter('ollama_model',    'llama2')
        self.declare_parameter('ollama_url',      'http://localhost:11434')
        self.declare_parameter('record_seconds',   5.0)
        self.declare_parameter('sample_rate',    16000)
        self.declare_parameter('nav_action',     'navigate_to_pose')
        self.declare_parameter('frame_id',       'map')

        g = self.get_parameter
        self._whisper_model_name = g('whisper_model').value
        self._ollama_model       = g('ollama_model').value
        self._ollama_url         = g('ollama_url').value
        self._record_sec         = g('record_seconds').value
        self._sample_rate        = int(g('sample_rate').value)
        self._frame_id           = g('frame_id').value

        # load locations
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory('diff_drive_robot')
        except Exception:
            share = os.path.join(
                os.path.expanduser('~'), 'rosnav', 'src', 'diff_drive_robot-main')
        self._locations = _load_locations(share)
        self.get_logger().info(
            f'Loaded locations: {list(self._locations.keys())}')

        # nav2 action client
        self._nav_client = ActionClient(
            self, NavigateToPose, g('nav_action').value)

        # text-command subscriber
        self.create_subscription(String, '/llm_nav/command',
                                 self._text_cmd_cb, 10)

        # pose tracking for accuracy reporting
        self._current_pose: tuple[float, float] | None = None
        self._goal_xy: tuple[float, float] | None = None
        self._nav_start_time: float | None = None
        self._recovery_count = 0
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)

        # lazy-load Whisper (takes a few seconds)
        self._whisper = None
        self._whisper_lock = threading.Lock()
        threading.Thread(target=self._load_whisper, daemon=True).start()

        self._busy = False
        self._busy_lock = threading.Lock()

        self.get_logger().info(
            f'LLM nav ready.  ollama={self._ollama_model}  '
            f'whisper={self._whisper_model_name}')
        self.get_logger().info(
            'Press Enter in this terminal to speak, '
            'or publish to /llm_nav/command')

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

        print(f'\n🎙  Recording {self._record_sec:.0f}s … (speak now)', flush=True)
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

    # ── LLM parse ─────────────────────────────────────────────────────────────

    def _parse_command(self, text: str) -> dict | None:
        location_list = ', '.join(self._locations.keys()) if self._locations else 'none'
        prompt = _SYSTEM.format(locations=location_list, command=text)
        try:
            raw = call_ollama(self._ollama_model, prompt, self._ollama_url)
        except (urllib.error.URLError, TimeoutError) as e:
            self.get_logger().error(f'ollama error: {e}')
            return None

        parsed = _extract_json(raw)
        if parsed is None:
            self.get_logger().error(f'LLM returned unparseable: {raw[:200]}')
        return parsed

    # ── navigation ────────────────────────────────────────────────────────────

    def _resolve_goal(self, parsed: dict) -> tuple[float, float, float] | None:
        action = parsed.get('action', '')

        if action == 'stop':
            self.get_logger().info('Stop command received — cancelling goals.')
            self._nav_client.cancel_all_goals_async()
            return None

        if action == 'unknown':
            self.get_logger().warn(f"LLM couldn't parse: {parsed.get('reason','?')}")
            return None

        if action != 'go':
            self.get_logger().warn(f'Unknown action: {action}')
            return None

        if 'location' in parsed:
            name = parsed['location']
            if name not in self._locations:
                self.get_logger().error(
                    f'Location "{name}" not in locations.yaml. '
                    f'Known: {list(self._locations.keys())}')
                return None
            coords = self._locations[name]
            x, y   = float(coords[0]), float(coords[1])
            yaw    = float(coords[2]) if len(coords) > 2 else 0.0
            return x, y, yaw

        if 'x' in parsed and 'y' in parsed:
            return float(parsed['x']), float(parsed['y']), float(parsed.get('yaw', 0.0))

        self.get_logger().error(f'Malformed go command: {parsed}')
        return None

    def _send_goal(self, x: float, y: float, yaw_deg: float):
        # Run in a background thread so we never block the ROS executor.
        threading.Thread(
            target=self._send_goal_thread, args=(x, y, yaw_deg), daemon=True).start()

    def _send_goal_thread(self, x: float, y: float, yaw_deg: float):
        waited = 0.0
        while not self._nav_client.wait_for_server(timeout_sec=5.0):
            waited += 5.0
            self.get_logger().warn(
                f'Waiting for NavigateToPose server… ({waited:.0f}s)')
            if waited >= 60.0:
                self.get_logger().error(
                    'NavigateToPose server did not come up after 60s — is Nav2 running?')
                with self._busy_lock:
                    self._busy = False
                return

        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.header.stamp    = self.get_clock().now().to_msg()
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
        self.get_logger().info(
            f'Navigating to ({x:.2f}, {y:.2f}, yaw={yaw_deg:.0f}°)…')

        future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_accepted_cb)

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        self._current_pose = (p.x, p.y)

    def _feedback_cb(self, fb):
        dist = fb.feedback.distance_remaining
        self._recovery_count = fb.feedback.number_of_recoveries
        if dist > 0.0:
            self.get_logger().info(f'  distance remaining: {dist:.2f}m', throttle_duration_sec=3.0)

    def _goal_accepted_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected by Nav2.')
            with self._busy_lock:
                self._busy = False
            return
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        from action_msgs.msg import GoalStatus
        status = future.result().status
        elapsed = time.time() - self._nav_start_time if self._nav_start_time else 0.0

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached!')
            if self._goal_xy and self._current_pose:
                gx, gy = self._goal_xy
                rx, ry = self._current_pose
                err = math.hypot(rx - gx, ry - gy)
                self.get_logger().info(
                    f'  Accuracy: {err:.3f} m from target  |  '
                    f'time: {elapsed:.1f}s  |  recoveries: {self._recovery_count}')
        else:
            self.get_logger().warn(
                f'Navigation ended with status {status}  |  '
                f'time: {elapsed:.1f}s  |  recoveries: {self._recovery_count}')
        with self._busy_lock:
            self._busy = False

    # ── entry points ──────────────────────────────────────────────────────────

    def _text_cmd_cb(self, msg: String):
        self._process(msg.data.strip())

    def handle_voice(self):
        """Called from UI thread when user presses Enter."""
        with self._busy_lock:
            busy = self._busy
        if busy:
            print('Still navigating — wait or say "stop".', flush=True)
            return
        text = self._transcribe()
        if text:
            self._process(text)

    def handle_typed(self, text: str):
        with self._busy_lock:
            busy = self._busy
        if busy:
            print('Still navigating — wait or say "stop".', flush=True)
            return
        self._process(text)

    def _process(self, text: str):
        self.get_logger().info(f'Command: "{text}"')
        print(f'   Asking {self._ollama_model}…', flush=True)
        parsed = self._parse_command(text)
        if parsed is None:
            return
        self.get_logger().info(f'LLM parsed: {parsed}')
        goal = self._resolve_goal(parsed)
        if goal:
            with self._busy_lock:
                self._busy = True
            self._send_goal(*goal)


# ── main ──────────────────────────────────────────────────────────────────────

def _ui_loop(node: LLMNavigator):
    """Runs on main thread — handles terminal input."""
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
