#!/usr/bin/env python3
"""
mission_server.py — High-level mission execution layer (top of 3-tier stack).

Sits above Nav2.  Accepts structured missions and breaks them into
NavigateToPose action calls.  Runs as a persistent daemon that any
client can send missions to via the /mission/execute topic.

Mission types
─────────────
  patrol    Loop endlessly through a waypoint list.
  sequence  Visit waypoints once in order, then stop.
  goto      Navigate to a single pose.

Waypoints may be raw coordinates [x, y, yaw_deg] or named locations
defined in config/locations.yaml (e.g. "room_a", "charging_dock").

Daemon mode (run once, persists):
  ros2 run diff_drive_robot mission_server.py

Send a mission from the command line (daemon must be running):
  ros2 run diff_drive_robot mission_server.py patrol  robot1 room_a room_b room_c
  ros2 run diff_drive_robot mission_server.py patrol  robot1 1,2,0  3,4,90
  ros2 run diff_drive_robot mission_server.py sequence robot1 0,0,0 2,0,0 2,2,90
  ros2 run diff_drive_robot mission_server.py goto    robot1 room_a
  ros2 run diff_drive_robot mission_server.py goto    robot1 3.0 -1.0 45
  ros2 run diff_drive_robot mission_server.py status
  ros2 run diff_drive_robot mission_server.py cancel
  ros2 run diff_drive_robot mission_server.py locations

Or via topic (JSON):
  ros2 topic pub --once /mission/execute std_msgs/msg/String \\
    '{data: "{\"type\":\"goto\",\"robot\":\"robot1\",\"waypoints\":[[\"room_a\"]]}"}'

State is published as JSON to /mission/state at 1 Hz.

FSM states
──────────
  IDLE        No active mission.
  NAVIGATING  Executing a waypoint.
  RECOVERING  Waypoint failed; patrol retrying next cycle.
  DONE        Sequence/goto completed successfully.
  FAILED      Mission aborted after unrecoverable failure.
"""

import json
import math
import os
import sys
import threading
import time
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False

# ── FSM state constants ────────────────────────────────────────────────────────
IDLE       = 'IDLE'
NAVIGATING = 'NAVIGATING'
RECOVERING = 'RECOVERING'
DONE       = 'DONE'
FAILED     = 'FAILED'


# ── Location helpers ───────────────────────────────────────────────────────────

def _locations_path() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(
            get_package_share_directory('diff_drive_robot'), 'config', 'locations.yaml')
    except Exception:
        return os.path.join(
            os.path.expanduser('~'), 'rosnav', 'src',
            'diff_drive_robot-main', 'config', 'locations.yaml')


def _load_locations(path: str = '') -> dict:
    if not HAS_YAML:
        return {}
    candidates = [path] if path else []
    candidates += [_locations_path(),
                   os.path.join(os.path.expanduser('~'), 'rosnav', 'locations.yaml')]
    for p in candidates:
        if p and os.path.isfile(p):
            with open(p) as f:
                data = yaml.safe_load(f) or {}
            return data.get('locations', {})
    return {}


def _resolve_wp(wp, locations: dict) -> list:
    """Return [x, y, yaw_deg] given a name string or a coordinate list/tuple."""
    if isinstance(wp, str):
        if wp not in locations:
            raise KeyError(f'Unknown location: {wp!r}. Known: {list(locations)}')
        return list(locations[wp])
    return [float(v) for v in wp]


# ── Pose helper ────────────────────────────────────────────────────────────────

def _make_pose(x: float, y: float, yaw_deg: float, stamp) -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp    = stamp
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    yaw = math.radians(float(yaw_deg))
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def _action_topic(ns: str) -> str:
    return f'/{ns}/navigate_to_pose' if ns else 'navigate_to_pose'


# ── Daemon node ────────────────────────────────────────────────────────────────

class MissionServer(Node):
    """Persistent daemon — receive missions via topic, execute via Nav2."""

    def __init__(self):
        super().__init__('mission_server')

        self.declare_parameter('goal_timeout', 120.0)
        self.declare_parameter('locations_file', '')

        self._goal_timeout = self.get_parameter('goal_timeout').value
        loc_file           = self.get_parameter('locations_file').value
        self._locations    = _load_locations(loc_file)

        self._lock        = threading.Lock()
        self._clients: dict[str, ActionClient] = {}
        self._missions: dict[str, dict] = {}

        self.create_subscription(String, '/mission/execute', self._mission_cb, 10)
        self._state_pub = self.create_publisher(String, '/mission/state', 10)
        self.create_timer(1.0, self._publish_state)

        loc_count = len(self._locations)
        self.get_logger().info(
            f'MissionServer ready — {loc_count} named location(s) loaded. '
            f'Listening on /mission/execute')

    # ── Action client (one per robot namespace) ────────────────────────────────

    def _client(self, ns: str) -> ActionClient:
        if ns not in self._clients:
            self._clients[ns] = ActionClient(self, NavigateToPose, _action_topic(ns))
        return self._clients[ns]

    def _ctx(self, ns: str) -> dict:
        if ns not in self._missions:
            self._missions[ns] = {
                'state': IDLE,
                'mission': {},
                'wp_idx': 0,
                'wp_labels': [],
                'goal_handle': None,
                'mission_gen': 0,
                'current_pose': None,
            }
        return self._missions[ns]

    # ── Mission intake ─────────────────────────────────────────────────────────

    def _mission_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Bad mission JSON: {e}')
            return

        if data.get('action', 'start').lower() == 'cancel':
            self._cancel_current()
            return

        mtype = data.get('type', '').lower()
        robot = data.get('robot', '')
        wps   = data.get('waypoints', [])

        if mtype == 'goto':
            pose = data.get('pose', wps[0] if wps else None)
            if pose is None:
                self.get_logger().error('goto requires pose: [x, y, yaw_deg] or "location_name"')
                return
            wps = [pose]

        if mtype not in ('patrol', 'sequence', 'goto'):
            self.get_logger().error(f'Unknown mission type: {mtype!r}')
            return

        if not wps:
            self.get_logger().error('Mission has no waypoints.')
            return

        # Resolve named locations → coordinates
        try:
            resolved = [_resolve_wp(wp, self._locations) for wp in wps]
        except KeyError as e:
            self.get_logger().error(str(e))
            return

        # Build human-readable labels for state publishing
        labels = [wp if isinstance(wp, str) else '' for wp in wps]

        with self._lock:
            ctx = self._ctx(robot)
            had_active = ctx['state'] in (NAVIGATING, RECOVERING)

        if had_active:
            self._cancel_current(robot, log_msg=False)

        with self._lock:
            ctx = self._ctx(robot)
            ctx['mission_gen'] += 1
            my_gen              = ctx['mission_gen']
            ctx['mission']      = {'type': mtype, 'waypoints': resolved}
            ctx['wp_labels']    = labels
            ctx['wp_idx']       = 0
            ctx['state']        = NAVIGATING
            ctx['current_pose'] = resolved[0]
            ctx['goal_handle']  = None

        self.get_logger().info(
            f'Mission accepted: type={mtype}  robot={robot or "/"}  '
            f'waypoints={len(resolved)}')

        t = threading.Thread(target=self._run_mission, args=(robot, my_gen), daemon=True)
        t.start()

    # ── Mission executor (runs in background thread) ───────────────────────────

    def _run_mission(self, ns: str, my_gen: int):
        with self._lock:
            ctx     = self._ctx(ns)
            mission = dict(ctx['mission'])

        mtype  = mission['type']
        wps    = mission['waypoints']
        client = self._client(ns)

        self.get_logger().info(f'Waiting for Nav2 ({ns or "/"}) …')
        if not client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('Nav2 not available — mission aborted.')
            with self._lock:
                if self._ctx(ns)['mission_gen'] == my_gen:
                    self._ctx(ns)['state'] = FAILED
            return

        loop = True
        while loop:
            for i, wp in enumerate(wps):
                with self._lock:
                    ctx = self._ctx(ns)
                    if ctx['mission_gen'] != my_gen:
                        return
                    ctx['wp_idx'] = i
                    ctx['state'] = NAVIGATING

                x   = float(wp[0])
                y   = float(wp[1])
                yaw = float(wp[2]) if len(wp) > 2 else 0.0

                with self._lock:
                    ctx = self._ctx(ns)
                    ctx['current_pose'] = [x, y, yaw]
                    labels = ctx['wp_labels']
                    label = labels[i] if i < len(labels) else ''
                loc_str = f' ({label})' if label else ''
                self.get_logger().info(
                    f'→ waypoint {i + 1}/{len(wps)}{loc_str}: '
                    f'({x:.2f}, {y:.2f}, {yaw:.0f}°)')

                ok = self._go(ns, client, x, y, yaw, my_gen)
                if not ok:
                    with self._lock:
                        if self._ctx(ns)['mission_gen'] != my_gen:
                            return
                    self.get_logger().warn(
                        f'Waypoint {i + 1} unreachable.  '
                        f'{"Continuing patrol — RECOVERING." if mtype == "patrol" else "Mission failed."}')
                    if mtype != 'patrol':
                        with self._lock:
                            if self._ctx(ns)['mission_gen'] == my_gen:
                                self._ctx(ns)['state'] = FAILED
                        return
                    # Patrol: mark RECOVERING before the next loop iteration
                    with self._lock:
                        if self._ctx(ns)['mission_gen'] == my_gen:
                            self._ctx(ns)['state'] = RECOVERING

            if mtype != 'patrol':
                loop = False

        with self._lock:
            if self._ctx(ns)['mission_gen'] == my_gen:
                self._ctx(ns)['state'] = DONE
        self.get_logger().info('Mission complete.')

    def _go(self, ns: str, client: ActionClient, x: float, y: float, yaw: float, my_gen: int) -> bool:
        goal      = NavigateToPose.Goal()
        goal.pose = _make_pose(x, y, yaw, self.get_clock().now().to_msg())

        future   = client.send_goal_async(goal)
        deadline = time.time() + 15.0
        while not future.done():
            if time.time() > deadline:
                self.get_logger().warn('Goal acceptance timed out (15 s).')
                return False
            time.sleep(0.05)

        handle = future.result()
        if handle is None or not handle.accepted:
            return False

        with self._lock:
            ctx = self._ctx(ns)
            if ctx['mission_gen'] != my_gen:
                handle.cancel_goal_async()
                return False
            ctx['goal_handle'] = handle

        result_future  = handle.get_result_async()
        nav_deadline   = time.time() + self._goal_timeout
        while not result_future.done():
            with self._lock:
                superseded = self._ctx(ns)['mission_gen'] != my_gen
            if superseded:
                handle.cancel_goal_async()
                return False
            if time.time() > nav_deadline:
                self.get_logger().warn(
                    f'Goal timeout ({self._goal_timeout:.0f}s) — cancelling.')
                handle.cancel_goal_async()
                return False
            time.sleep(0.1)

        return result_future.result().status == GoalStatus.STATUS_SUCCEEDED

    def _cancel_current(self, robot: str = '', log_msg: bool = True):
        with self._lock:
            targets = [robot] if robot in self._missions else list(self._missions)
            cancelled = []
            handles = []
            for ns in targets:
                ctx = self._ctx(ns)
                ctx['mission_gen'] += 1
                ctx['state'] = IDLE
                ctx['mission'] = {}
                ctx['wp_idx'] = 0
                ctx['wp_labels'] = []
                ctx['current_pose'] = None
                if ctx['goal_handle'] is not None:
                    handles.append(ctx['goal_handle'])
                ctx['goal_handle'] = None
                cancelled.append(ns)
        for gh in handles:
            gh.cancel_goal_async()
        if log_msg and cancelled:
            scope = ', '.join(cancelled)
            self.get_logger().info(f'Mission cancelled for: {scope}')

    # ── State publisher ────────────────────────────────────────────────────────

    def _publish_state(self):
        with self._lock:
            snapshots = []
            for ns, ctx in self._missions.items():
                mission = ctx['mission']
                idx = ctx['wp_idx']
                labels = ctx['wp_labels']
                snapshots.append({
                    'state':    ctx['state'],
                    'type':     mission.get('type', ''),
                    'robot':    ns,
                    'wp_index': idx,
                    'wp_total': len(mission.get('waypoints', [])),
                    'location': labels[idx] if labels and idx < len(labels) else '',
                    'pose':     ctx['current_pose'],
                })
        for payload in snapshots:
            msg      = String()
            msg.data = json.dumps(payload)
            self._state_pub.publish(msg)


# ── CLI helpers ────────────────────────────────────────────────────────────────

def _send(node: Node, payload: dict):
    pub = node.create_publisher(String, '/mission/execute', 10)
    time.sleep(0.5)
    msg      = String()
    msg.data = json.dumps(payload)
    pub.publish(msg)
    time.sleep(0.3)
    print(f'Sent: {payload}')


def _status(node: Node, robot_filter: str = ''):
    received: dict[str, dict] = {}

    def _cb(msg):
        data = json.loads(msg.data)
        robot = data.get('robot', '')
        if robot_filter and robot != robot_filter:
            return
        received[robot or '/'] = data

    node.create_subscription(String, '/mission/state', _cb, 10)
    spin = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin.start()
    deadline = time.time() + 3.0
    while not received and time.time() < deadline:
        time.sleep(0.05)

    if received:
        print('\n── Mission State ─────────────────────────')
        for robot in sorted(received):
            s = received[robot]
            print(f'  Robot   : {s["robot"] or "/"}')
            print(f'  State   : {s["state"]}')
            print(f'  Type    : {s["type"] or "—"}')
            if s['wp_total']:
                loc = f' ({s["location"]})' if s.get('location') else ''
                print(f'  Progress: {s["wp_index"] + 1}/{s["wp_total"]}{loc}')
        print('──────────────────────────────────────────\n')
    else:
        print('No mission server found (is it running as a daemon?)')


def _parse_wp(s: str, locations: dict):
    """Parse 'x,y[,yaw]' string or a location name."""
    if ',' in s:
        return [float(v) for v in s.split(',')]
    # location name — pass as string; server will resolve it
    if s in locations:
        return s
    # Try to interpret as a bare float (single number makes no sense as WP, error)
    raise ValueError(
        f'Not a valid waypoint or location name: {s!r}. '
        f'Known locations: {list(locations)}')


def _usage():
    print(__doc__)
    sys.exit(0)


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    argv = remove_ros_args(args=sys.argv)[1:]

    if not argv or argv[0] == '--daemon':
        rclpy.init()
        node = MissionServer()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            try:
                rclpy.shutdown()
            except Exception:
                pass
        return

    locations = _load_locations()

    rclpy.init()
    node = Node('mission_server_cli')
    spin = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin.start()
    time.sleep(0.5)

    cmd = argv[0].lower()

    if cmd == 'status':
        _status(node, argv[1] if len(argv) > 1 else '')

    elif cmd == 'cancel':
        _send(node, {'type': 'patrol', 'action': 'cancel', 'robot': '', 'waypoints': []})

    elif cmd == 'locations':
        if not locations:
            print('No locations.yaml found.')
        else:
            print('\n── Known Locations ───────────────────────')
            for name, coords in locations.items():
                print(f'  {name:<18} {coords}')
            print('──────────────────────────────────────────\n')

    elif cmd in ('patrol', 'sequence'):
        if len(argv) < 3:
            print(f'Usage: mission_server.py {cmd} <robot_ns> <wp_or_name> …')
            sys.exit(1)
        robot = argv[1]
        wps   = [_parse_wp(w, locations) for w in argv[2:]]
        _send(node, {'type': cmd, 'robot': robot, 'waypoints': wps})

    elif cmd == 'goto':
        if len(argv) < 3:
            print('Usage: mission_server.py goto <robot_ns> <location_name>|<x> <y> [yaw_deg]')
            sys.exit(1)
        robot = argv[1]
        # Detect: single location name vs raw x y [yaw]
        try:
            float(argv[2])
            is_numeric = True
        except ValueError:
            is_numeric = False

        if is_numeric:
            if len(argv) < 4:
                print('Usage: mission_server.py goto <robot_ns> <x> <y> [yaw_deg]')
                sys.exit(1)
            yaw  = float(argv[4]) if len(argv) > 4 else 0.0
            pose = [float(argv[2]), float(argv[3]), yaw]
        else:
            loc = argv[2]
            if loc not in locations:
                print(f'Unknown location: {loc!r}. Known: {list(locations)}')
                sys.exit(1)
            pose = locations[loc]

        _send(node, {'type': 'goto', 'robot': robot, 'pose': pose, 'waypoints': []})

    else:
        _usage()

    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
