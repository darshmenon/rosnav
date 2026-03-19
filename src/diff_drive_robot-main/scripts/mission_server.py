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

Daemon mode (run once, persists):
  ros2 run diff_drive_robot mission_server.py

Send a mission from the command line (daemon must be running):
  ros2 run diff_drive_robot mission_server.py patrol  robot1 1,2,0  3,4,90
  ros2 run diff_drive_robot mission_server.py sequence robot1 0,0,0 2,0,0 2,2,90
  ros2 run diff_drive_robot mission_server.py goto    robot1 3.0 -1.0 45
  ros2 run diff_drive_robot mission_server.py status
  ros2 run diff_drive_robot mission_server.py cancel

Or via topic (JSON):
  ros2 topic pub --once /mission/execute std_msgs/msg/String \\
    '{data: "{\"type\":\"patrol\",\"robot\":\"robot1\",\"waypoints\":[[1,2,0],[3,4,90]]}"}'

State is published as JSON to /mission/state at 1 Hz.
"""

import json
import math
import sys
import threading
import time
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

# ── State constants ────────────────────────────────────────────────────────────
IDLE       = 'IDLE'
NAVIGATING = 'NAVIGATING'
DONE       = 'DONE'
FAILED     = 'FAILED'


# ── Helpers ────────────────────────────────────────────────────────────────────

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
        self._goal_timeout = self.get_parameter('goal_timeout').value

        self._lock        = threading.Lock()
        self._state       = IDLE
        self._mission     = {}
        self._wp_idx      = 0
        self._robot_ns    = ''
        self._goal_handle = None
        self._clients: dict[str, ActionClient] = {}
        self._active_thread: Optional[threading.Thread] = None
        # Incremented each time a new mission starts; threads check this to
        # know if they have been superseded and should exit.
        self._mission_gen  = 0

        self.create_subscription(String, '/mission/execute', self._mission_cb, 10)
        self._state_pub = self.create_publisher(String, '/mission/state', 10)
        self.create_timer(1.0, self._publish_state)

        self.get_logger().info('MissionServer ready — listening on /mission/execute')

    # ── Action client (one per robot namespace) ────────────────────────────────

    def _client(self, ns: str) -> ActionClient:
        if ns not in self._clients:
            self._clients[ns] = ActionClient(self, NavigateToPose, _action_topic(ns))
        return self._clients[ns]

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
                self.get_logger().error('goto requires pose: [x, y, yaw_deg]')
                return
            wps = [pose]

        if mtype not in ('patrol', 'sequence', 'goto'):
            self.get_logger().error(f'Unknown mission type: {mtype!r}')
            return

        if not wps:
            self.get_logger().error('Mission has no waypoints.')
            return

        self._cancel_current()    # abort any running mission first

        with self._lock:
            self._mission_gen += 1
            my_gen             = self._mission_gen
            self._mission      = {'type': mtype, 'waypoints': wps}
            self._robot_ns     = robot
            self._wp_idx       = 0
            self._state        = NAVIGATING

        self.get_logger().info(
            f'Mission accepted: type={mtype}  robot={robot or "/"}  '
            f'waypoints={len(wps)}')

        t = threading.Thread(target=self._run_mission, args=(my_gen,), daemon=True)
        self._active_thread = t
        t.start()

    # ── Mission executor (runs in background thread) ───────────────────────────

    def _run_mission(self, my_gen: int):
        with self._lock:
            mission = dict(self._mission)
            ns      = self._robot_ns

        mtype  = mission['type']
        wps    = mission['waypoints']
        client = self._client(ns)

        self.get_logger().info(f'Waiting for Nav2 ({ns or "/"}) …')
        if not client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('Nav2 not available — mission aborted.')
            with self._lock:
                if self._mission_gen == my_gen:
                    self._state = FAILED
            return

        loop = True
        while loop:
            for i, wp in enumerate(wps):
                # Check if this thread has been superseded by a newer mission
                with self._lock:
                    if self._mission_gen != my_gen:
                        return

                with self._lock:
                    self._wp_idx = i

                x   = float(wp[0])
                y   = float(wp[1])
                yaw = float(wp[2]) if len(wp) > 2 else 0.0

                self.get_logger().info(
                    f'→ waypoint {i + 1}/{len(wps)}: '
                    f'({x:.2f}, {y:.2f}, {yaw:.0f}°)')

                ok = self._go(client, x, y, yaw, my_gen)
                if not ok:
                    # Check if cancelled/superseded before marking failed
                    with self._lock:
                        if self._mission_gen != my_gen:
                            return
                    self.get_logger().warn(
                        f'Waypoint {i + 1} unreachable.  '
                        f'{"Continuing patrol loop." if mtype == "patrol" else "Mission failed."}')
                    if mtype != 'patrol':
                        with self._lock:
                            if self._mission_gen == my_gen:
                                self._state = FAILED
                        return

            if mtype != 'patrol':
                loop = False

        with self._lock:
            if self._mission_gen == my_gen:
                self._state = DONE
        self.get_logger().info('Mission complete.')

    def _go(self, client: ActionClient, x: float, y: float, yaw: float, my_gen: int) -> bool:
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
            self._goal_handle = handle

        result_future  = handle.get_result_async()
        nav_deadline   = time.time() + self._goal_timeout
        while not result_future.done():
            with self._lock:
                superseded = self._mission_gen != my_gen
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

    def _cancel_current(self):
        with self._lock:
            self._mission_gen += 1   # invalidates any running _run_mission thread
            self._state = IDLE
            gh = self._goal_handle
        if gh is not None:
            gh.cancel_goal_async()
        self.get_logger().info('Mission cancelled.')

    # ── State publisher ────────────────────────────────────────────────────────

    def _publish_state(self):
        with self._lock:
            m = self._mission
            payload = {
                'state':    self._state,
                'type':     m.get('type', ''),
                'robot':    self._robot_ns,
                'wp_index': self._wp_idx,
                'wp_total': len(m.get('waypoints', [])),
            }
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


def _status(node: Node):
    received = [None]

    def _cb(msg):
        received[0] = json.loads(msg.data)

    node.create_subscription(String, '/mission/state', _cb, 10)
    deadline = time.time() + 3.0
    # The spin thread handles callbacks; just wait here without calling spin again.
    while received[0] is None and time.time() < deadline:
        time.sleep(0.05)

    if received[0]:
        s = received[0]
        print('\n── Mission State ─────────────────────────')
        print(f'  State   : {s["state"]}')
        print(f'  Type    : {s["type"] or "—"}')
        print(f'  Robot   : {s["robot"] or "/"}')
        if s['wp_total']:
            print(f'  Progress: {s["wp_index"] + 1}/{s["wp_total"]}')
        print('──────────────────────────────────────────\n')
    else:
        print('No mission server found (is it running as a daemon?)')


def _parse_wp(s: str):
    return [float(v) for v in s.split(',')]


def _usage():
    print(__doc__)
    sys.exit(0)


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    argv = sys.argv[1:]

    # No args or explicit --daemon → run as persistent daemon
    if not argv or argv[0] == '--daemon':
        rclpy.init()
        node = MissionServer()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return

    rclpy.init()
    node = Node('mission_server_cli')
    spin = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin.start()
    time.sleep(0.5)

    cmd = argv[0].lower()

    if cmd == 'status':
        _status(node)

    elif cmd == 'cancel':
        _send(node, {'type': 'patrol', 'action': 'cancel', 'robot': '', 'waypoints': []})

    elif cmd in ('patrol', 'sequence'):
        if len(argv) < 3:
            print(f'Usage: mission_server.py {cmd} <robot_ns> x1,y1[,yaw] …')
            sys.exit(1)
        robot = argv[1]
        wps   = [_parse_wp(w) for w in argv[2:]]
        _send(node, {'type': cmd, 'robot': robot, 'waypoints': wps})

    elif cmd == 'goto':
        if len(argv) < 4:
            print('Usage: mission_server.py goto <robot_ns> <x> <y> [yaw_deg]')
            sys.exit(1)
        robot = argv[1]
        yaw   = float(argv[4]) if len(argv) > 4 else 0.0
        _send(node, {
            'type':      'goto',
            'robot':     robot,
            'pose':      [float(argv[2]), float(argv[3]), yaw],
            'waypoints': [],
        })

    else:
        _usage()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
