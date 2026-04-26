#!/usr/bin/env python3
"""
multi_teleop.py — Interactive teleop for multi-robot setups.

Features
────────
• List all active robots and select one interactively.
• Drive the selected robot with keyboard (WASD / arrow keys).
• Switch robots on the fly without restarting.
• Dynamically spawn a new robot into Gazebo while the simulation is running.
• Cancel autonomous navigation before taking manual control.

Usage
─────
  ros2 run diff_drive_robot multi_teleop.py

Controls (while driving)
────────────────────────
  W / ↑   — forward
  S / ↓   — backward
  A / ←   — turn left
  D / →   — turn right
  Space   — stop
  R       — return to robot select menu
  N       — spawn a NEW robot (dynamic add)
  Q       — quit
"""

import math
import os
import subprocess
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

# ── Tuning ───────────────────────────────────────────────────────────────────
LINEAR_SPEED  = 0.22   # m/s
ANGULAR_SPEED = 1.0    # rad/s
STOP_TIMEOUT  = 0.3    # s without input → publish zero velocity

# Gazebo spawn settings for dynamic robot addition
SPAWN_Z   = 0.3
SPAWN_YAW = 0.0
PKG       = 'diff_drive_robot'


def _get_char():
    """Read one keypress without echo."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == '\x1b':            # ESC sequence (arrow keys)
            ch2 = sys.stdin.read(2)
            return ch + ch2
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _discover_robots(node: Node) -> list[str]:
    """Find namespaces that have an active cmd_vel topic."""
    topics = node.get_topic_names_and_types()
    robots = sorted({
        t.split('/')[1]
        for t, _ in topics
        if t.count('/') >= 2 and t.endswith('/cmd_vel')
    })
    return robots


def _cancel_nav(node: Node, ns: str):
    """Cancel any running NavigateToPose goal for this robot."""
    client = ActionClient(node, NavigateToPose, f'/{ns}/navigate_to_pose')
    if client.wait_for_server(timeout_sec=0.5):
        client.cancel_goal_async(None)


class MultiTeleop(Node):
    def __init__(self):
        super().__init__('multi_teleop')
        self._publishers: dict[str, any] = {}   # ns → Publisher
        self._current_ns: str | None = None
        self._last_cmd_time = 0.0
        self._running = True

        # Watchdog: send zero cmd_vel after STOP_TIMEOUT of no input
        self._watchdog = threading.Thread(target=self._watchdog_loop, daemon=True)
        self._watchdog.start()

    # ── Publisher helpers ─────────────────────────────────────────────────────
    def _get_publisher(self, ns: str):
        if ns not in self._publishers:
            self._publishers[ns] = self.create_publisher(
                Twist, f'/{ns}/cmd_vel', 10)
        return self._publishers[ns]

    def _publish(self, ns: str, linear: float, angular: float):
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self._get_publisher(ns).publish(msg)
        self._last_cmd_time = time.time()

    def _stop(self, ns: str):
        self._publish(ns, 0.0, 0.0)

    def _watchdog_loop(self):
        while self._running:
            time.sleep(0.05)
            ns = self._current_ns
            if ns and time.time() - self._last_cmd_time > STOP_TIMEOUT:
                self._publish(ns, 0.0, 0.0)

    # ── Dynamic robot spawner ─────────────────────────────────────────────────
    def spawn_robot(self, ns: str, x: float, y: float):
        """Spawn a new robot into Gazebo and start its Nav2 stack."""
        urdf_cmd = (
            f'ros2 run xacro xacro '
            f'$(ros2 pkg prefix {PKG})/share/{PKG}/urdf/robot.urdf.xacro '
            f'frame_prefix:={ns}/'
        )
        print(f'\nSpawning {ns} at ({x:.1f}, {y:.1f}) …')

        # Publish robot_description
        desc_proc = subprocess.Popen(
            ['bash', '-c',
             f'ros2 run robot_state_publisher robot_state_publisher '
             f'--ros-args -r __ns:=/{ns} '
             f'-p use_sim_time:=true '
             f'-p frame_prefix:={ns}/ '
             f'-p robot_description:="$({urdf_cmd})"'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        time.sleep(1.0)

        # Spawn in Gazebo
        subprocess.run([
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-topic', f'/{ns}/robot_description',
            '-name', ns,
            '-x', str(x), '-y', str(y), '-z', str(SPAWN_Z), '-Y', str(SPAWN_YAW),
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # Bridge (2D lidar + odom + cmd_vel + tf)
        bridge_args = [
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            f'/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            f'/{ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            f'/{ns}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ]
        subprocess.Popen(
            ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             '--ros-args', '-r', f'__ns:=/{ns}', '--'] + bridge_args,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        print(f'{ns} spawned. Nav2 must be started separately.')
        print('  Run: ros2 launch diff_drive_robot multi_robot.launch.py '
              'explore:=false  (with your robot added to the ROBOTS list)')
        print('  Or manually launch navigation for this robot.')

    # ── Select robot ──────────────────────────────────────────────────────────
    def select_robot(self) -> str | None:
        robots = _discover_robots(self)
        if not robots:
            print('\nNo active robots found. '
                  'Is multi_robot.launch.py running?\n')
            return None

        print('\n── Available robots ───────────────────────────────')
        for i, ns in enumerate(robots):
            marker = '*' if ns == self._current_ns else ' '
            print(f'  [{i + 1}]{marker} {ns}')
        print('  [N]  Spawn a NEW robot dynamically')
        print('  [Q]  Quit')
        print('────────────────────────────────────────────────────')
        print('Select: ', end='', flush=True)

        ch = _get_char()
        print(ch)

        if ch.lower() == 'q':
            self._running = False
            return None
        if ch.lower() == 'n':
            return '__spawn__'
        try:
            idx = int(ch) - 1
            if 0 <= idx < len(robots):
                return robots[idx]
        except ValueError:
            pass
        print('Invalid selection.')
        return None

    # ── Spawn wizard ──────────────────────────────────────────────────────────
    def spawn_wizard(self):
        print('\n── Spawn new robot ────────────────────────────────')
        try:
            ns  = input('  Namespace (e.g. robot3): ').strip()
            x   = float(input('  Spawn X position: ').strip())
            y   = float(input('  Spawn Y position: ').strip())
        except (ValueError, EOFError):
            print('Aborted.')
            return
        if not ns:
            print('Aborted.')
            return
        self.spawn_robot(ns, x, y)

    # ── Drive loop ────────────────────────────────────────────────────────────
    def drive_loop(self, ns: str):
        self._current_ns = ns
        _cancel_nav(self, ns)

        print(f'\n── Driving {ns} ─────────────────────────────────────')
        print('  W/S/A/D or arrow keys — move')
        print('  Space — stop  |  R — robot select  |  N — new robot  |  Q — quit')
        print('─────────────────────────────────────────────────────')

        KEY_MAP = {
            'w': ( LINEAR_SPEED, 0.0),
            's': (-LINEAR_SPEED, 0.0),
            'a': (0.0,  ANGULAR_SPEED),
            'd': (0.0, -ANGULAR_SPEED),
            '\x1b[A': ( LINEAR_SPEED, 0.0),   # ↑
            '\x1b[B': (-LINEAR_SPEED, 0.0),   # ↓
            '\x1b[D': (0.0,  ANGULAR_SPEED),  # ←
            '\x1b[C': (0.0, -ANGULAR_SPEED),  # →
            ' ': (0.0, 0.0),
        }

        while self._running:
            ch = _get_char().lower()

            if ch == 'q':
                self._stop(ns)
                self._running = False
                return 'quit'

            if ch == 'r':
                self._stop(ns)
                self._current_ns = None
                return 'select'

            if ch == 'n':
                self._stop(ns)
                self.spawn_wizard()
                continue

            if ch in KEY_MAP:
                lin, ang = KEY_MAP[ch]
                self._publish(ns, lin, ang)

        return 'quit'


def main():
    rclpy.init()
    node = MultiTeleop()

    spin_thread = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    print('Multi-Robot Teleop  (2D LiDAR | Nav2 | Gazebo Harmonic)')
    print('Press Ctrl-C to exit at any time.')

    try:
        while node._running:
            ns = node.select_robot()
            if ns is None:
                break
            if ns == '__spawn__':
                node.spawn_wizard()
                continue
            action = node.drive_loop(ns)
            if action == 'quit':
                break
    except KeyboardInterrupt:
        pass
    finally:
        if node._current_ns:
            node._stop(node._current_ns)
        node._running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
