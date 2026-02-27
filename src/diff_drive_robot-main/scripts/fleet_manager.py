#!/usr/bin/env python3
"""
fleet_manager.py — Product-like fleet management CLI.

Commands
────────
  list           List all active robots and their states
  status         Show map, SLAM, and Nav2 status
  add <ns> <x> <y>   Dynamically spawn a robot into a running simulation
  remove <ns>    Kill a robot's Nav2 stack (Gazebo entity stays)
  teleop <ns>    Interactive keyboard control for one robot
  goto <ns> <x> <y> [yaw]   Send a navigation goal
  explore <ns>   Start frontier exploration on a robot
  stop <ns>      Stop navigation / teleop for a robot
  savemap [path] Save the current SLAM map
  help           Show this help

Usage
─────
  ros2 run diff_drive_robot fleet_manager.py list
  ros2 run diff_drive_robot fleet_manager.py add robot3 1.0 2.0
  ros2 run diff_drive_robot fleet_manager.py teleop robot1
  ros2 run diff_drive_robot fleet_manager.py goto robot2 3.0 -1.0
  ros2 run diff_drive_robot fleet_manager.py savemap /tmp/my_map
  ros2 run diff_drive_robot fleet_manager.py explore robot2
"""

import math
import subprocess
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav2_msgs.action import NavigateToPose

# ─────────────────────────────────────────────────────────────────────────────
LINEAR_SPEED  = 0.22
ANGULAR_SPEED = 1.0
STOP_TIMEOUT  = 0.3
PKG = 'diff_drive_robot'


# ── Helpers ───────────────────────────────────────────────────────────────────
def _getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            ch += sys.stdin.read(2)
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _discover_robots(node: Node) -> list[str]:
    topics = node.get_topic_names_and_types()
    return sorted({
        t.split('/')[1]
        for t, _ in topics
        if t.count('/') >= 2 and t.endswith('/cmd_vel')
    })


def _yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


# ─────────────────────────────────────────────────────────────────────────────
class FleetNode(Node):
    def __init__(self):
        super().__init__('fleet_manager')
        self._pubs: dict[str, any] = {}
        self._map_received = False

        qos = QoSProfile(depth=1,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, qos)

    def _map_cb(self, _):
        self._map_received = True

    def _pub(self, ns: str):
        if ns not in self._pubs:
            self._pubs[ns] = self.create_publisher(
                Twist, f'/{ns}/cmd_vel', 10)
        return self._pubs[ns]

    def _send_vel(self, ns: str, lin: float, ang: float):
        msg = Twist()
        msg.linear.x  = lin
        msg.angular.z = ang
        self._pub(ns).publish(msg)

    # ── Commands ──────────────────────────────────────────────────────────────
    def cmd_list(self):
        robots = _discover_robots(self)
        if not robots:
            print('No active robots found.')
            return
        print(f'\n{"Robot":<12}  {"cmd_vel topic":<28}  {"scan topic"}')
        print('─' * 60)
        topics = {t for t, _ in self.get_topic_names_and_types()}
        for ns in robots:
            scan = '✓' if f'/{ns}/scan' in topics else '✗'
            print(f'  {ns:<10}  /{ns}/cmd_vel                  scan={scan}')
        print()

    def cmd_status(self):
        robots = _discover_robots(self)
        nodes = [n for n in self.get_node_names() if n]
        slam  = any('slam_toolbox' in n.lower() for n in nodes)
        nav   = any('bt_navigator' in n.lower() for n in nodes)
        print('\n── System Status ─────────────────────────────────────')
        print(f'  Active robots : {robots or "none"}')
        print(f'  SLAM running  : {"✓" if slam else "✗"}')
        print(f'  Nav2 running  : {"✓" if nav else "✗"}')
        print(f'  /map available: {"✓" if self._map_received else "✗"}')
        print('──────────────────────────────────────────────────────\n')

    def cmd_add(self, ns: str, x: float, y: float):
        """Dynamically spawn robot + bridge into a running sim."""
        print(f'\nSpawning {ns} at ({x:.1f}, {y:.1f}) …')

        # RSP
        subprocess.Popen([
            'ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
            '--ros-args',
            '-r', f'__ns:=/{ns}',
            '-p', 'use_sim_time:=true',
            '-p', f'frame_prefix:={ns}/',
            '-p', f'robot_description:=$(ros2 run xacro xacro '
                  f'$(ros2 pkg prefix {PKG})/share/{PKG}/urdf/robot.urdf.xacro)',
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(1.5)

        # Spawn
        r = subprocess.run([
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-topic', f'/{ns}/robot_description',
            '-name', ns,
            '-x', str(x), '-y', str(y), '-z', '0.3', '-Y', '0.0',
        ], capture_output=True, text=True)
        if 'OK' in r.stdout or r.returncode == 0:
            print(f'  Gazebo: {ns} spawned')
        else:
            print(f'  Gazebo spawn output: {r.stdout.strip() or r.stderr.strip()}')

        # Bridge
        subprocess.Popen([
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '--ros-args', '-r', f'__ns:=/{ns}', '--',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            f'/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            f'/{ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            f'/{ns}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        print(f'  {ns} bridge started.')
        print(f'  To start Nav2 for {ns}, add it to the ROBOTS list and relaunch.')
        print(f'  Or use: ros2 launch {PKG} multi_robot.launch.py explore:=false')

    def cmd_goto(self, ns: str, x: float, y: float, yaw: float = 0.0):
        client = ActionClient(self, NavigateToPose, f'/{ns}/navigate_to_pose')
        print(f'Waiting for {ns} navigate_to_pose action server … ', end='', flush=True)
        if not client.wait_for_server(timeout_sec=5.0):
            print('timeout! Is Nav2 running for this robot?')
            return
        print('connected.')

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        ox, oy, oz, ow = _yaw_to_quat(yaw)
        goal.pose.pose.orientation.x = ox
        goal.pose.pose.orientation.y = oy
        goal.pose.pose.orientation.z = oz
        goal.pose.pose.orientation.w = ow

        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            print('Goal rejected.')
            return
        print(f'Goal sent to {ns} → ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.1f}°)')

    def cmd_stop(self, ns: str):
        self._send_vel(ns, 0.0, 0.0)
        client = ActionClient(self, NavigateToPose, f'/{ns}/navigate_to_pose')
        if client.wait_for_server(timeout_sec=1.0):
            client.cancel_goal_async(None)
        print(f'{ns} stopped.')

    def cmd_explore(self, ns: str):
        print(f'Starting frontier explorer for {ns} …')
        subprocess.Popen([
            'ros2', 'run', PKG, 'frontier_explorer.py',
            '--ros-args',
            '-r', f'__ns:=/{ns}',
            '-p', f'base_frame:={ns}/base_link',
        ])
        print(f'Frontier explorer launched for {ns}.')

    def cmd_savemap(self, path: str):
        import os as _os
        _os.makedirs(_os.path.dirname(path) if '/' in path else '.', exist_ok=True)
        print(f'Saving map to {path} …')
        r = subprocess.run(
            ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', path],
            capture_output=True, text=True)
        if r.returncode == 0:
            print(f'Map saved: {path}.yaml / {path}.pgm')
        else:
            print(f'Map save failed: {r.stderr.strip()}')

    def cmd_teleop(self, ns: str):
        """Interactive keyboard teleop for one robot."""
        self.cmd_stop(ns)   # cancel any active nav goal first
        print(f'\n── Teleop {ns} ──────────────────────────────────────')
        print('  W/S/A/D or arrows — move  |  Space — stop')
        print('  Q — back to shell')
        print('─────────────────────────────────────────────────────')

        KEY_MAP = {
            'w': ( LINEAR_SPEED, 0.0),   's': (-LINEAR_SPEED, 0.0),
            'a': (0.0,  ANGULAR_SPEED),  'd': (0.0, -ANGULAR_SPEED),
            '\x1b[a': ( LINEAR_SPEED, 0.0),
            '\x1b[b': (-LINEAR_SPEED, 0.0),
            '\x1b[d': (0.0,  ANGULAR_SPEED),
            '\x1b[c': (0.0, -ANGULAR_SPEED),
            ' ': (0.0, 0.0),
        }

        last_t = [time.time()]

        def watchdog():
            while True:
                time.sleep(0.05)
                if time.time() - last_t[0] > STOP_TIMEOUT:
                    self._send_vel(ns, 0.0, 0.0)

        wd = threading.Thread(target=watchdog, daemon=True)
        wd.start()

        while True:
            ch = _getch().lower()
            if ch == 'q':
                break
            if ch in KEY_MAP:
                lin, ang = KEY_MAP[ch]
                self._send_vel(ns, lin, ang)
                last_t[0] = time.time()

        self._send_vel(ns, 0.0, 0.0)
        print(f'Teleop ended for {ns}.')


# ─────────────────────────────────────────────────────────────────────────────
def _usage():
    print(__doc__)
    sys.exit(0)


def main():
    args = sys.argv[1:]
    if not args or args[0] in ('-h', '--help', 'help'):
        _usage()

    rclpy.init()
    node = FleetNode()

    spin = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True)
    spin.start()

    time.sleep(0.5)   # let discovery settle

    cmd = args[0].lower()

    if cmd == 'list':
        node.cmd_list()

    elif cmd == 'status':
        node.cmd_status()

    elif cmd == 'add':
        if len(args) < 4:
            print('Usage: fleet_manager.py add <namespace> <x> <y>')
            sys.exit(1)
        node.cmd_add(args[1], float(args[2]), float(args[3]))

    elif cmd == 'teleop':
        if len(args) < 2:
            print('Usage: fleet_manager.py teleop <namespace>')
            sys.exit(1)
        node.cmd_teleop(args[1])

    elif cmd == 'goto':
        if len(args) < 4:
            print('Usage: fleet_manager.py goto <namespace> <x> <y> [yaw_deg]')
            sys.exit(1)
        yaw = math.radians(float(args[4])) if len(args) > 4 else 0.0
        node.cmd_goto(args[1], float(args[2]), float(args[3]), yaw)

    elif cmd == 'stop':
        if len(args) < 2:
            print('Usage: fleet_manager.py stop <namespace>')
            sys.exit(1)
        node.cmd_stop(args[1])

    elif cmd == 'explore':
        if len(args) < 2:
            print('Usage: fleet_manager.py explore <namespace>')
            sys.exit(1)
        node.cmd_explore(args[1])

    elif cmd == 'savemap':
        path = args[1] if len(args) > 1 else '/tmp/fleet_map'
        node.cmd_savemap(path)

    elif cmd == 'remove':
        if len(args) < 2:
            print('Usage: fleet_manager.py remove <namespace>')
            sys.exit(1)
        node.cmd_stop(args[1])
        print(f'Nav2 stopped for {args[1]}. Gazebo entity remains.')

    else:
        print(f'Unknown command: {cmd}')
        _usage()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
