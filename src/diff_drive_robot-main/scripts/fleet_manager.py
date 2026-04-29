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
  goto <ns> <location>       Send a navigation goal by location name
  goto <ns> <x> <y> [yaw]   Send a navigation goal by coordinates
  explore <ns>   Start frontier exploration on a robot
  stop <ns>      Stop navigation / teleop for a robot
  savemap [path] Save the current SLAM map
  locations      List all named locations from locations.yaml
  mission <ns> patrol <loc_or_wp> …   Loop through waypoints/locations
  mission <ns> sequence <loc_or_wp> … Visit once in order
  mission <ns> goto <location>        Single named-location mission
  mission <ns> goto <x> <y> [yaw]    Single-pose mission
  mission <ns> status                 Show mission state
  mission <ns> cancel                 Cancel active mission
  collision <ns>  Show collision monitor state for a robot
  tasks add <x> <y> <yaw> [label]   Add a task to the shared queue
  tasks status                       Show task queue and robot states
  tasks clear                        Remove all pending tasks
  health         Show per-robot health (odom/scan Hz, nav2, collision)
  help           Show this help

Usage
─────
  ros2 run diff_drive_robot fleet_manager.py list
  ros2 run diff_drive_robot fleet_manager.py locations
  ros2 run diff_drive_robot fleet_manager.py goto robot1 room_a
  ros2 run diff_drive_robot fleet_manager.py goto robot2 3.0 -1.0
  ros2 run diff_drive_robot fleet_manager.py mission robot1 patrol room_a room_b room_c
  ros2 run diff_drive_robot fleet_manager.py mission robot1 patrol 1,2,0 3,4,90
  ros2 run diff_drive_robot fleet_manager.py mission robot1 goto charging_dock
  ros2 run diff_drive_robot fleet_manager.py mission robot1 status
  ros2 run diff_drive_robot fleet_manager.py collision robot1
"""

import json
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
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False

# ─────────────────────────────────────────────────────────────────────────────
LINEAR_SPEED  = 0.22
ANGULAR_SPEED = 1.0
STOP_TIMEOUT  = 0.3
PKG = 'diff_drive_robot'


# ── Location helpers ──────────────────────────────────────────────────────────

def _load_locations() -> dict:
    if not HAS_YAML:
        return {}
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory(PKG)
    except Exception:
        share = os.path.join(
            os.path.expanduser('~'), 'rosnav', 'src', 'diff_drive_robot-main')
    candidates = [
        os.path.join(share, 'config', 'locations.yaml'),
        os.path.join(os.path.expanduser('~'), 'rosnav', 'locations.yaml'),
    ]
    for p in candidates:
        if os.path.isfile(p):
            with open(p) as f:
                data = yaml.safe_load(f) or {}
            return data.get('locations', {})
    return {}


def _resolve_location(arg: str, locations: dict) -> tuple[float, float, float]:
    """Return (x, y, yaw_rad) from a location name."""
    if arg not in locations:
        known = list(locations)
        raise KeyError(f'Unknown location: {arg!r}. Known: {known}')
    coords = locations[arg]
    return float(coords[0]), float(coords[1]), math.radians(float(coords[2]) if len(coords) > 2 else 0.0)


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
        # Goal-handle based cancel is not possible without tracking the handle;
        # zero-vel above halts the robot; use mission cancel to abort Nav2 goals.
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

    def cmd_mission(self, ns: str, sub: str, extra: list[str]):
        """Send a mission to the mission_server daemon or query its state."""
        sub = sub.lower()

        if sub == 'status':
            received = [None]

            def _cb(msg):
                data = json.loads(msg.data)
                if ns and data.get('robot', '') != ns:
                    return
                received[0] = data

            self.create_subscription(String, '/mission/state', _cb, 10)
            deadline = time.time() + 3.0
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
                print('No mission server found (is it running?)')
            return

        pub = self.create_publisher(String, '/mission/execute', 10)
        time.sleep(0.3)

        if sub == 'cancel':
            payload = {'type': 'patrol', 'action': 'cancel', 'robot': ns, 'waypoints': []}

        elif sub in ('patrol', 'sequence'):
            locations = _load_locations()
            wps = []
            for w in extra:
                if ',' in w:
                    wps.append([float(v) for v in w.split(',')])
                elif w in locations:
                    wps.append(w)   # pass name; mission_server resolves it
                else:
                    print(f'Unknown location: {w!r}. Known: {list(locations)}')
                    return
            payload = {'type': sub, 'robot': ns, 'waypoints': wps}

        elif sub == 'goto':
            if not extra:
                print('Usage: mission <ns> goto <location>|<x> <y> [yaw_deg]')
                return
            locations = _load_locations()
            try:
                float(extra[0])
                is_numeric = True
            except ValueError:
                is_numeric = False
            if is_numeric:
                if len(extra) < 2:
                    print('Usage: mission <ns> goto <x> <y> [yaw_deg]')
                    return
                yaw  = float(extra[2]) if len(extra) > 2 else 0.0
                pose = [float(extra[0]), float(extra[1]), yaw]
            else:
                if extra[0] not in locations:
                    print(f'Unknown location: {extra[0]!r}. Known: {list(locations)}')
                    return
                pose = list(locations[extra[0]])
            payload = {
                'type': 'goto', 'robot': ns,
                'pose': pose, 'waypoints': [],
            }
        else:
            print(f'Unknown mission sub-command: {sub}')
            return

        msg = String()
        msg.data = json.dumps(payload)
        pub.publish(msg)
        time.sleep(0.2)
        print(f'Mission sent to {ns}: {payload}')

    def cmd_tasks(self, sub: str, extra: list[str]):
        """Interact with the task_allocator daemon."""
        sub = sub.lower()

        if sub == 'status':
            received = [None]

            def _cb(msg):
                received[0] = json.loads(msg.data)

            self.create_subscription(String, '/task_queue/state', _cb, 10)
            deadline = time.time() + 3.0
            while received[0] is None and time.time() < deadline:
                time.sleep(0.05)
            if received[0]:
                s = received[0]
                tasks = s.get('tasks', [])
                bots  = s.get('robot_states', {})
                print('\n── Task Queue ────────────────────────────────────')
                for t in (tasks or []):
                    robot = f' → {t["robot"]}' if t['robot'] else ''
                    label = f' [{t["label"]}]' if t.get('label') else ''
                    print(f'  {t["id"]}  ({t["x"]:.2f},{t["y"]:.2f})'
                          f'{label}  {t["status"]}{robot}')
                if not tasks:
                    print('  (empty)')
                print('\n── Robots ────────────────────────────────────────')
                for ns, st in (bots.items() if bots else {}).items():
                    print(f'  {ns}: {st}')
                print('──────────────────────────────────────────────────\n')
            else:
                print('No task_allocator found (is it running?)')
            return

        pub = self.create_publisher(String, '/task_queue/add' if sub == 'add' else '/task_queue/clear', 10)
        time.sleep(0.3)
        if sub == 'add':
            if len(extra) < 3:
                print('Usage: tasks add <x> <y> <yaw_deg> [label]')
                return
            payload = {
                'x': float(extra[0]), 'y': float(extra[1]),
                'yaw': float(extra[2]),
                'label': extra[3] if len(extra) > 3 else '',
            }
            msg = String()
            msg.data = json.dumps(payload)
            pub.publish(msg)
            time.sleep(0.2)
            print(f'Task queued: {payload}')
        elif sub == 'clear':
            msg = String()
            msg.data = json.dumps({})
            pub.publish(msg)
            time.sleep(0.2)
            print('Queue cleared.')
        else:
            print(f'Unknown tasks sub-command: {sub}')

    def cmd_health(self):
        """Print fleet health from the fleet_health daemon."""
        received = [None]

        def _cb(msg):
            received[0] = json.loads(msg.data)

        self.create_subscription(String, '/fleet/health', _cb, 10)
        deadline = time.time() + 3.0
        while received[0] is None and time.time() < deadline:
            time.sleep(0.05)

        if received[0]:
            data = received[0]
            print('\n── Fleet Health ──────────────────────────────────────')
            for ns, s in data.items():
                icon = {'OK': 'OK', 'WARN': 'WARN', 'ERROR': 'ERR'}.get(s['overall'], '?')
                print(f'  [{icon}] {ns:<10}  '
                      f'odom={s["odom_hz"]:4.1f}Hz  '
                      f'scan={s["scan_hz"]:4.1f}Hz  '
                      f'nav2={"✓" if s["nav2"] else "✗"}  '
                      f'mission={s["mission"]:<12}  '
                      f'collision={s["collision"]}')
            if not data:
                print('  No robots detected.')
            print('──────────────────────────────────────────────────────\n')
        else:
            print('No fleet_health monitor found (is it running?)')

    def cmd_collision(self, ns: str):
        """Print the latest collision monitor state for a robot."""
        topic    = f'/{ns}/collision_monitor/state' if ns else '/collision_monitor/state'
        received = [None]

        def _cb(msg):
            received[0] = json.loads(msg.data)

        self.create_subscription(String, topic, _cb, 10)
        deadline = time.time() + 3.0
        while received[0] is None and time.time() < deadline:
            time.sleep(0.05)
        if received[0]:
            s = received[0]
            print(f'\n── Collision Monitor [{ns}] ──────────────')
            print(f'  State     : {s["state"]}')
            mr = s["min_range"]
            print(f'  Min range : {mr:.2f} m' if mr >= 0 else '  Min range : — (no scan)')
            print(f'  Stop zone : {s["stop_dist"]} m')
            print(f'  Slow zone : {s["slow_dist"]} m')
            print('──────────────────────────────────────────\n')
        else:
            print(f'No collision monitor found for {ns} (is it running?)')

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

    elif cmd == 'locations':
        locs = _load_locations()
        if not locs:
            print('No locations.yaml found.')
        else:
            print('\n── Known Locations ───────────────────────')
            for name, coords in locs.items():
                print(f'  {name:<18} {coords}')
            print('──────────────────────────────────────────\n')

    elif cmd == 'goto':
        if len(args) < 3:
            print('Usage: fleet_manager.py goto <namespace> <location>|<x> <y> [yaw_deg]')
            sys.exit(1)
        ns = args[1]
        try:
            float(args[2])
            is_numeric = True
        except ValueError:
            is_numeric = False
        if is_numeric:
            if len(args) < 4:
                print('Usage: fleet_manager.py goto <namespace> <x> <y> [yaw_deg]')
                sys.exit(1)
            yaw = math.radians(float(args[4])) if len(args) > 4 else 0.0
            node.cmd_goto(ns, float(args[2]), float(args[3]), yaw)
        else:
            locs = _load_locations()
            try:
                x, y, yaw = _resolve_location(args[2], locs)
            except KeyError as e:
                print(e)
                sys.exit(1)
            node.cmd_goto(ns, x, y, yaw)

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

    elif cmd == 'mission':
        # mission <ns> <sub> [extra …]
        if len(args) < 3:
            print('Usage: fleet_manager.py mission <ns> patrol|sequence|goto|status|cancel [args…]')
            sys.exit(1)
        node.cmd_mission(args[1], args[2], args[3:])

    elif cmd == 'collision':
        if len(args) < 2:
            print('Usage: fleet_manager.py collision <namespace>')
            sys.exit(1)
        node.cmd_collision(args[1])

    elif cmd == 'tasks':
        if len(args) < 2:
            print('Usage: fleet_manager.py tasks add|status|clear [args…]')
            sys.exit(1)
        node.cmd_tasks(args[1], args[2:])

    elif cmd == 'health':
        node.cmd_health()

    else:
        print(f'Unknown command: {cmd}')
        _usage()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
