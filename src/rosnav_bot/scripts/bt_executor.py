#!/usr/bin/env python3
"""
bt_executor.py — CLI front-end for Nav2 BT-driven navigation.

Sub-commands
------------
  patrol <robot_ns> <x1,y1> <x2,y2> ... [--loops N]
      Navigate through waypoints in a loop.  N=0 means infinite.
      Uses patrol_loop BT for each individual goal.

  goto <robot_ns> <x> <y> [yaw] [--bt BT_NAME]
      Send a single NavigateToPose goal, optionally using a named BT XML.
      BT_NAME is the stem of a file in config/bt/ (e.g. navigate_w_replanning).

  no_go [--map_res 0.05]
      Read config/no_go_zones.yaml, publish an OccupancyGrid to /no_go_mask
      with occupied cells inside every defined rectangle.
      Publishes continuously (latched QoS) until killed.

  list
      Print BT XML files found in config/bt/.

Examples
--------
  ros2 run rosnav_bot bt_executor.py patrol robot1 0,0 2,0 2,2 0,2 --loops 3
  ros2 run rosnav_bot bt_executor.py goto   robot1 3.0 1.5 90 --bt navigate_w_replanning
  ros2 run rosnav_bot bt_executor.py no_go  --map_res 0.05
  ros2 run rosnav_bot bt_executor.py list
"""

import argparse
import math
import os
import sys
import time

# ---------------------------------------------------------------------------
# Package-share resolver (mirrors waypoint_nav.py pattern)
# ---------------------------------------------------------------------------

def _pkg_share() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('rosnav_bot')
    except Exception:
        return os.path.join(
            os.path.expanduser('~'), 'rosnav', 'src', 'rosnav_bot')


def _bt_path(name: str) -> str:
    """Return absolute path for a BT XML stem, e.g. 'navigate_w_replanning'."""
    share = _pkg_share()
    # Installed share dir first, then source tree fallback
    candidates = [
        os.path.join(share, 'config', 'bt', f'{name}.xml'),
        os.path.join(
            os.path.expanduser('~'), 'rosnav', 'src',
            'rosnav_bot', 'config', 'bt', f'{name}.xml'),
    ]
    for p in candidates:
        if os.path.isfile(p):
            return p
    # Return the first candidate anyway so the error message is useful
    return candidates[0]


def _no_go_yaml_path() -> str:
    share = _pkg_share()
    candidates = [
        os.path.join(share, 'config', 'no_go_zones.yaml'),
        os.path.join(
            os.path.expanduser('~'), 'rosnav', 'src',
            'rosnav_bot', 'config', 'no_go_zones.yaml'),
    ]
    for p in candidates:
        if os.path.isfile(p):
            return p
    return candidates[0]


def _bt_dir() -> str:
    share = _pkg_share()
    p = os.path.join(share, 'config', 'bt')
    if os.path.isdir(p):
        return p
    fallback = os.path.join(
        os.path.expanduser('~'), 'rosnav', 'src',
        'rosnav_bot', 'config', 'bt')
    return fallback

# ---------------------------------------------------------------------------
# Pose helpers
# ---------------------------------------------------------------------------

def _make_pose_stamped(x: float, y: float, yaw_deg: float, stamp, frame: str = 'map'):
    from geometry_msgs.msg import PoseStamped
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = stamp
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    yaw = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose

# ---------------------------------------------------------------------------
# Sub-command: list
# ---------------------------------------------------------------------------

def cmd_list(_args):
    bt_dir = _bt_dir()
    if not os.path.isdir(bt_dir):
        print(f'[bt_executor] BT directory not found: {bt_dir}')
        return 1
    files = sorted(f for f in os.listdir(bt_dir) if f.endswith('.xml'))
    if not files:
        print(f'[bt_executor] No XML files in {bt_dir}')
        return 0
    print(f'BT XML files in {bt_dir}:')
    for f in files:
        print(f'  {f}')
    return 0

# ---------------------------------------------------------------------------
# Sub-command: goto
# ---------------------------------------------------------------------------

def cmd_goto(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
    from action_msgs.msg import GoalStatus

    robot_ns = args.robot_ns.lstrip('/')
    x        = float(args.x)
    y        = float(args.y)
    yaw_deg  = float(args.yaw) if args.yaw is not None else 0.0
    bt_name  = args.bt

    bt_file = ''
    if bt_name:
        bt_file = _bt_path(bt_name)
        if not os.path.isfile(bt_file):
            print(f'[bt_executor] WARNING: BT file not found: {bt_file}')

    rclpy.init()
    node = Node('bt_executor_goto')

    action_name = f'/{robot_ns}/navigate_to_pose'
    client = ActionClient(node, NavigateToPose, action_name)

    node.get_logger().info(f'Waiting for {action_name} ...')
    if not client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error('Action server not available after 10 s')
        rclpy.shutdown()
        return 1

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose = _make_pose_stamped(x, y, yaw_deg, node.get_clock().now().to_msg())
    if bt_file:
        goal_msg.behavior_tree = bt_file

    node.get_logger().info(
        f'Sending goal → ({x}, {y}, {yaw_deg}°)'
        + (f'  bt={bt_file}' if bt_file else ''))

    result_holder = {}

    def _result_cb(future):
        result_holder['result'] = future.result()

    def _goal_cb(future):
        handle = future.result()
        if not handle.accepted:
            node.get_logger().error('Goal rejected.')
            result_holder['result'] = None
            return
        node.get_logger().info('Goal accepted.')
        handle.get_result_async().add_done_callback(_result_cb)

    future = client.send_goal_async(goal_msg)
    future.add_done_callback(_goal_cb)

    while rclpy.ok() and 'result' not in result_holder:
        rclpy.spin_once(node, timeout_sec=0.1)

    res = result_holder.get('result')
    if res is None:
        node.get_logger().error('Goal was rejected or timed out.')
        rclpy.shutdown()
        return 1

    if res.status == GoalStatus.STATUS_SUCCEEDED:
        node.get_logger().info('Goal reached successfully.')
        rclpy.shutdown()
        return 0
    else:
        node.get_logger().error(f'Navigation failed. Status: {res.status}')
        rclpy.shutdown()
        return 1

# ---------------------------------------------------------------------------
# Sub-command: patrol
# ---------------------------------------------------------------------------

def cmd_patrol(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
    from action_msgs.msg import GoalStatus

    robot_ns = args.robot_ns.lstrip('/')
    loops    = int(args.loops)  # 0 = infinite

    # Parse waypoints: each is "x,y" or "x,y,yaw"
    waypoints = []
    for wp_str in args.waypoints:
        parts = wp_str.split(',')
        if len(parts) < 2:
            print(f'[bt_executor] Bad waypoint format: {wp_str!r}  expected x,y[,yaw]')
            return 1
        x   = float(parts[0])
        y   = float(parts[1])
        yaw = float(parts[2]) if len(parts) > 2 else 0.0
        waypoints.append((x, y, yaw))

    if not waypoints:
        print('[bt_executor] patrol requires at least one waypoint.')
        return 1

    bt_file = _bt_path('patrol_loop')
    if not os.path.isfile(bt_file):
        print(f'[bt_executor] WARNING: patrol_loop BT not found at {bt_file}')
        bt_file = ''

    rclpy.init()
    node = Node('bt_executor_patrol')

    action_name = f'/{robot_ns}/navigate_to_pose'
    client = ActionClient(node, NavigateToPose, action_name)

    node.get_logger().info(f'Waiting for {action_name} ...')
    if not client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error('Action server not available after 10 s')
        rclpy.shutdown()
        return 1

    total_laps = 0
    wp_idx     = 0

    def _send_next():
        nonlocal wp_idx
        x, y, yaw = waypoints[wp_idx]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = _make_pose_stamped(x, y, yaw, node.get_clock().now().to_msg())
        if bt_file:
            goal_msg.behavior_tree = bt_file
        node.get_logger().info(
            f'[lap {total_laps + 1}, wp {wp_idx + 1}/{len(waypoints)}] '
            f'→ ({x:.2f}, {y:.2f}, {yaw:.1f}°)')
        fut = client.send_goal_async(goal_msg)
        fut.add_done_callback(_goal_cb)

    done_flag = {'done': False, 'rc': 0}

    def _result_cb(future):
        nonlocal total_laps, wp_idx
        res = future.result()
        if res.status != GoalStatus.STATUS_SUCCEEDED:
            node.get_logger().error(
                f'Waypoint failed (status {res.status}). Continuing to next.')

        wp_idx += 1
        if wp_idx >= len(waypoints):
            total_laps += 1
            wp_idx = 0
            node.get_logger().info(f'Lap {total_laps} complete.')
            if loops != 0 and total_laps >= loops:
                node.get_logger().info('Patrol complete — all laps done.')
                done_flag['done'] = True
                return

        if not done_flag['done']:
            _send_next()

    def _goal_cb(future):
        handle = future.result()
        if not handle.accepted:
            node.get_logger().error('Goal rejected — aborting patrol.')
            done_flag['done'] = True
            done_flag['rc']   = 1
            return
        handle.get_result_async().add_done_callback(_result_cb)

    _send_next()

    while rclpy.ok() and not done_flag['done']:
        rclpy.spin_once(node, timeout_sec=0.1)

    rclpy.shutdown()
    return done_flag['rc']

# ---------------------------------------------------------------------------
# Sub-command: no_go
# ---------------------------------------------------------------------------

def cmd_no_go(args):
    import yaml
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import Header

    map_res = float(args.map_res)

    yaml_path = _no_go_yaml_path()
    if not os.path.isfile(yaml_path):
        print(f'[bt_executor] no_go_zones.yaml not found: {yaml_path}')
        return 1

    with open(yaml_path) as f:
        data = yaml.safe_load(f)

    zones = data.get('no_go_zones', [])
    if not zones:
        print('[bt_executor] no_go_zones list is empty — nothing to publish.')
        return 0

    # Determine bounding box across all zones to size the grid
    all_x = [z['x_min'] for z in zones] + [z['x_max'] for z in zones]
    all_y = [z['y_min'] for z in zones] + [z['y_max'] for z in zones]
    # Add 1-cell padding around the full extent
    padding = map_res
    origin_x = min(all_x) - padding
    origin_y = min(all_y) - padding
    max_x    = max(all_x) + padding
    max_y    = max(all_y) + padding

    width  = max(1, int(math.ceil((max_x - origin_x) / map_res)))
    height = max(1, int(math.ceil((max_y - origin_y) / map_res)))

    grid_data = [0] * (width * height)

    for zone in zones:
        name  = zone.get('name', '?')
        xmin  = zone['x_min']
        xmax  = zone['x_max']
        ymin  = zone['y_min']
        ymax  = zone['y_max']
        col0  = max(0, int((xmin - origin_x) / map_res))
        col1  = min(width,  int(math.ceil((xmax - origin_x) / map_res)))
        row0  = max(0, int((ymin - origin_y) / map_res))
        row1  = min(height, int(math.ceil((ymax - origin_y) / map_res)))
        for row in range(row0, row1):
            for col in range(col0, col1):
                grid_data[row * width + col] = 100
        print(f'[bt_executor] zone {name!r}: cells [{col0}:{col1}, {row0}:{row1}]')

    rclpy.init()
    node = Node('bt_executor_no_go')

    latched_qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    pub = node.create_publisher(OccupancyGrid, '/no_go_mask', latched_qos)

    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.resolution = map_res
    msg.info.width      = width
    msg.info.height     = height
    msg.info.origin.position.x = origin_x
    msg.info.origin.position.y = origin_y
    msg.info.origin.orientation.w = 1.0
    msg.data = grid_data

    def _publish():
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)

    # Publish immediately, then on a 1 Hz timer so late subscribers get it
    timer = node.create_timer(1.0, _publish)
    _publish()

    node.get_logger().info(
        f'Publishing /no_go_mask ({width}x{height} @ {map_res} m/cell) — Ctrl-C to stop.')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    timer.cancel()
    rclpy.shutdown()
    return 0

# ---------------------------------------------------------------------------
# Argument parsing + dispatch
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog='bt_executor.py',
        description='Nav2 BT-driven navigation utilities for rosnav_bot.')
    sub = p.add_subparsers(dest='command', metavar='COMMAND')

    # patrol
    sp_patrol = sub.add_parser('patrol', help='Navigate through waypoints in a loop.')
    sp_patrol.add_argument('robot_ns', help='Robot namespace (e.g. robot1)')
    sp_patrol.add_argument(
        'waypoints', nargs='+', metavar='x,y[,yaw]',
        help='One or more waypoints as x,y or x,y,yaw_degrees')
    sp_patrol.add_argument(
        '--loops', default='0', metavar='N',
        help='Number of patrol loops; 0 = infinite (default: 0)')

    # goto
    sp_goto = sub.add_parser('goto', help='Send a single NavigateToPose goal.')
    sp_goto.add_argument('robot_ns', help='Robot namespace (e.g. robot1)')
    sp_goto.add_argument('x',   type=float, help='Goal X (metres, map frame)')
    sp_goto.add_argument('y',   type=float, help='Goal Y (metres, map frame)')
    sp_goto.add_argument('yaw', nargs='?',  help='Goal yaw in degrees (default 0)')
    sp_goto.add_argument(
        '--bt', default=None, metavar='BT_NAME',
        help='BT XML stem from config/bt/ (e.g. navigate_w_replanning)')

    # no_go
    sp_nogo = sub.add_parser('no_go', help='Publish no-go zones as OccupancyGrid.')
    sp_nogo.add_argument(
        '--map_res', default='0.05', metavar='RES',
        help='Grid resolution in metres/cell (default: 0.05)')

    # list
    sub.add_parser('list', help='List available BT XML files.')

    return p


def main():
    parser = build_parser()
    args   = parser.parse_args()

    if args.command == 'list':
        sys.exit(cmd_list(args))
    elif args.command == 'goto':
        sys.exit(cmd_goto(args))
    elif args.command == 'patrol':
        sys.exit(cmd_patrol(args))
    elif args.command == 'no_go':
        sys.exit(cmd_no_go(args))
    else:
        parser.print_help()
        sys.exit(1)


if __name__ == '__main__':
    main()
