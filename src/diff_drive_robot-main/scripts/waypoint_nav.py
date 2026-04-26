#!/usr/bin/env python3
"""
Waypoint follower using Nav2's FollowWaypoints action.

Waypoints are loaded from a YAML file.  The default path is
<package_share>/config/waypoints.yaml (falls back to ~/rosnav/waypoints.yaml).
Override at runtime:
  ros2 run diff_drive_robot waypoint_nav.py --ros-args \\
      -p waypoints_file:=/path/to/waypoints.yaml

Waypoints YAML — two supported formats:

  # Raw coordinates
  waypoints:
    - [x, y, yaw_degrees]
    - [2.0, 0.0, 0.0]

  # Named locations (resolved from locations.yaml)
  waypoints:
    - room_a
    - hallway
    - room_b

  # Mixed
  waypoints:
    - room_a
    - [2.0, 0.0, 0.0]

If the file is not found, a built-in default square route is used.
"""

import math
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False

DEFAULT_WAYPOINTS = [
    (2.0,  0.0,   0.0),
    (2.0,  2.0,  90.0),
    (0.0,  2.0, 180.0),
    (0.0,  0.0, -90.0),
]


def _pkg_share() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('diff_drive_robot')
    except Exception:
        return os.path.join(
            os.path.expanduser('~'), 'rosnav', 'src', 'diff_drive_robot-main')


def _default_waypoints_path() -> str:
    return os.path.join(_pkg_share(), 'config', 'waypoints.yaml')


def _load_locations(share: str) -> dict:
    if not HAS_YAML:
        return {}
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


def load_waypoints(path: str) -> list[tuple]:
    if not HAS_YAML or not os.path.isfile(path):
        return DEFAULT_WAYPOINTS

    with open(path) as f:
        data = yaml.safe_load(f)

    raw = data.get('waypoints', [])
    if not raw:
        return DEFAULT_WAYPOINTS

    share     = _pkg_share()
    locations = _load_locations(share)
    result    = []

    for wp in raw:
        if isinstance(wp, str):
            if wp not in locations:
                raise ValueError(
                    f'Unknown location name {wp!r} in {path}. '
                    f'Known: {list(locations)}')
            coords = locations[wp]
            result.append((float(coords[0]), float(coords[1]),
                           float(coords[2]) if len(coords) > 2 else 0.0))
        else:
            result.append((float(wp[0]), float(wp[1]),
                           float(wp[2]) if len(wp) > 2 else 0.0))

    return result or DEFAULT_WAYPOINTS


def make_pose(x: float, y: float, yaw_deg: float, stamp) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = stamp
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    yaw = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        self.declare_parameter('waypoints_file', _default_waypoints_path())
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('action_name', 'follow_waypoints')

        waypoints_file = self.get_parameter('waypoints_file').value
        frame_id       = self.get_parameter('frame_id').value
        action_name    = self.get_parameter('action_name').value

        self._waypoints = load_waypoints(waypoints_file)
        self._frame_id  = frame_id

        self.get_logger().info(
            f'Loaded {len(self._waypoints)} waypoints from '
            f'{"file" if os.path.isfile(waypoints_file) else "defaults"}')

        self._client = ActionClient(self, FollowWaypoints, action_name)
        self.get_logger().info('Waiting for FollowWaypoints server...')
        self._client.wait_for_server()
        self._send_waypoints()

    def _send_waypoints(self):
        now = self.get_clock().now().to_msg()
        goal = FollowWaypoints.Goal()
        goal.poses = [make_pose(x, y, yaw, now) for x, y, yaw in self._waypoints]

        self.get_logger().info(f'Sending {len(goal.poses)} waypoints...')
        future = self._client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected.')
            return
        self.get_logger().info('Goal accepted.')
        handle.get_result_async().add_done_callback(self._result_cb)

    def _feedback_cb(self, feedback_msg):
        idx = feedback_msg.feedback.current_waypoint
        self.get_logger().info(
            f'Navigating to waypoint {idx + 1}/{len(self._waypoints)}')

    def _result_cb(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            missed = list(result.result.missed_waypoints)
            if missed:
                self.get_logger().warn(f'Done. Missed waypoints: {missed}')
            else:
                self.get_logger().info('All waypoints reached.')
        else:
            self.get_logger().error(f'Failed. Status: {result.status}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
