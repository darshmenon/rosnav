#!/usr/bin/env python3
"""
coverage_planner.py — Boustrophedon (lawnmower) coverage path planner.

Waits for a /map (OccupancyGrid), computes a full-coverage sweep over the
free space, and sends the resulting waypoint list to Nav2's FollowWaypoints
action server.

Algorithm
─────────
  1. Receive OccupancyGrid (from SLAM Toolbox or map_server).
  2. Identify FREE cells (value 0) with clearance ≥ robot_radius from walls.
  3. Sweep the bounding box of free space with horizontal scan lines spaced
     sweep_spacing metres apart, alternating direction (boustrophedon).
  4. Keep only waypoints that land on free + cleared cells.
  5. Send the full list to FollowWaypoints.

Parameters
──────────
  robot_ns          namespace prefix (default: '')
  sweep_spacing     metres between scan lines  (default: 0.5)
  robot_radius      cell-clearance radius in m (default: 0.25)
  map_topic         (default: /map)
  start_from_robot  if true, start sweep from robot's current position (default: true)
  action_name       (default: follow_waypoints)

Usage
─────
  ros2 run diff_drive_robot coverage_planner.py
  ros2 run diff_drive_robot coverage_planner.py --ros-args \\
      -p sweep_spacing:=0.4 -p robot_ns:=robot1
"""

import math

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import FollowWaypoints

try:
    import tf2_ros
    HAS_TF = True
except ImportError:
    HAS_TF = False


def _make_pose(x: float, y: float, yaw: float, stamp, frame: str) -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = frame
    p.header.stamp    = stamp
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


class CoveragePlanner(Node):
    def __init__(self):
        super().__init__('coverage_planner')

        self.declare_parameter('robot_ns',         '')
        self.declare_parameter('sweep_spacing',    0.5)
        self.declare_parameter('robot_radius',     0.25)
        self.declare_parameter('map_topic',        '/map')
        self.declare_parameter('start_from_robot', True)
        self.declare_parameter('action_name',      'follow_waypoints')

        ns             = self.get_parameter('robot_ns').value
        self._spacing  = self.get_parameter('sweep_spacing').value
        self._radius   = self.get_parameter('robot_radius').value
        map_topic      = self.get_parameter('map_topic').value
        self._from_bot = self.get_parameter('start_from_robot').value
        action_ns      = self.get_parameter('action_name').value

        pre = f'/{ns}' if ns else ''

        self._map: OccupancyGrid | None = None

        self._tf_buf = tf2_ros.Buffer()          if HAS_TF else None
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self) if HAS_TF else None

        self._client = ActionClient(self, FollowWaypoints, f'{pre}/{action_ns}')

        self.create_subscription(OccupancyGrid, map_topic, self._map_cb, 1)

        self.get_logger().info(
            f'CoveragePlanner  ns={ns or "/"}  spacing={self._spacing}m  '
            f'radius={self._radius}m  waiting for map on {map_topic} …')

    # ── Map callback ──────────────────────────────────────────────────────────

    def _map_cb(self, msg: OccupancyGrid):
        if self._map is not None:
            return          # already planning
        self._map = msg
        self.get_logger().info(
            f'Map received: {msg.info.width}×{msg.info.height} '
            f'res={msg.info.resolution:.3f}m')
        self._plan_and_send()

    # ── Planning ──────────────────────────────────────────────────────────────

    def _plan_and_send(self):
        waypoints = self._compute_coverage()
        if not waypoints:
            self.get_logger().error('No reachable coverage waypoints found.')
            return

        self.get_logger().info(f'Coverage plan: {len(waypoints)} waypoints.  '
                               'Waiting for FollowWaypoints server …')
        if not self._client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('FollowWaypoints server not available.')
            return

        now  = self.get_clock().now().to_msg()
        goal = FollowWaypoints.Goal()
        goal.poses = [_make_pose(x, y, yaw, now, 'map') for x, y, yaw in waypoints]

        self.get_logger().info(f'Sending {len(goal.poses)} poses to FollowWaypoints …')
        future = self._client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_resp_cb)

    def _compute_coverage(self) -> list[tuple[float, float, float]]:
        msg   = self._map
        info  = msg.info
        res   = info.resolution
        w, h  = info.width, info.height
        ox    = info.origin.position.x
        oy    = info.origin.position.y

        grid  = np.array(msg.data, dtype=np.int8).reshape((h, w))
        free  = (grid == 0)

        # Erode free space by robot_radius to ensure clearance from walls
        r_cells = max(1, int(math.ceil(self._radius / res)))
        from scipy.ndimage import binary_erosion
        struct  = np.ones((2 * r_cells + 1, 2 * r_cells + 1), dtype=bool)
        padded  = binary_erosion(free, structure=struct)

        # Bounding box of navigable cells
        rows, cols = np.where(padded)
        if len(rows) == 0:
            # Fall back to free (no erosion) if erosion kills all cells
            rows, cols = np.where(free)
        if len(rows) == 0:
            return []

        r_min, r_max = int(rows.min()), int(rows.max())
        c_min, c_max = int(cols.min()), int(cols.max())

        spacing_cells = max(1, int(round(self._spacing / res)))
        waypoints: list[tuple[float, float, float]] = []

        row = r_min
        left_to_right = True
        while row <= r_max:
            # Collect navigable cells on this scan line
            if len(rows) == len(np.where(padded)[0]):   # padded is valid
                line_cols = [c for c in range(c_min, c_max + 1) if padded[row, c]]
            else:
                line_cols = [c for c in range(c_min, c_max + 1) if free[row, c]]

            if not left_to_right:
                line_cols = list(reversed(line_cols))

            if line_cols:
                prev_c = None
                for c in line_cols:
                    # Only keep one waypoint per spacing_cells interval
                    if prev_c is None or abs(c - prev_c) >= spacing_cells:
                        wx = ox + (c + 0.5) * res
                        wy = oy + (row + 0.5) * res
                        # Yaw: face direction of travel along sweep line
                        yaw = 0.0 if left_to_right else math.pi
                        waypoints.append((wx, wy, yaw))
                        prev_c = c

            left_to_right = not left_to_right
            row += spacing_cells

        if not waypoints:
            return []

        # Optionally reorder so first waypoint is nearest robot's current pose
        if self._from_bot and HAS_TF:
            robot_x, robot_y = self._get_robot_pose()
            if robot_x is not None:
                waypoints.sort(key=lambda p: (p[0] - robot_x) ** 2 + (p[1] - robot_y) ** 2)

        return waypoints

    def _get_robot_pose(self):
        try:
            tf = self._tf_buf.lookup_transform(
                'map', 'base_link',
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:
            return None, None

    # ── Action callbacks ──────────────────────────────────────────────────────

    def _feedback_cb(self, fb):
        idx = fb.feedback.current_waypoint
        total = len(self._map.data) if self._map else '?'
        self.get_logger().info(f'Coverage: waypoint {idx + 1} reached.')

    def _goal_resp_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Coverage goal rejected.')
            return
        self.get_logger().info('Coverage goal accepted.')
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            missed = list(result.result.missed_waypoints)
            if missed:
                self.get_logger().warn(f'Coverage done. Missed waypoints: {missed}')
            else:
                self.get_logger().info('Coverage complete — all waypoints reached.')
        else:
            self.get_logger().error(f'Coverage failed. Status: {result.status}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
