#!/usr/bin/env python3
# frontier_explorer.py — fixed: reads real robot pose from TF
"""
Frontier-based autonomous exploration.

All tuning values are ROS 2 parameters — override at launch:
  ros2 run diff_drive_robot frontier_explorer.py --ros-args \
      -p min_frontier_size:=15 -p revisit_radius:=1.0

Run alongside slam.launch.py (mapping mode):
  ros2 launch diff_drive_robot slam.launch.py
  ros2 run diff_drive_robot frontier_explorer.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import tf2_ros

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

import numpy as np
import math
from collections import deque


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('revisit_radius',    0.3)
        self.declare_parameter('poll_period',       1.5)
        self.declare_parameter('map_topic',         '/map')
        self.declare_parameter('action_name',       'navigate_to_pose')
        self.declare_parameter('goal_frame',        'map')
        self.declare_parameter('base_frame',        'base_link')
        self.declare_parameter('min_goal_distance', 0.35)

        self._min_size      = self.get_parameter('min_frontier_size').value
        self._revisit_r     = self.get_parameter('revisit_radius').value
        self._goal_frame    = self.get_parameter('goal_frame').value
        self._base_frame    = self.get_parameter('base_frame').value
        self._min_goal_dist = self.get_parameter('min_goal_distance').value
        map_topic           = self.get_parameter('map_topic').value
        action_name         = self.get_parameter('action_name').value
        poll_period         = self.get_parameter('poll_period').value

        # ------------------------------------------------------------------
        # TF buffer for robot pose lookup
        # ------------------------------------------------------------------
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self._map: OccupancyGrid | None = None
        self._navigating = False
        self._visited: list[tuple[float, float]] = []

        # ------------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------------
        self._nav_client = ActionClient(self, NavigateToPose, action_name)
        self._map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self._map_callback, 1)

        self.get_logger().info('Waiting for Nav2 action server...')
        self._nav_client.wait_for_server()
        self.get_logger().info('Ready. Waiting for map...')

        self.create_timer(poll_period, self._explore)

    # ------------------------------------------------------------------
    # Map callback
    # ------------------------------------------------------------------
    def _map_callback(self, msg: OccupancyGrid):
        self._map = msg

    # ------------------------------------------------------------------
    # Main exploration loop
    # ------------------------------------------------------------------
    def _explore(self):
        if self._map is None or self._navigating:
            return

        frontiers = self._find_frontiers()
        if not frontiers:
            self.get_logger().info('No frontiers — exploration complete.')
            return

        goal = self._best_frontier(frontiers)
        if goal is None:
            self.get_logger().info('All frontiers already visited.')
            return

        self.get_logger().info(
            f'Navigating to frontier ({goal[0]:.2f}, {goal[1]:.2f})')
        self._visited.append(goal)
        self._send_goal(*goal)

    # ------------------------------------------------------------------
    # Frontier detection
    # ------------------------------------------------------------------
    def _find_frontiers(self) -> list[tuple[float, float]]:
        msg = self._map
        width, height = msg.info.width, msg.info.height
        res = msg.info.resolution
        ox  = msg.info.origin.position.x
        oy  = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        free_mask = data == 0
        unknown_mask = data == -1
        # A frontier cell is free and directly adjacent to unknown space.
        unknown_adjacent = np.zeros_like(unknown_mask, dtype=bool)
        unknown_adjacent[:-1, :] |= unknown_mask[1:, :]
        unknown_adjacent[1:, :]  |= unknown_mask[:-1, :]
        unknown_adjacent[:, :-1] |= unknown_mask[:, 1:]
        unknown_adjacent[:, 1:]  |= unknown_mask[:, :-1]
        frontier_mask = free_mask & unknown_adjacent

        if not frontier_mask.any():
            return []

        centroids = []
        visited = np.zeros_like(frontier_mask, dtype=bool)
        frontier_cells = np.argwhere(frontier_mask)
        for sy, sx in frontier_cells:
            if visited[sy, sx]:
                continue

            queue = deque([(int(sy), int(sx))])
            visited[sy, sx] = True
            cluster = []

            while queue:
                y, x = queue.popleft()
                cluster.append((y, x))
                for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                    if ny < 0 or ny >= height or nx < 0 or nx >= width:
                        continue
                    if visited[ny, nx] or not frontier_mask[ny, nx]:
                        continue
                    visited[ny, nx] = True
                    queue.append((ny, nx))

            if len(cluster) < self._min_size:
                continue

            cells = np.array(cluster, dtype=np.float32)
            cy, cx = cells.mean(axis=0)
            centroids.append((ox + (cx + 0.5) * res, oy + (cy + 0.5) * res))
        return centroids

    # ------------------------------------------------------------------
    # Pick nearest unvisited frontier
    # ------------------------------------------------------------------
    def _best_frontier(self, frontiers):
        pos = self._robot_position()
        if pos is None:
            return None
        rx, ry = pos
        best, best_dist = None, float('inf')
        for fx, fy in frontiers:
            if self._already_visited(fx, fy):
                continue
            d = math.hypot(fx - rx, fy - ry)
            if d < self._min_goal_dist:
                continue
            if d < best_dist:
                best_dist, best = d, (fx, fy)
        return best

    def _already_visited(self, fx, fy):
        return any(
            math.hypot(fx - vx, fy - vy) < self._revisit_r
            for vx, vy in self._visited)

    def _robot_position(self):
        """Return (x, y) of base_link in the map frame via TF lookup."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._goal_frame, self._base_frame, rclpy.time.Time())
            return (
                tf.transform.translation.x,
                tf.transform.translation.y,
            )
        except Exception:
            # TF not yet available.
            self.get_logger().debug(
                f'Waiting for TF {self._goal_frame} -> {self._base_frame}')
            return None

    # ------------------------------------------------------------------
    # Nav2 goal
    # ------------------------------------------------------------------
    def _send_goal(self, x: float, y: float):
        self._navigating = True
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self._goal_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal rejected. Trying next frontier.')
            self._navigating = False
            return
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Frontier reached. Searching for next...')
        else:
            self.get_logger().warn(f'Navigation failed (status={status}).')
        self._navigating = False


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
