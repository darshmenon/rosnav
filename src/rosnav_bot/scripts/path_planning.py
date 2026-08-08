#!/usr/bin/env python3
"""Standalone A* global planner.

Subscribes to an OccupancyGrid (`map_topic`, default `/map`) and treats
occupied/unknown cells as obstacles. If no map arrives within
`map_wait_sec`, falls back to a tiny hardcoded demo obstacle set so the
node still publishes a path for offline smoke tests.
"""

from __future__ import annotations

import heapq
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker

_MAP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

# Demo obstacles (world metres) used only when no /map is available.
_FALLBACK_OBSTACLES = [(2.0, 1.0), (4.0, 3.0), (5.0, 1.0)]


class ImprovedAStar(Node):
    def __init__(self):
        super().__init__('improved_astar')
        self.path_pub = self.create_publisher(Path, 'global_plan', 10)
        self.marker_pub = self.create_publisher(Marker, 'obstacle_markers', 10)

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_wait_sec', 5.0)
        self.declare_parameter('grid_size_x', 20)
        self.declare_parameter('grid_size_y', 20)
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('safety_margin', 0.3)
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('treat_unknown_as_obstacle', True)
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('goal_x', 8.0)
        self.declare_parameter('goal_y', 8.0)

        self._resolution = float(self.get_parameter('resolution').value)
        self._grid_w = int(self.get_parameter('grid_size_x').value)
        self._grid_h = int(self.get_parameter('grid_size_y').value)
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._map_ready = False
        self._using_fallback = False
        self._occupancy: np.ndarray | None = None  # True = blocked
        self.obstacles: list[tuple[int, int]] = []

        map_topic = self.get_parameter('map_topic').value
        self.create_subscription(OccupancyGrid, map_topic, self._map_cb, _MAP_QOS)
        self.get_logger().info(f'Waiting for OccupancyGrid on {map_topic} …')

        self.create_timer(1.0, self.plan_path)
        self._fallback_timer = self.create_timer(
            float(self.get_parameter('map_wait_sec').value),
            self._maybe_fallback,
        )

    def _maybe_fallback(self):
        self._fallback_timer.cancel()
        if self._map_ready or self._using_fallback:
            return
        self._using_fallback = True
        self.obstacles = self._world_pts_to_grid(_FALLBACK_OBSTACLES, self._resolution)
        self.get_logger().warn(
            'No /map received — using hardcoded demo obstacles. '
            'Publish an OccupancyGrid to plan on a real map.')
        self.publish_obstacles()

    def _map_cb(self, msg: OccupancyGrid):
        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            return
        res = msg.info.resolution
        data = np.asarray(msg.data, dtype=np.int16).reshape((h, w))
        occ_thresh = int(self.get_parameter('occupied_threshold').value)
        blocked = data >= occ_thresh
        if bool(self.get_parameter('treat_unknown_as_obstacle').value):
            blocked |= data < 0

        self._occupancy = blocked
        self._resolution = float(res)
        self._grid_w = int(w)
        self._grid_h = int(h)
        self._origin_x = float(msg.info.origin.position.x)
        self._origin_y = float(msg.info.origin.position.y)
        # Sparse list for marker viz (sample occupied cells).
        ys, xs = np.where(blocked)
        step = max(1, len(xs) // 2000)  # cap marker points
        self.obstacles = list(zip(xs[::step].tolist(), ys[::step].tolist()))
        self._map_ready = True
        self._using_fallback = False
        self.get_logger().info(
            f'Map received {w}x{h} @ {res:.3f} m — A* using OccupancyGrid obstacles.')
        self.publish_obstacles()

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def plan_path(self):
        if not self._map_ready and not self._using_fallback:
            return
        start = self.world_to_grid(
            float(self.get_parameter('start_x').value),
            float(self.get_parameter('start_y').value))
        goal = self.world_to_grid(
            float(self.get_parameter('goal_x').value),
            float(self.get_parameter('goal_y').value))
        if not self.valid_grid_position(start) or not self.valid_grid_position(goal):
            self.get_logger().warn(
                f'Start {start} or goal {goal} is blocked/out of bounds — skip plan.')
            return
        path = self.astar(start, goal)
        if path:
            self.publish_path(path)

    def astar(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {}
        g_score = {start: 0.0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            current = heapq.heappop(open_set)[1]
            if current == goal:
                return self.reconstruct_path(came_from, current)

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),
                           (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                if not self.valid_grid_position(neighbor):
                    continue
                temp_g = g_score[current] + math.hypot(dx, dy)
                if neighbor not in g_score or temp_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = temp_g
                    f_score[neighbor] = temp_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        return None

    def valid_grid_position(self, pos):
        x, y = pos
        if not (0 <= x < self._grid_w and 0 <= y < self._grid_h):
            return False
        if self._occupancy is not None:
            # Inflate by safety_margin in cells.
            margin = int(self.get_parameter('safety_margin').value / self._resolution)
            y0 = max(0, y - margin)
            y1 = min(self._grid_h, y + margin + 1)
            x0 = max(0, x - margin)
            x1 = min(self._grid_w, x + margin + 1)
            return not bool(self._occupancy[y0:y1, x0:x1].any())
        margin = int(self.get_parameter('safety_margin').value / self._resolution)
        return all(
            abs(x - ox) > margin or abs(y - oy) > margin
            for ox, oy in self.obstacles)

    def publish_path(self, path):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for grid_pos in path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position = self.grid_to_world(*grid_pos)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        return (
            int((x - self._origin_x) / self._resolution),
            int((y - self._origin_y) / self._resolution),
        )

    def grid_to_world(self, gx: int, gy: int) -> Point:
        p = Point()
        p.x = self._origin_x + (gx + 0.5) * self._resolution
        p.y = self._origin_y + (gy + 0.5) * self._resolution
        return p

    def reconstruct_path(self, came_from: dict, current: tuple) -> list:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def publish_obstacles(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'obstacles'
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = self._resolution
        marker.scale.y = self._resolution
        marker.color.r = 1.0
        marker.color.a = 1.0
        for gx, gy in self.obstacles:
            marker.points.append(self.grid_to_world(gx, gy))
        self.marker_pub.publish(marker)


def _world_pts_to_grid(raw_obstacles, resolution):
    return [(int(x / resolution), int(y / resolution)) for x, y in raw_obstacles]


def main(args=None):
    rclpy.init(args=args)
    node = ImprovedAStar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
