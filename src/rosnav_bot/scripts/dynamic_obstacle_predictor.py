#!/usr/bin/env python3
"""
dynamic_obstacle_predictor.py — live, velocity-projected KeepoutFilter mask
for moving obstacles, closing the gap documented in concepts.md §26:
obstacle_tracker.py already Kalman-tracks moving obstacles but never fed that
into Nav2 — avoidance was reactive-only (LaserScan -> obstacle_layer, after
the fact). This subscribes to obstacle_tracker.py's /obstacle_tracker/state
track feed and republishes a small nav_msgs/OccupancyGrid mask marking each
track's current footprint *and* a short velocity-projected "shadow" ahead of
it, so the costmap already carries cost where the obstacle is about to be —
same idea as "Dynamic Path Planning of a mobile robot adopting a costmap
layer approach in ROS2" (IEEE, 9921458), scoped down to reuse this repo's
existing costmap-filter-mask wiring (see camera_terrain_speed_mask.py, same
publish-mask-directly pattern) instead of a new C++ costmap_2d plugin.

Wire into Nav2 via config/dynamic_obstacle_filter.yaml (KeepoutFilter,
type: 0) + a costmap_filter_info_server + lifecycle_manager, same group
shape as the gs_keepout_mask wiring in slam_nav.launch.py.

No new Nav2 plugin, no new C++ build target — this node is the mask
*publisher*, same trick camera_terrain_speed_mask.py already uses.
"""
import json
import math

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                        ReliabilityPolicy)
from std_msgs.msg import String

_MASK_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class DynamicObstaclePredictor(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_predictor')

        self.declare_parameter('robot_ns', '')
        self.declare_parameter('mask_topic', '/dynobs/keepout_mask')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('resolution', 0.10)
        self.declare_parameter('obstacle_radius', 0.35)
        self.declare_parameter('predict_horizon_s', 1.5)
        self.declare_parameter('predict_steps', 3)
        self.declare_parameter('padding', 0.5)
        self.declare_parameter('publish_period', 0.3)
        self.declare_parameter('min_speed_to_predict', 0.05)

        ns = self.get_parameter('robot_ns').value
        pre = f'/{ns}' if ns else ''
        state_topic = f'{pre}/obstacle_tracker/state'

        self._mask_topic = self.get_parameter('mask_topic').value
        self._map_frame = self.get_parameter('map_frame').value
        self._resolution = float(self.get_parameter('resolution').value)
        self._obstacle_radius = float(self.get_parameter('obstacle_radius').value)
        self._horizon = float(self.get_parameter('predict_horizon_s').value)
        self._steps = max(1, int(self.get_parameter('predict_steps').value))
        self._padding = float(self.get_parameter('padding').value)
        self._min_speed = float(self.get_parameter('min_speed_to_predict').value)

        self._tracks: list[dict] = []

        self.create_subscription(String, state_topic, self._on_state, 10)
        self._pub = self.create_publisher(OccupancyGrid, self._mask_topic, _MASK_QOS)

        period = max(0.05, float(self.get_parameter('publish_period').value))
        self.create_timer(period, self._publish_mask)

        self.get_logger().info(
            f'dynamic_obstacle_predictor  state_topic={state_topic}  '
            f'mask_topic={self._mask_topic}  horizon={self._horizon}s  '
            f'steps={self._steps}  obstacle_radius={self._obstacle_radius}m')

    def _on_state(self, msg: String):
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        self._tracks = data.get('moving_obstacles', [])

    def _footprint_centers(self) -> list[tuple[float, float]]:
        """Current position + velocity-projected shadow points, per track."""
        centers: list[tuple[float, float]] = []
        for t in self._tracks:
            x, y = t['x'], t['y']
            vx, vy = t['vx'], t['vy']
            speed = math.hypot(vx, vy)
            centers.append((x, y))
            if speed < self._min_speed:
                continue
            for i in range(1, self._steps + 1):
                dt = self._horizon * i / self._steps
                centers.append((x + vx * dt, y + vy * dt))
        return centers

    def _publish_mask(self):
        centers = self._footprint_centers()

        if not centers:
            # Nothing tracked right now — publish a degenerate all-free
            # 1x1 grid so a stale keepout mark from a previous cycle never
            # lingers (TRANSIENT_LOCAL would otherwise keep serving the
            # last real mask forever to late subscribers).
            grid = OccupancyGrid()
            grid.header.stamp = self.get_clock().now().to_msg()
            grid.header.frame_id = self._map_frame
            grid.info.resolution = self._resolution
            grid.info.width = 1
            grid.info.height = 1
            grid.data = [0]
            self._pub.publish(grid)
            return

        pad = self._padding + self._obstacle_radius
        min_x = min(c[0] for c in centers) - pad
        max_x = max(c[0] for c in centers) + pad
        min_y = min(c[1] for c in centers) - pad
        max_y = max(c[1] for c in centers) + pad

        res = self._resolution
        width = max(1, int(math.ceil((max_x - min_x) / res)))
        height = max(1, int(math.ceil((max_y - min_y) / res)))

        value = [0] * (width * height)
        r_cells = max(1, int(math.ceil(self._obstacle_radius / res)))
        for cx, cy in centers:
            ccol = int((cx - min_x) / res)
            crow = int((cy - min_y) / res)
            for drow in range(-r_cells, r_cells + 1):
                for dcol in range(-r_cells, r_cells + 1):
                    if drow * drow + dcol * dcol > r_cells * r_cells:
                        continue
                    row, col = crow + drow, ccol + dcol
                    if 0 <= row < height and 0 <= col < width:
                        value[row * width + col] = 100

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self._map_frame
        grid.info.resolution = res
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = min_x
        grid.info.origin.position.y = min_y
        grid.info.origin.orientation.w = 1.0
        grid.data = value
        self._pub.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstaclePredictor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
