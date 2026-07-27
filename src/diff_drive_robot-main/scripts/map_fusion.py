#!/usr/bin/env python3
"""
map_fusion.py — Opt-in auxiliary-scan map fusion for multi-robot SLAM.

Problem: in multi_robot.launch.py, only robot1's lidar feeds slam_toolbox.
Every other robot only runs AMCL against robot1's map, so when the frontier
coordinator sends robot2+ into unexplored territory, their own scans never
update /map — the fleet doesn't map any faster than a single robot would.

This node fuses the *other* robots' scans into the cells slam_toolbox still
marks unknown, publishing the result on a separate topic (/map_fused by
default). It never touches slam_toolbox, AMCL, or /map itself — it only
reads /map and layers extra knowledge on top for consumers (e.g. the
frontier coordinator) that opt into the fused topic.

Parameters
----------
robot_namespaces  Comma-separated auxiliary robot names, e.g. "robot2,robot3"
                  (the SLAM-driving robot, e.g. robot1, should NOT be listed)
base_map_topic    Source OccupancyGrid to fuse into (default /map)
output_topic      Fused OccupancyGrid topic to publish (default /map_fused)
laser_frame_suffix  TF frame suffix for each robot's lidar (default laser_frame)
publish_rate      Hz to publish the fused grid (default 2.0)
max_range         Ignore beams beyond this range, metres (default 10.0)
"""

import math

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_geometry_msgs import do_transform_point

FREE = 0
OCCUPIED = 100
UNKNOWN = -1


class MapFusion(Node):
    def __init__(self):
        super().__init__('map_fusion')

        self.declare_parameter('robot_namespaces', '')
        self.declare_parameter('base_map_topic', '/map')
        self.declare_parameter('output_topic', '/map_fused')
        self.declare_parameter('laser_frame_suffix', 'laser_frame')
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('max_range', 10.0)

        raw = self.get_parameter('robot_namespaces').value
        self._robots = [ns.strip() for ns in raw.split(',') if ns.strip()]
        self._frame_suffix = self.get_parameter('laser_frame_suffix').value
        self._max_range = self.get_parameter('max_range').value
        base_map_topic = self.get_parameter('base_map_topic').value
        output_topic = self.get_parameter('output_topic').value
        publish_rate = self.get_parameter('publish_rate').value

        if not self._robots:
            self.get_logger().warn(
                'map_fusion started with no robot_namespaces — nothing to fuse.')

        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self)

        self._base_map: OccupancyGrid | None = None
        self._base_grid: np.ndarray | None = None
        # Sparse overlay of cells learned from auxiliary robots' scans, keyed
        # by WORLD coordinates (cell-center x, y) rather than grid indices.
        # slam_toolbox resizes/re-origins its grid as it discovers new area,
        # so raw (row, col) indices from an older grid would silently point
        # at the wrong cells once the grid changes shape. Re-deriving row/col
        # from world coords against the *current* grid on every use keeps
        # each entry pinned to the right physical location regardless of
        # how the underlying grid has grown. Value is FREE or OCCUPIED; only
        # ever applied where the current base map is still UNKNOWN, so
        # slam_toolbox's own classification always wins once it catches up.
        self._overlay: dict[tuple[float, float], int] = {}

        self.create_subscription(OccupancyGrid, base_map_topic, self._map_cb, 1)
        for ns in self._robots:
            self.create_subscription(
                LaserScan, f'/{ns}/scan',
                lambda msg, ns=ns: self._scan_cb(ns, msg), 5)

        self._pub = self.create_publisher(OccupancyGrid, output_topic, 1)
        self.create_timer(1.0 / publish_rate, self._publish_fused)

        self.get_logger().info(
            f'map_fusion fusing {self._robots} scans into {base_map_topic} '
            f'-> {output_topic}')

    # ------------------------------------------------------------------
    def _map_cb(self, msg: OccupancyGrid):
        self._base_map = msg
        self._base_grid = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))
        # Drop overlay entries the base map has since resolved itself, or
        # that have fallen outside the (possibly resized/re-origined) grid —
        # keeps the dict from growing unbounded and guarantees SLAM wins.
        if self._overlay:
            info = msg.info
            h, w = self._base_grid.shape
            kept = {}
            for (wx, wy), v in self._overlay.items():
                r, c = self._world_to_cell(wx, wy, info)
                if 0 <= r < h and 0 <= c < w and self._base_grid[r, c] == UNKNOWN:
                    kept[(wx, wy)] = v
            self._overlay = kept

    # ------------------------------------------------------------------
    @staticmethod
    def _world_to_cell(wx, wy, info):
        return (math.floor((wy - info.origin.position.y) / info.resolution),
                math.floor((wx - info.origin.position.x) / info.resolution))

    @staticmethod
    def _cell_center(r, c, info):
        return (info.origin.position.x + (c + 0.5) * info.resolution,
                info.origin.position.y + (r + 0.5) * info.resolution)

    # ------------------------------------------------------------------
    def _scan_cb(self, ns: str, scan: LaserScan):
        if self._base_map is None:
            return

        frame = f'{ns}/{self._frame_suffix}'
        try:
            tf = self._tf.lookup_transform(
                self._base_map.header.frame_id, frame, rclpy.time.Time())
        except Exception:
            return

        info = self._base_map.info
        h, w = self._base_grid.shape

        # Robot/sensor origin in map frame (ray start for every beam).
        origin_pt = PointStamped()
        origin_pt.header.frame_id = frame
        origin_pt.point.x = origin_pt.point.y = origin_pt.point.z = 0.0
        rx, ry = self._transform_xy(origin_pt, tf)
        if rx is None:
            return
        r0, c0 = self._world_to_cell(rx, ry, info)

        angle = scan.angle_min
        for rng in scan.ranges:
            beam_angle = angle
            angle += scan.angle_increment
            if not math.isfinite(rng) or rng < scan.range_min:
                continue
            # A reading at/beyond range_max means "no return" (free out to
            # that distance), not a detected obstacle.
            is_hit = rng < scan.range_max
            clipped = min(rng, self._max_range)

            pt = PointStamped()
            pt.header.frame_id = frame
            pt.point.x = clipped * math.cos(beam_angle)
            pt.point.y = clipped * math.sin(beam_angle)
            pt.point.z = 0.0
            wx, wy = self._transform_xy(pt, tf)
            if wx is None:
                continue
            r1, c1 = self._world_to_cell(wx, wy, info)

            for r, c in _bresenham(r0, c0, r1, c1):
                if not (0 <= r < h and 0 <= c < w):
                    break
                if self._base_grid[r, c] != UNKNOWN:
                    continue
                key = self._cell_center(r, c, info)
                if (r, c) == (r1, c1) and is_hit and rng <= self._max_range:
                    self._overlay[key] = OCCUPIED
                else:
                    # Ray passed cleanly through this cell. Even if an earlier
                    # scan marked it OCCUPIED, an unobstructed ray crossing it
                    # now is fresh evidence the obstacle is gone (e.g. another
                    # robot or a person that has since moved on) — let it
                    # clear back to FREE instead of sticking forever, since
                    # nothing else in this overlay ever un-sets OCCUPIED.
                    self._overlay[key] = FREE

    # ------------------------------------------------------------------
    def _transform_xy(self, point: PointStamped, tf):
        try:
            out = do_transform_point(point, tf)
            return out.point.x, out.point.y
        except Exception:
            return None, None

    # ------------------------------------------------------------------
    def _publish_fused(self):
        if self._base_map is None:
            return
        grid = self._base_grid.copy()
        h, w = grid.shape
        info = self._base_map.info
        for (wx, wy), v in self._overlay.items():
            r, c = self._world_to_cell(wx, wy, info)
            if 0 <= r < h and 0 <= c < w and grid[r, c] == UNKNOWN:
                grid[r, c] = v

        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._base_map.header.frame_id
        out.info = self._base_map.info
        out.data = grid.flatten().astype(np.int8).tolist()
        self._pub.publish(out)


# ──────────────────────────────────────────────────────────────────────────────
def _bresenham(r0, c0, r1, c1):
    """Grid cells from (r0,c0) to (r1,c1) inclusive, Bresenham's line algorithm."""
    cells = []
    dr, dc = abs(r1 - r0), abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    err = dr - dc
    r, c = r0, c0
    while True:
        cells.append((r, c))
        if r == r1 and c == c1:
            break
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc
    return cells


def main(args=None):
    rclpy.init(args=args)
    node = MapFusion()
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
