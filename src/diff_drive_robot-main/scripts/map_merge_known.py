#!/usr/bin/env python3
"""
map_merge_known.py — Robust known-pose multi-robot OccupancyGrid merge.

Each robot runs its own slam_toolbox instance and publishes /<ns>/map in its
local map frame (<ns>/map). This node stitches those grids into one global
/map using known initial poses (spawn offsets), the same pattern as
m-explore's map_merge with known_init_poses:=true.

Robustness choices
------------------
* Align in world coordinates, not grid indices — survives slam_toolbox
  resizing / re-originating each robot's local grid independently.
* Expand the output bounding box to the union of all transformed maps.
* Require matching resolution (warn + skip mismatched sources).
* Conflict policy: OCCUPIED > FREE > UNKNOWN (navigation-safe).
* Never mutates per-robot SLAM topics; only publishes the merged result.

Parameters
----------
robot_namespaces  Comma-separated names, e.g. "robot1,robot2"
init_poses_json   JSON list matching robots: [{"x":0,"y":0,"yaw":0}, ...]
                  Poses are robot local-map origin in the global map frame.
robot_map_topic   Per-robot map topic suffix (default "map" → /robotN/map)
output_topic      Merged OccupancyGrid (default /map)
world_frame       frame_id of the merged map (default map)
publish_rate      Hz (default 2.0)
"""

from __future__ import annotations

import json
import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

FREE = 0
OCCUPIED = 100
UNKNOWN = -1

_MAP_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class MapMergeKnown(Node):
    def __init__(self):
        super().__init__('map_merge_known')

        self.declare_parameter('robot_namespaces', 'robot1,robot2')
        self.declare_parameter('init_poses_json', '[]')
        self.declare_parameter('robot_map_topic', 'map')
        self.declare_parameter('output_topic', '/map')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('boundary_margin_cells', 10)

        raw = self.get_parameter('robot_namespaces').value
        self._robots = [ns.strip() for ns in raw.split(',') if ns.strip()]
        poses = json.loads(self.get_parameter('init_poses_json').value or '[]')
        if len(poses) != len(self._robots):
            raise ValueError(
                f'init_poses_json length {len(poses)} != robots {len(self._robots)}')

        self._poses = []
        for p in poses:
            self._poses.append({
                'x': float(p.get('x', 0.0)),
                'y': float(p.get('y', 0.0)),
                'yaw': float(p.get('yaw', 0.0)),
            })

        map_suffix = self.get_parameter('robot_map_topic').value.strip().strip('/')
        output_topic = self.get_parameter('output_topic').value
        self._world_frame = self.get_parameter('world_frame').value
        publish_rate = float(self.get_parameter('publish_rate').value)
        self._margin = max(0, int(self.get_parameter('boundary_margin_cells').value))

        self._maps: dict[str, OccupancyGrid] = {}
        self._res: float | None = None
        self._warned_res: set[str] = set()

        for ns in self._robots:
            topic = f'/{ns}/{map_suffix}'
            self.create_subscription(
                OccupancyGrid, topic,
                lambda msg, n=ns: self._map_cb(n, msg),
                _MAP_QOS)

        self._pub = self.create_publisher(OccupancyGrid, output_topic, _MAP_QOS)
        self.create_timer(1.0 / max(publish_rate, 0.1), self._publish)

        self.get_logger().info(
            f'map_merge_known: {self._robots} → {output_topic} '
            f'(frame={self._world_frame}, known poses)')

    def _map_cb(self, ns: str, msg: OccupancyGrid):
        res = float(msg.info.resolution)
        if res <= 0.0:
            return
        if self._res is None:
            self._res = res
        elif abs(res - self._res) > 1e-6:
            if ns not in self._warned_res:
                self.get_logger().warn(
                    f'[{ns}] resolution {res} != merge resolution {self._res}; skipping')
                self._warned_res.add(ns)
            return
        self._maps[ns] = msg

    def _publish(self):
        if len(self._maps) < 1 or self._res is None:
            return

        res = self._res
        # Collect occupied/free samples in global coordinates.
        # Key by quantized cell center so growing source grids stay aligned.
        cells: dict[tuple[int, int], int] = {}

        for ns, pose in zip(self._robots, self._poses):
            msg = self._maps.get(ns)
            if msg is None:
                continue
            self._stamp_grid(msg, pose, res, cells)

        if not cells:
            return

        xs = [c for c, _ in cells]
        ys = [r for _, r in cells]
        # Pad with a margin of UNKNOWN cells: this grid's bounding box is
        # recomputed from scratch every publish, so it can grow between
        # ticks. Without slack, a goal picked from a slightly newer snapshot
        # than the one a costmap's static_layer has buffered can land just
        # past that layer's (still shrunk) edge -> worldToMap failures.
        min_c, max_c = min(xs) - self._margin, max(xs) + self._margin
        min_r, max_r = min(ys) - self._margin, max(ys) + self._margin
        width = max_c - min_c + 1
        height = max_r - min_r + 1
        if width <= 0 or height <= 0 or width * height > 80_000_000:
            self.get_logger().error(
                f'Refused absurd merged size {width}x{height}')
            return

        grid = np.full((height, width), UNKNOWN, dtype=np.int8)
        for (c, r), val in cells.items():
            grid[r - min_r, c - min_c] = val

        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._world_frame
        out.info.resolution = res
        out.info.width = width
        out.info.height = height
        # Cell (min_c, min_r) is the lower-left cell; its corner is the origin.
        out.info.origin.position.x = min_c * res
        out.info.origin.position.y = min_r * res
        out.info.origin.position.z = 0.0
        out.info.origin.orientation.w = 1.0
        out.data = grid.flatten().tolist()
        self._pub.publish(out)

    def _stamp_grid(self, msg: OccupancyGrid, pose: dict, res: float,
                    cells: dict[tuple[int, int], int]):
        """Project a local OccupancyGrid into global quantized cells."""
        data = np.asarray(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        # Local map origin yaw is usually 0; compose with known init yaw.
        yaw = pose['yaw']
        c, s = math.cos(yaw), math.sin(yaw)
        tx, ty = pose['x'], pose['y']

        rows, cols = np.nonzero(data != UNKNOWN)
        if rows.size == 0:
            return

        # Vectorized local centers → global, then quantize.
        lx = ox + (cols.astype(np.float64) + 0.5) * res
        ly = oy + (rows.astype(np.float64) + 0.5) * res
        gx = tx + c * lx - s * ly
        gy = ty + s * lx + c * ly
        gcs = np.floor(gx / res).astype(np.int64)
        grs = np.floor(gy / res).astype(np.int64)
        vals = data[rows, cols]

        for gc, gr, val in zip(gcs.tolist(), grs.tolist(), vals.tolist()):
            prev = cells.get((gc, gr), UNKNOWN)
            cells[(gc, gr)] = _merge_cell(prev, int(val))


def _merge_cell(a: int, b: int) -> int:
    """Navigation-safe merge: occupied wins, then free, then unknown."""
    if a == OCCUPIED or b == OCCUPIED:
        return OCCUPIED
    if a == FREE or b == FREE:
        return FREE
    return UNKNOWN


def main(args=None):
    rclpy.init(args=args)
    node = MapMergeKnown()
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
