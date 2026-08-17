#!/usr/bin/env python3
"""
map_merge_known.py — Robust known-pose multi-robot OccupancyGrid merge.

Each robot runs its own slam_toolbox instance and publishes /<ns>/map in its
local map frame (<ns>/map). This node stitches those grids into one global
/map using known initial poses (spawn offsets) rather than scan matching.

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
                  Used until pose_topic (if set) delivers a live correction.
pose_topic        Optional std_msgs/String topic of JSON poses (same shape as
                   init_poses_json), published by collab_loop_closure.py.
                   Once a message arrives, it replaces init_poses_json for
                   that robot — corrects for independent-SLAM drift instead
                   of trusting the static spawn offset for the whole run.
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
from std_msgs.msg import String

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
        self.declare_parameter('pose_topic', '')
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

        self._pose_topic = self.get_parameter('pose_topic').value.strip()
        self._live_poses = False
        if self._pose_topic:
            self.create_subscription(
                String, self._pose_topic, self._pose_cb, 10)

        self._pub = self.create_publisher(OccupancyGrid, output_topic, _MAP_QOS)
        self.create_timer(1.0 / max(publish_rate, 0.1), self._publish)

        pose_note = f', pose corrections from {self._pose_topic}' if self._pose_topic else ', static poses'
        self.get_logger().info(
            f'map_merge_known: {self._robots} → {output_topic} '
            f'(frame={self._world_frame}{pose_note})')

    def _pose_cb(self, msg: String):
        try:
            poses = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warn(f'Malformed pose correction on {self._pose_topic!r}: {msg.data!r}')
            return
        if len(poses) != len(self._robots):
            return
        for i, p in enumerate(poses):
            self._poses[i] = {
                'x': float(p.get('x', 0.0)),
                'y': float(p.get('y', 0.0)),
                'yaw': float(p.get('yaw', 0.0)),
            }
        if not self._live_poses:
            self._live_poses = True
            self.get_logger().info(f'Switched to live pose corrections from {self._pose_topic}')

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
        # Quantized global cell coords in world coordinates, split by value
        # so the whole merge is a couple of vectorized numpy ops instead of
        # a per-cell Python dict rebuilt from scratch every tick — that loop
        # became the dominant cost as grids grow (hospital/warehouse worlds).
        # slam_toolbox only ever emits -1/0/100 (never partial probabilities),
        # so filtering to exactly FREE/OCCUPIED loses nothing.
        free_gc, free_gr, occ_gc, occ_gr = [], [], [], []
        for ns, pose in zip(self._robots, self._poses):
            msg = self._maps.get(ns)
            if msg is None:
                continue
            fgc, fgr, ogc, ogr = self._project_grid(msg, pose, res)
            if fgc.size:
                free_gc.append(fgc)
                free_gr.append(fgr)
            if ogc.size:
                occ_gc.append(ogc)
                occ_gr.append(ogr)

        if not free_gc and not occ_gc:
            return

        all_gc = np.concatenate(free_gc + occ_gc)
        all_gr = np.concatenate(free_gr + occ_gr)

        # Pad with a margin of UNKNOWN cells: this grid's bounding box is
        # recomputed from scratch every publish, so it can grow between
        # ticks. Without slack, a goal picked from a slightly newer snapshot
        # than the one a costmap's static_layer has buffered can land just
        # past that layer's (still shrunk) edge -> worldToMap failures.
        min_c, max_c = int(all_gc.min()) - self._margin, int(all_gc.max()) + self._margin
        min_r, max_r = int(all_gr.min()) - self._margin, int(all_gr.max()) + self._margin
        width = max_c - min_c + 1
        height = max_r - min_r + 1
        if width <= 0 or height <= 0 or width * height > 80_000_000:
            self.get_logger().error(
                f'Refused absurd merged size {width}x{height}')
            return

        grid = np.full((height, width), UNKNOWN, dtype=np.int8)
        # Free first, occupied second: occupied unconditionally wins wherever
        # the two disagree on the same cell, matching the OCCUPIED > FREE >
        # UNKNOWN merge policy.
        if free_gc:
            grid[np.concatenate(free_gr) - min_r, np.concatenate(free_gc) - min_c] = FREE
        if occ_gc:
            grid[np.concatenate(occ_gr) - min_r, np.concatenate(occ_gc) - min_c] = OCCUPIED

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

    def _project_grid(self, msg: OccupancyGrid, pose: dict, res: float):
        """Project a local OccupancyGrid's FREE/OCCUPIED cells into global
        quantized coords. Returns (free_gc, free_gr, occ_gc, occ_gr)."""
        data = np.asarray(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        # Local map origin yaw is usually 0; compose with known init yaw.
        yaw = pose['yaw']
        c, s = math.cos(yaw), math.sin(yaw)
        tx, ty = pose['x'], pose['y']

        def to_global(rows, cols):
            lx = ox + (cols.astype(np.float64) + 0.5) * res
            ly = oy + (rows.astype(np.float64) + 0.5) * res
            gx = tx + c * lx - s * ly
            gy = ty + s * lx + c * ly
            return np.floor(gx / res).astype(np.int64), np.floor(gy / res).astype(np.int64)

        free_rows, free_cols = np.nonzero(data == FREE)
        occ_rows, occ_cols = np.nonzero(data == OCCUPIED)
        free_gc, free_gr = to_global(free_rows, free_cols)
        occ_gc, occ_gr = to_global(occ_rows, occ_cols)
        return free_gc, free_gr, occ_gc, occ_gr


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
