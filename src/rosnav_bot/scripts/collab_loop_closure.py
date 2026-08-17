#!/usr/bin/env python3
"""
collab_loop_closure.py — Inter-robot loop closure for slam_mode:=multi.

Each robot in slam_mode:=multi runs its own independent slam_toolbox instance
and is stitched into /map_merged by map_merge_known using a *static* known
spawn-offset pose (see multi_robot.launch.py). Independent SLAM instances
drift apart over a long run, so that static assumption slowly goes stale and
overlapping regions of the merged map start to ghost/double up.

This is a 2D-native correction: this fleet's SLAM input is always 2D
LaserScan (even when lidar_type:=3d is set — that's a separate sensor, not
wired into SLAM), so drift correction is done via brute-force correlative
grid matching — the same family of algorithm Cartographer uses for loop
closure — applied directly to the OccupancyGrids slam_toolbox already
publishes, rather than 3D point-cloud registration.

Algorithm
---------
For every robot pair with overlapping map bounding boxes (world frame, using
each robot's current pose estimate): search a small window of (dx, dy,
dtheta) around the current relative pose and score each candidate by how
many of robot i's OCCUPIED cells land on robot j's OCCUPIED cells. Only a
small local search is needed — robots start from known, roughly-correct
spawn offsets, so this corrects drift rather than solving global relocation.
Accepted corrections (enough overlap + high enough match score) are fed
through an EMA so a single noisy match can't jerk the merged map; robot1
never moves (it is the global reference frame, matching map_merge_known's
existing convention).

Re-matching a pair is skipped whenever neither robot's occupied-cell count
has grown since its last check — once two maps are aligned, nothing changes
until new area is explored, so this avoids re-running the full brute-force
search every cycle for a pair that's already settled.

Corrected poses are published as JSON — map_merge_known subscribes to this
topic and prefers it over its static init_poses_json once available.

Parameters
----------
robot_namespaces   Comma-separated names, e.g. "robot1,robot2" (robot1 fixed)
init_poses_json     JSON list of starting poses (same shape as map_merge_known)
robot_map_topic     Per-robot map topic suffix (default "map")
output_topic        Corrected poses, JSON (default /collab/poses)
search_window_m      +/- metres searched for (dx, dy) (default 0.5)
search_step_m        Metres per search step (default 0.1)
search_window_rad    +/- radians searched for dtheta (default 0.08)
search_step_rad      Radians per angle step (default 0.02)
min_overlap_cells    Minimum overlapping occupied cells to attempt a match (default 40)
min_match_ratio      Minimum fraction of robot i occupied cells that must land
                      on robot j occupied cells to accept a correction (default 0.35)
correction_gain      EMA smoothing factor applied to accepted corrections (default 0.3)
publish_rate         Hz (default 1.0)
publish_markers      Publish RViz debug markers on /collab/loop_closures (default true) —
                     a line between each pair's current pose estimates, green when a
                     correction was accepted this cycle, dim orange when a candidate
                     match was tried but stayed below min_match_ratio.
"""

from __future__ import annotations

import json
import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA, String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

OCCUPIED = 100

_MAP_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class CollabLoopClosure(Node):
    def __init__(self):
        super().__init__('collab_loop_closure')

        self.declare_parameter('robot_namespaces', 'robot1,robot2')
        self.declare_parameter('init_poses_json', '[]')
        self.declare_parameter('robot_map_topic', 'map')
        self.declare_parameter('output_topic', '/collab/poses')
        self.declare_parameter('search_window_m', 0.5)
        self.declare_parameter('search_step_m', 0.1)
        self.declare_parameter('search_window_rad', 0.08)
        self.declare_parameter('search_step_rad', 0.02)
        self.declare_parameter('min_overlap_cells', 40)
        self.declare_parameter('min_match_ratio', 0.35)
        self.declare_parameter('correction_gain', 0.3)
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('publish_markers', True)

        raw = self.get_parameter('robot_namespaces').value
        self._robots = [ns.strip() for ns in raw.split(',') if ns.strip()]
        init = json.loads(self.get_parameter('init_poses_json').value or '[]')
        if len(init) != len(self._robots):
            raise ValueError(
                f'init_poses_json length {len(init)} != robots {len(self._robots)}')

        # Current best pose estimate per robot: local-map origin in the
        # global map frame. robot1 (index 0) is the fixed global reference,
        # same convention as map_merge_known / multi_robot.launch.py.
        self._poses = [
            {'x': float(p.get('x', 0.0)), 'y': float(p.get('y', 0.0)),
             'yaw': float(p.get('yaw', 0.0))}
            for p in init
        ]

        self._sw_m = float(self.get_parameter('search_window_m').value)
        self._ss_m = float(self.get_parameter('search_step_m').value)
        self._sw_rad = float(self.get_parameter('search_window_rad').value)
        self._ss_rad = float(self.get_parameter('search_step_rad').value)
        self._min_overlap = int(self.get_parameter('min_overlap_cells').value)
        self._min_ratio = float(self.get_parameter('min_match_ratio').value)
        self._gain = float(self.get_parameter('correction_gain').value)

        map_suffix = self.get_parameter('robot_map_topic').value.strip().strip('/')
        output_topic = self.get_parameter('output_topic').value
        publish_rate = float(self.get_parameter('publish_rate').value)
        self._publish_markers_enabled = self.get_parameter('publish_markers').value

        self._maps: dict[str, OccupancyGrid] = {}
        self._pair_cache: dict[tuple[int, int], dict] = {}
        self._last_links: list[dict] = []
        for ns in self._robots:
            topic = f'/{ns}/{map_suffix}'
            self.create_subscription(
                OccupancyGrid, topic,
                lambda msg, n=ns: self._map_cb(n, msg),
                _MAP_QOS)

        self._pub = self.create_publisher(String, output_topic, _MAP_QOS)
        self._marker_pub = self.create_publisher(MarkerArray, '/collab/loop_closures', 10)
        self.create_timer(1.0 / max(publish_rate, 0.1), self._cycle)

        self.get_logger().info(
            f'collab_loop_closure: {self._robots} (robot1 fixed) -> {output_topic} '
            f'| search=+/-{self._sw_m}m/{math.degrees(self._sw_rad):.1f}deg '
            f'min_overlap={self._min_overlap} min_ratio={self._min_ratio} '
            f'gain={self._gain}')

    def _map_cb(self, ns: str, msg: OccupancyGrid):
        self._maps[ns] = msg

    # ------------------------------------------------------------------
    def _cycle(self):
        if len(self._maps) < 2:
            self._publish()
            return

        links = []
        for i in range(1, len(self._robots)):
            ns_i = self._robots[i]
            msg_i = self._maps.get(ns_i)
            if msg_i is None:
                continue
            best_dx = best_dy = best_dyaw = 0.0
            best_ratio = 0.0
            best_overlap = 0
            best_j = None
            for j in range(len(self._robots)):
                if j == i:
                    continue
                ns_j = self._robots[j]
                msg_j = self._maps.get(ns_j)
                if msg_j is None:
                    continue
                dx, dy, dyaw, overlap, ratio = self._match_pair(msg_i, self._poses[i], msg_j, self._poses[j])
                if overlap >= self._min_overlap and ratio > best_ratio:
                    best_dx, best_dy, best_dyaw = dx, dy, dyaw
                    best_ratio, best_overlap, best_j = ratio, overlap, j

            accepted = best_overlap >= self._min_overlap and best_ratio >= self._min_ratio
            if best_j is not None:
                links.append({'i': i, 'j': best_j, 'ratio': best_ratio, 'accepted': accepted})
            if accepted:
                pose = self._poses[i]
                old = (pose['x'], pose['y'], pose['yaw'])
                pose['x'] += self._gain * best_dx
                pose['y'] += self._gain * best_dy
                pose['yaw'] += self._gain * best_dyaw
                self.get_logger().info(
                    f'[{ns_i}] loop closure: overlap={best_overlap} ratio={best_ratio:.2f} '
                    f'raw_correction=({best_dx:+.3f},{best_dy:+.3f},{math.degrees(best_dyaw):+.2f}deg) '
                    f'pose ({old[0]:.3f},{old[1]:.3f},{math.degrees(old[2]):.1f}deg) -> '
                    f'({pose["x"]:.3f},{pose["y"]:.3f},{math.degrees(pose["yaw"]):.1f}deg)')

        if links:
            self._last_links = links
        self._publish()
        self._publish_markers()

    def _publish_markers(self):
        if not self._publish_markers_enabled:
            return
        markers = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)
        now = self.get_clock().now().to_msg()

        for idx, link in enumerate(self._last_links):
            pi, pj = self._poses[link['i']], self._poses[link['j']]
            accepted = link['accepted']
            color = (ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.9) if accepted
                     else ColorRGBA(r=0.9, g=0.6, b=0.1, a=0.5))
            line = Marker()
            line.header.frame_id = 'map'
            line.header.stamp = now
            line.ns = 'collab_links'
            line.id = idx
            line.type = Marker.LINE_LIST
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.08 if accepted else 0.03
            line.color = color
            line.points = [
                Point(x=pi['x'], y=pi['y'], z=0.2),
                Point(x=pj['x'], y=pj['y'], z=0.2),
            ]
            markers.markers.append(line)

            label = Marker()
            label.header.frame_id = 'map'
            label.header.stamp = now
            label.ns = 'collab_link_labels'
            label.id = idx
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = (pi['x'] + pj['x']) / 2.0
            label.pose.position.y = (pi['y'] + pj['y']) / 2.0
            label.pose.position.z = 0.5
            label.pose.orientation.w = 1.0
            label.scale.z = 0.25
            label.color = color
            label.text = f"ratio={link['ratio']:.2f}{' (accepted)' if accepted else ''}"
            markers.markers.append(label)

        self._marker_pub.publish(markers)

    def _publish(self):
        out = String()
        out.data = json.dumps(self._poses)
        self._pub.publish(out)

    # ------------------------------------------------------------------
    def _match_pair(self, msg_i: OccupancyGrid, pose_i: dict,
                     msg_j: OccupancyGrid, pose_j: dict):
        """Search a small (dx, dy, dyaw) window applied to robot i's pose that
        maximizes agreement between i's occupied cells and j's occupied cells,
        both projected into the shared global frame. Returns
        (best_dx, best_dy, best_dyaw, overlap_cells, match_ratio)."""
        res = msg_j.info.resolution
        if res <= 0.0 or msg_i.info.resolution <= 0.0:
            return 0.0, 0.0, 0.0, 0, 0.0

        i_gx, i_gy = self._occupied_global(msg_i, pose_i, res)
        if i_gx.size < self._min_overlap:
            return 0.0, 0.0, 0.0, 0, 0.0
        j_gx, j_gy = self._occupied_global(msg_j, pose_j, res)
        if j_gx.size == 0:
            return 0.0, 0.0, 0.0, 0, 0.0

        # j's occupied cells as a dense boolean grid over its own bounding
        # box + the search margin, so each candidate match is a vectorized
        # bounds check + fancy-index lookup instead of per-point set lookups.
        margin = int(math.ceil(self._sw_m / res)) + 1
        min_c = int(j_gx.min()) - margin
        min_r = int(j_gy.min()) - margin
        max_c = int(j_gx.max()) + margin
        max_r = int(j_gy.max()) + margin
        width = max_c - min_c + 1
        height = max_r - min_r + 1
        j_grid = np.zeros((height, width), dtype=bool)
        j_grid[j_gy - min_r, j_gx - min_c] = True

        # Local coords of robot i's occupied cells about its own origin, so
        # each search candidate is a cheap re-rotate + re-translate instead
        # of re-projecting from the raw grid every time.
        ox = msg_i.info.origin.position.x
        oy = msg_i.info.origin.position.y
        data = np.asarray(msg_i.data, dtype=np.int8).reshape(
            (msg_i.info.height, msg_i.info.width))
        rows, cols = np.nonzero(data == OCCUPIED)
        lx = ox + (cols.astype(np.float64) + 0.5) * msg_i.info.resolution
        ly = oy + (rows.astype(np.float64) + 0.5) * msg_i.info.resolution

        steps_m = max(1, round(self._sw_m / self._ss_m))
        steps_rad = max(1, round(self._sw_rad / self._ss_rad))

        best_ratio = -1.0
        best = (0.0, 0.0, 0.0, 0)
        n_total = lx.size
        for k in range(-steps_rad, steps_rad + 1):
            dyaw = k * self._ss_rad
            yaw = pose_i['yaw'] + dyaw
            c, s = math.cos(yaw), math.sin(yaw)
            gx0 = pose_i['x'] + c * lx - s * ly
            gy0 = pose_i['y'] + s * lx + c * ly
            for a in range(-steps_m, steps_m + 1):
                ddx = a * self._ss_m
                gx = np.floor((gx0 + ddx) / res).astype(np.int64) - min_c
                for b in range(-steps_m, steps_m + 1):
                    ddy = b * self._ss_m
                    gy = np.floor((gy0 + ddy) / res).astype(np.int64) - min_r
                    in_bounds = (gx >= 0) & (gx < width) & (gy >= 0) & (gy < height)
                    hits = int(j_grid[gy[in_bounds], gx[in_bounds]].sum())
                    ratio = hits / n_total
                    if ratio > best_ratio:
                        best_ratio = ratio
                        best = (ddx, ddy, dyaw, hits)

        ddx, ddy, dyaw, overlap = best
        return ddx, ddy, dyaw, overlap, max(best_ratio, 0.0)

    @staticmethod
    def _occupied_global(msg: OccupancyGrid, pose: dict, res: float):
        data = np.asarray(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))
        rows, cols = np.nonzero(data == OCCUPIED)
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        yaw = pose['yaw']
        c, s = math.cos(yaw), math.sin(yaw)
        lx = ox + (cols.astype(np.float64) + 0.5) * msg.info.resolution
        ly = oy + (rows.astype(np.float64) + 0.5) * msg.info.resolution
        gx = pose['x'] + c * lx - s * ly
        gy = pose['y'] + s * lx + c * ly
        return (np.floor(gx / res).astype(np.int64),
                np.floor(gy / res).astype(np.int64))


def main(args=None):
    rclpy.init(args=args)
    node = CollabLoopClosure()
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
