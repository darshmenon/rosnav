#!/usr/bin/env python3
"""
slam_debug_monitor.py - live SLAM/TF stability diagnostics.

This is intentionally lightweight: it does not need ground truth. It watches
map->odom, odom->base_link, map->base_link, /map growth, /odom velocity, and
/scan proximity. The main signal is whether map->odom yaw/translation keeps
oscillating while map growth and robot motion are low.
"""

from __future__ import annotations

from collections import deque
import math
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
import tf2_ros

try:  # only present with ros-humble-rtabmap-ros
    from rtabmap_msgs.msg import Info as RtabInfo
except ImportError:
    RtabInfo = None


def _yaw_deg(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def _wrap_deg(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


class SlamDebugMonitor(Node):
    def __init__(self):
        super().__init__('slam_debug_monitor')

        self.declare_parameter('fixed_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('report_period_sec', 2.0)
        self.declare_parameter('window_sec', 20.0)
        self.declare_parameter('unstable_yaw_span_deg', 20.0)
        self.declare_parameter('unstable_translation_span_m', 0.5)
        self.declare_parameter('stationary_speed_mps', 0.03)
        self.declare_parameter('stationary_yaw_rate_radps', 0.05)
        self.declare_parameter('map_growth_epsilon_cells', 25)

        self._fixed_frame = str(self.get_parameter('fixed_frame').value)
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._window_sec = float(self.get_parameter('window_sec').value)
        self._unstable_yaw_span = float(
            self.get_parameter('unstable_yaw_span_deg').value)
        self._unstable_translation_span = float(
            self.get_parameter('unstable_translation_span_m').value)
        self._stationary_speed = float(
            self.get_parameter('stationary_speed_mps').value)
        self._stationary_yaw_rate = float(
            self.get_parameter('stationary_yaw_rate_radps').value)
        self._map_growth_epsilon = int(
            self.get_parameter('map_growth_epsilon_cells').value)

        self._tf_buffer = tf2_ros.Buffer(node=self)
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._map_total = 0
        self._map_free = 0
        self._map_occ = 0
        self._map_unknown = 0
        self._last_map_stamp = 0.0
        self._last_odom = None
        self._last_scan_min = None

        self._samples = deque()

        # Map-stream / pose-graph diagnostics (why does the map jump or look broken?)
        self._map_msgs = 0
        self._map_times = deque(maxlen=20)
        self._prev_map = None            # (width, height, ox, oy, data)
        self._last_map_changed = 0
        self._last_map_geom = ''
        self._last_map_geom_change = ''
        self._prev_map_odom = None
        self._max_graph_jump = (0.0, 0.0)
        self._graph_jumps = 0
        self._rtab_nodes = set()
        self._rtab_loop_closures = 0
        self._rtab_proximity = 0
        self._rtab_last_ref = -1

        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('map_topic').value),
            self._on_map,
            5,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._on_odom,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self._on_scan,
            qos_profile_sensor_data,
        )

        if RtabInfo is not None:
            self.create_subscription(RtabInfo, '/info', self._on_rtab_info, 5)

        period = max(0.5, float(self.get_parameter('report_period_sec').value))
        self.create_timer(period, self._report)
        self.get_logger().info(
            f'slam_debug_monitor frames={self._fixed_frame}->{self._odom_frame}'
            f'->{self._base_frame} window={self._window_sec:.1f}s')

    def _on_map(self, msg: OccupancyGrid) -> None:
        data = np.asarray(msg.data, dtype=np.int16)
        self._map_total = int(data.size)
        self._map_free = int(np.count_nonzero(data == 0))
        self._map_occ = int(np.count_nonzero(data > 50))
        self._map_unknown = int(np.count_nonzero(data < 0))
        self._last_map_stamp = time.monotonic()
        self._map_msgs += 1
        self._map_times.append(self._last_map_stamp)

        geom = (msg.info.width, msg.info.height,
                round(msg.info.origin.position.x, 3), round(msg.info.origin.position.y, 3))
        self._last_map_geom = f'{geom[0]}x{geom[1]}@({geom[2]},{geom[3]})'
        if self._prev_map is not None:
            pgeom, pdata = self._prev_map
            if pgeom == geom:
                # cells whose state flipped free<->occupied (unknown->known is normal growth)
                both = (pdata >= 0) & (data >= 0)
                self._last_map_changed = int(np.count_nonzero(
                    both & ((pdata > 50) != (data > 50))))
            else:
                self._last_map_changed = -1
                self._last_map_geom_change = f'{pgeom}->{geom}'
                self.get_logger().info(
                    f'slam_debug MAP RESIZED/SHIFTED {pgeom} -> {geom}')
        self._prev_map = (geom, data)

        # map->odom step across consecutive map updates = pose-graph optimisation jump
        tf = self._lookup(self._fixed_frame, self._odom_frame)
        if tf is not None:
            cur = (tf.transform.translation.x, tf.transform.translation.y,
                   _yaw_deg(tf.transform.rotation))
            if self._prev_map_odom is not None:
                dpos = math.hypot(cur[0] - self._prev_map_odom[0], cur[1] - self._prev_map_odom[1])
                dyaw = abs(_wrap_deg(cur[2] - self._prev_map_odom[2]))
                if dpos > 0.15 or dyaw > 3.0:
                    self._graph_jumps += 1
                    if dpos > self._max_graph_jump[0]:
                        self._max_graph_jump = (dpos, dyaw)
                    self.get_logger().warn(
                        f'slam_debug GRAPH JUMP map->odom moved {dpos:.2f}m / {dyaw:.1f}deg '
                        f'between map updates (loop closure / graph optimisation?)')
            self._prev_map_odom = cur

    def _on_rtab_info(self, msg) -> None:
        self._rtab_nodes.add(int(msg.ref_id))
        self._rtab_last_ref = int(msg.ref_id)
        if msg.loop_closure_id > 0:
            self._rtab_loop_closures += 1
            self.get_logger().info(
                f'slam_debug RTAB LOOP CLOSURE node {msg.ref_id} <-> {msg.loop_closure_id}')
        if msg.proximity_detection_id > 0:
            self._rtab_proximity += 1
            self.get_logger().info(
                f'slam_debug RTAB PROXIMITY link node {msg.ref_id} <-> {msg.proximity_detection_id}')

    def _on_odom(self, msg: Odometry) -> None:
        self._last_odom = msg

    def _on_scan(self, msg: LaserScan) -> None:
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        ranges = ranges[np.isfinite(ranges)]
        ranges = ranges[(ranges >= msg.range_min) & (ranges <= msg.range_max)]
        self._last_scan_min = float(np.min(ranges)) if ranges.size else None

    def _lookup(self, target: str, source: str):
        try:
            return self._tf_buffer.lookup_transform(
                target, source, Time(), timeout=Duration(seconds=0.03))
        except Exception:
            return None

    def _report(self) -> None:
        now = time.monotonic()
        map_odom = self._lookup(self._fixed_frame, self._odom_frame)
        odom_base = self._lookup(self._odom_frame, self._base_frame)
        map_base = self._lookup(self._fixed_frame, self._base_frame)

        if map_odom is None or odom_base is None or map_base is None:
            self.get_logger().warn('slam_debug: missing map/odom/base_link TF')
            return

        mot = map_odom.transform.translation
        moyaw = _yaw_deg(map_odom.transform.rotation)
        mbt = map_base.transform.translation
        mbyaw = _yaw_deg(map_base.transform.rotation)
        obt = odom_base.transform.translation
        obyaw = _yaw_deg(odom_base.transform.rotation)

        vx = 0.0
        wz = 0.0
        if self._last_odom is not None:
            vx = float(self._last_odom.twist.twist.linear.x)
            wz = float(self._last_odom.twist.twist.angular.z)

        known = self._map_free + self._map_occ
        coverage = (known / self._map_total * 100.0) if self._map_total else 0.0
        sample = {
            't': now,
            'map_odom_x': float(mot.x),
            'map_odom_y': float(mot.y),
            'map_odom_yaw': float(moyaw),
            'known': known,
            'vx': abs(vx),
            'wz': abs(wz),
        }
        self._samples.append(sample)
        while self._samples and now - self._samples[0]['t'] > self._window_sec:
            self._samples.popleft()

        yaw_values = [s['map_odom_yaw'] for s in self._samples]
        yaw_span = 0.0
        if yaw_values:
            base = yaw_values[0]
            unwrapped = [base + _wrap_deg(y - base) for y in yaw_values]
            yaw_span = max(unwrapped) - min(unwrapped)

        xs = [s['map_odom_x'] for s in self._samples]
        ys = [s['map_odom_y'] for s in self._samples]
        trans_span = 0.0
        if xs and ys:
            trans_span = math.hypot(max(xs) - min(xs), max(ys) - min(ys))

        known_growth = known - self._samples[0]['known'] if self._samples else 0
        max_vx = max((s['vx'] for s in self._samples), default=0.0)
        max_wz = max((s['wz'] for s in self._samples), default=0.0)
        stationary = max_vx < self._stationary_speed and max_wz < self._stationary_yaw_rate
        map_flat = known_growth < self._map_growth_epsilon
        unstable = (
            len(self._samples) >= 3
            and map_flat
            and stationary
            and (
                yaw_span >= self._unstable_yaw_span
                or trans_span >= self._unstable_translation_span
            )
        )

        status = 'SLAM_UNSTABLE' if unstable else 'ok'
        scan = 'n/a' if self._last_scan_min is None else f'{self._last_scan_min:.2f}m'
        map_age = now - self._last_map_stamp if self._last_map_stamp else float('inf')
        self.get_logger().info(
            f'slam_debug status={status} '
            f'map_odom=({mot.x:.2f},{mot.y:.2f},{moyaw:.1f}deg) '
            f'odom_base=({obt.x:.2f},{obt.y:.2f},{obyaw:.1f}deg) '
            f'map_base=({mbt.x:.2f},{mbt.y:.2f},{mbyaw:.1f}deg) '
            f'window yaw_span={yaw_span:.1f}deg trans_span={trans_span:.2f}m '
            f'known_growth={known_growth} coverage={coverage:.1f}% '
            f'vx={vx:.3f} wz={wz:.3f} nearest_scan={scan} map_age={map_age:.1f}s')
        hz = 0.0
        if len(self._map_times) >= 2 and self._map_times[-1] > self._map_times[0]:
            hz = (len(self._map_times) - 1) / (self._map_times[-1] - self._map_times[0])
        rtab = ''
        if RtabInfo is not None:
            rtab = (f' rtab_nodes={len(self._rtab_nodes)} loop_closures={self._rtab_loop_closures}'
                    f' proximity={self._rtab_proximity}')
        self.get_logger().info(
            f'slam_debug_map msgs={self._map_msgs} rate={hz:.2f}Hz geom={self._last_map_geom} '
            f'flipped_cells_last_update={self._last_map_changed} '
            f'graph_jumps={self._graph_jumps} max_jump={self._max_graph_jump[0]:.2f}m/'
            f'{self._max_graph_jump[1]:.1f}deg{rtab}')


def main(args=None):
    rclpy.init(args=args)
    node = SlamDebugMonitor()
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
