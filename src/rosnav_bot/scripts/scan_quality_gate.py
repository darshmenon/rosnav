#!/usr/bin/env python3
"""
scan_quality_gate.py — Find / reject malformed LaserScans before SLAM eats them.

laser_filters already drops near-range and shadow points. This node catches
*structurally* bad frames that still corrupt slam_toolbox / cartographer /
RTAB-Map when they slip through: empty beams, broken angle metadata, all-NaN,
too few finite hits, stamp/frame corruption, sudden beam-count jumps.

Modes:
  monitor (default)  Subscribe, log rejects + rate stats. Does not republish.
  gate               Subscribe scan_in, republish only clean scans on scan_out.

Usage (monitor live /scan while SLAM runs):
  ros2 run rosnav_bot scan_quality_gate.py --ros-args -p use_sim_time:=true

Usage (gate — insert between laser_filters and SLAM):
  # laser_filters: scan_filtered → scan_pre
  # this node:     scan_in=scan_pre, scan_out=scan
  ros2 run rosnav_bot scan_quality_gate.py --ros-args \\
      -p mode:=gate -p use_sim_time:=true \\
      -r scan_in:=scan_pre -r scan_out:=scan
"""

from __future__ import annotations

import math
from collections import Counter

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def _finite(x: float) -> bool:
    return math.isfinite(x)


class ScanQualityGate(Node):
    def __init__(self):
        super().__init__('scan_quality_gate')

        self.declare_parameter('mode', 'monitor')  # monitor | gate
        self.declare_parameter('scan_in', 'scan')
        self.declare_parameter('scan_out', 'scan_clean')
        self.declare_parameter('min_valid_ratio', 0.05)
        self.declare_parameter('min_valid_beams', 8)
        self.declare_parameter('max_nan_ratio', 0.50)
        self.declare_parameter('allow_beam_count_jump', 0.25)
        self.declare_parameter('require_frame_id', True)
        self.declare_parameter('stats_period_sec', 5.0)

        mode = str(self.get_parameter('mode').value).strip().lower()
        if mode not in ('monitor', 'gate'):
            raise SystemExit(f"mode must be monitor|gate, got {mode!r}")
        self._gate = mode == 'gate'

        self._min_valid_ratio = float(self.get_parameter('min_valid_ratio').value)
        self._min_valid_beams = int(self.get_parameter('min_valid_beams').value)
        self._max_nan_ratio = float(self.get_parameter('max_nan_ratio').value)
        self._beam_jump = float(self.get_parameter('allow_beam_count_jump').value)
        self._require_frame = bool(self.get_parameter('require_frame_id').value)

        scan_in = str(self.get_parameter('scan_in').value)
        scan_out = str(self.get_parameter('scan_out').value)

        self._pub = None
        if self._gate:
            self._pub = self.create_publisher(
                LaserScan, scan_out, qos_profile_sensor_data)

        self.create_subscription(
            LaserScan, scan_in, self._on_scan, qos_profile_sensor_data)

        self._ok = 0
        self._bad = 0
        self._reasons: Counter[str] = Counter()
        self._last_n_beams: int | None = None
        self._last_stamp_ns: int | None = None

        period = float(self.get_parameter('stats_period_sec').value)
        self.create_timer(max(0.5, period), self._report)

        self.get_logger().info(
            f'scan_quality_gate mode={mode} in={scan_in}'
            + (f' out={scan_out}' if self._gate else ' (monitor only)'))

    def _classify(self, msg: LaserScan) -> str | None:
        """Return reject reason, or None if the scan is usable."""
        if self._require_frame and not (msg.header.frame_id or '').strip():
            return 'empty_frame_id'

        n = len(msg.ranges)
        if n < 2:
            return 'too_few_beams'

        if not _finite(msg.angle_increment) or msg.angle_increment <= 0.0:
            return 'bad_angle_increment'
        if not _finite(msg.angle_min) or not _finite(msg.angle_max):
            return 'bad_angle_bounds'
        if msg.angle_max <= msg.angle_min:
            return 'angle_max_le_min'
        if not _finite(msg.range_min) or not _finite(msg.range_max):
            return 'bad_range_limits'
        if msg.range_max <= msg.range_min:
            return 'range_max_le_min'

        expected = int(round((msg.angle_max - msg.angle_min) / msg.angle_increment)) + 1
        # Allow ±2 beams — Gazebo / pointcloud_to_scan sometimes off-by-one.
        if abs(expected - n) > 2:
            return 'beam_count_vs_angles'

        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if stamp_ns <= 0:
            return 'zero_stamp'
        if self._last_stamp_ns is not None and stamp_ns < self._last_stamp_ns:
            return 'stamp_went_backwards'

        if self._last_n_beams is not None and self._last_n_beams > 0:
            jump = abs(n - self._last_n_beams) / float(self._last_n_beams)
            if jump > self._beam_jump:
                return 'beam_count_jump'

        nan_n = 0
        valid_n = 0
        for r in msg.ranges:
            if not _finite(r):
                nan_n += 1
                continue
            if msg.range_min <= r <= msg.range_max:
                valid_n += 1

        if nan_n / n > self._max_nan_ratio:
            return 'too_many_nan'
        if valid_n < self._min_valid_beams:
            return 'too_few_valid'
        if valid_n / n < self._min_valid_ratio:
            return 'low_valid_ratio'

        self._last_n_beams = n
        self._last_stamp_ns = stamp_ns
        return None

    def _on_scan(self, msg: LaserScan) -> None:
        reason = self._classify(msg)
        if reason is None:
            self._ok += 1
            if self._pub is not None:
                self._pub.publish(msg)
            return

        self._bad += 1
        self._reasons[reason] += 1
        self.get_logger().warning(
            f'reject {reason} beams={len(msg.ranges)} '
            f'frame={msg.header.frame_id!r}',
            throttle_duration_sec=2.0)

    def _report(self) -> None:
        total = self._ok + self._bad
        if total == 0:
            self.get_logger().warn('no LaserScan received yet')
            return
        top = ', '.join(f'{k}={v}' for k, v in self._reasons.most_common(5)) or '-'
        pct = 100.0 * self._bad / total
        self.get_logger().info(
            f'scans ok={self._ok} rejected={self._bad} ({pct:.1f}%) reasons: {top}')


def main(args=None):
    rclpy.init(args=args)
    node = ScanQualityGate()
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
