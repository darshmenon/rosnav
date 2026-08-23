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
import time
from collections import Counter

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf2_ros


def _finite(x: float) -> bool:
    return math.isfinite(x)


def _yaw_deg(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


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
        self.declare_parameter('diagnose_tf', True)
        self.declare_parameter('fixed_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('warn_scan_age_sec', 0.25)
        self.declare_parameter('diagnose_odom', True)
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('warn_odom_age_sec', 0.25)

        mode = str(self.get_parameter('mode').value).strip().lower()
        if mode not in ('monitor', 'gate'):
            raise SystemExit(f"mode must be monitor|gate, got {mode!r}")
        self._gate = mode == 'gate'

        self._min_valid_ratio = float(self.get_parameter('min_valid_ratio').value)
        self._min_valid_beams = int(self.get_parameter('min_valid_beams').value)
        self._max_nan_ratio = float(self.get_parameter('max_nan_ratio').value)
        self._beam_jump = float(self.get_parameter('allow_beam_count_jump').value)
        self._require_frame = bool(self.get_parameter('require_frame_id').value)
        self._diagnose_tf = bool(self.get_parameter('diagnose_tf').value)
        self._fixed_frame = str(self.get_parameter('fixed_frame').value)
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._warn_scan_age = float(self.get_parameter('warn_scan_age_sec').value)
        self._diagnose_odom = bool(self.get_parameter('diagnose_odom').value)
        self._odom_topic = str(self.get_parameter('odom_topic').value)
        self._warn_odom_age = float(self.get_parameter('warn_odom_age_sec').value)

        scan_in = str(self.get_parameter('scan_in').value)
        scan_out = str(self.get_parameter('scan_out').value)

        self._tf_buffer = None
        self._tf_listener = None
        if self._diagnose_tf:
            self._tf_buffer = tf2_ros.Buffer(node=self)
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._pub = None
        if self._gate:
            self._pub = self.create_publisher(
                LaserScan, scan_out, qos_profile_sensor_data)

        self.create_subscription(
            LaserScan, scan_in, self._on_scan, qos_profile_sensor_data)
        if self._diagnose_odom:
            self.create_subscription(
                Odometry, self._odom_topic, self._on_odom, qos_profile_sensor_data)

        self._ok = 0
        self._bad = 0
        self._reasons: Counter[str] = Counter()
        self._last_n_beams: int | None = None
        self._last_stamp_ns: int | None = None
        self._last_rx_monotonic: float | None = None
        self._last_frame = ''
        self._last_age_sec: float | None = None
        self._last_rx_period_sec: float | None = None
        self._last_valid_ratio: float | None = None
        self._last_nan_ratio: float | None = None
        self._last_angle_span_deg: float | None = None
        self._last_odom_rx_monotonic: float | None = None
        self._last_odom_rx_period_sec: float | None = None
        self._last_odom_age_sec: float | None = None
        self._last_odom_frame = ''
        self._last_odom_child = ''
        self._last_odom_x: float | None = None
        self._last_odom_y: float | None = None
        self._last_odom_yaw_deg: float | None = None
        self._last_odom_vx: float | None = None
        self._last_odom_wz: float | None = None

        period = float(self.get_parameter('stats_period_sec').value)
        self.create_timer(max(0.5, period), self._report)

        self.get_logger().info(
            f'scan_quality_gate mode={mode} in={scan_in}'
            + (f' out={scan_out}' if self._gate else ' (monitor only)')
            + (f' odom={self._odom_topic}' if self._diagnose_odom else ' odom=off'))

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
        self._record_scan_stats(msg)
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

    def _record_scan_stats(self, msg: LaserScan) -> None:
        now_mono = time.monotonic()
        if self._last_rx_monotonic is not None:
            self._last_rx_period_sec = now_mono - self._last_rx_monotonic
        self._last_rx_monotonic = now_mono

        self._last_frame = msg.header.frame_id or ''
        self._last_angle_span_deg = math.degrees(msg.angle_max - msg.angle_min)

        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        now_ns = self.get_clock().now().nanoseconds
        self._last_age_sec = (now_ns - stamp_ns) / 1e9 if stamp_ns > 0 else None

        n = len(msg.ranges)
        if n <= 0:
            self._last_valid_ratio = None
            self._last_nan_ratio = None
            return
        nan_n = 0
        valid_n = 0
        for r in msg.ranges:
            if not _finite(r):
                nan_n += 1
            elif msg.range_min <= r <= msg.range_max:
                valid_n += 1
        self._last_valid_ratio = valid_n / n
        self._last_nan_ratio = nan_n / n

    def _on_odom(self, msg: Odometry) -> None:
        now_mono = time.monotonic()
        if self._last_odom_rx_monotonic is not None:
            self._last_odom_rx_period_sec = now_mono - self._last_odom_rx_monotonic
        self._last_odom_rx_monotonic = now_mono

        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        now_ns = self.get_clock().now().nanoseconds
        self._last_odom_age_sec = (now_ns - stamp_ns) / 1e9 if stamp_ns > 0 else None
        self._last_odom_frame = msg.header.frame_id or ''
        self._last_odom_child = msg.child_frame_id or ''
        self._last_odom_x = msg.pose.pose.position.x
        self._last_odom_y = msg.pose.pose.position.y
        self._last_odom_yaw_deg = _yaw_deg(msg.pose.pose.orientation)
        self._last_odom_vx = msg.twist.twist.linear.x
        self._last_odom_wz = msg.twist.twist.angular.z

    def _report(self) -> None:
        total = self._ok + self._bad
        if total == 0:
            self.get_logger().warn('no LaserScan received yet')
            return
        top = ', '.join(f'{k}={v}' for k, v in self._reasons.most_common(5)) or '-'
        pct = 100.0 * self._bad / total
        scan_bits = [
            f'frame={self._last_frame!r}',
            f'beams={self._last_n_beams if self._last_n_beams is not None else "?"}',
        ]
        if self._last_valid_ratio is not None:
            scan_bits.append(f'valid={100.0 * self._last_valid_ratio:.1f}%')
        if self._last_nan_ratio is not None:
            scan_bits.append(f'nan={100.0 * self._last_nan_ratio:.1f}%')
        if self._last_angle_span_deg is not None:
            scan_bits.append(f'span={self._last_angle_span_deg:.1f}deg')
        if self._last_age_sec is not None:
            scan_bits.append(f'stamp_age={self._last_age_sec:.3f}s')
        if self._last_rx_period_sec is not None:
            scan_bits.append(f'rx_dt={self._last_rx_period_sec:.3f}s')
        tf_bits = self._tf_summary()
        odom_bits = self._odom_summary()
        self.get_logger().info(
            f'scans ok={self._ok} rejected={self._bad} ({pct:.1f}%) '
            f'reasons: {top}; scan_diag: {" ".join(scan_bits)}'
            + (f'; odom_diag: {odom_bits}' if odom_bits else '')
            + (f'; tf_diag: {tf_bits}' if tf_bits else ''))

        if self._last_age_sec is not None and self._last_age_sec > self._warn_scan_age:
            throttle_sec = max(0.5, float(self.get_parameter('stats_period_sec').value))
            self.get_logger().warn(
                f'scan timestamp lag {self._last_age_sec:.3f}s exceeds '
                f'warn_scan_age_sec={self._warn_scan_age:.3f}s',
                throttle_duration_sec=throttle_sec)
        if self._last_odom_age_sec is not None and self._last_odom_age_sec > self._warn_odom_age:
            throttle_sec = max(0.5, float(self.get_parameter('stats_period_sec').value))
            self.get_logger().warn(
                f'odom timestamp lag {self._last_odom_age_sec:.3f}s exceeds '
                f'warn_odom_age_sec={self._warn_odom_age:.3f}s',
                throttle_duration_sec=throttle_sec)

    def _odom_summary(self) -> str:
        if not self._diagnose_odom:
            return ''
        if self._last_odom_rx_monotonic is None:
            return f'{self._odom_topic}=no_msg'
        parts = [
            f'frame={self._last_odom_frame!r}',
            f'child={self._last_odom_child!r}',
        ]
        if self._last_odom_age_sec is not None:
            parts.append(f'stamp_age={self._last_odom_age_sec:.3f}s')
        if self._last_odom_rx_period_sec is not None:
            parts.append(f'rx_dt={self._last_odom_rx_period_sec:.3f}s')
        if self._last_odom_x is not None and self._last_odom_y is not None:
            parts.append(f'pose=({self._last_odom_x:.2f},{self._last_odom_y:.2f})')
        if self._last_odom_yaw_deg is not None:
            parts.append(f'yaw={self._last_odom_yaw_deg:.1f}deg')
        if self._last_odom_vx is not None and self._last_odom_wz is not None:
            parts.append(f'twist=(vx:{self._last_odom_vx:.2f},wz:{self._last_odom_wz:.2f})')
        return ' '.join(parts)

    def _tf_summary(self) -> str:
        if self._tf_buffer is None:
            return ''
        pairs = (
            ('map_odom', self._fixed_frame, self._odom_frame),
            ('odom_base', self._odom_frame, self._base_frame),
            ('map_base', self._fixed_frame, self._base_frame),
        )
        parts = []
        for name, target, source in pairs:
            try:
                tf = self._tf_buffer.lookup_transform(
                    target, source, Time(),
                    timeout=Duration(seconds=0.02))
            except Exception as exc:
                parts.append(f'{name}=missing({type(exc).__name__})')
                continue
            tr = tf.transform.translation
            yaw = _yaw_deg(tf.transform.rotation)
            parts.append(f'{name}=x:{tr.x:.2f} y:{tr.y:.2f} yaw:{yaw:.1f}deg')
        return ' '.join(parts)


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
