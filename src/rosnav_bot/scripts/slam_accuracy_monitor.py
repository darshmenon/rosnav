#!/usr/bin/env python3
"""
slam_accuracy_monitor.py — Quantify SLAM/localization accuracy over time.

Always reports the map->odom correction (position + yaw) that slam_toolbox/
AMCL is applying on top of raw odometry — a useful drift-correction signal on
its own, with no extra setup.

If a ground-truth pose topic is bridged (e.g. Gazebo's own pose for the robot
model, bridged to ROS as nav_msgs/Odometry or geometry_msgs/PoseStamped), also
tracks live position/yaw error, RMSE, and max error against the SLAM estimate
(map->base_link). Ground truth and the map frame don't share an origin or
heading in general (e.g. Gazebo world frame vs. a map frame zeroed at the
robot's spawn pose) — the node self-aligns on the first sample pair instead of
assuming they're numerically comparable.

Usage (drift-only, no ground truth needed):
  ros2 run rosnav_bot slam_accuracy_monitor.py --ros-args -p use_sim_time:=true

Usage (with ground truth bridged as nav_msgs/Odometry on /ground_truth/odom):
  ros2 run rosnav_bot slam_accuracy_monitor.py --ros-args -p use_sim_time:=true \\
      -p ground_truth_topic:=/ground_truth/odom -p ground_truth_type:=odometry

Usage (ground truth as geometry_msgs/PoseStamped):
  ros2 run rosnav_bot slam_accuracy_monitor.py --ros-args -p use_sim_time:=true \\
      -p ground_truth_topic:=/ground_truth/pose -p ground_truth_type:=pose_stamped
"""

from __future__ import annotations

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import tf2_ros


def _yaw_deg(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def _wrap_deg(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


class SlamAccuracyMonitor(Node):
    def __init__(self):
        super().__init__('slam_accuracy_monitor')

        self.declare_parameter('fixed_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('report_period_sec', 5.0)
        self.declare_parameter('ground_truth_topic', '')  # '' = disabled
        self.declare_parameter('ground_truth_type', 'odometry')  # odometry | pose_stamped

        self._fixed_frame = str(self.get_parameter('fixed_frame').value)
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)

        self._tf_buffer = tf2_ros.Buffer(node=self)
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        gt_topic = str(self.get_parameter('ground_truth_topic').value).strip()
        gt_type = str(self.get_parameter('ground_truth_type').value).strip().lower()
        self._gt_enabled = bool(gt_topic)
        self._gt_topic = gt_topic
        self._gt_xyz_yaw: tuple[float, float, float] | None = None

        if self._gt_enabled:
            if gt_type not in ('odometry', 'pose_stamped'):
                raise SystemExit(
                    f"ground_truth_type must be odometry|pose_stamped, got {gt_type!r}")
            if gt_type == 'odometry':
                self.create_subscription(
                    Odometry, gt_topic, self._on_gt_odom, qos_profile_sensor_data)
            else:
                self.create_subscription(
                    PoseStamped, gt_topic, self._on_gt_pose, qos_profile_sensor_data)

        # Alignment computed once, from the first (ground_truth, map->base_link) pair.
        self._aligned = False
        self._align_dx = 0.0
        self._align_dy = 0.0
        self._align_dyaw = 0.0

        self._n_samples = 0
        self._sum_sq_err = 0.0
        self._max_err = 0.0
        self._last_pos_err: float | None = None
        self._last_yaw_err: float | None = None

        period = float(self.get_parameter('report_period_sec').value)
        self.create_timer(max(0.5, period), self._report)

        self.get_logger().info(
            f'slam_accuracy_monitor fixed_frame={self._fixed_frame} '
            f'odom_frame={self._odom_frame} base_frame={self._base_frame} '
            + (f'ground_truth={gt_topic} ({gt_type})' if self._gt_enabled
               else 'ground_truth=off (drift-only mode)'))

    def _on_gt_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._gt_xyz_yaw = (p.x, p.y, _yaw_deg(msg.pose.pose.orientation))

    def _on_gt_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._gt_xyz_yaw = (p.x, p.y, _yaw_deg(msg.pose.orientation))

    def _lookup(self, target: str, source: str):
        try:
            return self._tf_buffer.lookup_transform(
                target, source, Time(), timeout=Duration(seconds=0.05))
        except Exception:
            return None

    def _report(self) -> None:
        map_odom = self._lookup(self._fixed_frame, self._odom_frame)
        map_base = self._lookup(self._fixed_frame, self._base_frame)

        if map_odom is None or map_base is None:
            self.get_logger().warn('map/odom/base_link TF not available yet')
            return

        tr = map_odom.transform.translation
        yaw = _yaw_deg(map_odom.transform.rotation)
        drift_bits = f'map_odom_correction: x:{tr.x:.3f} y:{tr.y:.3f} yaw:{yaw:.1f}deg'

        if not self._gt_enabled:
            self.get_logger().info(drift_bits)
            return

        if self._gt_xyz_yaw is None:
            self.get_logger().info(f'{drift_bits}; ground_truth: waiting for {self._gt_topic}')
            return

        gx, gy, gyaw = self._gt_xyz_yaw
        mb = map_base.transform.translation
        myaw = _yaw_deg(map_base.transform.rotation)

        if not self._aligned:
            # Rigid 2D alignment: rotate+translate ground truth's frame onto
            # the map frame using this first sample pair, so a constant
            # frame offset (different origin/heading) doesn't show up as
            # bogus "error".
            self._align_dyaw = myaw - gyaw
            rad = math.radians(self._align_dyaw)
            cos_a, sin_a = math.cos(rad), math.sin(rad)
            self._align_dx = mb.x - (gx * cos_a - gy * sin_a)
            self._align_dy = mb.y - (gx * sin_a + gy * cos_a)
            self._aligned = True
            self.get_logger().info(
                f'ground truth aligned to map frame (dyaw={self._align_dyaw:.1f}deg)')

        rad = math.radians(self._align_dyaw)
        cos_a, sin_a = math.cos(rad), math.sin(rad)
        aligned_x = gx * cos_a - gy * sin_a + self._align_dx
        aligned_y = gx * sin_a + gy * cos_a + self._align_dy
        aligned_yaw = _wrap_deg(gyaw + self._align_dyaw)

        pos_err = math.hypot(mb.x - aligned_x, mb.y - aligned_y)
        yaw_err = _wrap_deg(myaw - aligned_yaw)

        self._n_samples += 1
        self._sum_sq_err += pos_err * pos_err
        self._max_err = max(self._max_err, pos_err)
        self._last_pos_err = pos_err
        self._last_yaw_err = yaw_err
        rmse = math.sqrt(self._sum_sq_err / self._n_samples)

        self.get_logger().info(
            f'{drift_bits}; accuracy: pos_err={pos_err:.3f}m yaw_err={yaw_err:.1f}deg '
            f'rmse={rmse:.3f}m max_err={self._max_err:.3f}m n={self._n_samples}')


def main(args=None):
    rclpy.init(args=args)
    node = SlamAccuracyMonitor()
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
