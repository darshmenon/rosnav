#!/usr/bin/env python3
"""
scan_to_odom_cloud.py — republish /scan as a PointCloud2 already expressed in the odom frame.

Why: bisecting the Nav2 controller_server `map->odom` freeze (2026-11-01) showed it only appears when
the local costmap's ObstacleLayer/VoxelLayer has a LaserScan observation source. That source uses a
tf2 MessageFilter on controller_server's TF buffer (laser_frame -> odom at every scan stamp); with no
obstacle layer at all there was no freeze in 550 s, with it the buffer stopped absorbing /tf after
35-260 s. A cloud whose frame_id is already the costmap's global frame needs no transform wait.
This node does the one TF lookup in its own process (slam_nav.launch.py local_scan_cloud:=true points the
local costmap at the result with sensor_frame base_link for the raytrace origin).

Params: scan_topic (/scan), cloud_topic (/scan_odom_cloud), target_frame (odom), tf_timeout (0.05), max_tf_lag (0.25 s: fall back to the latest TF if the scan stamp is just ahead of it),
        z (0.1)   height the 2D points are given
"""
import math

import numpy as np
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from sensor_msgs_py import point_cloud2


class ScanToOdomCloud(Node):
    def __init__(self):
        super().__init__('scan_to_odom_cloud')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cloud_topic', '/scan_odom_cloud')
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('tf_timeout', 0.05)
        self.declare_parameter('max_tf_lag', 0.25)
        self.declare_parameter('z', 0.1)
        self._target = self.get_parameter('target_frame').value
        self._timeout = Duration(seconds=float(self.get_parameter('tf_timeout').value))
        self._z = float(self.get_parameter('z').value)
        self._max_lag = float(self.get_parameter('max_tf_lag').value)
        self._n_fallback = 0
        self._tf = tf2_ros.Buffer()
        self._tfl = tf2_ros.TransformListener(self._tf, self)
        self._pub = self.create_publisher(
            point_cloud2.PointCloud2, self.get_parameter('cloud_topic').value, qos_profile_sensor_data)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self._on_scan,
                                 qos_profile_sensor_data)
        self._n = self._fail = 0
        self._last_err = ''
        self.create_timer(10.0, lambda: self.get_logger().info(
            f'scan_to_odom_cloud: {self._n} clouds published ({self._n_fallback} via latest-TF fallback), {self._fail} scans skipped (no TF)'
            + (f' | last error: {self._last_err}' if self._last_err else '')))
        self.get_logger().info(f'scan_to_odom_cloud: {self.get_parameter("scan_topic").value} -> '
                               f'{self.get_parameter("cloud_topic").value} in {self._target}')

    def _on_scan(self, msg: LaserScan) -> None:
        try:
            t = self._tf.lookup_transform(self._target, msg.header.frame_id, Time.from_msg(msg.header.stamp),
                                          timeout=self._timeout)
        except tf2_ros.ExtrapolationException as e:
            # Scan stamp is slightly ahead of the newest odom TF (slow sim, 10 Hz scan vs ~30 Hz TF).
            # For a 10 Hz costmap cloud the latest transform is accurate enough if it is recent.
            try:
                t = self._tf.lookup_transform(self._target, msg.header.frame_id, Time())
                lag = (Time.from_msg(msg.header.stamp) - Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
                if abs(lag) > self._max_lag:
                    raise e
                self._n_fallback += 1
            except tf2_ros.TransformException:
                self._fail += 1
                self._last_err = str(e)[:160]
                return
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
            self._fail += 1
            self._last_err = str(e)[:160]
            return
        r = np.asarray(msg.ranges, float)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        ok = np.isfinite(r) & (r > max(msg.range_min, 0.02)) & (r < msg.range_max)
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        px, py = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        pts = np.stack([c * px - s * py + t.transform.translation.x,
                        s * px + c * py + t.transform.translation.y,
                        np.full(px.shape, self._z)], axis=1).astype(np.float32)
        cloud = point_cloud2.create_cloud_xyz32(msg.header, pts)
        cloud.header.frame_id = self._target
        self._pub.publish(cloud)
        self._n += 1


def main():
    rclpy.init()
    node = ScanToOdomCloud()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # normal teardown (SIGINT from launch) — no traceback
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
