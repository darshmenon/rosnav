#!/usr/bin/env python3
"""Project a 3D PointCloud2 into a 2D LaserScan for Nav2 / laser_filters / AMCL.

Used when lidar_type:=3d so /scan keeps flowing after the 2D gpu_lidar is
replaced by the VLP-16-style cloud on /points.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


class PointCloudToScan(Node):
    def __init__(self):
        super().__init__('pointcloud_to_scan')
        self.declare_parameter('target_frame', '')
        self.declare_parameter('min_height', 0.05)
        self.declare_parameter('max_height', 0.55)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', math.pi / 180.0)
        self.declare_parameter('scan_time', 0.1)
        self.declare_parameter('range_min', 0.30)
        self.declare_parameter('range_max', 30.0)
        self.declare_parameter('use_inf', True)

        self._sub = self.create_subscription(
            PointCloud2, 'cloud_in', self._on_cloud, qos_profile_sensor_data)
        self._pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.get_logger().info(
            'pointcloud_to_scan: cloud_in → scan '
            f'(z in [{self.get_parameter("min_height").value}, '
            f'{self.get_parameter("max_height").value}])')

    def _on_cloud(self, cloud: PointCloud2) -> None:
        min_h = float(self.get_parameter('min_height').value)
        max_h = float(self.get_parameter('max_height').value)
        angle_min = float(self.get_parameter('angle_min').value)
        angle_max = float(self.get_parameter('angle_max').value)
        angle_inc = float(self.get_parameter('angle_increment').value)
        range_min = float(self.get_parameter('range_min').value)
        range_max = float(self.get_parameter('range_max').value)
        use_inf = bool(self.get_parameter('use_inf').value)
        target_frame = str(self.get_parameter('target_frame').value).strip()

        if angle_inc <= 0.0 or angle_max <= angle_min:
            return
        n_bins = int(math.floor((angle_max - angle_min) / angle_inc)) + 1
        if n_bins < 2:
            return

        ranges = np.full(n_bins, np.inf if use_inf else range_max + 1.0, dtype=np.float32)
        for x, y, z in point_cloud2.read_points(
                cloud, field_names=('x', 'y', 'z'), skip_nans=True):
            if z < min_h or z > max_h:
                continue
            r = math.hypot(x, y)
            if r < range_min or r > range_max:
                continue
            angle = math.atan2(y, x)
            if angle < angle_min or angle > angle_max:
                continue
            idx = int((angle - angle_min) / angle_inc)
            if 0 <= idx < n_bins and r < ranges[idx]:
                ranges[idx] = r

        scan = LaserScan()
        scan.header = cloud.header
        if target_frame:
            scan.header.frame_id = target_frame
        scan.angle_min = angle_min
        scan.angle_max = angle_min + (n_bins - 1) * angle_inc
        scan.angle_increment = angle_inc
        scan.time_increment = 0.0
        scan.scan_time = float(self.get_parameter('scan_time').value)
        scan.range_min = range_min
        scan.range_max = range_max
        scan.ranges = ranges.tolist()
        self._pub.publish(scan)


def main():
    rclpy.init()
    node = PointCloudToScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
