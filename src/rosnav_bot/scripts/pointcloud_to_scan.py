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
        # Terrain mode: obstacle = point sitting `obstacle_min..obstacle_max` above the
        # lowest point (local ground) in its grid cell, instead of a fixed sensor-height band.
        self.declare_parameter('ground_segment', False)
        self.declare_parameter('ground_cell', 0.3)
        self.declare_parameter('obstacle_min', 0.15)
        self.declare_parameter('obstacle_max', 1.2)

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
        if bool(self.get_parameter('ground_segment').value):
            self._project_ground(cloud, ranges, angle_min, angle_max, angle_inc, range_min, range_max)
            self._publish(cloud, ranges, n_bins, angle_min, angle_inc, range_min, range_max, target_frame)
            return
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

        self._publish(cloud, ranges, n_bins, angle_min, angle_inc, range_min, range_max, target_frame)

    def _project_ground(self, cloud, ranges, angle_min, angle_max, angle_inc, range_min, range_max):
        pts = point_cloud2.read_points(cloud, field_names=('x', 'y', 'z'), skip_nans=True)
        if pts.size == 0:
            return
        x = np.asarray(pts['x'], dtype=np.float64)
        y = np.asarray(pts['y'], dtype=np.float64)
        z = np.asarray(pts['z'], dtype=np.float64)
        fin = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        x, y, z = x[fin], y[fin], z[fin]
        if x.size == 0:
            return
        cell = float(self.get_parameter('ground_cell').value)
        h_min = float(self.get_parameter('obstacle_min').value)
        h_max = float(self.get_parameter('obstacle_max').value)
        ix = np.floor(x / cell).astype(np.int64)
        iy = np.floor(y / cell).astype(np.int64)
        key = ix * 100003 + iy
        uniq, inv = np.unique(key, return_inverse=True)
        zmin = np.full(uniq.size, np.inf)
        np.minimum.at(zmin, inv, z)
        h = z - zmin[inv]
        r = np.hypot(x, y)
        ang = np.arctan2(y, x)
        ok = (h >= h_min) & (h <= h_max) & (r >= range_min) & (r <= range_max) \
            & (ang >= angle_min) & (ang <= angle_max)
        idx = ((ang[ok] - angle_min) / angle_inc).astype(np.int64)
        good = (idx >= 0) & (idx < ranges.size)
        np.minimum.at(ranges, idx[good], r[ok][good].astype(np.float32))

    def _publish(self, cloud, ranges, n_bins, angle_min, angle_inc, range_min, range_max, target_frame):
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
