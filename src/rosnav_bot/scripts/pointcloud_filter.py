#!/usr/bin/env python3
"""
pointcloud_filter.py — cheap 3D point-cloud preprocessing for the 3D-lidar SLAM paths (numpy only).

Pipeline per cloud (each step switchable by a parameter):
  1. crop box        keep min_range <= r <= max_range, z_min <= z <= z_max     (drops robot body hits, sky, far noise)
  2. ground removal  drop points with z < ground_z                              (flat-ground approximation)
  3. voxel grid      keep one point (the centroid) per voxel_size cube           (the "voxel reduce")
  4. outlier removal drop voxels holding fewer than min_pts raw points           (isolated returns / noise)
Order is crop -> ground -> voxel -> outlier, so the outlier test uses the point counts per voxel.
Topics: `cloud_in` (PointCloud2) -> `cloud_out` (PointCloud2, same frame/stamp, xyz[+intensity] float32).
Parameters: voxel_size (0.10, 0 = off), min_range (0.3), max_range (30), z_min (-1.0), z_max (3.0),
            ground_z (-99 = off), min_pts (1 = off)
  ros2 run rosnav_bot pointcloud_filter.py --ros-args -r cloud_in:=/lidar/points -r cloud_out:=/lidar/points_filtered
Wired (optional, off by default) via slam_nav.launch.py cloud_filter:=true -> RTAB-Map scan_cloud. Offline test: python3 pointcloud_filter.py --selftest
"""
import sys

import numpy as np


def filter_cloud(xyz, voxel_size=0.10, min_range=0.3, max_range=30.0, z_min=-1.0, z_max=3.0, ground_z=-99.0, min_pts=1):
    """xyz: (N,3) float array -> filtered (M,3). Voxel step returns the centroid of each occupied voxel."""
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    r = np.linalg.norm(xyz[:, :2], axis=1)
    xyz = xyz[(r >= min_range) & (r <= max_range) & (xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max) & (xyz[:, 2] >= ground_z)]
    if voxel_size <= 0 or len(xyz) == 0:
        return xyz
    keys = np.floor(xyz / voxel_size).astype(np.int64)
    _, inv, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    inv = inv.ravel()
    sums = np.zeros((len(counts), 3), dtype=np.float64)
    np.add.at(sums, inv, xyz)
    cent = (sums / counts[:, None]).astype(np.float32)
    return cent[counts >= min_pts]


def selftest():
    rng = np.random.default_rng(0)
    wall = np.column_stack([rng.uniform(-5, 5, 40000), np.full(40000, 4.0) + rng.normal(0, 0.005, 40000), rng.uniform(0, 2, 40000)])
    ground = np.column_stack([rng.uniform(-8, 8, 20000), rng.uniform(-8, 8, 20000), np.full(20000, -0.45)])
    noise = rng.uniform(-8, 8, (50, 3))                          # isolated returns
    body = rng.normal(0, 0.05, (500, 3))                         # hits on the robot itself
    cloud = np.vstack([wall, ground, noise, body]).astype(np.float32)
    out = filter_cloud(cloud, voxel_size=0.10, ground_z=-0.40, min_pts=3)
    print(f'{len(cloud)} -> {len(out)} points ({100 * len(out) / len(cloud):.1f}%)')
    assert len(out) < 0.10 * len(cloud), 'voxel grid did not reduce enough'
    assert not (out[:, 2] < -0.40).any(), 'ground not removed'
    assert not (np.linalg.norm(out[:, :2], axis=1) < 0.3).any(), 'robot body not cropped'
    assert abs(np.median(out[np.abs(out[:, 1] - 4.0) < 0.3][:, 1]) - 4.0) < 0.05, 'wall position moved'
    assert len(filter_cloud(cloud, voxel_size=0.10, ground_z=-0.40, min_pts=3)) <= len(filter_cloud(cloud, voxel_size=0.10, ground_z=-0.40, min_pts=1))
    print('selftest OK (wall stays within 5 cm, ground + body + isolated noise gone)')


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2 as pc2

    class N(Node):
        def __init__(self):
            super().__init__('pointcloud_filter')
            d = dict(voxel_size=0.10, min_range=0.3, max_range=30.0, z_min=-1.0, z_max=3.0, ground_z=-99.0, min_pts=1)
            for k, v in d.items():
                self.declare_parameter(k, v)
            self.k = {k: self.get_parameter(k).value for k in d}
            self.pub = self.create_publisher(PointCloud2, 'cloud_out', qos_profile_sensor_data)
            self.create_subscription(PointCloud2, 'cloud_in', self.cb, qos_profile_sensor_data)
            self.n, self.last = 0, 0.0
            self.get_logger().info(f'pointcloud_filter {self.k}')

        def cb(self, m):
            pts = pc2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True)
            out = filter_cloud(pts, **self.k)
            self.pub.publish(pc2.create_cloud_xyz32(m.header, out.tolist()))
            self.n += 1
            t = self.get_clock().now().nanoseconds / 1e9
            if t - self.last > 10.0:
                self.last = t
                self.get_logger().info(f'{len(pts)} -> {len(out)} points ({100 * len(out) / max(1, len(pts)):.0f}%), {self.n} clouds')

    rclpy.init()
    n = N()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    selftest() if '--selftest' in sys.argv else main()
