#!/usr/bin/env python3
"""
orb_slam3_occupancy_bridge.py — Turn ORB-SLAM3's map->odom TF + the RGB-D
depth image into a real nav_msgs/OccupancyGrid on /map, so slam_algo:=orbslam3
plugs into Nav2/the frontier explorer the same way every other slam_algo does.

Why this exists: ORB-SLAM3 (docker/orb_slam3/, run as a separate sidecar
container — see its README for why it can't join this workspace) is a sparse
feature-based tracker. It publishes map->odom TF and a sparse landmark point
cloud, not an occupancy grid — unlike slam_toolbox/Cartographer/RTAB-Map,
which all build one internally. This node closes that gap the same way
RTAB-Map's Grid/Sensor=1 does for slam_algo:=vslam: project the depth image
through the current map->base_link transform, ray-trace free space, mark
occupied endpoints. It only depends on ORB-SLAM3's TF output (map->odom),
not on the sidecar being in this ROS graph in any other way — so it degrades
gracefully (keeps the last map, logs a warning) if that container isn't
running yet or drops out.

Height-filtered the same way pointcloud_to_scan.py is (obstacle_layer, not
voxel_layer — see concepts.md's "2D Lidar" notes): points between
min_height/max_height above base_link count as obstacles, everything else is
ignored (ceiling, floor noise).

Usage (started automatically by slam_nav.launch.py slam_algo:=orbslam3):
  ros2 run rosnav_bot orb_slam3_occupancy_bridge.py --ros-args -p use_sim_time:=true
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros

UNKNOWN = -1
FREE = 0
OCCUPIED = 100


class OrbSlam3OccupancyBridge(Node):
    def __init__(self):
        super().__init__('orb_slam3_occupancy_bridge')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('resolution', 0.05)     # matches other backends' Grid/CellSize
        self.declare_parameter('initial_size_m', 20.0)  # grid starts this wide, grows on demand
        self.declare_parameter('min_height', 0.05)      # base_link frame, matches pointcloud_to_scan.py
        self.declare_parameter('max_height', 0.55)
        self.declare_parameter('max_range', 8.0)        # matches rtabmap_vslam_node's Grid/RangeMax
        self.declare_parameter('depth_stride', 4)        # subsample the depth image for speed
        self.declare_parameter('publish_period_sec', 0.5)
        self.declare_parameter('tf_timeout_sec', 0.2)

        self._resolution = float(self.get_parameter('resolution').value)
        self._min_h = float(self.get_parameter('min_height').value)
        self._max_h = float(self.get_parameter('max_height').value)
        self._max_range = float(self.get_parameter('max_range').value)
        self._stride = max(1, int(self.get_parameter('depth_stride').value))
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._tf_timeout = Duration(
            seconds=float(self.get_parameter('tf_timeout_sec').value))

        size_m = float(self.get_parameter('initial_size_m').value)
        n = int(math.ceil(size_m / self._resolution))
        self._grid = np.full((n, n), UNKNOWN, dtype=np.int8)
        # origin_x/y: world-frame coords of cell (0, 0) — grid is centered on
        # wherever the robot is when the first usable TF/depth pair arrives.
        self._origin_x = None
        self._origin_y = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._camera_info = None
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._on_camera_info, 5)

        latched_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._map_pub = self.create_publisher(OccupancyGrid, '/map', latched_qos)

        self.create_subscription(
            Image, self.get_parameter('depth_topic').value, self._on_depth, 5)

        period = float(self.get_parameter('publish_period_sec').value)
        self._publish_timer = self.create_timer(period, self._publish_map)
        self._warned_no_tf = False

        self.get_logger().info(
            f'orb_slam3_occupancy_bridge: {self.get_parameter("depth_topic").value} '
            f'+ {self._map_frame}->{self._base_frame} TF -> /map '
            f'(res={self._resolution}m, height=[{self._min_h}, {self._max_h}]m). '
            'Waiting for the ORB-SLAM3 sidecar to publish map->odom TF '
            '(docker compose up orb_slam3, or docker/orb_slam3/build_bare_metal.sh) — '
            '/map stays unknown until then.')

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _ensure_bounds(self, xs: np.ndarray, ys: np.ndarray) -> None:
        """Grow self._grid (numpy pad) if any world point falls outside it."""
        if self._origin_x is None:
            return
        col = ((xs - self._origin_x) / self._resolution).astype(np.int64)
        row = ((ys - self._origin_y) / self._resolution).astype(np.int64)
        if col.size == 0:
            return
        min_c, max_c = int(col.min()), int(col.max())
        min_r, max_r = int(row.min()), int(row.max())
        pad_left = max(0, -min_c)
        pad_right = max(0, max_c - (self._grid.shape[1] - 1))
        pad_bottom = max(0, -min_r)
        pad_top = max(0, max_r - (self._grid.shape[0] - 1))
        if pad_left or pad_right or pad_bottom or pad_top:
            self._grid = np.pad(
                self._grid,
                ((pad_bottom, pad_top), (pad_left, pad_right)),
                mode='constant', constant_values=UNKNOWN)
            self._origin_x -= pad_left * self._resolution
            self._origin_y -= pad_bottom * self._resolution

    def _world_to_cell(self, x: float, y: float):
        col = int((x - self._origin_x) / self._resolution)
        row = int((y - self._origin_y) / self._resolution)
        return row, col

    def _raytrace_free(self, r0: int, c0: int, r1: int, c1: int) -> None:
        """Bresenham line, marking every cell up to (not including) the
        endpoint as FREE — endpoint is set OCCUPIED separately."""
        dr, dc = abs(r1 - r0), abs(c1 - c0)
        sr = 1 if r0 < r1 else -1
        sc = 1 if c0 < c1 else -1
        err = dr - dc
        r, c = r0, c0
        h, w = self._grid.shape
        while (r, c) != (r1, c1):
            if 0 <= r < h and 0 <= c < w and self._grid[r, c] != OCCUPIED:
                self._grid[r, c] = FREE
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r += sr
            if e2 < dr:
                err += dr
                c += sc

    def _on_depth(self, msg: Image) -> None:
        if self._camera_info is None:
            return
        try:
            tf_map_base = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, Time(),
                timeout=self._tf_timeout)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            if not self._warned_no_tf:
                self.get_logger().warn(
                    f'{self._map_frame}->{self._base_frame} TF not available yet — '
                    'is the ORB-SLAM3 sidecar running? /map stays unknown until it publishes.',
                    throttle_duration_sec=10.0)
                self._warned_no_tf = True
            return
        self._warned_no_tf = False

        try:
            tf_map_cam = self._tf_buffer.lookup_transform(
                self._map_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp),
                timeout=self._tf_timeout)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        fx, fy = self._camera_info.k[0], self._camera_info.k[4]
        cx, cy = self._camera_info.k[2], self._camera_info.k[5]

        vs, us = np.mgrid[0:msg.height:self._stride, 0:msg.width:self._stride]
        z = depth[vs, us]
        valid = np.isfinite(z) & (z > 0.05) & (z < self._max_range)
        if not np.any(valid):
            return
        us, vs, z = us[valid].astype(np.float32), vs[valid].astype(np.float32), z[valid]

        # Camera-optical-frame point cloud (Z forward, X right, Y down —
        # matches camera.xacro's camera_optical_joint convention).
        x_cam = (us - cx) * z / fx
        y_cam = (vs - cy) * z / fy
        z_cam = z

        cq = tf_map_cam.transform.rotation
        R = _quat_to_matrix(cq.x, cq.y, cq.z, cq.w)
        t = tf_map_cam.transform.translation
        pts_cam = np.stack([x_cam, y_cam, z_cam], axis=1)
        pts_map = pts_cam @ R.T + np.array([t.x, t.y, t.z])

        base_z = tf_map_base.transform.translation.z

        height = pts_map[:, 2] - base_z
        keep = (height > self._min_h) & (height < self._max_h)
        if not np.any(keep):
            return
        xs, ys = pts_map[keep, 0], pts_map[keep, 1]

        if self._origin_x is None:
            self._origin_x = t.x - self._grid.shape[1] * self._resolution / 2.0
            self._origin_y = t.y - self._grid.shape[0] * self._resolution / 2.0

        self._ensure_bounds(np.append(xs, t.x), np.append(ys, t.y))
        r0, c0 = self._world_to_cell(t.x, t.y)
        for x, y in zip(xs, ys):
            r1, c1 = self._world_to_cell(x, y)
            self._raytrace_free(r0, c0, r1, c1)
            if 0 <= r1 < self._grid.shape[0] and 0 <= c1 < self._grid.shape[1]:
                self._grid[r1, c1] = OCCUPIED

    def _publish_map(self) -> None:
        if self._origin_x is None:
            return
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        info = MapMetaData()
        info.resolution = self._resolution
        info.width = self._grid.shape[1]
        info.height = self._grid.shape[0]
        info.origin.position.x = self._origin_x
        info.origin.position.y = self._origin_y
        info.origin.position.z = 0.0
        info.origin.orientation.w = 1.0
        msg.info = info
        msg.data = self._grid.flatten().tolist()
        self._map_pub.publish(msg)


def _quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    rclpy.init()
    node = OrbSlam3OccupancyBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
