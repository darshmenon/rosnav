#!/usr/bin/env python3
"""
camera_terrain_speed_mask.py — live, heuristic terrain-roughness SpeedFilter
mask driven by the robot's own RGB-D camera, instead of
gen_terrain_speed_mask.py's static SDF-ground-truth bake (concepts.md §36/§37).
Reuses the exact same Nav2 costmap-filter wiring (SpeedFilter plugin,
costmap_filter_info_server, base=100/multiplier=-1 mask-value convention) —
this node just becomes the mask *publisher* in place of the static
filter_mask_server for a run. No new Nav2 plugin.

Heuristic (no ML dependency, consistent with this repo's other classical-CV
scripts like aruco_dock.py): unprojects the depth image into 3D points via
pinhole intrinsics (vectorized numpy — a downsampled pixel grid, not a
per-point Python loop, since a 1280x960 depth frame is too big to iterate one
pixel at a time every publish cycle), transforms them into base_link via TF,
keeps only points in a thin height band near the ground
(--ground-z-min/--ground-z-max — this robot has no calibrated ground-
clearance spec, so the band is a generous heuristic, not a measured one), and
uses per-cell Z variance among those points as a roughness proxy: flat
ground -> low variance -> full speed; uneven/rough ground -> high variance ->
near-stop. This is explicitly best-effort — there is no runtime ground-truth
terrain label to validate against, only the SDF friction values
gen_terrain_speed_mask.py already used for the static/baked version.

Publishes nav_msgs/OccupancyGrid directly on the mask topic (default
/gs/speed_filter_mask, TRANSIENT_LOCAL like map_server's latched publish) at
--publish-period Hz. The grid is small and re-centered on the robot's
map-frame pose each cycle — cells outside it are simply not part of the
message, same graceful "unrestricted where there's no data" behavior
gen_terrain_speed_mask.py relies on for unexplored zones.

Requires enable_rgbd:=true (slam_nav.launch.py terrain_live_camera:=true
forces it, same pattern slam_algo:=vslam already uses for enable_rgbd) and
the depth image + camera_info topics gz_bridge_rgbd.yaml already bridges.
"""
import math

import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                        ReliabilityPolicy, qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

_MASK_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def quaternion_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class CameraTerrainSpeedMask(Node):
    def __init__(self):
        super().__init__('camera_terrain_speed_mask')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('mask_topic', '/gs/speed_filter_mask')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('stride', 8)
        self.declare_parameter('max_range', 6.0)
        self.declare_parameter('resolution', 0.10)
        self.declare_parameter('grid_ahead', 3.0)
        self.declare_parameter('grid_behind', 0.5)
        self.declare_parameter('grid_width', 3.0)
        self.declare_parameter('ground_z_min', -0.05)
        self.declare_parameter('ground_z_max', 0.15)
        self.declare_parameter('smooth_variance', 0.00005)
        self.declare_parameter('rough_variance', 0.0015)
        self.declare_parameter('min_points_per_cell', 4)
        self.declare_parameter('publish_period', 1.0)

        self._bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._latest_depth = None
        self._cam_info = None

        self.create_subscription(
            Image, self.get_parameter('depth_topic').value, self._on_depth,
            qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value, self._on_info,
            qos_profile_sensor_data)
        self._pub = self.create_publisher(
            OccupancyGrid, self.get_parameter('mask_topic').value, _MASK_QOS)
        self._timer = self.create_timer(
            float(self.get_parameter('publish_period').value), self._on_timer)
        self.get_logger().info(
            'camera_terrain_speed_mask: live heuristic SpeedFilter mask from '
            f"{self.get_parameter('depth_topic').value} -> "
            f"{self.get_parameter('mask_topic').value} (best-effort, see concepts.md §37)")

    def _on_depth(self, msg: Image):
        self._latest_depth = msg

    def _on_info(self, msg: CameraInfo):
        self._cam_info = msg

    def _on_timer(self):
        depth_msg = self._latest_depth
        info = self._cam_info
        if depth_msg is None or info is None:
            return

        depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        depth = np.asarray(depth, dtype=np.float64)
        if depth_msg.encoding in ('16UC1', 'mono16'):
            depth = depth / 1000.0  # mm -> m

        base_frame = self.get_parameter('base_frame').value
        map_frame = self.get_parameter('map_frame').value
        camera_frame = depth_msg.header.frame_id or info.header.frame_id
        try:
            cam_to_base = self._tf_buffer.lookup_transform(
                base_frame, camera_frame, Time.from_msg(depth_msg.header.stamp))
            base_to_map = self._tf_buffer.lookup_transform(
                map_frame, base_frame, Time.from_msg(depth_msg.header.stamp))
        except Exception as exc:  # noqa: BLE001 — tf2 raises several distinct exception types
            self.get_logger().warn(f'TF lookup failed: {exc}', throttle_duration_sec=5.0)
            return

        stride = max(1, int(self.get_parameter('stride').value))
        max_range = float(self.get_parameter('max_range').value)
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]

        d = depth[::stride, ::stride]
        vs, us = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
        valid = np.isfinite(d) & (d > 0.05) & (d <= max_range)
        if not np.any(valid):
            return
        d, us, vs = d[valid], us[valid], vs[valid]

        # Pinhole unprojection in the optical frame (X right, Y down, Z forward
        # — see camera.xacro's "Optical frame" comment).
        x_cam = (us - cx) * d / fx
        y_cam = (vs - cy) * d / fy
        pts_cam = np.stack([x_cam, y_cam, d], axis=-1)

        r_cb = quaternion_to_matrix(
            cam_to_base.transform.rotation.x, cam_to_base.transform.rotation.y,
            cam_to_base.transform.rotation.z, cam_to_base.transform.rotation.w)
        t_cb = np.array([
            cam_to_base.transform.translation.x, cam_to_base.transform.translation.y,
            cam_to_base.transform.translation.z])
        base_pts = pts_cam @ r_cb.T + t_cb

        z_min = float(self.get_parameter('ground_z_min').value)
        z_max = float(self.get_parameter('ground_z_max').value)
        ground = base_pts[(base_pts[:, 2] >= z_min) & (base_pts[:, 2] <= z_max)]

        ahead = float(self.get_parameter('grid_ahead').value)
        behind = float(self.get_parameter('grid_behind').value)
        half_w = float(self.get_parameter('grid_width').value) / 2.0
        resolution = float(self.get_parameter('resolution').value)
        width = max(1, int(math.ceil((ahead + behind) / resolution)))
        height = max(1, int(math.ceil((2 * half_w) / resolution)))

        value = np.zeros(height * width, dtype=np.float64)  # default: full speed (0)
        min_pts = int(self.get_parameter('min_points_per_cell').value)
        if ground.shape[0] >= min_pts:
            col = np.floor((ground[:, 0] + behind) / resolution).astype(np.int64)
            row = np.floor((ground[:, 1] + half_w) / resolution).astype(np.int64)
            in_bounds = (col >= 0) & (col < width) & (row >= 0) & (row < height)
            col, row, gz = col[in_bounds], row[in_bounds], ground[in_bounds, 2]
            idx = row * width + col

            count = np.zeros(height * width, dtype=np.float64)
            sum_z = np.zeros(height * width, dtype=np.float64)
            sum_z2 = np.zeros(height * width, dtype=np.float64)
            np.add.at(count, idx, 1.0)
            np.add.at(sum_z, idx, gz)
            np.add.at(sum_z2, idx, gz * gz)

            valid_cells = count >= min_pts
            mean = np.divide(sum_z, count, out=np.zeros_like(sum_z), where=valid_cells)
            var = np.divide(sum_z2, count, out=np.zeros_like(sum_z), where=valid_cells) - mean * mean
            var = np.maximum(var, 0.0)

            smooth_var = float(self.get_parameter('smooth_variance').value)
            rough_var = float(self.get_parameter('rough_variance').value)
            frac = np.clip((var - smooth_var) / (rough_var - smooth_var), 0.0, 1.0)
            value[valid_cells] = frac[valid_cells] * 100.0

        # base_link-frame grid origin (behind/left corner) -> map frame.
        r_bm = quaternion_to_matrix(
            base_to_map.transform.rotation.x, base_to_map.transform.rotation.y,
            base_to_map.transform.rotation.z, base_to_map.transform.rotation.w)
        t_bm = np.array([
            base_to_map.transform.translation.x, base_to_map.transform.translation.y,
            base_to_map.transform.translation.z])
        origin_map = r_bm @ np.array([-behind, -half_w, 0.0]) + t_bm
        yaw = math.atan2(r_bm[1, 0], r_bm[0, 0])

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = map_frame
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = float(origin_map[0])
        grid.info.origin.position.y = float(origin_map[1])
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.z = math.sin(yaw / 2.0)
        grid.info.origin.orientation.w = math.cos(yaw / 2.0)
        grid.data = np.round(value).astype(np.int8).tolist()
        self._pub.publish(grid)


def main():
    rclpy.init()
    node = CameraTerrainSpeedMask()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
