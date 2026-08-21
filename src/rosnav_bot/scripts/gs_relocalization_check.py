#!/usr/bin/env python3
"""
gs_relocalization_check.py — coarse localization-confidence signal from the
splat, on top of the geometric one RTAB-Map/AMCL already provide: at the
robot's current estimated pose, project the splat's own point colors through
the camera (same TF + intrinsics projection as gs_semantic_fusion.py, see
concepts.md §29) into a low-res color image, and compare it against the live
camera frame downsampled to the same resolution. If the estimated pose is
right, the two should roughly agree in average color per region; a bad
pose (drift, kidnapped robot, bad loop closure) desyncs them.

This is deliberately NOT full photometric splat rendering (Splat-Nav-style
render-and-compare needs the actual differentiable Gaussian rasterizer —
gsplat/nerfstudio's rendering pipeline running live inside a ROS node, a
much heavier GPU/dependency lift than this spike takes on, see concepts.md
§34). This is a coarse proxy: binned average point color vs. binned average
pixel color, nothing photoreal. Treat its output as a soft confidence signal
to log/monitor, not something to actually gate localization on — no such
consumer is wired up here.

Usage (needs a running camera + a converged splat's npz):
  ros2 run rosnav_bot gs_relocalization_check.py --ros-args \\
      -p use_sim_time:=true -p npz_path:=/home/asimov/gs_data/cafe_points_clean2.npz

Publishes std_msgs/Float32 on gs_reloc/color_error (0 = perfect color
agreement in every compared bin, higher = worse) and gs_reloc/coverage
(fraction of the downsampled grid where the splat actually had points to
compare against — low coverage means the error number isn't meaningful,
e.g. camera pointed somewhere the splat never captured).
"""
import cv_bridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformListener


def quaternion_to_matrix(x, y, z, w) -> np.ndarray:
    """Same hand-rolled quaternion -> rotation-matrix helper as
    gs_semantic_fusion.py (kept duplicated rather than shared, matching this
    repo's existing convention of small standalone gs_*.py scripts)."""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xs, ys, zs = x * s, y * s, z * s
    wx, wy, wz = w * xs, w * ys, w * zs
    xx, xy, xz = x * xs, x * ys, x * zs
    yy, yz, zz = y * ys, y * zs, z * zs
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


class GsRelocalizationCheck(Node):
    def __init__(self):
        super().__init__('gs_relocalization_check')
        self.declare_parameter('npz_path', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('image_topic', 'camera/image_raw')
        self.declare_parameter('camera_info_topic', 'camera/camera_info')
        self.declare_parameter('max_range', 8.0)
        self.declare_parameter('grid_bins', 16)  # NxN comparison grid
        self.declare_parameter('min_points_per_bin', 3)
        self.declare_parameter('check_period_sec', 2.0)

        npz_path = str(self.get_parameter('npz_path').value)
        if not npz_path:
            self.get_logger().error("Required parameter 'npz_path' not set — see gs_splat_to_pointcloud.py")
            raise SystemExit(1)

        d = np.load(npz_path)
        self._points = d['xyz'].astype(np.float64)
        self._colors = d['rgb'].astype(np.float64)  # uint8-range values, kept as float for averaging
        self.get_logger().info(f'Loaded {self._points.shape[0]} splat points from {npz_path}')

        self._map_frame = str(self.get_parameter('map_frame').value)
        self._max_range = float(self.get_parameter('max_range').value)
        self._bins = int(self.get_parameter('grid_bins').value)
        self._min_pts = int(self.get_parameter('min_points_per_bin').value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._bridge = cv_bridge.CvBridge()

        self._cam_info = None
        self._latest_image = None
        self.create_subscription(
            CameraInfo, str(self.get_parameter('camera_info_topic').value), self._camera_info_cb, 5)
        self.create_subscription(
            Image, str(self.get_parameter('image_topic').value), self._image_cb, 5)

        self._error_pub = self.create_publisher(Float32, 'gs_reloc/color_error', 5)
        self._coverage_pub = self.create_publisher(Float32, 'gs_reloc/coverage', 5)
        self.create_timer(float(self.get_parameter('check_period_sec').value), self._check)

        self.get_logger().info('gs_relocalization_check ready — waiting for camera_info + image')

    def _camera_info_cb(self, msg: CameraInfo):
        self._cam_info = msg

    def _image_cb(self, msg: Image):
        self._latest_image = msg

    def _check(self):
        if self._cam_info is None or self._latest_image is None:
            return

        camera_frame = self._cam_info.header.frame_id or self._latest_image.header.frame_id
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, camera_frame, Time.from_msg(self._latest_image.header.stamp))
        except Exception as exc:  # noqa: BLE001 — tf2 raises several distinct exception types
            self.get_logger().warn(f'TF lookup {self._map_frame}<-{camera_frame} failed: {exc}', throttle_duration_sec=5.0)
            return

        t = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
        q = tf.transform.rotation
        r = quaternion_to_matrix(q.x, q.y, q.z, q.w)

        offsets = self._points - t
        in_range = np.linalg.norm(offsets, axis=1) < self._max_range
        if not np.any(in_range):
            self.get_logger().info('No splat points within range of current pose — nothing to compare.')
            self._coverage_pub.publish(Float32(data=0.0))
            return
        cam_pts = offsets[in_range] @ r
        colors = self._colors[in_range]

        img_w, img_h = self._cam_info.width, self._cam_info.height
        fx, fy = self._cam_info.k[0], self._cam_info.k[4]
        cx, cy = self._cam_info.k[2], self._cam_info.k[5]
        in_front = cam_pts[:, 2] > 0.1
        z = cam_pts[in_front, 2]
        u = fx * cam_pts[in_front, 0] / z + cx
        v = fy * cam_pts[in_front, 1] / z + cy
        colors = colors[in_front]
        on_screen = (u >= 0) & (u < img_w) & (v >= 0) & (v < img_h)
        u, v, colors = u[on_screen], v[on_screen], colors[on_screen]
        if u.shape[0] == 0:
            self.get_logger().info('No splat points project on-screen at current pose — nothing to compare.')
            self._coverage_pub.publish(Float32(data=0.0))
            return

        live = self._bridge.imgmsg_to_cv2(self._latest_image, desired_encoding='rgb8').astype(np.float64)

        bins = self._bins
        bin_u = np.clip((u / img_w * bins).astype(int), 0, bins - 1)
        bin_v = np.clip((v / img_h * bins).astype(int), 0, bins - 1)

        errors = []
        for by in range(bins):
            for bx in range(bins):
                sel = (bin_u == bx) & (bin_v == by)
                if sel.sum() < self._min_pts:
                    continue
                synth_color = colors[sel].mean(axis=0)
                u0, u1 = int(bx / bins * img_w), int((bx + 1) / bins * img_w)
                v0, v1 = int(by / bins * img_h), int((by + 1) / bins * img_h)
                live_patch = live[v0:v1, u0:u1]
                if live_patch.size == 0:
                    continue
                live_color = live_patch.reshape(-1, 3).mean(axis=0)
                errors.append(float(np.abs(synth_color - live_color).mean()))

        coverage = len(errors) / (bins * bins)
        self._coverage_pub.publish(Float32(data=coverage))
        if not errors:
            self.get_logger().info(f'coverage=0.0 (no bin had >= {self._min_pts} splat points)')
            return
        mean_error = float(np.mean(errors))
        self._error_pub.publish(Float32(data=mean_error))
        self.get_logger().info(
            f'color_error={mean_error:.1f}/255, coverage={coverage:.2f} '
            f'({len(errors)}/{bins * bins} bins compared)')


def main(args=None):
    rclpy.init(args=args)
    node = GsRelocalizationCheck()
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
