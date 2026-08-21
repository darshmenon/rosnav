#!/usr/bin/env python3
"""
gs_semantic_fusion.py — lift yolo_detector.py's 2D image detections into real
3D map-frame regions by projecting the pre-trained Gaussian Splat point cloud
(see gs_splat_to_pointcloud.py) through the camera and keeping whatever splat
points land inside each detection's bounding box.

This is the key advantage over a plain camera-only semantic costmap layer
(see ROADMAP_CONCEPTS.md's "Semantic costmap layer from YOLO detections"):
that idea has no depth, so it must assume detections sit on a flat ground
plane. Here the splat already carries real 3D geometry from GS training, so
"this bounding box is a chair" becomes an actual 3D position/extent, not a
ground-plane guess — closer to what Splat-Nav-style GS-fused navigation does,
scoped down to just the perception/labeling step (see concepts.md §29).

Not wired into Nav2 costmaps yet — this only publishes visualization_msgs/
MarkerArray for RViz. Feeding labeled regions into a live costmap filter
would need a mutable filter_mask_server (the existing gs_keepout_filter.yaml
pipeline is a static offline mask) — left as follow-up, see concepts.md §29.

Usage (needs enable_yolo:=true so yolo_detector.py is running):
  ros2 run rosnav_bot gs_semantic_fusion.py --ros-args \\
      -p npz_path:=/home/asimov/gs_data/cafe_points_clean2.npz -p use_sim_time:=true

Then in RViz: add a MarkerArray display on gs_semantic/markers.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray

# Fixed per-class colors so the same class always renders the same hue,
# rather than a hash-based color that changes between runs.
_CLASS_COLORS = {
    'person': (0.90, 0.20, 0.20),
    'chair': (0.20, 0.60, 0.90),
    'dining table': (0.90, 0.70, 0.10),
    'couch': (0.60, 0.30, 0.90),
    'potted plant': (0.20, 0.80, 0.30),
}
_DEFAULT_COLOR = (0.70, 0.70, 0.70)


def quaternion_to_matrix(x, y, z, w) -> np.ndarray:
    """Standard quaternion -> 3x3 rotation matrix (hand-rolled, same style as
    gs_capture.py's _matrix_to_quaternion, to avoid an extra tf_transformations
    apt dependency for one conversion)."""
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


class GsSemanticFusion(Node):
    def __init__(self):
        super().__init__('gs_semantic_fusion')
        self.declare_parameter('npz_path', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('detections_topic', 'yolo/detections')
        self.declare_parameter('camera_info_topic', 'camera/camera_info')
        self.declare_parameter('marker_topic', 'gs_semantic/markers')
        self.declare_parameter('max_range', 8.0)
        self.declare_parameter('min_points', 5)

        npz_path = str(self.get_parameter('npz_path').value)
        if not npz_path:
            self.get_logger().error("Required parameter 'npz_path' not set — see gs_splat_to_pointcloud.py")
            raise SystemExit(1)

        d = np.load(npz_path)
        self._points = d['xyz'].astype(np.float64)
        self.get_logger().info(f'Loaded {self._points.shape[0]} splat points from {npz_path}')

        self._map_frame = str(self.get_parameter('map_frame').value)
        self._max_range = float(self.get_parameter('max_range').value)
        self._min_points = int(self.get_parameter('min_points').value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._cam_info = None
        self.create_subscription(
            CameraInfo, str(self.get_parameter('camera_info_topic').value), self._camera_info_cb, 5)
        self.create_subscription(
            Detection2DArray, str(self.get_parameter('detections_topic').value), self._detections_cb, 5)
        self._marker_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter('marker_topic').value), 5)

        self.get_logger().info('gs_semantic_fusion ready — waiting for camera_info + detections')

    def _camera_info_cb(self, msg: CameraInfo):
        self._cam_info = msg

    def _detections_cb(self, msg: Detection2DArray):
        if self._cam_info is None:
            return
        if not msg.detections:
            return

        camera_frame = self._cam_info.header.frame_id or msg.header.frame_id
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, camera_frame, Time.from_msg(msg.header.stamp))
        except Exception as exc:  # noqa: BLE001 — tf2 raises several distinct exception types
            self.get_logger().warn(f'TF lookup {self._map_frame}<-{camera_frame} failed: {exc}', throttle_duration_sec=5.0)
            return

        t = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
        q = tf.transform.rotation
        r = quaternion_to_matrix(q.x, q.y, q.z, q.w)

        # Coarse pre-crop by range from the camera before projecting — cheap
        # vectorized pass that keeps the per-detection bbox test fast even
        # with a several-hundred-thousand-point splat.
        offsets = self._points - t
        in_range = np.linalg.norm(offsets, axis=1) < self._max_range
        candidates_map = self._points[in_range]
        if candidates_map.shape[0] == 0:
            return

        # p_map = R @ p_cam + t  =>  p_cam = R^T @ (p_map - t); batched over
        # row vectors this is (p_map - t) @ R.
        cam_pts = offsets[in_range] @ r
        fx, fy = self._cam_info.k[0], self._cam_info.k[4]
        cx, cy = self._cam_info.k[2], self._cam_info.k[5]
        in_front = cam_pts[:, 2] > 0.1
        z = cam_pts[in_front, 2]
        u = fx * cam_pts[in_front, 0] / z + cx
        v = fy * cam_pts[in_front, 1] / z + cy
        candidates_map = candidates_map[in_front]

        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for i, det in enumerate(msg.detections):
            if not det.results:
                continue
            label = det.results[0].hypothesis.class_id
            score = det.results[0].hypothesis.score
            x1 = det.bbox.center.position.x - det.bbox.size_x / 2.0
            x2 = det.bbox.center.position.x + det.bbox.size_x / 2.0
            y1 = det.bbox.center.position.y - det.bbox.size_y / 2.0
            y2 = det.bbox.center.position.y + det.bbox.size_y / 2.0

            in_box = (u >= x1) & (u <= x2) & (v >= y1) & (v <= y2)
            hits = candidates_map[in_box]
            if hits.shape[0] < self._min_points:
                continue

            centroid = np.median(hits, axis=0)
            extent = np.percentile(np.abs(hits - centroid), 90, axis=0) * 2.0
            extent = np.clip(extent, 0.05, 3.0)
            color = _CLASS_COLORS.get(label, _DEFAULT_COLOR)

            box = Marker()
            box.header.frame_id = self._map_frame
            box.header.stamp = msg.header.stamp
            box.ns = 'gs_semantic'
            box.id = i * 2
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x, box.pose.position.y, box.pose.position.z = centroid
            box.pose.orientation.w = 1.0
            box.scale.x, box.scale.y, box.scale.z = extent
            box.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.5)
            box.lifetime.sec = 1
            markers.markers.append(box)

            text = Marker()
            text.header.frame_id = self._map_frame
            text.header.stamp = msg.header.stamp
            text.ns = 'gs_semantic'
            text.id = i * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x, text.pose.position.y = centroid[0], centroid[1]
            text.pose.position.z = centroid[2] + extent[2] / 2.0 + 0.15
            text.pose.orientation.w = 1.0
            text.scale.z = 0.2
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = f'{label} {score:.2f} ({hits.shape[0]}pt)'
            text.lifetime.sec = 1
            markers.markers.append(text)

            self.get_logger().info(
                f'{label} ({score:.2f}) -> map=({centroid[0]:.2f}, {centroid[1]:.2f}, {centroid[2]:.2f}), '
                f'{hits.shape[0]} splat points')

        self._marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = GsSemanticFusion()
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
