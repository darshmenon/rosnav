#!/usr/bin/env python3
"""
yolo_collect.py — save RGB frames (+ optional empty labels) for YOLO fine-tuning.

Run while slam_nav (or any camera bringup) is up:

  ros2 launch rosnav_bot slam_nav.launch.py world_name:=cafe enable_camera:=true
  ros2 run rosnav_bot yolo_collect.py --ros-args \\
      -p namespace:= -p out_dir:=$HOME/yolo_data/cafe -p max_frames:=200

Then label with any YOLO-format tool (or hand-edit labels/*.txt) and train:

  ros2 run rosnav_bot yolo_train.py --data $HOME/yolo_data/cafe/dataset.yaml

Auto-labeling (no manual annotation): if the world has known-pose static
objects (e.g. worlds/cafe.world's table1..table5), pass auto_label_config
pointing at a YAML describing them (see config/yolo_auto_label_cafe.yaml) and
labels are computed by projecting each object's 3D AABB into every captured
frame via TF + camera intrinsics, instead of being left empty:

  ros2 launch rosnav_bot slam_nav.launch.py world_name:=cafe enable_rgbd:=true explore:=true
  ros2 run rosnav_bot yolo_collect.py --ros-args \\
      -p out_dir:=$HOME/yolo_data/cafe_auto -p classes:=table \\
      -p auto_label_config:=$(pwd)/src/rosnav_bot/config/yolo_auto_label_cafe.yaml \\
      -p use_sim_time:=true
"""

from __future__ import annotations

import math
import os
import time

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def quaternion_to_matrix(x, y, z, w) -> np.ndarray:
    """Standard quaternion -> 3x3 rotation matrix (same hand-rolled helper as
    gs_semantic_fusion.py, to avoid an extra tf_transformations apt dependency
    for one conversion)."""
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


def _load_object_corners(cfg: dict) -> list[tuple[str, np.ndarray]]:
    """Each object's 8 AABB corners in world/map frame, from its (x, y, z,
    yaw) pose and the shared half-extents/z-range in the config."""
    hx, hy = float(cfg['aabb_half_x']), float(cfg['aabb_half_y'])
    z0, z1 = float(cfg['z_min']), float(cfg['z_max'])
    local_corners = np.array([
        [sx * hx, sy * hy, sz]
        for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (z0, z1)
    ])
    objects = []
    for obj in cfg['objects']:
        x, y, z, yaw = (float(v) for v in obj['pose'])
        c, s = math.cos(yaw), math.sin(yaw)
        rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        world_corners = local_corners @ rot.T + np.array([x, y, z])
        objects.append((str(obj.get('name', 'object')), world_corners))
    return objects


class YoloCollect(Node):
    def __init__(self):
        super().__init__('yolo_collect')
        self.declare_parameter('namespace', '')
        self.declare_parameter('image_topic', 'camera/image_raw')
        self.declare_parameter('out_dir', os.path.expanduser('~/yolo_data/sim'))
        self.declare_parameter('max_frames', 200)
        self.declare_parameter('min_interval_s', 0.5)
        self.declare_parameter('classes', 'chair,table,person')
        self.declare_parameter('split', 'train')  # train | val
        self.declare_parameter('auto_label_config', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('camera_info_topic', 'camera/camera_info')
        self.declare_parameter('min_visible_px', 12)

        ns = self.get_parameter('namespace').value
        pre = f'/{ns}' if ns else ''
        topic = f'{pre}/{self.get_parameter("image_topic").value}'.replace('//', '/')
        self._out = os.path.expanduser(str(self.get_parameter('out_dir').value))
        self._max = int(self.get_parameter('max_frames').value)
        self._min_interval = float(self.get_parameter('min_interval_s').value)
        self._split = str(self.get_parameter('split').value).strip() or 'train'
        self._classes = [
            c.strip() for c in str(self.get_parameter('classes').value).split(',')
            if c.strip()
        ]

        img_dir = os.path.join(self._out, 'images', self._split)
        lbl_dir = os.path.join(self._out, 'labels', self._split)
        os.makedirs(img_dir, exist_ok=True)
        os.makedirs(lbl_dir, exist_ok=True)
        self._img_dir = img_dir
        self._lbl_dir = lbl_dir
        self._write_dataset_yaml(self._classes)

        self._auto_label_objects = None
        self._auto_label_class_id = None
        self._cam_info = None
        auto_label_path = os.path.expanduser(str(self.get_parameter('auto_label_config').value))
        if auto_label_path:
            with open(auto_label_path, 'r', encoding='utf-8') as f:
                cfg = yaml.safe_load(f)
            label = str(cfg['label'])
            if label not in self._classes:
                self.get_logger().error(
                    f"auto_label_config label '{label}' not in classes={self._classes} — "
                    f"pass -p classes:=... including '{label}'")
                raise SystemExit(2)
            self._auto_label_class_id = self._classes.index(label)
            self._auto_label_objects = _load_object_corners(cfg)
            self._map_frame = str(self.get_parameter('map_frame').value)
            self._min_visible_px = int(self.get_parameter('min_visible_px').value)
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            cam_info_topic = f'{pre}/{self.get_parameter("camera_info_topic").value}'.replace('//', '/')
            self.create_subscription(CameraInfo, cam_info_topic, self._camera_info_cb, 5)
            self.get_logger().info(
                f'Auto-labeling {len(self._auto_label_objects)} "{label}" object(s) '
                f'from {auto_label_path}')

        self._bridge = CvBridge()
        self._count = 0
        self._last_save = 0.0
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, topic, self._on_image, qos)
        self.get_logger().info(
            f'Collecting up to {self._max} frames from {topic} → {self._out} '
            + ('(auto-labeled)' if self._auto_label_objects is not None
               else '(empty labels; annotate before training)'))

    def _write_dataset_yaml(self, classes: list[str]) -> None:
        path = os.path.join(self._out, 'dataset.yaml')
        lines = [
            f'path: {os.path.abspath(self._out)}',
            'train: images/train',
            'val: images/val',
            f'nc: {len(classes)}',
            'names:',
        ]
        for i, name in enumerate(classes):
            lines.append(f'  {i}: {name}')
        with open(path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(lines) + '\n')
        # Ensure val dirs exist even if only train is collected first.
        os.makedirs(os.path.join(self._out, 'images', 'val'), exist_ok=True)
        os.makedirs(os.path.join(self._out, 'labels', 'val'), exist_ok=True)

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self._cam_info = msg

    def _compute_auto_labels(self, msg: Image) -> list[str] | None:
        """Returns YOLO label lines, or None if not ready yet (camera_info or
        TF not available for this frame's timestamp — caller should retry on
        the next frame rather than save a bogus empty/background label)."""
        if self._cam_info is None:
            return None
        camera_frame = self._cam_info.header.frame_id or msg.header.frame_id
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, camera_frame, Time.from_msg(msg.header.stamp))
        except Exception as exc:  # noqa: BLE001 — tf2 raises several distinct exception types
            self.get_logger().warn(
                f'TF lookup {self._map_frame}<-{camera_frame} failed: {exc}',
                throttle_duration_sec=5.0)
            return None

        t = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
        q = tf.transform.rotation
        r = quaternion_to_matrix(q.x, q.y, q.z, q.w)
        fx, fy = self._cam_info.k[0], self._cam_info.k[4]
        cx, cy = self._cam_info.k[2], self._cam_info.k[5]
        width, height = self._cam_info.width, self._cam_info.height

        lines = []
        for _name, corners in self._auto_label_objects:
            cam_pts = (corners - t) @ r
            # Require ALL 8 corners in front of the camera plane, not just
            # some: a near-crossing object (only a few corners still ahead)
            # produces a 2D hull of just those survivors that can stretch
            # across most of the frame — a degenerate box, not the object's
            # actual silhouette. Simpler to skip these than to get them
            # partially right.
            if not np.all(cam_pts[:, 2] > 0.1):
                continue
            z = cam_pts[:, 2]
            u = fx * cam_pts[:, 0] / z + cx
            v = fy * cam_pts[:, 1] / z + cy
            x1, x2 = float(np.clip(u.min(), 0, width)), float(np.clip(u.max(), 0, width))
            y1, y2 = float(np.clip(v.min(), 0, height)), float(np.clip(v.max(), 0, height))
            if (x2 - x1) < self._min_visible_px or (y2 - y1) < self._min_visible_px:
                continue
            cxn = (x1 + x2) / 2.0 / width
            cyn = (y1 + y2) / 2.0 / height
            wn = (x2 - x1) / width
            hn = (y2 - y1) / height
            lines.append(f'{self._auto_label_class_id} {cxn:.6f} {cyn:.6f} {wn:.6f} {hn:.6f}\n')
        return lines

    def _on_image(self, msg: Image) -> None:
        if self._count >= self._max:
            return
        now = time.monotonic()
        if now - self._last_save < self._min_interval:
            return

        label_lines = None
        if self._auto_label_objects is not None:
            label_lines = self._compute_auto_labels(msg)
            if label_lines is None:
                return  # camera_info/TF not ready — retry next frame, don't burn a slot

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge failed: {exc}')
            return

        stem = f'frame_{self._count:05d}'
        img_path = os.path.join(self._img_dir, f'{stem}.jpg')
        lbl_path = os.path.join(self._lbl_dir, f'{stem}.txt')
        cv2.imwrite(img_path, frame)
        if label_lines is not None:
            with open(lbl_path, 'w', encoding='utf-8') as f:
                f.writelines(label_lines)
        elif not os.path.exists(lbl_path):
            # Empty label file = background image (valid for YOLO). Fill with
            # class cx cy w h (normalized) after annotation.
            open(lbl_path, 'a', encoding='utf-8').close()

        self._count += 1
        self._last_save = now
        if self._count % 10 == 0 or self._count == self._max:
            self.get_logger().info(f'Saved {self._count}/{self._max} → {img_path}')
        if self._count >= self._max:
            self.get_logger().info(
                f'Done. Annotate labels under {self._lbl_dir}, then: '
                f'ros2 run rosnav_bot yolo_train.py --data {self._out}/dataset.yaml')
            raise SystemExit(0)


def main():
    rclpy.init()
    node = YoloCollect()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
