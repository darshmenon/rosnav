#!/usr/bin/env python3
"""
yolo_detector.py — Pluggable YOLO object detection on the robot's RGB camera.

Optional add-on node: subscribes to a robot's `camera/image_raw`, runs
Ultralytics YOLO inference at a fixed, throttled rate (decoupled from the
camera's publish rate so inference lag never backs up the image queue), and
publishes vision_msgs/Detection2DArray plus (optionally) an annotated debug
image. Nothing else in the stack depends on this node — nav2, SLAM, and
frontier exploration all run identically whether it's on or off.

Requires the `ultralytics` pip package (not a rosdep — install separately):
  pip install ultralytics
If it's missing, the node logs how to install it and exits rather than
spinning uselessly.

Usage
─────
  ros2 run rosnav_bot yolo_detector.py --ros-args -p namespace:=robot1
  ros2 run rosnav_bot yolo_detector.py --ros-args \\
      -p model_path:=yolov8n.pt -p confidence:=0.6 -p max_rate_hz:=5.0
"""

from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2D, Detection2DArray, ObjectHypothesisWithPose,
)

LOG_PERIOD = 5.0        # summary log cadence, seconds
NO_FRAME_WARN_PERIOD = 10.0  # warn if no camera frames arrive for this long


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameter('namespace', '')
        self.declare_parameter('image_topic', 'camera/image_raw')
        self.declare_parameter('detections_topic', 'yolo/detections')
        self.declare_parameter('annotated_topic', 'yolo/image_annotated')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('iou', 0.45)
        self.declare_parameter('device', 'auto')
        self.declare_parameter('max_rate_hz', 5.0)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('classes', '')  # comma-separated allow-list, empty = all

        ns = self.get_parameter('namespace').value
        pre = f'/{ns}' if ns else ''
        image_topic = f'{pre}/{self.get_parameter("image_topic").value}'
        detections_topic = f'{pre}/{self.get_parameter("detections_topic").value}'
        annotated_topic = f'{pre}/{self.get_parameter("annotated_topic").value}'

        self._confidence = float(self.get_parameter('confidence').value)
        self._iou = float(self.get_parameter('iou').value)
        self._max_rate_hz = max(0.1, float(self.get_parameter('max_rate_hz').value))
        self._publish_annotated = bool(self.get_parameter('publish_annotated').value)
        classes_raw = str(self.get_parameter('classes').value or '').strip()
        self._class_allowlist = (
            {c.strip().lower() for c in classes_raw.split(',') if c.strip()}
            if classes_raw else None
        )

        model_path = str(self.get_parameter('model_path').value)
        device_arg = str(self.get_parameter('device').value)

        try:
            from ultralytics import YOLO
        except ImportError:
            self.get_logger().error(
                "ultralytics is not installed — YOLO detection can't run. "
                "Install it with: pip install ultralytics")
            self._ready = False
            self._model = None
            return

        device = self._resolve_device(device_arg)
        self.get_logger().info(
            f'Loading YOLO model "{model_path}" on device={device} '
            f'(confidence={self._confidence}, iou={self._iou}) …')
        try:
            self._model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model "{model_path}": {e}')
            self._ready = False
            self._model = None
            return

        self._device = device
        self._ready = True
        self._bridge = CvBridge()
        self._latest_frame = None
        self._latest_stamp = None
        self._last_processed_stamp = None
        self._last_summary_log = None
        self._last_frame_recv = None
        self._detections_seen_total = 0

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, image_topic, self._image_cb, qos)
        self._det_pub = self.create_publisher(Detection2DArray, detections_topic, 10)
        self._annotated_pub = (
            self.create_publisher(Image, annotated_topic, 1)
            if self._publish_annotated else None
        )

        allow_str = ', '.join(sorted(self._class_allowlist)) if self._class_allowlist else 'all'
        self.get_logger().info(
            f'YOLO detector ready — image={image_topic} → detections={detections_topic}'
            + (f', annotated={annotated_topic}' if self._publish_annotated else '')
            + f' | rate={self._max_rate_hz}Hz classes=[{allow_str}]')

        self.create_timer(1.0 / self._max_rate_hz, self._infer_tick)
        self.create_timer(2.0, self._check_no_frames)

    @staticmethod
    def _resolve_device(device_arg: str) -> str:
        if device_arg and device_arg != 'auto':
            return device_arg
        try:
            import torch
            return 'cuda:0' if torch.cuda.is_available() else 'cpu'
        except ImportError:
            return 'cpu'

    def _image_cb(self, msg: Image):
        self._latest_frame = msg
        self._latest_stamp = time.monotonic()
        self._last_frame_recv = self._latest_stamp

    def _check_no_frames(self):
        if not self._ready:
            return
        if self._last_frame_recv is None:
            return
        if time.monotonic() - self._last_frame_recv > NO_FRAME_WARN_PERIOD:
            self.get_logger().warning(
                f'No camera frames received in over {NO_FRAME_WARN_PERIOD:.0f}s — '
                'is the camera bridge running?')

    def _infer_tick(self):
        if not self._ready or self._latest_frame is None:
            return
        if self._latest_stamp == self._last_processed_stamp:
            return  # no new frame since last inference — skip rather than reprocess
        self._last_processed_stamp = self._latest_stamp
        msg = self._latest_frame

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        try:
            results = self._model.predict(
                frame, conf=self._confidence, iou=self._iou,
                device=self._device, verbose=False)
        except Exception as e:
            self.get_logger().error(f'YOLO inference failed: {e}')
            return

        result = results[0]
        names = result.names
        det_array = Detection2DArray()
        det_array.header = msg.header

        counts = {}
        for box in result.boxes:
            cls_id = int(box.cls[0])
            label = names.get(cls_id, str(cls_id)) if isinstance(names, dict) else str(cls_id)
            if self._class_allowlist is not None and label.lower() not in self._class_allowlist:
                continue

            score = float(box.conf[0])
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]

            det = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x = (x1 + x2) / 2.0
            det.bbox.center.position.y = (y1 + y2) / 2.0
            det.bbox.size_x = x2 - x1
            det.bbox.size_y = y2 - y1

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = label
            hyp.hypothesis.score = score
            det.results.append(hyp)
            det_array.detections.append(det)

            counts[label] = counts.get(label, 0) + 1

        self._det_pub.publish(det_array)
        self._detections_seen_total += len(det_array.detections)

        if self._annotated_pub is not None:
            annotated = result.plot()
            out_msg = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            out_msg.header = msg.header
            self._annotated_pub.publish(out_msg)

        self._log_summary(counts)

    def _log_summary(self, counts: dict):
        now = time.monotonic()
        if self._last_summary_log is not None and now - self._last_summary_log < LOG_PERIOD:
            return
        self._last_summary_log = now
        if counts:
            summary = ', '.join(f'{label}×{n}' for label, n in sorted(counts.items()))
            self.get_logger().info(f'Detections: {summary} (total seen so far: {self._detections_seen_total})')
        else:
            self.get_logger().info('No detections in latest frame.')


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    if not getattr(node, '_ready', False):
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
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
