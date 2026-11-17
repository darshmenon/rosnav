#!/usr/bin/env python3
"""
ped_detector.py — pedestrian detections for the social-nav stack, from a camera OR the 2D lidar.

  camera mode : YOLO person boxes on /camera/image_raw + depth /camera/depth/image_raw + CameraInfo
                (RGB-D worlds: gz_bridge_rgbd.yaml / gz_bridge_3d_rgbd.yaml, urdf/rgbd_camera.xacro)
  lidar mode  : person-sized clusters in /scan (the default diff_drive has no camera)
  auto        : camera if image+depth+camera_info arrive within `auto_wait` s, else lidar

Both go through ped_detector_core (back-projection / clustering, unit-tested offline), then ped_tracker_core's
Kalman + Hungarian tracker. Output topics:
  /ped_detector/detections    std_msgs/String JSON {t, frame, detections:[{x,y,conf,src,...}]}  (raw, map frame)
  /obstacle_tracker/state     std_msgs/String JSON, SAME format as obstacle_tracker.py  (publish_state:=true)
                              -> social_nav_node.py / dynamic_obstacle_predictor.py consume it unchanged
  /ped_detector/markers       MarkerArray (RViz: detections as red spheres, tracks as cyan cylinders + ids)

Run it INSTEAD of obstacle_tracker.py (same state topic), or with publish_state:=false next to it.

  ros2 run rosnav_bot ped_detector.py --ros-args -p mode:=auto -p use_sim_time:=true
  ros2 run rosnav_bot ped_detector.py --ros-args -p mode:=lidar -p map:=<map yaml>   # drop wall-corner clusters
"""
import json
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ped_detector_core as dc  # noqa: E402
from ped_tracker_core import PedTracker  # noqa: E402

YOLO_CANDIDATES = ['~/rosnav/yolov8n.pt', '~/yolov8n.pt', 'yolov8n.pt']


def image_to_array(msg):
    """sensor_msgs/Image -> numpy without cv_bridge. rgb8/bgr8 -> HxWx3 uint8; 32FC1 -> metres; 16UC1 -> metres (mm/1000)."""
    enc = msg.encoding
    buf = np.frombuffer(msg.data, np.uint8)
    if enc in ('rgb8', 'bgr8'):
        img = buf.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        return img if enc == 'rgb8' else img[..., ::-1]
    if enc == '32FC1':
        return np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width).copy()
    if enc == '16UC1':
        return np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.width).astype(np.float32) / 1000.0
    raise ValueError(f'unsupported image encoding {enc}')


class PedDetector(Node):
    def __init__(self):
        super().__init__('ped_detector')
        for k, v in dict(mode='auto', auto_wait=6.0, rate_hz=5.0, map_frame='map', min_conf=0.4, body_offset=0.2,
                         yolo_model='', device='cpu', imgsz=480, publish_state=True,
                         state_topic='/obstacle_tracker/state', detections_topic='/ped_detector/detections',
                         image_topic='/camera/image_raw', depth_topic='/camera/depth/image_raw',
                         info_topic='/camera/camera_info', scan_topic='/scan', map='', wall_clearance=0.30,
                         lidar_max_range=6.0).items():
            self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value
        self.p = {k: g(k) for k in ('mode', 'auto_wait', 'rate_hz', 'map_frame', 'min_conf', 'body_offset', 'device',
                                    'imgsz', 'publish_state', 'wall_clearance', 'lidar_max_range')}
        self.rgb = self.depth = self.info = self.scan = None
        self.t0 = time.monotonic()
        self.chosen = None
        self.model = None
        self.static_fn = None
        if g('map'):
            import social_force_core as sf
            w = sf.MapWorld(g('map'))
            wc = float(g('wall_clearance'))
            self.static_fn = lambda x, y: w.clearance(x, y) < wc
            self.get_logger().info(f'lidar static filter: drop clusters within {wc} m of walls in {g("map")}')
        import tf2_ros
        self.buf = tf2_ros.Buffer()
        self.tl = tf2_ros.TransformListener(self.buf, self)
        self.create_subscription(Image, g('image_topic'), lambda m: setattr(self, 'rgb', m), 2)
        self.create_subscription(Image, g('depth_topic'), lambda m: setattr(self, 'depth', m), 2)
        self.create_subscription(CameraInfo, g('info_topic'), lambda m: setattr(self, 'info', m), 2)
        self.create_subscription(LaserScan, g('scan_topic'), lambda m: setattr(self, 'scan', m), 2)
        self.pub_det = self.create_publisher(String, g('detections_topic'), 10)
        self.pub_state = self.create_publisher(String, g('state_topic'), 10) if g('publish_state') else None
        self.pub_mark = self.create_publisher(MarkerArray, '/ped_detector/markers', 10)
        self.tracker = PedTracker(dt=1.0 / float(g('rate_hz')), meas_var=0.06, max_age=6, min_hits=3)
        self.n_frames = self.n_dets = 0
        self.create_timer(1.0 / float(g('rate_hz')), self._tick)
        self.create_timer(10.0, self._status)
        self.get_logger().info(f'ped_detector up: mode={self.p["mode"]} rate={self.p["rate_hz"]} Hz, '
                               f'state -> {g("state_topic") if self.pub_state else "(off)"}')

    # ---- source selection
    def _have_camera(self):
        return self.rgb is not None and self.depth is not None and self.info is not None

    def _pick(self):
        if self.chosen:
            return self.chosen
        m = self.p['mode']
        if m == 'camera' or (m == 'auto' and self._have_camera()):
            self.chosen = 'camera'
        elif m == 'lidar' or (m == 'auto' and time.monotonic() - self.t0 > self.p['auto_wait']):
            self.chosen = 'lidar'
        if self.chosen:
            self.get_logger().info(f'detector source = {self.chosen}')
            if self.chosen == 'camera':
                self._load_yolo()
        return self.chosen

    def _load_yolo(self):
        from ultralytics import YOLO
        path = self.get_parameter('yolo_model').value
        cands = [path] if path else [os.path.expanduser(c) for c in YOLO_CANDIDATES]
        for c in cands:
            if c and os.path.exists(c):
                self.model = YOLO(c)
                self.get_logger().info(f'YOLO weights: {c} (device={self.p["device"]})')
                return
        raise RuntimeError(f'no YOLO weights found in {cands}; pass -p yolo_model:=/path/to/yolov8n.pt '
                           f'(or use mode:=lidar)')

    def _T(self, frame):
        t = self.buf.lookup_transform(self.p['map_frame'], frame, rclpy.time.Time()).transform
        return dc.make_transform(t.translation.x, t.translation.y, t.translation.z,
                                 t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)

    # ---- detection
    def _camera(self):
        rgb, depth, info = self.rgb, self.depth, self.info
        try:
            T = self._T(info.header.frame_id or depth.header.frame_id)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'no TF {self.p["map_frame"]}<-{info.header.frame_id}: {e}', throttle_duration_sec=5)
            return None
        img = np.ascontiguousarray(image_to_array(rgb))
        res = self.model(img, classes=[0], conf=self.p['min_conf'], imgsz=int(self.p['imgsz']), verbose=False,
                         device=self.p['device'])[0]
        boxes = [(*b[:4], c) for b, c in zip(res.boxes.xyxy.cpu().numpy().tolist(), res.boxes.conf.cpu().numpy().tolist())]
        return dc.detections_from_boxes(boxes, image_to_array(depth), dc.intrinsics(info.k), T,
                                        body_offset=self.p['body_offset'], min_conf=self.p['min_conf'])

    def _lidar(self):
        sc = self.scan
        if sc is None:
            return None
        try:
            T = self._T(sc.header.frame_id)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'no TF {self.p["map_frame"]}<-{sc.header.frame_id}: {e}', throttle_duration_sec=5)
            return None
        return dc.lidar_clusters(sc.ranges, sc.angle_min, sc.angle_increment, T, max_range=self.p['lidar_max_range'],
                                 static_fn=self.static_fn)

    def _tick(self):
        src = self._pick()
        if src is None:
            return
        t0 = time.perf_counter()
        dets = self._camera() if src == 'camera' else self._lidar()
        if dets is None:
            return
        self.n_frames += 1
        self.n_dets += len(dets)
        now = self.get_clock().now()
        tracks = self.tracker.update([(d['x'], d['y']) for d in dets])
        self.pub_det.publish(String(data=dc.to_detections_json(dets, now.nanoseconds / 1e9, self.p['map_frame'])))
        if self.pub_state is not None:
            self.pub_state.publish(String(data=dc.to_state_json(tracks)))
        self._markers(dets, tracks, now.to_msg())
        self.last_ms = (time.perf_counter() - t0) * 1e3

    def _markers(self, dets, tracks, stamp):
        ma = MarkerArray()
        clr = Marker(); clr.action = Marker.DELETEALL; ma.markers.append(clr)
        for i, d in enumerate(dets):
            m = Marker(); m.header.frame_id, m.header.stamp = self.p['map_frame'], stamp
            m.ns, m.id, m.type = 'det', i, Marker.SPHERE
            m.pose.position.x, m.pose.position.y, m.pose.position.z = d['x'], d['y'], 0.9
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.25
            m.color.r, m.color.a = 1.0, 0.9
            ma.markers.append(m)
        for t in tracks:
            m = Marker(); m.header.frame_id, m.header.stamp = self.p['map_frame'], stamp
            m.ns, m.id, m.type = 'trk', int(t['id']), Marker.TEXT_VIEW_FACING
            m.pose.position.x, m.pose.position.y, m.pose.position.z = t['x'], t['y'], 1.5
            m.pose.orientation.w = 1.0
            m.scale.z = 0.3
            m.color.r = m.color.g = m.color.b = m.color.a = 1.0
            m.text = f'#{t["id"]}'
            ma.markers.append(m)
        self.pub_mark.publish(ma)

    def _status(self):
        self.get_logger().info(
            f'source={self.chosen or "waiting"} frames={self.n_frames} detections={self.n_dets} '
            f'tracks={len([t for t in self.tracker.tracks if t.hits >= 3 and t.miss == 0])} '
            f'last_ms={getattr(self, "last_ms", 0.0):.1f} '
            f'inputs: rgb={self.rgb is not None} depth={self.depth is not None} info={self.info is not None} '
            f'scan={self.scan is not None}')


def main():
    rclpy.init()
    n = PedDetector()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
