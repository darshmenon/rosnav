#!/usr/bin/env python3
"""Record a Gaussian-Splatting training set from the robot's own camera while it drives.

Unlike gs_capture.py (teleported camera rig), this runs alongside the normal robot stack
(slam_nav.launch.py enable_camera:=true) and saves a frame whenever the robot has moved or
turned enough. Poses are Gazebo ground truth (/ground_truth, world frame) composed with the
static base_link -> camera_optical_frame TF, so the splat lands in true world metres like
gs_capture.py output (see concepts.md §29b). Output is nerfstudio-data format:
  <out_dir>/images/frame_%05d.png + <out_dir>/transforms.json

Usage:
  ros2 run rosnav_bot gs_record_robot.py --ros-args -p use_sim_time:=true -p out_dir:=$HOME/gs_data/cafe_robot
  # drive/explore the robot, Ctrl-C when done (transforms.json is written on shutdown and every 25 frames)
Train: ns-train splatfacto --data <out_dir> --pipeline.model.random-init True nerfstudio-data
"""
import json
import math
import os

import cv2
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

# optical (X right, Y down, Z fwd) -> nerfstudio OpenGL (X right, Y up, Z back)
_OPT_TO_GL = np.diag([1.0, -1.0, -1.0])


def _quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class RobotGsRecorder(Node):
    def __init__(self):
        super().__init__('gs_record_robot')
        self.declare_parameter('out_dir', os.path.expanduser('~/gs_data/robot'))
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('info_topic', '/camera/camera_info')
        self.declare_parameter('gt_topic', '/ground_truth')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('min_move_m', 0.15)
        self.declare_parameter('min_turn_deg', 10.0)
        self.declare_parameter('max_speed_mps', 0.25)   # skip motion-blurred frames
        self.declare_parameter('blur_min_laplacian_var', 15.0)
        self.declare_parameter('max_frames', 2000)

        self._out = os.path.expanduser(self.get_parameter('out_dir').value)
        self._img_dir = os.path.join(self._out, 'images')
        os.makedirs(self._img_dir, exist_ok=True)
        self._min_move = float(self.get_parameter('min_move_m').value)
        self._min_turn = math.radians(float(self.get_parameter('min_turn_deg').value))
        self._max_speed = float(self.get_parameter('max_speed_mps').value)
        self._min_lap = float(self.get_parameter('blur_min_laplacian_var').value)
        self._max_frames = int(self.get_parameter('max_frames').value)
        self._base = self.get_parameter('base_frame').value
        self._cam = self.get_parameter('camera_frame').value

        self._bridge = CvBridge()
        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self)
        self._gt = None
        self._info = None
        self._frames = []
        self._last = None  # (xyz, yaw) of last saved camera pose
        self._skipped = {'close': 0, 'fast': 0, 'blur': 0, 'notf': 0}

        self.create_subscription(Odometry, self.get_parameter('gt_topic').value, self._gt_cb, 10)
        self.create_subscription(CameraInfo, self.get_parameter('info_topic').value, self._info_cb, 1)
        self.create_subscription(Image, self.get_parameter('image_topic').value, self._image_cb, 2)
        self.create_timer(10.0, self._status)
        self.get_logger().info(
            f'Recording robot camera -> {self._out} (move>={self._min_move}m or turn>={math.degrees(self._min_turn):.0f}deg, '
            f'speed<={self._max_speed}m/s). Waiting for {self.get_parameter("gt_topic").value}, camera_info and images...')

    def _gt_cb(self, msg):
        self._gt = msg

    def _info_cb(self, msg):
        self._info = msg

    def _status(self):
        self.get_logger().info(
            f'{len(self._frames)} frames saved; skipped {self._skipped} '
            f'(gt={"ok" if self._gt else "MISSING"}, info={"ok" if self._info else "MISSING"})')

    def _image_cb(self, msg):
        if self._gt is None or self._info is None or len(self._frames) >= self._max_frames:
            return
        v = self._gt.twist.twist.linear
        if math.hypot(v.x, v.y) > self._max_speed:
            self._skipped['fast'] += 1
            return
        try:
            tf = self._tf.lookup_transform(self._base, self._cam, rclpy.time.Time())
        except Exception:
            self._skipped['notf'] += 1
            return

        p = self._gt.pose.pose
        t_wb = np.eye(4)
        t_wb[:3, :3] = _quat_to_rot(p.orientation)
        t_wb[:3, 3] = [p.position.x, p.position.y, p.position.z]
        t_bc = np.eye(4)
        t_bc[:3, :3] = _quat_to_rot(tf.transform.rotation)
        t_bc[:3, 3] = [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]
        t_wc = t_wb @ t_bc

        fwd = t_wc[:3, 2]  # optical +Z in world
        yaw = math.atan2(fwd[1], fwd[0])
        if self._last is not None:
            dpos = float(np.linalg.norm(t_wc[:3, 3] - self._last[0]))
            dyaw = abs((yaw - self._last[1] + math.pi) % (2 * math.pi) - math.pi)
            if dpos < self._min_move and dyaw < self._min_turn:
                self._skipped['close'] += 1
                return

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if cv2.Laplacian(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var() < self._min_lap:
            self._skipped['blur'] += 1
            return

        name = f'frame_{len(self._frames):05d}.png'
        cv2.imwrite(os.path.join(self._img_dir, name), frame)
        t_gl = t_wc.copy()
        t_gl[:3, :3] = t_wc[:3, :3] @ _OPT_TO_GL
        self._frames.append({'file_path': f'images/{name}', 'transform_matrix': t_gl.tolist()})
        self._last = (t_wc[:3, 3].copy(), yaw)
        if len(self._frames) % 25 == 0:
            self.write_transforms()

    def write_transforms(self):
        if self._info is None:
            return
        k = self._info.k
        data = {
            'camera_model': 'OPENCV',
            'fl_x': k[0], 'fl_y': k[4], 'cx': k[2], 'cy': k[5],
            'w': self._info.width, 'h': self._info.height,
            'k1': 0.0, 'k2': 0.0, 'p1': 0.0, 'p2': 0.0,
            'frames': self._frames,
        }
        path = os.path.join(self._out, 'transforms.json')
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)
        self.get_logger().info(f'wrote {path} ({len(self._frames)} frames)')


def main(args=None):
    rclpy.init(args=args)
    node = RobotGsRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_transforms()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
