#!/usr/bin/env python3
"""
gs_capture.py — Gaussian Splatting capture rig driver.

Feasibility-spike tool: teleports the free-floating gs_capture_rig camera
(models/gs_capture_rig/model.sdf) through a grid of poses in a Gazebo world
via the /world/<world>/set_pose service, saving each RGB frame together with
the *exact* pose this script just commanded — Gazebo ground truth, no SLAM or
odometry involved, so downstream 3D Gaussian Splatting training gets exact
camera poses for free instead of needing COLMAP structure-from-motion.

Output: <out_dir>/images/frame_%05d.png + <out_dir>/transforms.json, in
nerfstudio-data format — train with (in the nerfstudio venv):
  ns-train splatfacto --data <out_dir>

Requires Gazebo + gs_capture_rig already up:
  ros2 launch rosnav_bot gs_capture.launch.py world_name:=cafe

Usage:
  ros2 run rosnav_bot gs_capture.py --ros-args \\
      -p world_name:=cafe -p out_dir:=/home/asimov/gs_data/cafe
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

SETTLE_TIME_S = 0.35      # let the renderer catch up after a teleport before trusting a frame
FRAME_WAIT_TIMEOUT_S = 5.0
BLACK_FRAME_STDDEV = 3.0  # below this, the rig is almost certainly inside geometry — drop the frame


def _look_rotation(forward: np.ndarray) -> np.ndarray:
    """gz-sim camera sensor local frame (X forward, Y left, Z up — same
    body-frame convention as the robot's camera_link, NOT the optical frame:
    confirmed empirically, identity orientation renders looking down +X, not
    +Z) rotation matrix (columns = world-frame direction of each local axis)
    for a given forward direction, no roll, world up = +Z."""
    forward = forward / np.linalg.norm(forward)
    world_up = np.array([0.0, 0.0, 1.0])
    right = np.cross(forward, world_up)
    right = right / np.linalg.norm(right)
    up = np.cross(right, forward)
    left = -right
    return np.column_stack([forward, left, up])


def _matrix_to_quaternion(r: np.ndarray):
    """3x3 rotation matrix -> (w, x, y, z), Shepperd's method."""
    tr = np.trace(r)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (r[2, 1] - r[1, 2]) / s
        y = (r[0, 2] - r[2, 0]) / s
        z = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s
    return w, x, y, z


def generate_waypoints(centers_xy, heights, yaw_steps, pitch_deg_list):
    """(x, y, z, yaw_rad, pitch_rad) grid: positions x heights, each looking
    outward through a full yaw sweep at each pitch."""
    waypoints = []
    for (cx, cy) in centers_xy:
        for z in heights:
            for pitch_deg in pitch_deg_list:
                pitch = math.radians(pitch_deg)
                for i in range(yaw_steps):
                    yaw = 2 * math.pi * i / yaw_steps
                    waypoints.append((cx, cy, z, yaw, pitch))
    return waypoints


class GsCapture(Node):
    def __init__(self):
        super().__init__('gs_capture')

        self.declare_parameter('world_name', 'cafe')
        self.declare_parameter('model_name', 'gs_capture_rig')
        self.declare_parameter('image_topic', 'gs_capture/image_raw')
        self.declare_parameter('out_dir', os.path.expanduser('~/gs_data/cafe'))
        # Default grid tuned to cafe.world's furniture layout (tables at
        # x in [-1.5, 2.4], y in [-1.6, -9.0]) — centers sit in the aisles
        # between tables rather than on top of them.
        self.declare_parameter('centers_x', [-1.0, 1.0, 2.5])
        self.declare_parameter('centers_y', [-1.0, -4.0, -7.0, -10.0])
        self.declare_parameter('heights', [0.8, 1.6])
        self.declare_parameter('yaw_steps', 8)
        self.declare_parameter('pitch_deg', [0.0, -15.0])
        self.declare_parameter('horizontal_fov_rad', 1.5708)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)

        self._world = str(self.get_parameter('world_name').value)
        self._model = str(self.get_parameter('model_name').value)
        image_topic = str(self.get_parameter('image_topic').value)
        self._out_dir = str(self.get_parameter('out_dir').value)
        centers_x = list(self.get_parameter('centers_x').value)
        centers_y = list(self.get_parameter('centers_y').value)
        heights = list(self.get_parameter('heights').value)
        yaw_steps = int(self.get_parameter('yaw_steps').value)
        pitch_deg = list(self.get_parameter('pitch_deg').value)
        self._hfov = float(self.get_parameter('horizontal_fov_rad').value)
        self._width = int(self.get_parameter('image_width').value)
        self._height = int(self.get_parameter('image_height').value)

        self._images_dir = os.path.join(self._out_dir, 'images')
        os.makedirs(self._images_dir, exist_ok=True)

        centers_xy = [(x, y) for x in centers_x for y in centers_y]
        self._waypoints = generate_waypoints(centers_xy, heights, yaw_steps, pitch_deg)

        self._bridge = CvBridge()
        self._latest_frame = None
        self._latest_recv_time = None

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, image_topic, self._image_cb, qos)

        self.get_logger().info(
            f'gs_capture ready — {len(self._waypoints)} waypoints planned, '
            f'world={self._world}, model={self._model}, out_dir={self._out_dir}')

    def _image_cb(self, msg: Image):
        self._latest_frame = msg
        self._latest_recv_time = time.monotonic()

    def _set_pose(self, x, y, z, yaw, pitch) -> bool:
        forward = np.array([
            math.cos(yaw) * math.cos(pitch),
            math.sin(yaw) * math.cos(pitch),
            math.sin(pitch),
        ])
        r_cw = _look_rotation(forward)
        w, qx, qy, qz = _matrix_to_quaternion(r_cw)

        req = (
            f'name: "{self._model}", '
            f'position: {{x: {x}, y: {y}, z: {z}}}, '
            f'orientation: {{w: {w}, x: {qx}, y: {qy}, z: {qz}}}'
        )
        result = subprocess.run(
            ['gz', 'service', '-s', f'/world/{self._world}/set_pose',
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '2000', '--req', req],
            capture_output=True, text=True,
        )
        if result.returncode != 0:
            self.get_logger().error(f'set_pose failed: {result.stderr.strip()}')
            return False
        return True

    def _wait_for_fresh_frame(self, after_time: float):
        deadline = time.monotonic() + FRAME_WAIT_TIMEOUT_S
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._latest_recv_time is not None and self._latest_recv_time > after_time:
                return self._latest_frame
        return None

    def run(self):
        # Map from gz-sim's local camera frame (X forward, Y left, Z up) to
        # nerfstudio's OpenGL/NeRF convention (X right, Y up, Z backward):
        # new_right = -left, new_up = up, new_back = -forward.
        r_gl_remap = np.array([
            [0.0, 0.0, -1.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ])
        frames = []
        dropped = 0

        for idx, (x, y, z, yaw, pitch) in enumerate(self._waypoints):
            if not self._set_pose(x, y, z, yaw, pitch):
                dropped += 1
                continue
            command_time = time.monotonic()
            time.sleep(SETTLE_TIME_S)
            msg = self._wait_for_fresh_frame(command_time)
            if msg is None:
                self.get_logger().warning(f'[{idx}] no fresh frame — skipping')
                dropped += 1
                continue

            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if float(np.std(frame)) < BLACK_FRAME_STDDEV:
                self.get_logger().warning(
                    f'[{idx}] near-uniform frame at ({x:.1f},{y:.1f},{z:.1f}) — '
                    'rig is likely inside geometry, dropping')
                dropped += 1
                continue

            filename = f'frame_{idx:05d}.png'
            cv2.imwrite(os.path.join(self._images_dir, filename), frame)

            forward = np.array([
                math.cos(yaw) * math.cos(pitch),
                math.sin(yaw) * math.cos(pitch),
                math.sin(pitch),
            ])
            r_cw = _look_rotation(forward)
            r_gl = r_cw @ r_gl_remap
            transform = np.eye(4)
            transform[:3, :3] = r_gl
            transform[:3, 3] = [x, y, z]

            frames.append({
                'file_path': f'images/{filename}',
                'transform_matrix': transform.tolist(),
            })

            if idx % 20 == 0:
                self.get_logger().info(f'[{idx}/{len(self._waypoints)}] captured {len(frames)}, dropped {dropped}')

        fx = self._width / (2 * math.tan(self._hfov / 2))
        transforms = {
            'camera_model': 'OPENCV',
            'fl_x': fx,
            'fl_y': fx,
            'cx': self._width / 2.0,
            'cy': self._height / 2.0,
            'w': self._width,
            'h': self._height,
            'k1': 0.0, 'k2': 0.0, 'p1': 0.0, 'p2': 0.0,
            'frames': frames,
        }
        transforms_path = os.path.join(self._out_dir, 'transforms.json')
        with open(transforms_path, 'w') as f:
            json.dump(transforms, f, indent=2)

        self.get_logger().info(
            f'Done — {len(frames)} frames captured, {dropped} dropped, '
            f'wrote {transforms_path}. Train with (nerfstudio venv):\n'
            f'  ns-train splatfacto --data {self._out_dir}')


def main(args=None):
    rclpy.init(args=args)
    node = GsCapture()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
