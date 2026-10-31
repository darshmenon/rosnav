#!/usr/bin/env python3
"""
mpc_controller.py — nonlinear MPC local controller (see mpc_core.py) as a standalone ROS 2 node.

Follows Nav2's global plan (`/plan`, published by planner_server for the active goal) while keeping a
hard clearance from static obstacles (every `scan_stride`-th /scan beam) and from moving obstacles
(obstacle_tracker.py's JSON on /obstacle_tracker/state, constant-velocity prediction). Everything is
computed in `control_frame` (default odom), where scan, tracker and robot pose agree; the plan is
transformed there on arrival.

Research controller, same status as rl_policy_node.py: it publishes `cmd_vel_topic` directly, so run it
instead of Nav2's controller output (for example point it at /cmd_vel_nav and give the DWB/RPP
controller_server a goal-less run, or use a launch that leaves controller_server idle).

  ros2 run rosnav_bot mpc_controller.py --ros-args -p cmd_vel_topic:=/cmd_vel_nav -p v_ref:=0.3

Parameters: control_frame (odom), base_frame (base_link), cmd_vel_topic (/cmd_vel_nav), plan_topic (/plan),
            scan_topic (/scan), tracks_topic (/obstacle_tracker/state), rate_hz (10), goal_tol (0.15),
            scan_stride (6), plus every field of mpc_core.MPCConfig (horizon, dt, v_ref, r_hard, ...).
Logs one status line per second: speed command, goal distance, obstacles used, solver ok/iters/ms.
"""
from __future__ import annotations

import dataclasses
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mpc_core import MPCConfig, UnicycleMPC  # noqa: E402


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        self.declare_parameter('control_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('tracks_topic', '/obstacle_tracker/state')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('goal_tol', 0.15)
        self.declare_parameter('scan_stride', 6)
        self.declare_parameter('track_timeout', 1.0)
        cfg = MPCConfig()
        for f in dataclasses.fields(cfg):
            self.declare_parameter(f.name, getattr(cfg, f.name))
        cfg = MPCConfig(**{f.name: type(getattr(cfg, f.name))(self.get_parameter(f.name).value)
                           for f in dataclasses.fields(cfg)})
        self._mpc = UnicycleMPC(cfg)

        self._ctl = self.get_parameter('control_frame').value
        self._base = self.get_parameter('base_frame').value
        self._goal_tol = float(self.get_parameter('goal_tol').value)
        self._stride = max(1, int(self.get_parameter('scan_stride').value))
        self._track_timeout = float(self.get_parameter('track_timeout').value)

        self._tf = tf2_ros.Buffer()
        self._tfl = tf2_ros.TransformListener(self._tf, self)
        self._path = None                       # Nx2 in control_frame
        self._scan_xy = np.empty((0, 2))        # static obstacle points, control_frame
        self._tracks = np.empty((0, 4))
        self._tracks_t = 0.0
        self._v = self._w = 0.0
        self._n_solves = self._n_fail = 0
        self._ms = []

        self.create_subscription(Path, self.get_parameter('plan_topic').value, self._on_plan, 1)
        self.create_subscription(LaserScan, self.get_parameter('scan_topic').value, self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(String, self.get_parameter('tracks_topic').value, self._on_tracks, 5)
        self._pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self._tick)
        self.create_timer(1.0, self._report)
        self.get_logger().info(
            f'mpc_controller: plan {self.get_parameter("plan_topic").value} + scan + tracks -> '
            f'{self.get_parameter("cmd_vel_topic").value} in {self._ctl} '
            f'(horizon={cfg.horizon}x{cfg.dt}s, v_ref={cfg.v_ref}, r_hard={cfg.r_hard} m)')

    # ---- inputs ----------------------------------------------------------------------------
    def _pose(self, stamp=None):
        """Base pose (x, y, yaw) in control_frame, or None."""
        try:
            t = self._tf.lookup_transform(self._ctl, self._base, stamp or Time(),
                                          timeout=Duration(seconds=0.05))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        tr = t.transform
        return tr.translation.x, tr.translation.y, _yaw(tr.rotation)

    def _on_plan(self, msg: Path) -> None:
        if not msg.poses:
            self._path = None
            return
        try:
            t = self._tf.lookup_transform(self._ctl, msg.header.frame_id, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'plan frame {msg.header.frame_id}->{self._ctl} unavailable: {e}',
                                   throttle_duration_sec=5.0)
            return
        yaw = _yaw(t.transform.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        p = np.array([[ps.pose.position.x, ps.pose.position.y] for ps in msg.poses])
        self._path = np.stack([c * p[:, 0] - s * p[:, 1] + t.transform.translation.x,
                               s * p[:, 0] + c * p[:, 1] + t.transform.translation.y], axis=1)
        self._mpc.reset()
        self.get_logger().info(f'new plan: {len(p)} poses, goal ({self._path[-1][0]:.2f}, '
                               f'{self._path[-1][1]:.2f}) in {self._ctl}')

    def _on_scan(self, msg: LaserScan) -> None:
        pose = self._pose()
        if pose is None:
            return
        # beams are in the scan frame; laser_frame sits on the robot, so use base pose + beam angle
        r = np.asarray(msg.ranges[::self._stride], float)
        a = msg.angle_min + np.arange(0, len(msg.ranges), self._stride) * msg.angle_increment
        ok = np.isfinite(r) & (r > max(msg.range_min, 0.05)) & (r < min(msg.range_max, 4.0))
        x, y, th = pose
        self._scan_xy = np.stack([x + r[ok] * np.cos(th + a[ok]), y + r[ok] * np.sin(th + a[ok])], axis=1)

    def _on_tracks(self, msg: String) -> None:
        try:
            tr = json.loads(msg.data).get('moving_obstacles', [])
        except (ValueError, AttributeError):
            return
        self._tracks = np.array([[t['x'], t['y'], t['vx'], t['vy']] for t in tr], float).reshape(-1, 4)
        self._tracks_t = time.monotonic()

    # ---- control loop ----------------------------------------------------------------------
    def _tick(self) -> None:
        if self._path is None:
            return
        pose = self._pose()
        if pose is None:
            return
        gdist = math.hypot(self._path[-1][0] - pose[0], self._path[-1][1] - pose[1])
        if gdist < self._goal_tol:
            self._pub.publish(Twist())
            self._v = self._w = 0.0
            self._path = None
            self.get_logger().info(f'goal reached (dist {gdist:.2f} m) — stopping')
            return
        parts = [self._scan_xy.reshape(-1, 2) if len(self._scan_xy) else np.empty((0, 2))]
        obs = np.hstack([parts[0], np.zeros((len(parts[0]), 2))])
        if len(self._tracks) and time.monotonic() - self._tracks_t < self._track_timeout:
            obs = np.vstack([self._tracks, obs])        # solver keeps the nearest max_obstacles overall
        t0 = time.process_time()
        v, w, info = self._mpc.solve(pose, self._v, self._w, self._path, obs)
        self._ms.append((time.process_time() - t0) * 1e3)
        self._n_solves += 1
        self._n_fail += 0 if info['ok'] else 1
        self._v, self._w = v, w
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        self._pub.publish(cmd)

    def _report(self) -> None:
        if self._path is None and not self._n_solves:
            return
        ms = f'{np.median(self._ms):.0f}' if self._ms else '-'
        self.get_logger().info(
            f'v={self._v:.2f} w={self._w:.2f} static_pts={len(self._scan_xy)} '
            f'moving={len(self._tracks)} solves={self._n_solves} failed={self._n_fail} '
            f'median_solve={ms}ms', throttle_duration_sec=1.0)
        self._ms = self._ms[-50:]


def main():
    rclpy.init()
    rclpy.spin(MPCController())


if __name__ == '__main__':
    main()
