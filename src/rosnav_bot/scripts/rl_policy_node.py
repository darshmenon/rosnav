#!/usr/bin/env python3
"""
rl_policy_node.py — run a trained ScanNav policy on live /scan + goal.

Loads either:
  - pure-PyTorch checkpoint from train_ppo.py (*.pt), or
  - stable-baselines3 zip (*.zip) if SB3 works on your machine.

Research controller: publishes /cmd_vel. Disable Nav2's controller when testing.

  python3 src/rosnav_bot/scripts/train_ppo.py --map src/rosnav_bot/maps/map_maze.yaml
  ros2 run rosnav_bot rl_policy_node.py --ros-args \\
      -p model_path:=runs/rl/ppo_scan_nav/ppo_scan_nav.pt \\
      -p goal_x:=2.0 -p goal_y:=1.0
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class _TorchPolicy:
    def __init__(self, path: str, n_beams: int):
        import torch
        from rosnav_bot.rl.policy import ActorCritic

        try:
            ckpt = torch.load(path, map_location='cpu', weights_only=False)
        except TypeError:
            ckpt = torch.load(path, map_location='cpu')
        obs_dim = int(ckpt.get('obs_dim', n_beams + 3))
        act_dim = int(ckpt.get('act_dim', 2))
        self.net = ActorCritic(obs_dim, act_dim)
        self.net.load_state_dict(ckpt['state_dict'])
        self.net.eval()
        self.n_beams = int(ckpt.get('n_beams', n_beams))
        self.max_range = float(ckpt.get('max_range', 8.0))
        self.v_max = float(ckpt.get('v_max', 0.5))
        self.w_max = float(ckpt.get('w_max', 1.0))
        self._torch = torch

    def predict(self, obs: np.ndarray) -> np.ndarray:
        with self._torch.no_grad():
            ot = self._torch.as_tensor(obs, dtype=self._torch.float32).unsqueeze(0)
            action, _, _ = self.net.act(ot, deterministic=True)
            return action.squeeze(0).cpu().numpy()


class _Sb3Policy:
    def __init__(self, path: str, n_beams: int, v_max: float, w_max: float):
        from stable_baselines3 import PPO
        self.model = PPO.load(path)
        self.n_beams = n_beams
        self.max_range = 8.0
        self.v_max = v_max
        self.w_max = w_max

    def predict(self, obs: np.ndarray) -> np.ndarray:
        action, _ = self.model.predict(obs, deterministic=True)
        return np.asarray(action, dtype=np.float32)


class RlPolicyNode(Node):
    def __init__(self):
        super().__init__('rl_policy')
        self.declare_parameter('model_path', 'runs/rl/ppo_scan_nav/ppo_scan_nav.pt')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('n_beams', 36)
        self.declare_parameter('max_range', 8.0)
        self.declare_parameter('v_max', 0.5)
        self.declare_parameter('w_max', 1.0)
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('control_hz', 10.0)
        self.declare_parameter('enabled', True)

        model_path = os.path.expanduser(str(self.get_parameter('model_path').value))
        n_beams = int(self.get_parameter('n_beams').value)
        v_max = float(self.get_parameter('v_max').value)
        w_max = float(self.get_parameter('w_max').value)

        # Allow `ros2 run` without relying solely on installed package path.
        here = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        if here not in sys.path:
            sys.path.insert(0, here)

        self._policy = None
        if not os.path.isfile(model_path):
            self.get_logger().error(f'model not found: {model_path}')
            return
        try:
            if model_path.endswith('.zip'):
                self._policy = _Sb3Policy(model_path, n_beams, v_max, w_max)
            else:
                self._policy = _TorchPolicy(model_path, n_beams)
        except Exception as exc:
            self.get_logger().error(f'failed to load policy: {exc}')
            self._policy = None
            return

        self._n_beams = self._policy.n_beams
        self._max_range = float(getattr(self._policy, 'max_range',
                                        self.get_parameter('max_range').value))
        self._v_max = float(getattr(self._policy, 'v_max', v_max))
        self._w_max = float(getattr(self._policy, 'w_max', w_max))
        self._enabled = bool(self.get_parameter('enabled').value)

        self._scan = None
        self._xy = np.zeros(2)
        self._yaw = 0.0
        self._have_odom = False
        self._goal = np.array([
            float(self.get_parameter('goal_x').value),
            float(self.get_parameter('goal_y').value),
        ], dtype=np.float64)

        self._cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self._on_scan, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._on_odom, 10)
        self.create_subscription(
            PoseStamped, self.get_parameter('goal_topic').value, self._on_goal, 10)

        hz = max(1.0, float(self.get_parameter('control_hz').value))
        self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(
            f'RL policy loaded from {model_path} (n_beams={self._n_beams})')

    def _on_scan(self, msg: LaserScan) -> None:
        self._scan = msg

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._xy[:] = (p.x, p.y)
        self._yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)
        self._have_odom = True

    def _on_goal(self, msg: PoseStamped) -> None:
        self._goal[:] = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'goal ← ({self._goal[0]:.2f}, {self._goal[1]:.2f})')

    def _downsample_scan(self, msg: LaserScan) -> np.ndarray:
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        ranges = np.nan_to_num(ranges, nan=self._max_range, posinf=self._max_range)
        ranges = np.clip(ranges, 0.0, self._max_range)
        if ranges.size == 0:
            return np.ones(self._n_beams, dtype=np.float32)
        idx = np.linspace(0, ranges.size - 1, self._n_beams).astype(np.int32)
        return ranges[idx] / self._max_range

    def _goal_relative(self) -> np.ndarray:
        dx = self._goal[0] - self._xy[0]
        dy = self._goal[1] - self._xy[1]
        c, s = math.cos(self._yaw), math.sin(self._yaw)
        rx = c * dx + s * dy
        ry = -s * dx + c * dy
        return np.array([rx, ry, math.atan2(ry, rx)], dtype=np.float32)

    def _tick(self) -> None:
        if self._policy is None or not self._enabled:
            return
        if self._scan is None or not self._have_odom:
            return
        obs = np.concatenate([
            self._downsample_scan(self._scan),
            self._goal_relative(),
        ]).astype(np.float32)
        action = self._policy.predict(obs)
        cmd = Twist()
        cmd.linear.x = float(np.clip(action[0], -1, 1) * self._v_max)
        cmd.angular.z = float(np.clip(action[1], -1, 1) * self._w_max)
        self._cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = RlPolicyNode()
    if node._policy is None:
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
