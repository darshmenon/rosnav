"""Offline scan-nav Gymnasium env for PPO (research local planner).

Loads a Nav2 occupancy PGM+YAML, simulates a differential-drive robot with
raycast LiDAR, and rewards progress toward a random free-space goal.

Training does NOT need Gazebo — deploy the policy with rl_policy_node.py
against live /scan + TF.
"""

from __future__ import annotations

import math
import os
from typing import Optional, Tuple

import numpy as np
import yaml

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError as exc:  # pragma: no cover
    raise ImportError('pip install gymnasium') from exc

from PIL import Image

OCCUPIED_MAX = 50   # PGM: 0=occupied … 254=free (Nav2 trinary)
FREE_MIN = 200


def load_occupancy(
    map_yaml: str,
) -> Tuple[np.ndarray, np.ndarray, float, Tuple[float, float, float]]:
    with open(map_yaml, encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    resolution = float(meta['resolution'])
    origin = tuple(float(x) for x in meta['origin'][:3])
    img_path = meta['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.abspath(map_yaml)), img_path)
    img = np.array(Image.open(img_path).convert('L'))
    # row 0 = top of image = max y in map frame
    occupied = img < OCCUPIED_MAX
    free = img > FREE_MIN
    return occupied, free, resolution, origin


class ScanNavEnv(gym.Env):
    """Discrete-time differential drive on a static occupancy grid."""

    metadata = {'render_modes': []}

    def __init__(
        self,
        map_yaml: str,
        n_beams: int = 36,
        max_range: float = 8.0,
        dt: float = 0.1,
        max_steps: int = 400,
        v_max: float = 0.5,
        w_max: float = 1.0,
        goal_tol: float = 0.35,
        seed: Optional[int] = None,
    ):
        super().__init__()
        occupied, free, resolution, origin = load_occupancy(map_yaml)
        self.occupied = occupied
        self.free = free
        self.resolution = resolution
        self.origin = origin  # (x, y, yaw)
        self.h, self.w = occupied.shape
        self.n_beams = n_beams
        self.max_range = max_range
        self.dt = dt
        self.max_steps = max_steps
        self.v_max = v_max
        self.w_max = w_max
        self.goal_tol = goal_tol

        # obs: scan[n] + goal_dx, goal_dy, goal_dyaw  (all robot-frame)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(n_beams + 3,), dtype=np.float32)
        # action: [v, w] normalized to [-1, 1]
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

        self._rng = np.random.default_rng(seed)
        self._xy = np.zeros(2, dtype=np.float64)
        self._yaw = 0.0
        self._goal = np.zeros(2, dtype=np.float64)
        self._steps = 0
        self._prev_dist = 0.0

    def world_to_px(self, x: float, y: float) -> Tuple[int, int]:
        col = int((x - self.origin[0]) / self.resolution)
        row = self.h - 1 - int((y - self.origin[1]) / self.resolution)
        return row, col

    def _in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.h and 0 <= col < self.w

    def _is_free(self, x: float, y: float) -> bool:
        row, col = self.world_to_px(x, y)
        if not self._in_bounds(row, col):
            return False
        return bool(self.free[row, col]) and not bool(self.occupied[row, col])

    def _sample_free(self) -> np.ndarray:
        free_idx = np.argwhere(self.free & ~self.occupied)
        if len(free_idx) == 0:
            raise RuntimeError('map has no free cells')
        for _ in range(200):
            r, c = free_idx[self._rng.integers(0, len(free_idx))]
            x = self.origin[0] + (c + 0.5) * self.resolution
            y = self.origin[1] + (self.h - 1 - r + 0.5) * self.resolution
            if self._is_free(x, y):
                return np.array([x, y], dtype=np.float64)
        r, c = free_idx[0]
        return np.array([
            self.origin[0] + (c + 0.5) * self.resolution,
            self.origin[1] + (self.h - 1 - r + 0.5) * self.resolution,
        ], dtype=np.float64)

    def _raycast(self) -> np.ndarray:
        ranges = np.full(self.n_beams, self.max_range, dtype=np.float32)
        for i in range(self.n_beams):
            ang = self._yaw + (-math.pi + (2 * math.pi * i) / self.n_beams)
            dx, dy = math.cos(ang), math.sin(ang)
            dist = 0.0
            step = self.resolution * 0.5
            while dist < self.max_range:
                dist += step
                x = self._xy[0] + dx * dist
                y = self._xy[1] + dy * dist
                row, col = self.world_to_px(x, y)
                if not self._in_bounds(row, col) or self.occupied[row, col]:
                    ranges[i] = float(dist)
                    break
        return ranges

    def _goal_relative(self) -> np.ndarray:
        dx = self._goal[0] - self._xy[0]
        dy = self._goal[1] - self._xy[1]
        c, s = math.cos(self._yaw), math.sin(self._yaw)
        # robot frame
        rx = c * dx + s * dy
        ry = -s * dx + c * dy
        dyaw = math.atan2(ry, rx)
        return np.array([rx, ry, dyaw], dtype=np.float32)

    def _obs(self) -> np.ndarray:
        scan = self._raycast() / self.max_range
        return np.concatenate([scan, self._goal_relative()]).astype(np.float32)

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        self._xy = self._sample_free()
        self._yaw = float(self._rng.uniform(-math.pi, math.pi))
        self._goal = self._sample_free()
        # ensure goal is not on top of start
        for _ in range(50):
            if np.linalg.norm(self._goal - self._xy) > 1.0:
                break
            self._goal = self._sample_free()
        self._steps = 0
        self._prev_dist = float(np.linalg.norm(self._goal - self._xy))
        return self._obs(), {}

    def step(self, action):
        action = np.asarray(action, dtype=np.float32).reshape(2)
        v = float(np.clip(action[0], -1, 1) * self.v_max)
        w = float(np.clip(action[1], -1, 1) * self.w_max)

        # unicycle
        self._yaw = (self._yaw + w * self.dt + math.pi) % (2 * math.pi) - math.pi
        nx = self._xy[0] + v * math.cos(self._yaw) * self.dt
        ny = self._xy[1] + v * math.sin(self._yaw) * self.dt

        collided = not self._is_free(nx, ny)
        if not collided:
            self._xy[0], self._xy[1] = nx, ny

        self._steps += 1
        dist = float(np.linalg.norm(self._goal - self._xy))
        progress = self._prev_dist - dist
        self._prev_dist = dist

        reward = progress * 2.0 - 0.01  # progress + small time cost
        if collided:
            reward -= 1.0
        terminated = dist < self.goal_tol
        if terminated:
            reward += 5.0
        truncated = self._steps >= self.max_steps

        return self._obs(), float(reward), terminated, truncated, {
            'dist': dist, 'collided': collided,
        }
