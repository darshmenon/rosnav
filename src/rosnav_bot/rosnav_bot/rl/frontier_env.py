"""Offline frontier-selection Gymnasium env for PPO (research explorer policy).

Loads a Nav2 occupancy PGM+YAML (same loader as scan_nav_env.py) and plays out
a simplified exploration episode: at each step the agent picks one of up to
K candidate frontiers, "travels" there, and reveals what a scan would see —
matching the same per-candidate feature semantics scripts/frontier_explorer.py
computes at runtime (distance, size_m, info_gain, clearance, suspicious_ratio,
hysteresis) so a policy trained here transfers directly to that node's
_score_frontier() via rl/frontier_policy.py.

Training does NOT need Gazebo — deploy with frontier_explorer.py's
frontier_scorer:=learned + learned_model_path.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Optional

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError as exc:  # pragma: no cover
    raise ImportError('pip install gymnasium') from exc

from rosnav_bot.rl.scan_nav_env import load_occupancy

N_FEATURES = 6  # distance, size_m, info_gain, clearance, suspicious_ratio, hysteresis
MIN_CLUSTER_SIZE = 3
SENSOR_RANGE_M = 3.5
INFO_GAIN_RADIUS_M = 2.0
HYSTERESIS_RADIUS_M = 2.0


class FrontierSelectEnv(gym.Env):
    """Semi-MDP: one step = pick a frontier, travel to it, reveal a scan there."""

    metadata = {'render_modes': []}

    def __init__(
        self,
        map_yaml: str,
        k_candidates: int = 8,
        max_steps: int = 60,
        target_coverage: float = 0.95,
        seed: Optional[int] = None,
    ):
        super().__init__()
        occupied, free, resolution, origin = load_occupancy(map_yaml)
        self.occupied = occupied
        self.free = free
        self.resolution = resolution
        self.origin = origin
        self.h, self.w = occupied.shape
        self.k = k_candidates
        self.max_steps = max_steps
        self.target_coverage = target_coverage
        self._total_known_cells = max(1, int((free | occupied).sum()))

        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(k_candidates * N_FEATURES,), dtype=np.float32)
        self.action_space = spaces.Discrete(k_candidates)

        self._rng = np.random.default_rng(seed)
        self._known = np.zeros_like(occupied, dtype=bool)
        self._pos_rc = (0, 0)
        self._last_goal_rc: Optional[tuple] = None
        self._steps = 0
        self._candidates: list[dict] = []

    def world_xy(self, row: int, col: int) -> tuple:
        x = self.origin[0] + (col + 0.5) * self.resolution
        y = self.origin[1] + (self.h - 1 - row + 0.5) * self.resolution
        return x, y

    def _reveal(self, row: int, col: int, radius_m: float) -> int:
        r_cells = max(1, math.ceil(radius_m / self.resolution))
        r0, r1 = max(0, row - r_cells), min(self.h, row + r_cells + 1)
        c0, c1 = max(0, col - r_cells), min(self.w, col + r_cells + 1)
        yy, xx = np.mgrid[r0:r1, c0:c1]
        disk = (yy - row) ** 2 + (xx - col) ** 2 <= r_cells ** 2
        before = int(self._known[r0:r1, c0:c1].sum())
        self._known[r0:r1, c0:c1] |= disk
        after = int(self._known[r0:r1, c0:c1].sum())
        return after - before

    def _known_free(self) -> np.ndarray:
        return self._known & self.free & ~self.occupied

    def _known_occupied(self) -> np.ndarray:
        return self._known & self.occupied

    def _frontier_mask(self) -> np.ndarray:
        kfree = self._known_free()
        unknown = ~self._known
        adj = np.zeros_like(unknown, dtype=bool)
        adj[:-1, :] |= unknown[1:, :]
        adj[1:, :] |= unknown[:-1, :]
        adj[:, :-1] |= unknown[:, 1:]
        adj[:, 1:] |= unknown[:, :-1]
        return kfree & adj

    def _clearance_m(self, row: int, col: int, max_r_m: float = 2.0) -> float:
        max_r_cells = max(1, math.ceil(max_r_m / self.resolution))
        for r in range(1, max_r_cells + 1):
            r0, r1 = max(0, row - r), min(self.h, row + r + 1)
            c0, c1 = max(0, col - r), min(self.w, col + r + 1)
            if self.occupied[r0:r1, c0:c1].any():
                return (r - 1) * self.resolution
        return max_r_m

    def _find_candidates(self) -> list[dict]:
        mask = self._frontier_mask()
        if not mask.any():
            return []
        ry, rx = self._pos_rc
        visited = np.zeros_like(mask, dtype=bool)
        clusters = []
        ys, xs = np.nonzero(mask)
        for sy, sx in zip(ys.tolist(), xs.tolist()):
            if visited[sy, sx]:
                continue
            q = deque([(sy, sx)])
            visited[sy, sx] = True
            cluster = []
            while q:
                y, x = q.popleft()
                cluster.append((y, x))
                for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                    if 0 <= ny < self.h and 0 <= nx < self.w and mask[ny, nx] and not visited[ny, nx]:
                        visited[ny, nx] = True
                        q.append((ny, nx))
            if len(cluster) < MIN_CLUSTER_SIZE:
                continue
            cy, cx = cluster[len(cluster) // 2]
            # Straight-line distance, not true path distance — a deliberate
            # cheap approximation for the offline training signal (avoids
            # the O(known-area) BFS that made this the per-step bottleneck
            # as the known region grows; frontier_explorer.py's live
            # deployment still uses real path distance via Nav2/costmap).
            d = math.hypot(cy - ry, cx - rx) * self.resolution
            if d <= 1e-6:
                continue
            size_m = len(cluster) * self.resolution
            clearance = self._clearance_m(cy, cx)
            unknown = ~self._known
            r_cells = max(1, math.ceil(INFO_GAIN_RADIUS_M / self.resolution))
            r0, r1 = max(0, cy - r_cells), min(self.h, cy + r_cells + 1)
            c0, c1 = max(0, cx - r_cells), min(self.w, cx + r_cells + 1)
            info_gain = float(unknown[r0:r1, c0:c1].sum()) * self.resolution ** 2
            suspicious_ratio = size_m / max(clearance + 0.5, 1e-3)
            hysteresis = 0.0
            if self._last_goal_rc is not None:
                lgy, lgx = self._last_goal_rc
                if math.hypot(cy - lgy, cx - lgx) * self.resolution <= HYSTERESIS_RADIUS_M:
                    hysteresis = 1.0
            clusters.append({
                'goal_rc': (cy, cx),
                'distance': d,
                'size_m': size_m,
                'info_gain': info_gain,
                'clearance': clearance,
                'suspicious_ratio': suspicious_ratio,
                'hysteresis': hysteresis,
            })
        clusters.sort(key=lambda c: c['distance'])
        return clusters[: self.k]

    def _obs_and_mask(self):
        obs = np.zeros((self.k, N_FEATURES), dtype=np.float32)
        valid = np.zeros(self.k, dtype=bool)
        for i, c in enumerate(self._candidates):
            obs[i] = (c['distance'], c['size_m'], c['info_gain'],
                      c['clearance'], c['suspicious_ratio'], c['hysteresis'])
            valid[i] = True
        return obs.reshape(-1), valid

    def _sample_free_rc(self):
        free_idx = np.argwhere(self.free & ~self.occupied)
        if len(free_idx) == 0:
            raise RuntimeError('map has no free cells')
        r, c = free_idx[self._rng.integers(0, len(free_idx))]
        return int(r), int(c)

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        self._known[:] = False
        self._pos_rc = self._sample_free_rc()
        self._reveal(*self._pos_rc, SENSOR_RANGE_M)
        self._last_goal_rc = None
        self._steps = 0
        self._candidates = self._find_candidates()
        # A spawn in a small fully-enclosed pocket can have its whole
        # reachable area covered by one sensor sweep, leaving zero frontiers
        # right at reset — an all-invalid action mask that would reach the
        # policy's Categorical(logits=all -inf) and produce NaN. Retry a
        # handful of spawns rather than ever handing back an empty mask.
        retries = 0
        while not self._candidates and retries < 20:
            self._known[:] = False
            self._pos_rc = self._sample_free_rc()
            self._reveal(*self._pos_rc, SENSOR_RANGE_M)
            self._candidates = self._find_candidates()
            retries += 1
        obs, valid = self._obs_and_mask()
        return obs, {'action_mask': valid}

    def step(self, action: int):
        self._steps += 1
        valid_candidates = self._candidates
        if not valid_candidates or action >= len(valid_candidates):
            # No legal candidate (padding slot or exhausted map) — end episode.
            obs, valid = self._obs_and_mask()
            return obs, -1.0, True, False, {'action_mask': valid}

        chosen = valid_candidates[action]
        gy, gx = chosen['goal_rc']
        traveled = chosen['distance']
        gained = self._reveal(gy, gx, SENSOR_RANGE_M)
        self._pos_rc = (gy, gx)
        self._last_goal_rc = (gy, gx)

        coverage_gain_m2 = gained * self.resolution ** 2
        reward = coverage_gain_m2 - 0.05 * traveled - 0.02

        self._candidates = self._find_candidates()
        coverage = float(self._known.sum()) / self._total_known_cells
        terminated = coverage >= self.target_coverage or not self._candidates
        if terminated and coverage >= self.target_coverage:
            reward += 5.0
        truncated = self._steps >= self.max_steps

        obs, valid = self._obs_and_mask()
        return obs, float(reward), terminated, truncated, {
            'action_mask': valid, 'coverage': coverage,
        }
