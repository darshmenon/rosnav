#!/usr/bin/env python3
"""Unit tests for rosnav_bot/rl/frontier_env.py (learned frontier scorer).

Pure-logic / no-ROS: FrontierSelectEnv only needs a saved PGM+YAML map, so
these construct it directly against one of the repo's existing small maps
instead of mocking anything.
"""
import os
import sys
import time

import numpy as np
import pytest

_PKG_ROOT = os.path.join(os.path.dirname(__file__), '..')
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from rosnav_bot.rl.frontier_env import FrontierSelectEnv, N_FEATURES  # noqa: E402

_MAP = os.path.join(_PKG_ROOT, 'maps', 'map_bench_room_small.yaml')
pytestmark = pytest.mark.skipif(not os.path.isfile(_MAP), reason='test map not present')


def test_reset_never_returns_all_invalid_mask():
    # Regression test: a spawn whose sensor sweep covers its whole pocket
    # used to hand back an all-False mask straight from reset(), which
    # produces Categorical(logits=all -inf) -> NaN downstream in training.
    env = FrontierSelectEnv(_MAP, k_candidates=8, seed=1)
    for seed in range(15):
        obs, info = env.reset(seed=seed)
        assert info['action_mask'].any(), f'seed={seed} produced an all-invalid mask'
        assert obs.shape == (8 * N_FEATURES,)
        assert np.isfinite(obs).all()


def test_step_terminates_quickly_and_stays_finite():
    # Regression test for the BFS-relaxation blowup (see frontier_env.py's
    # _path_distance_map history) and the O(known-area) slowdown that made
    # per-step cost grow with the known region — both would show up here as
    # a multi-second single step.
    env = FrontierSelectEnv(_MAP, k_candidates=8, max_steps=40, seed=2)
    obs, info = env.reset(seed=2)
    coverages = []
    for _ in range(40):
        mask = info['action_mask']
        action = int(np.argmax(mask))  # first valid candidate
        t0 = time.time()
        obs, reward, terminated, truncated, info = env.step(action)
        elapsed = time.time() - t0
        assert elapsed < 2.0, f'single env.step() took {elapsed:.2f}s — perf regression'
        assert np.isfinite(obs).all()
        assert np.isfinite(reward)
        if 'coverage' in info:
            coverages.append(info['coverage'])
        if terminated or truncated:
            break
    assert coverages, 'expected at least one successful step before termination'
    assert coverages[-1] >= coverages[0]


def test_observation_and_action_space_shapes():
    env = FrontierSelectEnv(_MAP, k_candidates=8, seed=3)
    assert env.observation_space.shape == (8 * N_FEATURES,)
    assert env.action_space.n == 8
