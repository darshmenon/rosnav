#!/usr/bin/env python3
"""Unit tests for rosnav_bot/rl/frontier_policies.py (pure numpy, no ROS / no env)."""
import math
import os
import sys

import numpy as np

_PKG_ROOT = os.path.join(os.path.dirname(__file__), '..')
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)

from rosnav_bot.rl import frontier_policies as fp  # noqa: E402


def feats(rows):
    """rows: (distance, size, gain, clearance, susp, hyst) -> (k, 6) float array."""
    return np.array(rows, dtype=float)


def test_nearest_picks_smallest_valid_distance():
    f = feats([[3, 1, 1, 1, 0, 0], [1, 1, 1, 1, 0, 0], [2, 1, 1, 1, 0, 0]])
    assert fp.nearest(f, np.array([True, True, True])) == 1
    assert fp.nearest(f, np.array([True, False, True])) == 2


def test_max_gain_ignores_masked_slots():
    f = feats([[1, 1, 5, 1, 0, 0], [1, 1, 9, 1, 0, 0], [1, 1, 7, 1, 0, 0]])
    assert fp.max_gain(f, np.array([True, False, True])) == 2


def test_gain_per_cost_trades_distance_against_gain():
    f = feats([[1, 1, 4, 1, 0, 0], [10, 1, 12, 1, 0, 0]])
    m = np.array([True, True])
    assert fp.make_gain_per_cost(1.0)(f, m) == 0       # 4/2 vs 12/11
    assert fp.make_gain_per_cost(0.0)(f, m) == 1       # distance ignored
    assert fp.make_gain_per_cost(2.0)(f, m) == 0


def test_live_hysteresis_bonus_breaks_close_tie():
    f = feats([[2.0, 3, 1, 2, 0, 0], [2.3, 3, 1, 2, 0, 1]])
    m = np.array([True, True])
    assert fp.make_live()(f, m) == 1                   # -0.9 penalty < +1.5 bonus
    assert fp.make_live(hyst_gain=0.0)(f, m) == 0


def test_live_suspicious_frontier_penalised_and_hard_rejected():
    # candidate 0: huge cluster squeezed to the wall (ratio 40/1.1 > hard 6)
    f = feats([[1.0, 40, 1, 0.1, 0, 0], [2.0, 4, 1, 2.0, 0, 0]])
    m = np.array([True, True])
    assert fp.make_live()(f, m) == 1
    # with only the suspicious one valid it must still be chosen (pool never empties)
    assert fp.make_live()(f, np.array([True, False])) == 0


def test_live_min_goal_distance_falls_back_when_all_too_close():
    f = feats([[0.1, 3, 1, 2, 0, 0], [0.2, 3, 1, 2, 0, 0]])
    assert fp.make_live()(f, np.array([True, True])) == 0


def test_live_failed_cooldown_excludes_goal_within_radius():
    f = feats([[1.0, 5, 1, 2, 0, 0], [3.0, 3, 1, 2, 0, 0]])
    ctx = {'goals': np.array([[1.0, 0.0], [8.0, 0.0]]), 'failed': [(1.2, 0.0, 0.75)]}
    m = np.array([True, True])
    assert fp.make_live()(f, m, None) == 0
    assert fp.make_live()(f, m, ctx) == 1
    # blacklisting everything must not empty the pool
    ctx['failed'].append((8.0, 0.0, 1.0))
    assert fp.make_live()(f, m, ctx) in (0, 1)


def test_failed_radius_clamped():
    assert fp.failed_radius(0.1) == fp.FAILED_MIN_R
    assert fp.failed_radius(100.0) == fp.FAILED_MAX_R
    assert math.isclose(fp.failed_radius(4.0), 1.4)


def test_turn_angle_cases():
    assert fp.turn_angle((1, 0), (0, 0), (2, 0)) == 0.0
    assert math.isclose(fp.turn_angle((1, 0), (0, 0), (1, 1)), math.pi / 2)
    assert math.isclose(fp.turn_angle((1, 0), (0, 0), (0, 0)), math.pi)
    assert fp.turn_angle((1, 0), None, (5, 5)) == 0.0


def test_cost_aware_avoids_backtracking_when_gain_comparable():
    # robot moved +x; candidate 0 is straight ahead, candidate 1 is behind, slightly nearer
    f = feats([[2.0, 3, 10, 2, 0, 0], [1.8, 3, 10.5, 2, 0, 0]])
    ctx = {'goals': np.array([[3.0, 0.0], [-0.8, 0.0]]),
           'pos': np.array([1.0, 0.0]), 'prev': np.array([0.0, 0.0])}
    m = np.array([True, True])
    assert fp.make_gain_per_cost(1.0)(f, m, ctx) == 1
    assert fp.make_cost_aware()(f, m, ctx) == 0


def test_cost_aware_without_ctx_degrades_to_gain_per_dist():
    f = feats([[1, 1, 4, 1, 0, 0], [10, 1, 12, 1, 0, 0]])
    m = np.array([True, True])
    assert fp.make_cost_aware()(f, m) == fp.make_gain_per_cost(1.0)(f, m)


def test_random_only_returns_valid_slots_and_is_seeded():
    f = feats([[1, 1, 1, 1, 0, 0]] * 4)
    m = np.array([False, True, False, True])
    a = [fp.make_random(3)(f, m) for _ in range(1)]
    p = fp.make_random(3)
    seq1 = [p(f, m) for _ in range(20)]
    p2 = fp.make_random(3)
    seq2 = [p2(f, m) for _ in range(20)]
    assert seq1 == seq2 and set(seq1) <= {1, 3} and a[0] in (1, 3)


def test_make_learned_missing_file_returns_none():
    assert fp.make_learned('/nonexistent/model.pt') is None
    assert 'learned' not in fp.all_policies('/nonexistent/model.pt')


def test_all_policies_return_valid_index():
    rng = np.random.default_rng(0)
    f = rng.uniform(0.5, 5, (8, 6))
    m = np.array([True] * 5 + [False] * 3)
    ctx = {'goals': rng.uniform(-5, 5, (8, 2)), 'pos': np.zeros(2), 'prev': np.array([-1., 0]),
           'failed': []}
    for name, p in fp.all_policies(None).items():
        assert m[p(f, m, ctx)], name
