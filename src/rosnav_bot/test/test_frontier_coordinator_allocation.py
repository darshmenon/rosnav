#!/usr/bin/env python3
"""Unit tests for FrontierCoordinator._hungarian_plan (no ROS graph needed)."""
import importlib.util
import os

_SCRIPT = os.path.join(
    os.path.dirname(__file__), '..', 'scripts', 'frontier_coordinator.py')
_spec = importlib.util.spec_from_file_location('frontier_coordinator', _SCRIPT)
fc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fc)


class _Log:
    def info(self, *a, **k):
        pass


def make_bare(pick_table, assign_r=1.0):
    """pick_table: {robot: [(score, (x, y)), ...]} returned by a stubbed _pick."""
    obj = fc.FrontierCoordinator.__new__(fc.FrontierCoordinator)
    obj.get_logger = _Log
    obj._assign_r = assign_r

    def _pick(frontiers, pos, ns, collect=None):
        for score, xy in pick_table[ns]:
            collect.append((score, xy, {'score': score, 'distance': 1.0, 'size_m': 1.0,
                                        'info_gain': 0.0, 'clearance': 0.0,
                                        'suspicious_ratio': 0.0}))
        return None, {'n_total': 0}
    obj._pick = _pick
    return obj


def test_hungarian_beats_greedy_order_bias():
    # Greedy in order would give r1 frontier A (10) leaving r2 only B (0):
    # total 10. Optimal: r1->B (8), r2->A (9): total 17.
    A, B = (0.0, 0.0), (10.0, 0.0)
    node = make_bare({'r1': [(10, A), (8, B)], 'r2': [(9, A), (0, B)]})
    plan = node._hungarian_plan(['r1', 'r2'], [], {'r1': (0, 0), 'r2': (0, 0)})
    assert plan['r1'][0] == B and plan['r2'][0] == A


def test_close_frontiers_not_double_assigned():
    A, A2 = (0.0, 0.0), (0.5, 0.0)   # within assign_radius=1.0 of each other
    node = make_bare({'r1': [(5, A)], 'r2': [(5, A2)]})
    plan = node._hungarian_plan(['r1', 'r2'], [], {'r1': (0, 0), 'r2': (0, 0)})
    assert len(plan) == 1


def test_single_candidate_robot_returns_empty_plan():
    node = make_bare({'r1': [(5, (0.0, 0.0))], 'r2': []})
    assert node._hungarian_plan(['r1', 'r2'], [], {'r1': (0, 0), 'r2': (0, 0)}) == {}
