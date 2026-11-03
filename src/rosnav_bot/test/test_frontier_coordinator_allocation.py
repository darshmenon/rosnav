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


def _bare_scorer(sep_w):
    obj = fc.FrontierCoordinator.__new__(fc.FrontierCoordinator)
    obj._sep_w, obj._sep_r = sep_w, 6.0
    obj._assigned = {'r1': None, 'r2': (0.0, 0.0)}
    return obj


def test_separation_penalty_off_by_default():
    assert _bare_scorer(0.0)._separation_penalty({'point': (1.0, 0.0)}, 'r1') == 0.0


def test_separation_penalty_scales_with_closeness():
    node = _bare_scorer(2.0)
    near = node._separation_penalty({'point': (0.0, 0.0)}, 'r1')
    mid = node._separation_penalty({'point': (3.0, 0.0)}, 'r1')
    far = node._separation_penalty({'point': (7.0, 0.0)}, 'r1')
    assert near == 2.0 and abs(mid - 1.0) < 1e-9 and far == 0.0
    # own goal is ignored
    assert node._separation_penalty({'point': (0.0, 0.0)}, 'r2') == 0.0


def test_parse_robot_floats_and_per_robot_min_goal():
    assert fc.parse_robot_floats('robot2:0.8, bad, robot3:x,robot4:0.5') == {
        'robot2': 0.8, 'robot4': 0.5}
    node = fc.FrontierCoordinator.__new__(fc.FrontierCoordinator)
    node._min_goal_d, node._min_goal_by_robot = 0.35, {'r2': 0.8}
    assert node._min_goal_for('r2') == 0.8 and node._min_goal_for('r1') == 0.35


def _pick_node(min_gain):
    node = fc.FrontierCoordinator.__new__(fc.FrontierCoordinator)
    for k, v in dict(_assigned={}, _visited=[], _revisit_r=0.5, _assign_r=1.0, _suspect_hard_ratio=6.0,
                     _min_goal_d=0.0, _min_gain=min_gain, _scorer='nearest', _hyst_r=0, _hyst_gain=0,
                     _sep_w=0.0, _sep_r=6.0, _suspect_ratio=3.0, _suspect_penalty=8.0,
                     _suspect_buffer=1.0).items():
        setattr(node, k, v)
    node._path_distance_map = lambda pos: None
    node._frontier_path_distance = staticmethod(lambda f, pd: f['d'])
    node._recently_failed = lambda x, y: False
    node._costmap_allows = lambda x, y, ns: True
    return node


def _fr(x, gain, d):
    return {'point': (x, 0.0), 'size_m': 1.0, 'info_gain': gain, 'clearance': 1.0, 'd': d}


def test_tiny_frontiers_skipped_while_larger_exist():
    node = _pick_node(0.05)
    goal, diag = node._pick([_fr(1.0, 0.02, 1.0), _fr(5.0, 0.20, 4.0)], (0, 0), 'r1')
    assert goal == (5.0, 0.0) and diag['n_tiny'] == 1


def test_tiny_frontiers_used_when_nothing_else_left():
    node = _pick_node(0.05)
    goal, _ = node._pick([_fr(1.0, 0.02, 1.0)], (0, 0), 'r1')
    assert goal == (1.0, 0.0)


def test_stale_tf_is_detected():
    import types

    class _Clock:
        def now(self): return fc.rclpy.time.Time(seconds=100)

    node = fc.FrontierCoordinator.__new__(fc.FrontierCoordinator)
    node.get_clock = lambda: _Clock()
    node._max_tf_age = 3.0
    mk = lambda sec: types.SimpleNamespace(header=types.SimpleNamespace(
        stamp=types.SimpleNamespace(sec=sec, nanosec=0)))
    assert node._tf_is_stale(mk(90)) is True       # 10 s old
    assert node._tf_is_stale(mk(99)) is False      # 1 s old
    assert node._tf_is_stale(mk(0)) is False       # static chain
    node._max_tf_age = 0.0
    assert node._tf_is_stale(mk(10)) is False      # disabled
