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


def _grid_node(rows, reach):
    """Bare coordinator with a tiny OccupancyGrid (-1 unknown, 0 free, 100 occupied), 1 m cells."""
    import types
    import numpy as np
    h, w = len(rows), len(rows[0])
    info = types.SimpleNamespace(width=w, height=h, resolution=1.0,
                                 origin=types.SimpleNamespace(position=types.SimpleNamespace(x=0.0, y=0.0)))
    node = fc.FrontierCoordinator.__new__(fc.FrontierCoordinator)
    node._map = types.SimpleNamespace(data=[v for r in rows for v in r], info=info)
    node._unknown_reach = reach
    return node


def test_robot_in_unknown_cells_reaches_mapped_free_space():
    # free blob on the left, two unknown cells, then the robot standing in unknown territory
    rows = [[0, 0, 0, -1, -1, -1]]
    far = _grid_node(rows, 0.0)._path_distance_map((5.5, 0.5))
    near = _grid_node(rows, 3.0)._path_distance_map((5.5, 0.5))
    assert not (far[0, :3] < float('inf')).any()        # strict: robot cut off from the free blob
    assert (near[0, :3] < float('inf')).all()           # unknown_reach lets it connect


def test_unknown_reach_never_crosses_occupied_cells():
    rows = [[0, 0, 100, -1, -1]]
    d = _grid_node(rows, 5.0)._path_distance_map((4.5, 0.5))
    assert not (d[0, :2] < float('inf')).any()          # the wall still blocks


class _WarnLog:
    def __init__(self): self.warns = []
    def info(self, *a, **k): pass
    def warn(self, msg, *a, **k): self.warns.append(msg)


def _goal_node(sent_ago, nav_ready=True, timeout=150.0):
    import time
    node = fc.FrontierCoordinator.__new__(fc.FrontierCoordinator)
    log = _WarnLog()
    node.get_logger = lambda: log
    node._log = log
    node._robots = ['r1']
    node._state = {'r1': fc._State.NAVIGATING}
    node._goal_sent_t = {'r1': time.monotonic() - sent_ago}
    node._assigned = {'r1': (2.0, 1.0)}
    node._assigned_meta = {'r1': {'score': 1.0}}
    node._nav_ready = {'r1': nav_ready}
    node._goal_timeout = timeout
    node._goal_gen = {'r1': 3}
    node._failed_goals = 0
    cancelled = []
    node._goal_handle = {'r1': types_ns(cancel_goal_async=lambda: cancelled.append(1))}
    node._cancelled = cancelled
    node._failed = []
    node._remember_failed_frontier = lambda g, m: node._failed.append(g)
    node._publish_stats = lambda: None
    return node


def types_ns(**kw):
    import types
    return types.SimpleNamespace(**kw)


def test_goal_with_no_result_is_abandoned_after_timeout():
    node = _goal_node(sent_ago=200.0)
    node._expire_stale_goals()
    assert node._state['r1'] == fc._State.IDLE and node._assigned['r1'] is None
    assert node._failed_goals == 1 and node._failed == [(2.0, 1.0)]
    assert node._cancelled == [1] and node._goal_gen['r1'] == 4      # late replies now ignored


def test_recent_goal_with_live_server_is_kept():
    node = _goal_node(sent_ago=20.0)
    node._expire_stale_goals()
    assert node._state['r1'] == fc._State.NAVIGATING and node._failed_goals == 0


def test_goal_freed_when_action_server_disappears():
    node = _goal_node(sent_ago=12.0, nav_ready=False)
    node._expire_stale_goals()
    assert node._state['r1'] == fc._State.IDLE and node._failed_goals == 1
    # but a brief blip (<5 s since the goal was sent) is tolerated
    young = _goal_node(sent_ago=2.0, nav_ready=False)
    young._expire_stale_goals()
    assert young._state['r1'] == fc._State.NAVIGATING


def test_goal_timeout_zero_disables_expiry():
    node = _goal_node(sent_ago=9999.0, timeout=0.0)
    node._expire_stale_goals()
    assert node._state['r1'] == fc._State.NAVIGATING


def test_unknown_reach_not_applied_when_robot_is_on_known_free_space():
    # robot stands on a known-free cell cut off from the blob by unknown cells: strict search
    rows = [[0, 0, -1, -1, 0]]
    d = _grid_node(rows, 5.0)._path_distance_map((4.5, 0.5))
    assert not (d[0, :2] < float('inf')).any()


# ----------------------------------------------------------------------
# Cold start: a suspicious-only frontier must still be pickable before any goal exists
# ----------------------------------------------------------------------
def _cold_node(assigned=None, done=0):
    obj = fc.FrontierCoordinator.__new__(fc.FrontierCoordinator)
    obj._assigned = assigned or {'r1': None, 'r2': None}
    obj._succeeded_goals, obj._failed_goals = done, 0
    obj._visited, obj._revisit_r, obj._assign_r = [], 0.3, 1.0
    obj._suspect_hard_ratio, obj._suspect_ratio, obj._suspect_penalty = 6.0, 3.0, 8.0
    obj._min_gain = 0.0
    obj._path_distance_map = lambda pos: None
    obj._frontier_path_distance = lambda fr, pd: 2.0
    obj._costmap_allows = lambda x, y, ns: True
    obj._recently_failed = lambda x, y: False
    obj._min_goal_for = lambda ns: 0.2
    obj._score_frontier = lambda fr, d, ns: 1.0
    return obj


_SUSPECT = {'point': (1.0, 1.0), 'suspicious_ratio': 12.0, 'size_m': 5.0, 'info_gain': 1.0}


def test_cold_start_picks_suspicious_frontier():
    goal, pick = _cold_node()._pick([dict(_SUSPECT)], (0.0, 0.0), 'r1')
    assert goal == (1.0, 1.0) and pick['n_suspicious'] == 0


def test_after_first_goal_suspicious_frontier_rejected():
    goal, pick = _cold_node(done=1)._pick([dict(_SUSPECT)], (0.0, 0.0), 'r1')
    assert goal is None and pick['n_suspicious'] == 1
    goal, pick = _cold_node(assigned={'r1': None, 'r2': (3.0, 3.0)})._pick([dict(_SUSPECT)], (0.0, 0.0), 'r1')
    assert goal is None and pick['n_suspicious'] == 1
