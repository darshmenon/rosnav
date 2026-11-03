#!/usr/bin/env python3
"""FrontierCoordinator spurious-success guard + _on_result bookkeeping (no ROS graph needed)."""
import importlib.util
import os
import time

from action_msgs.msg import GoalStatus

_SCRIPT = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'frontier_coordinator.py')
_spec = importlib.util.spec_from_file_location('frontier_coordinator', _SCRIPT)
fc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fc)


class _Log:
    def info(self, *a, **k): pass
    warn = info
    error = info


class _Fut:
    def __init__(self, status):
        self._s = status

    def result(self):
        return type('R', (), {'status': self._s})()


def make(start, pos, sent_ago, goal=(0.0, 3.0)):
    n = fc.FrontierCoordinator.__new__(fc.FrontierCoordinator)
    n.get_logger = _Log
    n._min_goal_d = 0.5
    n._robots = ['r1']
    n._goal_start = {'r1': start}
    n._goal_sent_t = {'r1': time.monotonic() - sent_ago}
    n._hold_until = {'r1': 0.0}
    n._spurious = {'r1': 0}
    n._robot_pos = lambda ns: pos
    n._assigned = {'r1': goal}
    n._assigned_meta = {'r1': None}
    n._state = {'r1': fc._State.NAVIGATING}
    n._visited = []
    n._succeeded_goals = n._failed_goals = 0
    n._failed_frontiers = []
    n._publish_stats = lambda: None
    n._remember_failed_frontier = lambda g, m: n._failed_frontiers.append(g)
    return n


def test_success_without_motion_is_spurious_and_not_visited():
    n = make(start=(0.0, 0.0), pos=(0.0, 0.02), sent_ago=3.0)
    n._on_result(_Fut(GoalStatus.STATUS_SUCCEEDED), 'r1')
    assert n._visited == [] and n._succeeded_goals == 0
    assert n._hold_until['r1'] > time.monotonic() and n._spurious['r1'] == 1
    assert n._state['r1'] == fc._State.IDLE and n._assigned['r1'] is None


def test_instant_success_under_half_a_second_is_spurious_even_if_robot_moved():
    n = make(start=(0.0, 0.0), pos=(0.0, 1.5), sent_ago=0.1)
    assert n._spurious_success('r1', (0.0, 3.0))


def test_real_arrival_is_visited_and_clears_the_streak():
    n = make(start=(0.0, 0.0), pos=(0.0, 2.95), sent_ago=8.0)
    n._spurious['r1'] = 3
    n._on_result(_Fut(GoalStatus.STATUS_SUCCEEDED), 'r1')
    assert n._visited == [(0.0, 3.0)] and n._succeeded_goals == 1 and n._spurious['r1'] == 0


def test_goal_already_within_tolerance_is_a_genuine_success_without_motion():
    n = make(start=(0.0, 2.9), pos=(0.0, 2.9), sent_ago=0.2, goal=(0.0, 3.0))
    assert not n._spurious_success('r1', (0.0, 3.0))


def test_backoff_grows_and_caps_at_30s():
    n = make(start=(0.0, 0.0), pos=(0.0, 0.0), sent_ago=3.0)
    for _ in range(20):
        n._spurious_success('r1', (0.0, 3.0))
    assert n._spurious['r1'] == 20
    assert 29.0 < n._hold_until['r1'] - time.monotonic() <= 30.0


def test_no_tf_means_no_verdict():
    n = make(start=None, pos=None, sent_ago=0.1)
    assert not n._spurious_success('r1', (0.0, 3.0))
