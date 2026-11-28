"""coverage_planner: wait for the robot pose without blocking the map callback, and log each waypoint once."""
import importlib.util
import os
from types import SimpleNamespace as NS

import pytest

_SCRIPT = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'coverage_planner.py')
# No module-level skip: it aborts collection of the whole session under ROS's launch_testing pytest hook.
try:
    _spec = importlib.util.spec_from_file_location('coverage_planner', _SCRIPT)
    cp = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(cp)
except ImportError:
    cp = None

pytestmark = pytest.mark.skipif(cp is None, reason='ROS Nav2 messages not installed')


class Stub:
    """The planner's state without a ROS node; `pose` is what TF currently knows."""

    def __init__(self, pose_timeout=60.0):
        self.pose = (None, None)
        self.planned, self.logs, self.cancelled = [], [], False
        self._map, self._waypoint, self._pose_timeout = None, None, pose_timeout

    def create_timer(self, _period, callback):
        self.tick = callback
        return NS(cancel=lambda: setattr(self, 'cancelled', True))

    def get_logger(self):
        return NS(info=self.logs.append, warn=self.logs.append, error=self.logs.append)

    def _get_robot_pose(self):
        return self.pose

    def _plan_and_send(self):
        self.planned.append(self.pose)

    def _plan_when_located(self):
        cp.CoveragePlanner._plan_when_located(self)


def _map_msg():
    return NS(info=NS(width=4, height=4, resolution=0.1))


def _receive_map(stub):
    cp.CoveragePlanner._map_cb(stub, _map_msg())


def test_does_not_plan_inside_the_map_callback():
    s = Stub()
    _receive_map(s)
    assert s.planned == []


def test_plans_once_the_pose_arrives():
    s = Stub()
    _receive_map(s)
    s.tick()
    assert s.planned == [] and not s.cancelled
    s.pose = (1.0, 2.0)
    s.tick()
    assert s.planned == [(1.0, 2.0)] and s.cancelled


def test_plans_without_pose_after_the_timeout_and_says_so():
    s = Stub(pose_timeout=0.0)
    _receive_map(s)
    s.tick()
    assert s.planned == [(None, None)] and s.cancelled
    assert any('without the robot pose' in m for m in s.logs)


def test_each_waypoint_is_logged_once():
    s = Stub()
    for i in (0, 0, 0, 1, 1, 2, 2):
        cp.CoveragePlanner._feedback_cb(s, NS(feedback=NS(current_waypoint=i)))
    assert s.logs == [f'Coverage: heading to waypoint {n}.' for n in (1, 2, 3)]
