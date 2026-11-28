"""coverage_planner: start_from_robot may pick which corner the sweep starts from, but must keep it a sweep."""
import importlib.util
import math
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

RES, SIZE = 0.1, 40          # 4 m x 4 m free room, origin (0, 0)


def _plan(robot=None, from_robot=True):
    origin = NS(position=NS(x=0.0, y=0.0))
    stub = NS(_map=NS(info=NS(resolution=RES, width=SIZE, height=SIZE, origin=origin), data=[0] * SIZE * SIZE),
              _radius=0.2, _spacing=0.5, _from_bot=from_robot,
              _rx_min=-1e9, _rx_max=1e9, _ry_min=-1e9, _ry_max=1e9,
              _get_robot_pose=lambda: robot or (None, None))
    return cp.CoveragePlanner._compute_coverage(stub)


def _length(path):
    return sum(math.dist(a[:2], b[:2]) for a, b in zip(path, path[1:]))


@pytest.mark.parametrize('robot', [(0.0, 0.0), (4.0, 0.0), (0.0, 4.0), (4.0, 4.0), (2.1, 1.9)])
def test_order_stays_a_sweep(robot):
    sweep = _plan(from_robot=False)
    path = _plan(robot)
    assert sorted(p[:2] for p in path) == sorted(p[:2] for p in sweep)
    assert _length(path) == pytest.approx(_length(sweep))


@pytest.mark.parametrize('robot', [(0.0, 0.0), (4.0, 0.0), (0.0, 4.0), (4.0, 4.0)])
def test_starts_at_nearest_corner(robot):
    path = _plan(robot)
    assert math.dist(path[0][:2], robot) < 0.6


@pytest.mark.parametrize('robot', [(0.0, 0.0), (4.0, 0.0), (0.0, 4.0), (4.0, 4.0)])
def test_yaw_faces_travel_direction(robot):
    path = _plan(robot)
    for a, b in zip(path, path[1:]):
        if a[1] == b[1]:
            assert math.cos(a[2]) * (b[0] - a[0]) > 0


def test_unknown_pose_keeps_sweep():
    assert _plan(robot=None) == _plan(from_robot=False)
