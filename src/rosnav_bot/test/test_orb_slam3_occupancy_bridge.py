#!/usr/bin/env python3
"""Unit tests for the pure-logic pieces of scripts/orb_slam3_occupancy_bridge.py.

No ROS graph: the node is built with __new__() and only the attributes each
method reads are set (same approach as test_frontier_explorer.py).
"""
import importlib.util
import math
import os
from collections import deque

import numpy as np

_SCRIPT = os.path.join(
    os.path.dirname(__file__), '..', 'scripts', 'orb_slam3_occupancy_bridge.py')
_spec = importlib.util.spec_from_file_location('orb_slam3_occupancy_bridge', _SCRIPT)
bridge = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(bridge)

EMPTY = np.empty((0, 2))


def _wall(x, n=200):
    return np.c_[np.full(n, x), np.linspace(-1.5, 1.5, n)]


def _wall_x(grid):
    cols = np.argwhere(grid.to_occupancy() == bridge.OCCUPIED)[:, 1]
    return cols.mean() * grid.res + grid.ox if len(cols) else None


def _node():
    class _Log:
        def info(self, *_a, **_k):
            pass

    class _Clock:
        class _T:
            nanoseconds = int(100e9)

        def now(self):
            return self._T()

    n = object.__new__(bridge.OrbSlam3OccupancyBridge)
    n.get_logger = lambda: _Log()
    n.get_clock = lambda: _Clock()
    n._resolution = 0.05
    n._grid = bridge.LogOddsGrid(0.05, 20.0)
    n._grid.center_on(0.0, 0.0)
    n._rng = np.random.default_rng(0)
    n._kf = deque(maxlen=500)
    n._kf_pts = 600
    n._jump_t = 0.12
    n._jump_r = math.radians(2.0)
    n._rebuild_gap = 8.0
    n._last_rebuild = 0.0
    n._rebuilds = 0
    n._ref_mo = (0.0, 0.0, 0.0)
    return n


def test_wall_needs_repeated_hits_to_become_occupied():
    g = bridge.LogOddsGrid(0.05, 20.0)
    g.center_on(0.0, 0.0)
    g.integrate(0.0, 0.0, EMPTY, _wall(3.0))
    assert _wall_x(g) is None          # one hit is not enough
    g.integrate(0.0, 0.0, EMPTY, _wall(3.0))
    assert abs(_wall_x(g) - 3.0) < 0.1


def test_grid_grows_to_fit_far_points():
    g = bridge.LogOddsGrid(0.05, 2.0)
    g.center_on(0.0, 0.0)
    g.integrate(0.0, 0.0, EMPTY, _wall(30.0, 20))
    assert g.l.shape[1] * g.res > 30.0


def test_floor_hits_clear_space_but_do_not_mark_occupied():
    g = bridge.LogOddsGrid(0.05, 20.0)
    g.center_on(0.0, 0.0)
    for _ in range(3):
        g.integrate(0.0, 0.0, _wall(3.0), EMPTY)
    occ = g.to_occupancy()
    assert (occ == bridge.OCCUPIED).sum() == 0
    assert (occ == bridge.FREE).sum() > 0


def test_rebuild_moves_old_walls_with_a_map_odom_shift():
    n = _node()
    for _ in range(6):
        n._grid.integrate(0.0, 0.0, EMPTY, _wall(3.0))
        n._store_keyframe(n._ref_mo, (0.0, 0.0), EMPTY, _wall(3.0))
    assert abs(_wall_x(n._grid) - 3.0) < 0.1
    n._maybe_rebuild((0.5, 0.0, 0.0))
    assert abs(_wall_x(n._grid) - 3.5) < 0.1
    assert n._rebuilds == 1


def test_small_shift_does_not_rebuild():
    n = _node()
    n._store_keyframe(n._ref_mo, (0.0, 0.0), EMPTY, _wall(3.0))
    n._maybe_rebuild((0.05, 0.0, math.radians(0.5)))
    assert n._rebuilds == 0


def test_rebuild_respects_min_interval():
    n = _node()
    n._store_keyframe(n._ref_mo, (0.0, 0.0), EMPTY, _wall(3.0))
    n._last_rebuild = 100.0 - 1.0      # clock says t=100 s, 1 s ago
    n._maybe_rebuild((0.5, 0.0, 0.0))
    assert n._rebuilds == 0


def test_odom_map_roundtrip_with_rotation():
    mo = (1.2, -0.7, math.radians(37))
    pts = np.array([[1.0, 2.0], [-3.0, 0.5]])
    back = bridge.OrbSlam3OccupancyBridge._odom_to_map(
        bridge.OrbSlam3OccupancyBridge._map_to_odom(pts, mo), mo)
    assert np.allclose(back, pts)


def test_ground_plane_flat_floor_gives_floor_mask():
    xs, ys = np.meshgrid(np.linspace(0.5, 2.9, 30), np.linspace(-1.5, 1.5, 30))
    pts = np.c_[xs.ravel(), ys.ravel(), np.zeros(900)]
    plane = bridge.fit_ground_plane(pts)
    assert plane is not None
    floor, band = bridge.split_by_height(pts, 0.05, 0.55, plane)
    assert floor.all() and not band.any()
