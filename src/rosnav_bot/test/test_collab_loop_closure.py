#!/usr/bin/env python3
"""collab_loop_closure._match_pair: does the correlative matcher recover a known pose error?
Ground truth = real cafe map split into two overlapping robots; robot B's pose is perturbed."""
import importlib.util
import math
import os

import numpy as np
from nav_msgs.msg import OccupancyGrid

_HERE = os.path.dirname(__file__)
_spec = importlib.util.spec_from_file_location('collab_loop_closure', os.path.join(_HERE, '..', 'scripts', 'collab_loop_closure.py'))
clc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(clc)

RES, OX, OY = 0.05, -5.93, -5.6


def _gt():
    with open(os.path.join(_HERE, '..', 'maps', 'map_cafe.pgm'), 'rb') as f:
        assert f.readline().strip() == b'P5'
        w, h = map(int, f.readline().split())
        f.readline()
        im = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)[::-1]
    g = np.full(im.shape, -1, dtype=np.int8)
    g[im >= 191], g[im <= 90] = 0, 100
    return g


def _msg(grid):
    m = OccupancyGrid()
    m.info.resolution, m.info.width, m.info.height = RES, grid.shape[1], grid.shape[0]
    m.info.origin.position.x, m.info.origin.position.y = OX, OY
    m.data = grid.flatten().tolist()
    return m


def _matcher():
    n = clc.CollabLoopClosure.__new__(clc.CollabLoopClosure)
    n._sw_m, n._ss_m, n._sw_rad, n._ss_rad = 0.5, 0.1, 0.08, 0.02
    n._min_overlap = 40
    return n


def _pair():
    g = _gt()
    w = g.shape[1]
    a, b = g.copy(), g.copy()
    a[:, int(w * 0.65):] = -1
    b[:, :int(w * 0.35)] = -1
    return _msg(a), _msg(b)


def _match(err):
    a, b = _pair()
    true = dict(x=0.0, y=0.0, yaw=0.0)
    wrong = dict(x=err[0], y=err[1], yaw=err[2])
    # robot i = B with the wrong pose, j = A with the true pose; the correction applied to B's pose
    return _matcher()._match_pair(b, wrong, a, true)


def test_zero_error_stays_put_with_high_match_ratio():
    dx, dy, dyaw, overlap, ratio = _match((0.0, 0.0, 0.0))
    assert (dx, dy, dyaw) == (0.0, 0.0, 0.0) and ratio > 0.1


def test_recovers_translation_error_within_one_search_step():
    for ex, ey in [(0.3, 0.0), (-0.3, 0.2), (0.0, -0.4), (0.5, 0.5)]:
        dx, dy, dyaw, overlap, ratio = _match((ex, ey, 0.0))
        assert abs(dx + ex) <= 0.1 + 1e-9 and abs(dy + ey) <= 0.1 + 1e-9, (ex, ey, dx, dy)


def test_recovers_yaw_error_within_one_search_step():
    for eyaw in (0.04, -0.06, 0.08):
        dx, dy, dyaw, overlap, ratio = _match((0.0, 0.0, eyaw))
        assert abs(dyaw + eyaw) <= 0.02 + 1e-9, (eyaw, dyaw)


def test_error_outside_the_window_is_not_fully_recovered():
    """Documents the limit: a 1.0 m error is beyond the 0.5 m window, residual stays >= 0.4 m."""
    dx, dy, _, _, _ = _match((1.0, 0.0, 0.0))
    assert abs(dx + 1.0) >= 0.4
