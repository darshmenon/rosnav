#!/usr/bin/env python3
"""Frontier detection accuracy of FrontierCoordinator._find_frontiers on the real cafe map.
Partial exploration = ground truth revealed only inside a disk around the start (what a robot has seen)."""
import importlib.util
import os
import time

import numpy as np
from nav_msgs.msg import OccupancyGrid
from scipy import ndimage

_HERE = os.path.dirname(__file__)
_spec = importlib.util.spec_from_file_location('frontier_coordinator', os.path.join(_HERE, '..', 'scripts', 'frontier_coordinator.py'))
fc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fc)

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


def _explored(radius_m, centre=(0.0, 0.0)):
    gt = _gt()
    yy, xx = np.mgrid[0:gt.shape[0], 0:gt.shape[1]]
    wx, wy = OX + (xx + 0.5) * RES, OY + (yy + 0.5) * RES
    inside = np.hypot(wx - centre[0], wy - centre[1]) <= radius_m
    g = np.where(inside, gt, -1).astype(np.int8)
    return g


def _node(grid):
    n = fc.FrontierCoordinator.__new__(fc.FrontierCoordinator)
    n._detector, n._min_size = 'wfd', 5
    n._frontier_clearance, n._goal_pullback, n._goal_search_radius = 0.55, 0.55, 1.25
    n._suspect_ratio, n._suspect_buffer = 3.0, 1.0
    n._costmaps, n._failed_frontiers = {}, []
    n._failed_frontier_min_radius, n._failed_frontier_max_radius = 0.75, 3.0
    n._failed_frontier_radius_scale, n._failed_goal_cooldown, n._failed_goal_r = 0.35, 45.0, 0.75
    n._last_pullback_relax_warn, n._map_saved, n._map_topic, n._save_path = 0.0, False, '/map', ''
    n._rrt_iterations, n._rrt_step = 300, 0.5
    n._elapsed_sec = lambda: 100.0
    n.get_logger = lambda: type('L', (), {'info': lambda *a, **k: None, 'warn': lambda *a, **k: None,
                                          'debug': lambda *a, **k: None, 'error': lambda *a, **k: None})()
    m = OccupancyGrid()
    m.info.resolution, m.info.width, m.info.height = RES, grid.shape[1], grid.shape[0]
    m.info.origin.position.x, m.info.origin.position.y = OX, OY
    m.data = grid.flatten().tolist()
    n._map = m
    return n


def _true_frontier_cells(grid):
    free, unk = grid == 0, grid == -1
    near_unknown = ndimage.binary_dilation(unk, structure=ndimage.generate_binary_structure(2, 1))
    return free & near_unknown


def _world(cell_rc):
    return OX + (cell_rc[1] + 0.5) * RES, OY + (cell_rc[0] + 0.5) * RES


def test_every_reported_frontier_is_a_real_boundary_and_goals_are_free():
    grid = _explored(2.5)
    n = _node(grid)
    fr = n._find_frontiers([(0.0, 0.0)])
    assert fr, 'no frontiers found on a partially explored map'
    truth = np.argwhere(_true_frontier_cells(grid))
    tw = np.array([_world(c) for c in truth])
    from scipy.spatial import cKDTree
    tree = cKDTree(tw)
    for f in fr:
        # a frontier's centroid / point lies near a real free-unknown boundary cell (goal is pulled back from it)
        d, _ = tree.query(f['point'])
        assert d <= 1.5, (f['point'], d)
        gy = int((f['point'][1] - OY) / RES)
        gx = int((f['point'][0] - OX) / RES)
        assert grid[gy, gx] == 0, 'goal not on known free cell'


def test_recall_every_large_true_frontier_cluster_has_a_detected_frontier():
    grid = _explored(2.5)
    n = _node(grid)
    fr = n._find_frontiers([(0.0, 0.0)])
    pts = np.array([f['point'] for f in fr])
    lab, k = ndimage.label(_true_frontier_cells(grid), structure=np.ones((3, 3)))
    missed = 0
    big = 0
    from scipy.spatial import cKDTree
    tree = cKDTree(pts)
    for i in range(1, k + 1):
        cells = np.argwhere(lab == i)
        if len(cells) < 10:                  # < 0.5 m of boundary: legitimately ignored as noise
            continue
        big += 1
        cw = np.array([_world(c) for c in cells])
        if tree.query(cw)[0].min() > 1.5:
            missed += 1
    assert big > 0 and missed == 0, f'{missed}/{big} large frontier clusters missed'


def test_fully_explored_map_has_no_frontiers_and_runs_fast():
    n = _node(_explored(100.0))
    t = time.time()
    fr = n._find_frontiers([(0.0, 0.0)])
    assert time.time() - t < 2.0
    assert len(fr) == 0
