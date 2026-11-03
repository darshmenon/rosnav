#!/usr/bin/env python3
"""map_merge_known.merge_grids: pure merge of per-robot grids, checked against a real map (cafe)."""
import importlib.util
import math
import os

import numpy as np

_HERE = os.path.dirname(__file__)
_spec = importlib.util.spec_from_file_location('map_merge_known', os.path.join(_HERE, '..', 'scripts', 'map_merge_known.py'))
mm = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mm)

RES = 0.05


def _cafe():
    """Ground truth cafe grid, row 0 = y min, values FREE/OCCUPIED/UNKNOWN. Origin (-5.93, -5.6)."""
    path = os.path.join(_HERE, '..', 'maps', 'map_cafe.pgm')
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5'
        w, h = map(int, f.readline().split())
        f.readline()
        im = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)[::-1]
    g = np.full(im.shape, mm.UNKNOWN, dtype=np.int8)
    g[im >= 191] = mm.FREE
    g[im <= 90] = mm.OCCUPIED
    return g, -5.93, -5.6


def _crop(gt, c0, c1):
    """Robot that only saw columns [c0, c1): everything else unknown."""
    g = np.full_like(gt, mm.UNKNOWN)
    g[:, c0:c1] = gt[:, c0:c1]
    return g


def _to_world(grid_min, min_c, min_r):
    """merged grid -> dict {(col_world, row_world): value} of known cells, in global cell coords."""
    rr, cc = np.nonzero(grid_min != mm.UNKNOWN)
    return {(int(c) + min_c, int(r) + min_r): int(grid_min[r, c]) for r, c in zip(rr, cc)}


def _agreement(merged, gt, ox, oy):
    grid, min_c, min_r = merged
    known = _to_world(grid, min_c, min_r)
    base_c, base_r = int(round(ox / RES)), int(round(oy / RES))
    same = sum(1 for (c, r), v in known.items()
               if 0 <= r - base_r < gt.shape[0] and 0 <= c - base_c < gt.shape[1]
               and gt[r - base_r, c - base_c] == v)
    return same / max(1, len(known))


def test_two_overlapping_robots_reproduce_the_world():
    gt, ox, oy = _cafe()
    w = gt.shape[1]
    a, b = _crop(gt, 0, int(w * 0.6)), _crop(gt, int(w * 0.4), w)
    merged = mm.merge_grids([(a, ox, oy, dict(x=0, y=0, yaw=0)), (b, ox, oy, dict(x=0, y=0, yaw=0))], RES)
    grid, _, _ = merged
    assert (grid != mm.UNKNOWN).sum() >= (gt != mm.UNKNOWN).sum()      # nothing lost
    assert _agreement(merged, gt, ox, oy) > 0.999


def test_robot_local_frame_offset_is_undone_by_its_pose():
    """Robot B's local map origin is shifted by (1.5, -0.8) m and rotated 90 deg: pose undoes it."""
    gt, ox, oy = _cafe()
    w = gt.shape[1]
    a = _crop(gt, 0, int(w * 0.6))
    b = _crop(gt, int(w * 0.4), w)
    # pose maps local->world: g = t + R(90deg) l, so local cell = (gy - ty, tx - gx).
    # Local columns therefore run along world rows, local rows against world columns.
    yaw, tx, ty = math.pi / 2, 1.5, -0.8
    w_b = b.shape[1]
    b_local = b.T[::-1, :]
    lox = oy - ty
    loy = tx - (ox + w_b * RES)
    merged = mm.merge_grids([(a, ox, oy, dict(x=0, y=0, yaw=0)),
                             (b_local, lox, loy, dict(x=tx, y=ty, yaw=yaw))], RES)
    # shifts are not whole cells, so edges quantise by <= 1 cell
    assert _agreement(merged, gt, ox, oy) > 0.95


def test_conflict_policy_occupied_beats_free_beats_unknown():
    free = np.full((4, 4), mm.FREE, dtype=np.int8)
    occ = np.full((4, 4), mm.UNKNOWN, dtype=np.int8)
    occ[1, 1] = mm.OCCUPIED
    unk = np.full((4, 4), mm.UNKNOWN, dtype=np.int8)
    pose = dict(x=0, y=0, yaw=0)
    grid, min_c, min_r = mm.merge_grids([(free, 0, 0, pose), (occ, 0, 0, pose), (unk, 0, 0, pose)], RES)
    assert grid[1 - min_r, 1 - min_c] == mm.OCCUPIED
    assert grid[0 - min_r, 0 - min_c] == mm.FREE


def test_disjoint_maps_expand_the_bounding_box_and_margin_adds_unknown_border():
    one = np.zeros((2, 2), dtype=np.int8)
    g1, c1, r1 = mm.merge_grids([(one, 0.0, 0.0, dict(x=0, y=0, yaw=0))], RES, margin=3)
    assert g1.shape == (2 + 6, 2 + 6) and g1[0, 0] == mm.UNKNOWN
    g2, _, _ = mm.merge_grids([(one, 0.0, 0.0, dict(x=0, y=0, yaw=0)),
                               (one, 0.0, 0.0, dict(x=5.0, y=0, yaw=0))], RES)
    assert g2.shape[1] >= 100                      # 5 m apart at 5 cm


def test_empty_and_absurd_inputs_return_none():
    unk = np.full((3, 3), mm.UNKNOWN, dtype=np.int8)
    assert mm.merge_grids([], RES) is None
    assert mm.merge_grids([(unk, 0, 0, dict(x=0, y=0, yaw=0))], RES) is None
    one = np.zeros((2, 2), dtype=np.int8)
    assert mm.merge_grids([(one, 0, 0, dict(x=0, y=0, yaw=0)), (one, 0, 0, dict(x=1e4, y=1e4, yaw=0))], RES) is None
