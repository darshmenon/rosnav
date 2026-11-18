"""Geometry checks for the Gazebo decentral scenarios: starts/goals sit in free map space, groups face each other."""
import importlib.util
import os
import sys

import numpy as np
import yaml

_D = os.path.join(os.path.dirname(__file__), '..')
_s = importlib.util.spec_from_file_location('dgs', os.path.join(_D, 'scripts', 'decentral_gz_scenarios.py'))
dgs = importlib.util.module_from_spec(_s)
sys.modules['dgs'] = dgs
_s.loader.exec_module(dgs)


def setup_module(module):
    dgs.gen()          # worlds/maps are generated (and gitignored): make sure they exist on a clean checkout


def _grid(name):
    meta = yaml.safe_load(open(os.path.join(_D, 'maps', name)))
    with open(os.path.join(_D, 'maps', meta['image']), 'rb') as f:
        f.readline()
        w, h = map(int, f.readline().split())
        f.readline()
        img = np.frombuffer(f.read(), np.uint8).reshape(h, w)
    return img, meta['resolution'], meta['origin']


def _free(img, res, org, x, y, clear=0.4):
    r = int(clear / res)
    c, rw = int((x - org[0]) / res), int(img.shape[0] - 1 - (y - org[1]) / res)
    win = img[max(rw - r, 0):rw + r + 1, max(c - r, 0):c + r + 1]
    return win.size > 0 and (win > 250).all()


def test_all_starts_and_goals_are_free_with_clearance():
    for kind, n in (('swap', 8), ('corridor', 6), ('crowd', 6)):
        if kind == 'crowd' and not os.path.exists(os.path.join(_D, 'maps', 'map_crowd.yaml')):
            continue   # crowd map comes from nav_explore_eval truth-from-world (not generated here)
        s = dgs.build(kind, n)
        img, res, org = _grid(s['map'])
        for r in s['robots']:
            assert _free(img, res, org, r['x'], r['y']), (kind, r)
        for g in s['goals']:
            assert _free(img, res, org, g[0], g[1]), (kind, g)


def test_swap_goals_are_antipodal_and_corridor_groups_cross():
    s = dgs.build('swap', 8)
    for r, g in zip(s['robots'], s['goals']):
        assert abs(r['x'] + g[0]) < 0.01 and abs(r['y'] + g[1]) < 0.01
    c = dgs.build('corridor', 6)
    for r, g in zip(c['robots'], c['goals']):
        assert r['x'] * g[0] < 0                      # starts and goals in opposite rooms


def test_corridor_width_is_1p5_m():
    boxes = dgs.corridor_boxes()
    img, org = dgs.raster(boxes, 0.05, 1.0)
    col = img[:, int((0.0 - org[0]) / 0.05)]
    free = np.sum(col[int(1.0 / 0.05) + 2: -int(1.0 / 0.05) - 2] > 200)
    assert abs(free * 0.05 - 1.5) < 0.12
