#!/usr/bin/env python3
"""Closed-loop checks of scripts/lqr_core.py: a kinematic unicycle driven by the LQR, no ROS."""
import importlib.util
import math
import os
import sys

import numpy as np

_D = os.path.join(os.path.dirname(__file__), '..', 'scripts')
for _n in ('mpc_core', 'lqr_core'):
    _s = importlib.util.spec_from_file_location(_n, os.path.join(_D, _n + '.py'))
    _m = importlib.util.module_from_spec(_s)
    sys.modules[_n] = _m
    _s.loader.exec_module(_m)
lqr_core = sys.modules['lqr_core']
DT = 0.1


def _drive(path, start, steps=300):
    lqr = lqr_core.UnicycleLQR(lqr_core.LQRConfig(dt=DT))
    x = np.array(start, float)
    v = w = 0.0
    errs, vs, ws = [], [], []
    for _ in range(steps):
        v, w, info = lqr.solve(x, v, w, path)
        assert info['ok']
        x = x + DT * np.array([v * math.cos(x[2]), v * math.sin(x[2]), w])
        errs.append(float(np.min(np.hypot(path[:, 0] - x[0], path[:, 1] - x[1]))))
        vs.append(v)
        ws.append(w)
        if np.hypot(*(path[-1] - x[:2])) < 0.1:
            break
    return x, errs, vs, ws


def test_converges_to_straight_line_from_offset():
    path = np.c_[np.linspace(0, 4, 80), np.zeros(80)]
    x, errs, _, _ = _drive(path, (0.0, 0.4, 0.3))
    assert np.hypot(*(path[-1] - x[:2])) < 0.15
    assert errs[-1] < 0.05                      # cross-track error has been regulated away


def test_follows_a_u_turn_and_reaches_goal():
    a = np.linspace(-math.pi / 2, math.pi / 2, 80)
    path = np.r_[np.c_[np.linspace(0, 2, 40), np.zeros(40)],
                 np.c_[2 + 0.7 * np.cos(a), 0.7 + 0.7 * np.sin(a)],
                 np.c_[np.linspace(2, 0, 40), 1.4 * np.ones(40)]]
    x, errs, _, _ = _drive(path, (0.0, -0.2, 0.0))
    assert np.hypot(*(path[-1] - x[:2])) < 0.15
    assert max(errs) < 0.35


def test_respects_velocity_and_accel_limits():
    path = np.c_[np.linspace(0, 3, 60), np.zeros(60)]
    _, _, vs, ws = _drive(path, (0.0, 1.0, 2.0), steps=60)   # big error -> saturating command
    c = lqr_core.LQRConfig()
    assert max(vs) <= c.v_max + 1e-9 and min(vs) >= c.v_min - 1e-9
    assert max(abs(w) for w in ws) <= c.w_max + 1e-9
    assert max(abs(np.diff(vs))) <= c.a_max * DT + 1e-9
