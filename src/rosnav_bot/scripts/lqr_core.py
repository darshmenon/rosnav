#!/usr/bin/env python3
"""
lqr_core.py — LQR path tracker for a unicycle (diff-drive) robot. No ROS; same `solve()` signature as
mpc_core.UnicycleMPC so mpc_controller.py can run either (`solver:=lqr`).

Error in the robot frame, e = [ex (along), ey (cross), eth], to the reference point on the path:
    e_dot = A e + B du,  A = [[0, wr, 0], [-wr, 0, vr], [0, 0, 0]],  B = [[-1, 0], [0, 0], [0, -1]]
(Kanayama error dynamics linearised at e=0, du = [v - vr, w - wr]). Discretised with dt and solved with the
discrete algebraic Riccati equation; u = [vr, wr] - K e. wr is the path's curvature x vr (feed-forward).
K is re-solved when (vr, wr) changes, cached otherwise, so a solve is ~0.1 ms (MPC: ~10-50 ms).

Limits: v/w clipped to the box, accelerations to a_max/alpha_max. Obstacles are NOT considered - LQR is a pure
tracker; rely on the planner + collision monitor (this is the honest trade-off vs the MPC).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.linalg import solve_discrete_are

try:                                    # works both as a script dir import and via importlib in tests
    from mpc_core import wrap, sample_reference
except ImportError:                     # pragma: no cover
    import importlib.util
    import os
    import sys
    _p = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'mpc_core.py')
    _s = importlib.util.spec_from_file_location('mpc_core', _p)
    _m = importlib.util.module_from_spec(_s)
    sys.modules.setdefault('mpc_core', _m)
    _s.loader.exec_module(_m)
    wrap, sample_reference = _m.wrap, _m.sample_reference


@dataclass
class LQRConfig:
    dt: float = 0.1
    v_max: float = 0.5
    v_min: float = 0.0
    w_max: float = 1.5
    a_max: float = 1.0
    alpha_max: float = 3.0
    v_ref: float = 0.35
    lookahead: float = 0.25          # reference point this far ahead along the path (m)
    q_along: float = 1.0
    q_cross: float = 6.0
    q_yaw: float = 2.0
    r_v: float = 1.0
    r_w: float = 0.5
    slow_dist: float = 0.8           # ramp v_ref down inside this distance of the goal
    turn_slow: float = 1.0           # v_ref scaled by 1/(1 + turn_slow*|kappa|)


class UnicycleLQR:
    def __init__(self, cfg: LQRConfig | None = None):
        self.cfg = cfg or LQRConfig()
        self._cache = {}
        self._v_prev = 0.0
        self._w_prev = 0.0

    def reset(self):
        self._v_prev = self._w_prev = 0.0

    def _gain(self, vr: float, wr: float):
        key = (round(vr, 2), round(wr, 2))
        k = self._cache.get(key)
        if k is None:
            c = self.cfg
            a = np.array([[0.0, wr, 0.0], [-wr, 0.0, vr], [0.0, 0.0, 0.0]])
            b = np.array([[-1.0, 0.0], [0.0, 0.0], [0.0, -1.0]])
            ad, bd = np.eye(3) + c.dt * a, c.dt * b          # Euler discretisation
            q = np.diag([c.q_along, c.q_cross, c.q_yaw])
            r = np.diag([c.r_v, c.r_w])
            p = solve_discrete_are(ad, bd, q, r)
            k = np.linalg.solve(r + bd.T @ p @ bd, bd.T @ p @ ad)
            if len(self._cache) > 256:
                self._cache.clear()
            self._cache[key] = k
        return k

    def solve(self, state, v_now, w_now, path_xy, obstacles=None):
        c = self.cfg
        x, y, th = (float(s) for s in state)
        path_xy = np.asarray(path_xy, float)
        goal_d = float(np.hypot(*(path_xy[-1] - np.array([x, y]))))
        pts, hd = sample_reference(path_xy, (x, y), 2, max(c.lookahead, 1e-3))
        # reference = point `lookahead` ahead; curvature from the heading change over that span
        rx, ry, rth = pts[1, 0], pts[1, 1], float(hd[1])
        span = float(np.hypot(*(pts[2] - pts[0]))) or 1e-3
        kappa = float(wrap(hd[2] - hd[0])) / span
        v_goal = c.v_ref * min(1.0, goal_d / c.slow_dist) if c.slow_dist > 0 else c.v_ref
        vr = max(0.05, v_goal / (1.0 + c.turn_slow * abs(kappa)))
        wr = vr * kappa
        dx, dy = rx - x, ry - y
        e = np.array([math.cos(th) * dx + math.sin(th) * dy,
                      -math.sin(th) * dx + math.cos(th) * dy,
                      float(wrap(rth - th))])
        du = -self._gain(vr, wr) @ e
        v = vr + float(du[0])
        w = wr + float(du[1])
        v = float(np.clip(v, c.v_min, c.v_max))
        w = float(np.clip(w, -c.w_max, c.w_max))
        v = float(np.clip(v, v_now - c.a_max * c.dt, v_now + c.a_max * c.dt))
        w = float(np.clip(w, w_now - c.alpha_max * c.dt, w_now + c.alpha_max * c.dt))
        return v, w, {'ok': True, 'cost': float(e @ e), 'goal_dist': goal_d,
                      'cross_err': float(e[1]), 'yaw_err': float(e[2]), 'kappa': kappa}
