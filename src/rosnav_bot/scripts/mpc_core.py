#!/usr/bin/env python3
"""
mpc_core.py — nonlinear model-predictive controller for a unicycle (diff-drive) robot.

No ROS in here, so it can be unit-tested and simulated offline. Each call solves

    min  sum_k  q_pos |p_k - ref_k|^2 + q_yaw wrap(th_k - ref_th_k)^2     (track the path)
         + r_v (v_k - v_ref)^2 + r_w w_k^2                                  (effort)
         + r_dv dv_k^2 + r_dw dw_k^2                                        (smoothness)
         + q_term |p_N - ref_N|^2 + q_obs sum hinge(r_soft - d_ki)^2        (terminal, soft clearance)
    s.t. x_{k+1} = x_k + dt (v cos th, v sin th, w)                         (unicycle)
         v in [v_min, v_max], w in [-w_max, w_max], |dv| <= a_max dt, |dw| <= alpha_max dt
         |p_k - o_i(k)| >= r_hard   for the nearest `max_obstacles` points   (hard clearance)

with SciPy SLSQP, warm-started from the previous solution shifted by one step. Obstacles may carry a
velocity (constant-velocity prediction), which is what makes it avoid *moving* obstacles. When the
robot is already closer than r_hard to an obstacle the hard radius is relaxed to what is currently
achievable so the problem stays feasible.

Typical use:
    mpc = UnicycleMPC(MPCConfig())
    v, w, info = mpc.solve(state=(x, y, th), v_now=v, w_now=w, path_xy=Nx2, obstacles=Mx4 [x,y,vx,vy])
"""
from __future__ import annotations

import math
import warnings
from dataclasses import dataclass

import numpy as np
from scipy.optimize import minimize


@dataclass
class MPCConfig:
    horizon: int = 12
    dt: float = 0.2
    v_max: float = 0.5
    v_min: float = 0.0            # no reversing by default
    w_max: float = 1.5
    a_max: float = 1.0            # m/s^2
    alpha_max: float = 3.0        # rad/s^2
    v_ref: float = 0.35
    q_pos: float = 4.0            # along-track error weight
    q_cross: float = 1.0          # cross-track weight: lower, so a sidestep is cheaper than stopping
    q_yaw: float = 0.5
    q_term: float = 10.0
    r_v: float = 0.1
    r_w: float = 0.1
    r_dv: float = 0.5
    r_dw: float = 0.3
    q_obs: float = 30.0
    r_hard: float = 0.30          # hard clearance, robot centre to obstacle point (m)
    r_soft: float = 0.50          # soft margin (m)
    max_obstacles: int = 6
    obstacle_range: float = 3.0
    max_iter: int = 25


def wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def rollout(x0, v, w, dt):
    """x0=(x,y,th); v,w arrays (N,). Returns (N+1, 3) states."""
    n = len(v)
    out = np.empty((n + 1, 3))
    out[0] = x0
    th = x0[2] + np.concatenate(([0.0], np.cumsum(w * dt)))
    # exact-in-heading Euler: use heading at the start of each step
    out[:, 2] = th
    out[1:, 0] = x0[0] + np.cumsum(v * np.cos(th[:-1]) * dt)
    out[1:, 1] = x0[1] + np.cumsum(v * np.sin(th[:-1]) * dt)
    return out


def sample_reference(path_xy: np.ndarray, pos, n: int, step: float):
    """n+1 reference points along the path starting at the path point closest to `pos`, spaced `step`
    metres apart (clamped to the last point). Returns (pts (n+1,2), headings (n+1,))."""
    path_xy = np.asarray(path_xy, float)
    if len(path_xy) == 1:
        pts = np.repeat(path_xy, n + 1, axis=0)
        return pts, np.zeros(n + 1)
    seg = np.diff(path_xy, axis=0)
    seg_len = np.hypot(seg[:, 0], seg[:, 1])
    cum = np.concatenate(([0.0], np.cumsum(seg_len)))
    i0 = int(np.argmin(np.hypot(path_xy[:, 0] - pos[0], path_xy[:, 1] - pos[1])))
    s = cum[i0] + step * np.arange(n + 1)
    s = np.minimum(s, cum[-1])
    x = np.interp(s, cum, path_xy[:, 0])
    y = np.interp(s, cum, path_xy[:, 1])
    pts = np.stack([x, y], axis=1)
    hd = np.arctan2(np.gradient(y), np.gradient(x))
    if cum[-1] - cum[i0] < 1e-6:
        hd[:] = 0.0
    return pts, hd


class UnicycleMPC:
    def __init__(self, cfg: MPCConfig | None = None):
        self.cfg = cfg or MPCConfig()
        self._u_prev = None       # (2N,) previous solution [v..., w...]

    def reset(self):
        self._u_prev = None

    def solve(self, state, v_now, w_now, path_xy, obstacles=None):
        c = self.cfg
        n, dt = c.horizon, c.dt
        x0 = np.asarray(state, float)
        pts, hd = sample_reference(path_xy, x0[:2], n, c.v_ref * dt)
        goal_d = float(np.hypot(*(np.asarray(path_xy)[-1] - x0[:2])))

        obs = self._select_obstacles(x0, obstacles)           # (M, 4)
        # predicted obstacle positions per step: (n+1, M, 2)
        tk = (np.arange(n + 1) * dt)[:, None, None]
        o_pred = obs[None, :, :2] + tk * obs[None, :, 2:4] if len(obs) else np.empty((n + 1, 0, 2))

        d_now = np.hypot(obs[:, 0] - x0[0], obs[:, 1] - x0[1]) if len(obs) else np.empty(0)
        r_hard = np.minimum(c.r_hard, np.maximum(d_now - 0.02, 0.0)) if len(obs) else d_now

        def split(u):
            return u[:n], u[n:]

        def cost(u):
            v, w = split(u)
            st = rollout(x0, v, w, dt)
            e = st[1:, :2] - pts[1:]
            ca, sa = np.cos(hd[1:]), np.sin(hd[1:])
            e_al = ca * e[:, 0] + sa * e[:, 1]
            e_cr = -sa * e[:, 0] + ca * e[:, 1]
            j = c.q_pos * np.sum(e_al * e_al) + c.q_cross * np.sum(e_cr * e_cr)
            j += c.q_yaw * np.sum(wrap(st[1:, 2] - hd[1:]) ** 2)
            j += c.q_term * np.sum((st[-1, :2] - pts[-1]) ** 2)
            j += c.r_v * np.sum((v - c.v_ref) ** 2) + c.r_w * np.sum(w * w)
            dv = np.diff(np.concatenate(([v_now], v)))
            dw = np.diff(np.concatenate(([w_now], w)))
            j += c.r_dv * np.sum(dv * dv) + c.r_dw * np.sum(dw * dw)
            if len(obs):
                d = np.linalg.norm(st[1:, None, :2] - o_pred[1:], axis=2)
                j += c.q_obs * np.sum(np.maximum(c.r_soft - d, 0.0) ** 2)
            return j

        cons = []
        if len(obs):
            def clearance(u):
                v, w = split(u)
                st = rollout(x0, v, w, dt)
                d = np.linalg.norm(st[1:, None, :2] - o_pred[1:], axis=2)   # (n, M)
                return (d - r_hard[None, :]).ravel()
            cons.append({'type': 'ineq', 'fun': clearance})

        # |dv|, |dw| limits are linear in u: build the constraint matrix once (exact Jacobian)
        D = np.eye(n) - np.eye(n, k=-1)                       # dv_k = v_k - v_{k-1}
        A_acc = np.zeros((4 * n, 2 * n))
        A_acc[:n, :n], A_acc[n:2 * n, :n] = -D, D
        A_acc[2 * n:3 * n, n:], A_acc[3 * n:, n:] = -D, D

        def accel(u):
            v, w = split(u)
            dv = v - np.concatenate(([v_now], v[:-1]))
            dw = w - np.concatenate(([w_now], w[:-1]))
            return np.concatenate([c.a_max * dt - dv, c.a_max * dt + dv,
                                   c.alpha_max * dt - dw, c.alpha_max * dt + dw])
        cons.append({'type': 'ineq', 'fun': accel, 'jac': lambda u: A_acc})

        bounds = [(c.v_min, c.v_max)] * n + [(-c.w_max, c.w_max)] * n

        if self._u_prev is not None and len(self._u_prev) == 2 * n:
            vp, wp = split(self._u_prev)
            u0 = np.concatenate([np.append(vp[1:], vp[-1]), np.append(wp[1:], wp[-1])])
        else:
            u0 = np.concatenate([np.full(n, min(c.v_ref, max(v_now, 0.1))), np.zeros(n)])
        # scale speed down near the goal so it does not overshoot
        if goal_d < c.v_ref * dt * n:
            u0[:n] = np.minimum(u0[:n], max(goal_d / (n * dt), 0.0))
        u0 = np.clip(u0, [b[0] for b in bounds], [b[1] for b in bounds])

        # Multi-start: an obstacle dead ahead on a straight path is a symmetric local minimum
        # (braking beats picking a side), so also try a left- and a right-curving initial guess
        # and keep the best valid solution.
        def run(us):
            with warnings.catch_warnings():
                warnings.simplefilter('ignore', RuntimeWarning)   # SLSQP clipping to bounds
                r = minimize(cost, us, method='SLSQP', bounds=bounds, constraints=cons,
                             options={'maxiter': c.max_iter, 'ftol': 1e-4})
            good = np.all(np.isfinite(r.x)) and (bool(r.success) or r.status == 9)
            if good and len(obs) and np.min(clearance(r.x)) < -1e-3:
                good = False
            return r, good

        def tight(r):
            """True when the plan still squeezes an obstacle: worth trying other sides."""
            d = np.linalg.norm(rollout(x0, r.x[:n], r.x[n:], dt)[1:, None, :2] - o_pred[1:], axis=2)
            return bool(np.min(d) < c.r_soft)

        best, r = None, None
        r, good = run(u0)
        if good:
            best = r
        stalled = good and goal_d > 1.0 and float(np.mean(r.x[:n])) < 0.3 * c.v_ref
        if len(obs) and (not good or tight(r) or stalled):
            for sign in (1.0, -1.0):
                # S-curve at reference speed: swing out, swing back, then straight. Starting from a
                # slow guess lets the solver collapse to v=0 (where heading has no effect).
                us = u0.copy()
                us[:n] = c.v_ref
                k3 = max(1, n // 3)
                prof = np.zeros(n)
                prof[:k3], prof[k3:2 * k3] = 0.6 * c.w_max, -0.6 * c.w_max
                us[n:] = sign * prof
                us = np.clip(us, [b[0] for b in bounds], [b[1] for b in bounds])
                r2, good2 = run(us)
                if good2 and (best is None or r2.fun < best.fun):
                    best = r2
        if best is None:
            res, ok, u = r, False, u0
        else:
            res, ok, u = best, True, best.x
        if not ok:
            # safe fallback: brake along the previous heading plan
            v_cmd = max(0.0, v_now - c.a_max * dt)
            self._u_prev = None
            return v_cmd, 0.0, {'ok': False, 'cost': float(res.fun), 'msg': str(res.message),
                                'goal_dist': goal_d, 'n_obstacles': len(obs)}
        self._u_prev = u
        v, w = split(u)
        pred = rollout(x0, v, w, dt)[:, :2]
        return float(v[0]), float(w[0]), {'ok': True, 'cost': float(res.fun), 'goal_dist': goal_d,
                                           'n_obstacles': len(obs), 'pred': pred,
                                           'iters': int(res.nit)}

    def _select_obstacles(self, x0, obstacles):
        if obstacles is None or len(obstacles) == 0:
            return np.empty((0, 4))
        o = np.asarray(obstacles, float).reshape(-1, 4 if np.ndim(obstacles) == 2 and
                                                  np.shape(obstacles)[1] >= 4 else 2)
        if o.shape[1] == 2:
            o = np.hstack([o, np.zeros((len(o), 2))])
        d = np.hypot(o[:, 0] - x0[0], o[:, 1] - x0[1])
        keep = np.where(d < self.cfg.obstacle_range)[0]
        keep = keep[np.argsort(d[keep])][: self.cfg.max_obstacles]
        return o[keep]
