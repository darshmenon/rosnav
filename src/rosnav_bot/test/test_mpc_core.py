#!/usr/bin/env python3
"""Closed-loop checks of scripts/mpc_core.py: a kinematic unicycle driven by the MPC, no ROS."""
import importlib.util
import math
import os
import sys
import time

import numpy as np

_SCRIPT = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'mpc_core.py')
_spec = importlib.util.spec_from_file_location('mpc_core', _SCRIPT)
mpc_core = importlib.util.module_from_spec(_spec)
sys.modules['mpc_core'] = mpc_core     # @dataclass needs the module registered
_spec.loader.exec_module(mpc_core)

DT_SIM = 0.1


def _line(x0, y0, x1, y1, n=80):
    return np.c_[np.linspace(x0, x1, n), np.linspace(y0, y1, n)]


def _run(path, obstacles_fn, start=(0.0, 0.0, 0.0), steps=200, cfg=None):
    """Returns (trajectory (T,3), min clearance, median solve ms, all solves ok)."""
    mpc = mpc_core.UnicycleMPC(cfg or mpc_core.MPCConfig())
    x = np.array(start, float)
    v = w = 0.0
    traj, clr, times, oks = [x.copy()], [], [], []
    for k in range(steps):
        obs = obstacles_fn(k * DT_SIM)
        t0 = time.process_time()
        v, w, info = mpc.solve(x, v, w, path, obs)
        times.append((time.process_time() - t0) * 1e3)
        oks.append(info['ok'])
        x = x + DT_SIM * np.array([v * math.cos(x[2]), v * math.sin(x[2]), w])
        traj.append(x.copy())
        if obs is not None and len(obs):
            o = np.asarray(obs)
            clr.append(float(np.min(np.hypot(o[:, 0] - x[0], o[:, 1] - x[1]))))
        if np.hypot(*(path[-1] - x[:2])) < 0.1:
            break
    return np.array(traj), (min(clr) if clr else None), float(np.median(times)), all(oks)


def test_tracks_a_straight_path_and_stops_at_the_goal():
    path = _line(0, 0, 4, 0)
    traj, _, ms, ok = _run(path, lambda t: None)
    assert np.hypot(*(path[-1] - traj[-1, :2])) < 0.15
    assert np.max(np.abs(traj[:, 1])) < 0.1
    assert ok


def test_recovers_from_a_lateral_offset():
    path = _line(0, 0, 5, 0)
    traj, _, _, _ = _run(path, lambda t: None, start=(0.0, 0.6, 0.0), steps=120)
    assert abs(traj[-1, 1]) < 0.15


def test_goes_around_a_static_obstacle_on_the_path():
    path = _line(0, 0, 6, 0)
    obs = np.array([[3.0, 0.0, 0.0, 0.0]])
    traj, clr, _, _ = _run(path, lambda t: obs)
    assert clr >= 0.28                     # r_hard = 0.30, small slack for the 0.1 s sim step
    assert np.hypot(*(path[-1] - traj[-1, :2])) < 0.3


def test_yields_to_a_crossing_moving_obstacle():
    path = _line(0, 0, 6, 0)

    def crossing(t):                       # walks across the path at 0.5 m/s from y=+2
        return np.array([[2.5, 2.0 - 0.5 * t, 0.0, -0.5]])
    traj, clr, _, _ = _run(path, crossing)
    assert clr >= 0.28


def test_starting_inside_the_hard_radius_stays_feasible():
    path = _line(0, 0, 4, 0)
    obs = np.array([[0.2, 0.1, 0.0, 0.0]])    # closer than r_hard = 0.30
    mpc = mpc_core.UnicycleMPC()
    v, w, info = mpc.solve((0.0, 0.0, 0.0), 0.0, 0.0, path, obs)
    assert math.isfinite(v) and math.isfinite(w)


def test_respects_velocity_and_acceleration_limits():
    cfg = mpc_core.MPCConfig()
    path = _line(0, 0, 5, 0)
    mpc = mpc_core.UnicycleMPC(cfg)
    v, w, _ = mpc.solve((0.0, 0.0, 0.0), 0.0, 0.0, path, None)
    assert cfg.v_min - 1e-6 <= v <= cfg.v_max + 1e-6
    assert v <= cfg.a_max * cfg.dt + 1e-3          # first step limited by accel from standstill
    assert abs(w) <= cfg.w_max + 1e-6


def test_solve_time_is_realtime_capable():
    path = _line(0, 0, 6, 0)
    obs = np.array([[2.0, 0.1, 0.0, 0.0], [3.0, -0.5, 0.0, 0.0], [2.5, 1.0, 0.0, 0.0]])
    _, _, ms, _ = _run(path, lambda t: obs, steps=30)
    assert ms < 120.0                              # median CPU ms per solve; control period is 100-200 ms


def test_sidesteps_an_obstacle_seen_only_by_its_surface_points():
    """Scan-like input: a few points on the face of a small obstacle dead ahead on the path.
    Regression: all starts used to collapse to v=0 and the robot stopped in front of it."""
    path = _line(0, 0, 6, 0)
    ang = np.linspace(math.pi - 0.9, math.pi + 0.9, 8)
    obs = np.c_[2.5 + 0.15 * np.cos(ang), 0.15 * np.sin(ang), np.zeros(8), np.zeros(8)]
    traj, clr, _, _ = _run(path, lambda t: obs, steps=300)
    assert clr >= 0.28
    assert np.hypot(*(path[-1] - traj[-1, :2])) < 0.3
