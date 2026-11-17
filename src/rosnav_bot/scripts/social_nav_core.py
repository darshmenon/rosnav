#!/usr/bin/env python3
"""
social_nav_core.py — pure-Python social-navigation core (no ROS, no sim).

Pieces, each usable from a ROS node later:
  * predict_cv()        constant-velocity pedestrian trajectory prediction
  * PredictivePlanner   picks a velocity that stays clear of *predicted*
                        pedestrian paths (velocity-sampling, time-indexed)
  * ReactivePlanner     baseline: goal-seek, stop when a person is close
  * safety_filter()     last-stage clamp between any planner and cmd_vel
                        (max speed, min clearance -> brake)
  * simulate()/metrics  crowd scenarios + collision rate, min clearance,
                        smoothness, time-to-goal, personal-space intrusion
                        seconds (the "social comfort" metric)

Learned predictors (Trajectron++, transformer) plug in by replacing
predict_cv() with anything returning the same (T, 2) array.
"""
import math
import random

import numpy as np

DT = 0.1
ROBOT_R = 0.30
PED_R = 0.25
PERSONAL_SPACE = 1.2   # m, centre-to-centre; Hall's "social distance" lower bound
V_MAX = 0.8


def predict_cv(pos, vel, horizon_s=3.0, dt=DT):
    """Constant-velocity prediction -> (T, 2) array of future positions."""
    t = np.arange(1, int(horizon_s / dt) + 1)[:, None] * dt
    return np.asarray(pos)[None, :] + t * np.asarray(vel)[None, :]


def safety_filter(v, robot_pos, peds, v_max=V_MAX, brake_dist=0.8):
    """Clamp speed; scale down linearly inside brake_dist of any pedestrian."""
    v = np.asarray(v, float)
    n = np.linalg.norm(v)
    if n > v_max:
        v = v / n * v_max
    if len(peds):
        d = min(np.linalg.norm(robot_pos - p) for p in peds) - ROBOT_R - PED_R
        if d < brake_dist:
            v = v * max(d, 0.0) / brake_dist
    return v


class ReactivePlanner:
    name = 'reactive'

    def __init__(self, stop_dist=1.0):
        self.stop_dist = stop_dist

    def plan(self, pos, goal, ped_pos, ped_vel):
        to_goal = goal - pos
        dist = np.linalg.norm(to_goal)
        v = to_goal / max(dist, 1e-6) * min(V_MAX, dist)
        for p in ped_pos:
            if np.linalg.norm(p - pos) < self.stop_dist:
                return np.zeros(2)
        return v


class PredictivePlanner:
    """Sample candidate velocities; score by predicted clearance + goal progress."""
    name = 'predictive'

    def __init__(self, horizon_s=3.0, w_goal=1.0, w_social=2.0):
        self.h, self.wg, self.ws = horizon_s, w_goal, w_social
        ang = np.linspace(-math.pi, math.pi, 25, endpoint=False)
        self.dirs = np.stack([np.cos(ang), np.sin(ang)], 1)

    def plan(self, pos, goal, ped_pos, ped_vel, preds=None):
        # `preds`: optional list of (T,2) predicted paths at DT spacing from a
        # learned predictor; falls back to constant velocity.
        if preds is None:
            preds = [predict_cv(p, v, self.h) for p, v in zip(ped_pos, ped_vel)]
        t = np.arange(1, int(self.h / DT) + 1)[:, None] * DT
        best, best_cost = np.zeros(2), 1e9
        evade, evade_clear = np.zeros(2), -1.0
        for speed in (0.0, 0.3, 0.55, V_MAX):
            for d in self.dirs:
                v = d * speed
                traj = pos[None, :] + t * v[None, :]
                min_clear = 1e9
                for pr in preds:
                    min_clear = min(min_clear,
                                    np.linalg.norm(traj - pr, axis=1).min())
                if min_clear > evade_clear:
                    evade, evade_clear = v, min_clear
                if min_clear < ROBOT_R + PED_R + 0.1:
                    continue  # hard reject: predicted collision
                intrusion = max(0.0, PERSONAL_SPACE - min_clear)
                end = traj[-1]
                cost = (self.wg * np.linalg.norm(goal - end)
                        + self.ws * intrusion * 3.0)
                if cost < best_cost:
                    best, best_cost = v, cost
        # every candidate predicted a collision -> evade (max-clearance
        # velocity) instead of freezing in the pedestrian's path
        return best if best_cost < 1e9 else evade


class RiskPlanner:
    """DRA-MPPI-flavoured: Monte Carlo collision probability over *joint*
    pedestrian futures (uncertainty grows with lookahead), minimised together
    with goal distance. No hard reject -> no freezing robot."""
    name = 'risk'

    def __init__(self, horizon_s=3.0, samples=24, w_risk=10.0, w_goal=1.0,
                 w_soc=1.0, w_commit=0.6, w_prog=1.5, sigma_scale=1.0,
                 conformal_radii=None, clear_fn=None, w_wall=8.0, wall_hard=0.34,
                 wall_soft=0.7, seed=0):
        # clear_fn: optional (...,2) -> distance-to-nearest-wall (m); candidates whose rollout
        # comes closer than wall_hard are rejected, closer than wall_soft are penalised.
        self.clear_fn, self.ww, self.wall_hard, self.wall_soft = clear_fn, w_wall, wall_hard, wall_soft
        self.h, self.K, self.wr, self.wg, self.ws = horizon_s, samples, w_risk, w_goal, w_soc
        self.wc, self.wp, self.sscale = w_commit, w_prog, sigma_scale
        self.prev_v = np.zeros(2)         # commitment: bias toward last chosen velocity
        # per-step uncertainty: conformal quantile radii (r_k ~ 2.15 sigma for a 2D
        # isotropic Gaussian at alpha=0.1) when given, else a hand-set linear growth.
        self.conf = None if conformal_radii is None else np.asarray(conformal_radii) / 2.146
        self.rng = np.random.default_rng(seed)
        ang = np.linspace(-math.pi, math.pi, 25, endpoint=False)
        d = np.stack([np.cos(ang), np.sin(ang)], 1)
        sp = np.array([0.0, 0.3, 0.55, V_MAX])
        self.cands = (d[None] * sp[:, None, None]).reshape(-1, 2)      # (C,2)

    def plan(self, pos, goal, ped_pos, ped_vel, preds=None):
        T = int(self.h / DT)
        t = np.arange(1, T + 1)[:, None] * DT
        traj = pos[None, None, :] + t[None] * self.cands[:, None, :]    # (C,T,2)
        wall = np.zeros(len(self.cands))
        if self.clear_fn is not None:
            mc = self.clear_fn(traj).min(axis=1)                        # (C,) worst clearance on rollout
            wall = (self.ww * np.maximum(0.0, self.wall_soft - mc) / self.wall_soft
                    + 100.0 * (mc < self.wall_hard))
        if len(ped_pos) == 0:
            gd = np.linalg.norm(goal - traj[:, -1], axis=1)
            self.prev_v = self.cands[int(np.argmin(gd + wall))]
            return self.prev_v
        if preds is None:
            preds = [predict_cv(p, v, self.h) for p, v in zip(ped_pos, ped_vel)]
        P = np.stack([q[:T] for q in preds])                            # (N,T,2)
        if self.conf is not None:       # interpolate conformal sigma_k (0.4 s grid) to DT
            sig = np.interp(t[:, 0], (np.arange(len(self.conf)) + 1) * 0.4, self.conf)
            sigma = (self.sscale * sig)[None, None, :, None]
        else:
            sigma = (0.10 + 0.22 * t)[None, None]                       # (1,1,T,1)
        S = P[None] + self.rng.normal(size=(self.K,) + P.shape) * sigma  # (K,N,T,2)
        dist = np.linalg.norm(traj[:, None, None] - S[None], axis=-1)   # (C,K,N,T)
        hit = (dist < ROBOT_R + PED_R + 0.15).any(axis=(2, 3))          # (C,K) joint
        risk = hit.mean(axis=1)                                         # P(collision)
        intr = np.maximum(0.0, PERSONAL_SPACE - dist.min(axis=(2, 3))).mean(axis=1)
        gdir = (goal - pos) / max(np.linalg.norm(goal - pos), 1e-6)
        cost = (self.wg * np.linalg.norm(goal - traj[:, -1], axis=1)
                + self.wr * risk + self.ws * 3.0 * intr + wall
                - self.wp * (self.cands @ gdir)                          # progress
                + self.wc * np.linalg.norm(self.cands - self.prev_v, axis=1))  # commitment
        self.prev_v = self.cands[int(np.argmin(cost))]
        return self.prev_v


def make_scenario(kind, rng, n_crowd=8):
    """Returns (start, goal, ped_starts, ped_vels). 6x? m open area."""
    start, goal = np.array([0.0, 0.0]), np.array([10.0, 0.0])
    if kind == 'crossing':     # pedestrians cross the robot's path
        ps = [np.array([3.0 + 2 * i, 3.0 * (-1) ** i]) for i in range(4)]
        pv = [np.array([0.0, -1.2 * (-1) ** i]) for i in range(4)]
    elif kind == 'head_on':    # pedestrians walk toward the robot
        ps = [np.array([10.0, -0.6 + 0.6 * i]) for i in range(3)]
        pv = [np.array([-1.1, 0.0]) for _ in range(3)]
    else:                      # 'crowd': random flows
        ps = [np.array([rng.uniform(2, 9), rng.uniform(-3, 3)]) for _ in range(n_crowd)]
        pv = [np.array([rng.uniform(-1, 1), rng.uniform(-1, 1)]) * 0.9
              for _ in range(n_crowd)]
    return start, goal, ps, pv


def simulate(planner, kind, seed=0, max_t=40.0, use_filter=True, n_crowd=8):
    rng = random.Random(seed)
    pos, goal, ps, pv = make_scenario(kind, rng, n_crowd)
    ps = [p.copy() for p in ps]
    pv = [v + np.array([rng.gauss(0, .05), rng.gauss(0, .05)]) for v in pv]
    pos = pos.copy()
    prev_v = np.zeros(2)
    log = dict(collided=False, min_clear=1e9, intrusion_s=0.0, jerk=0.0,
               reached=False, t=max_t)
    for k in range(int(max_t / DT)):
        # pedestrians bounce off a 14x8 box (keeps them in play)
        for i in range(len(ps)):
            ps[i] += pv[i] * DT
            if abs(ps[i][1]) > 4: pv[i][1] *= -1
            if ps[i][0] < -1 or ps[i][0] > 11: pv[i][0] *= -1
        v = planner.plan(pos, goal, ps, pv)
        if use_filter:
            v = safety_filter(v, pos, ps)
        log['jerk'] += np.linalg.norm(v - prev_v) / DT
        prev_v = v
        pos = pos + v * DT
        d = min(np.linalg.norm(pos - p) for p in ps)
        log['min_clear'] = min(log['min_clear'], d - ROBOT_R - PED_R)
        if d < ROBOT_R + PED_R:
            log['collided'] = True
        if d < PERSONAL_SPACE:
            log['intrusion_s'] += DT
        if np.linalg.norm(pos - goal) < 0.3:
            log['reached'], log['t'] = True, k * DT
            break
    log['jerk'] /= max(k, 1)
    return log


def evaluate(planners, kinds=('crossing', 'head_on', 'crowd'), seeds=20):
    rows = []
    for pl in planners:
        for kind in kinds:
            runs = [simulate(pl, kind, s) for s in range(seeds)]
            rows.append(dict(
                planner=pl.name, scenario=kind,
                collision_rate=np.mean([r['collided'] for r in runs]),
                success=np.mean([r['reached'] for r in runs]),
                min_clearance=np.mean([r['min_clear'] for r in runs]),
                intrusion_s=np.mean([r['intrusion_s'] for r in runs]),
                smoothness=np.mean([r['jerk'] for r in runs]),
                time_s=np.mean([r['t'] for r in runs])))
    return rows


def density_sweep(planners, counts=(4, 8, 16, 24), seeds=15):
    """Crowd-size sweep -> rows of collision/success/intrusion per planner."""
    rows = []
    for pl in planners:
        for n in counts:
            runs = [simulate(pl, 'crowd', s, n_crowd=n) for s in range(seeds)]
            rows.append(dict(planner=pl.name, peds=n,
                             collision_rate=np.mean([r['collided'] for r in runs]),
                             success=np.mean([r['reached'] for r in runs]),
                             intrusion_s=np.mean([r['intrusion_s'] for r in runs]),
                             time_s=np.mean([r['t'] for r in runs])))
    return rows


if __name__ == '__main__':
    import sys
    if 'sweep' in sys.argv:
        for r in density_sweep([ReactivePlanner(), PredictivePlanner()]):
            print(r['planner'].ljust(11), f"peds={r['peds']:>2}",
                  f"coll={r['collision_rate']:.2f} succ={r['success']:.2f}",
                  f"intr={r['intrusion_s']:.1f}s t={r['time_s']:.1f}s")
        sys.exit(0)
    rows = evaluate([ReactivePlanner(), PredictivePlanner()])
    hdr = ['planner', 'scenario', 'collision_rate', 'success', 'min_clearance',
           'intrusion_s', 'smoothness', 'time_s']
    print(' | '.join(f'{h:>14}' for h in hdr))
    for r in rows:
        print(' | '.join(f'{r[h]:>14.2f}' if not isinstance(r[h], str)
                         else f'{r[h]:>14}' for h in hdr))
