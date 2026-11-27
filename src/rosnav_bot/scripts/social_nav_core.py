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


try:
    from numba import njit
except ImportError:  # the numpy loops below score the same hits
    njit = None


if njit is not None:
    @njit(cache=False)
    def _aligned_hits(traj, S, thresh2):
        """Same-time gap and hit mask. traj (C,T,2), S (K,N,T,2)."""
        C, T, _ = traj.shape
        K, N, _, _ = S.shape
        min_d2 = np.empty((C, K))
        hit_t = np.zeros((C, K, T), dtype=np.bool_)
        for c in range(C):
            for k in range(K):
                md = 1e300
                for tr in range(T):
                    x, y = traj[c, tr, 0], traj[c, tr, 1]
                    close = False
                    for n in range(N):
                        dx = x - S[k, n, tr, 0]
                        dy = y - S[k, n, tr, 1]
                        d2 = dx * dx + dy * dy
                        if d2 < md:
                            md = d2
                        if d2 < thresh2[c]:
                            close = True
                    if close:
                        hit_t[c, k, tr] = True
                min_d2[c, k] = md
        return min_d2, hit_t

    @njit(cache=False)
    def _union_hits(traj, S, thresh2):
        """Hit if any robot time is within thresh of any sample time."""
        C, T, _ = traj.shape
        K, N, _, _ = S.shape
        hit = np.zeros((C, K), dtype=np.bool_)
        for c in range(C):
            for k in range(K):
                found = False
                for n in range(N):
                    if found:
                        break
                    for tr in range(T):
                        if found:
                            break
                        x, y = traj[c, tr, 0], traj[c, tr, 1]
                        for tp in range(T):
                            dx = x - S[k, n, tp, 0]
                            dy = y - S[k, n, tp, 1]
                            if dx * dx + dy * dy < thresh2[c]:
                                found = True
                                break
                hit[c, k] = found
        return hit
else:
    _aligned_hits = None


def _plan_extra(cands, pos, ped_pos, ped_vel, base=0.15):
    """Extra disc per candidate, in metres. Never above `base`.

    The fixed conformal schedule is not widened. A candidate that is not
    closing on a person keeps a smaller disc. These pedestrians do not react,
    so this is not a radius rebuilt from a plan-conditioned human simulator.
    """
    rel = np.asarray(ped_pos, float) - np.asarray(pos, float)
    direction = rel / np.maximum(np.linalg.norm(rel, axis=1, keepdims=True), 1e-6)
    closing = np.sum((cands[:, None, :] - np.asarray(ped_vel, float)[None]) * direction[None], axis=-1)
    peak = np.maximum(closing, 0.0).max(axis=1)
    return base * np.clip(peak / V_MAX, 0.0, 1.0)


def _thresh_per_candidate(thresh, n):
    """Scalar disc, or one disc per candidate. Never broadcast by accident."""
    th = np.asarray(thresh, dtype=np.float64).reshape(-1)
    if th.size == 1:
        th = np.full(n, float(th[0]))
    if th.size != n:
        raise ValueError(f'thresh length {th.size} != {n} candidates')
    return np.ascontiguousarray(th)


def _hits_numpy(traj, S, thresh):
    """Same masks as _aligned_hits / _union_hits, without the (C,K,N,T) temp."""
    C, T, _ = traj.shape
    K = S.shape[0]
    th = _thresh_per_candidate(thresh, C)
    min_d = np.full((C, K), np.inf)
    hit_t = np.zeros((C, K, T), dtype=bool)
    hit_u = np.zeros((C, K), dtype=bool)
    thresh2 = th * th
    a2 = np.sum(traj * traj, axis=-1)
    for n in range(S.shape[1]):
        ped = S[:, n]
        aligned = np.linalg.norm(traj[:, None] - ped[None], axis=-1)    # (C,K,T)
        np.minimum(min_d, aligned.min(axis=-1), out=min_d)
        hit_t |= aligned < th[:, None, None]
        b2 = np.sum(ped * ped, axis=-1)
        dot = (traj.reshape(-1, 2) @ ped.reshape(-1, 2).T).reshape(C, T, K, T)
        d2 = a2[:, :, None, None] + b2[None, None, :, :] - 2.0 * dot
        hit_u |= (d2 < thresh2[:, None, None, None]).any(axis=(1, 3))
    return min_d, hit_t, hit_u


def _collision_risk(traj, S, thresh, mode, horizon_discount):
    """Collision probability and closest time-aligned gap for each candidate.

    traj is (C, T, 2), S is (K, N, T, 2). 'any' is a hit at the same step.
    'discount' weights that hit from 1 down to horizon_discount. 'union' is a
    hit at any pair of times. min_dist is (C, K), the nearest same-time gap.
    thresh is one disc radius, or one per candidate.
    """
    T = traj.shape[1]
    th = _thresh_per_candidate(thresh, traj.shape[0])
    if _aligned_hits is not None:
        traj_c = np.ascontiguousarray(traj, dtype=np.float64)
        S_c = np.ascontiguousarray(S, dtype=np.float64)
        th2 = np.ascontiguousarray(th * th)
        min_d2, hit_t = _aligned_hits(traj_c, S_c, th2)
        min_d = np.sqrt(min_d2)
        if mode == 'union':
            risk = _union_hits(traj_c, S_c, th2).mean(axis=1)
            return risk, min_d
    else:
        min_d, hit_t, hit_u = _hits_numpy(traj, S, th)
        if mode == 'union':
            return hit_u.mean(axis=1), min_d
    if mode == 'discount':
        w = np.linspace(1.0, float(horizon_discount), T)
        risk = (hit_t * w).sum(axis=-1).mean(axis=1) / w.sum()
    else:
        risk = hit_t.any(axis=-1).mean(axis=1)
    return risk, min_d


class RiskPlanner:
    """DRA-MPPI-flavoured: Monte Carlo collision probability over *joint*
    pedestrian futures (uncertainty grows with lookahead), minimised together
    with goal distance. No hard reject -> no freezing robot."""
    name = 'risk'

    def __init__(self, horizon_s=3.0, samples=24, w_risk=10.0, w_goal=1.0,
                 w_soc=1.0, w_commit=0.6, w_prog=1.5, sigma_scale=1.0,
                 conformal_radii=None, clear_fn=None, w_wall=8.0, wall_hard=0.34,
                 wall_soft=0.7, seed=0, horizon_discount=None, adapt_when=None,
                 adapt_scale=0.25, union=False, plan_radius=False, name='risk'):
        # clear_fn: optional (...,2) -> distance-to-nearest-wall (m); candidates whose rollout
        # comes closer than wall_hard are rejected, closer than wall_soft are penalised.
        # horizon_discount: weight of a hit at the last step (1 at the first). None keeps the
        # any-time collision probability. adapt_when: if every moving candidate is at least this
        # risky, multiply w_risk by adapt_scale for that cycle. union: a hit is any spatial
        # overlap of the rollout with a pedestrian sample, ignoring time.
        # plan_radius: the extra 0.15 m disc shrinks when this candidate is not
        # closing on anyone, and never grows past 0.15 m. The crowd does not react.
        self.clear_fn, self.ww, self.wall_hard, self.wall_soft = clear_fn, w_wall, wall_hard, wall_soft
        self.h, self.K, self.wr, self.wg, self.ws = horizon_s, samples, w_risk, w_goal, w_soc
        self.wc, self.wp, self.sscale = w_commit, w_prog, sigma_scale
        self.horizon_discount, self.adapt_when, self.adapt_scale = horizon_discount, adapt_when, adapt_scale
        self.union, self.plan_radius = union, plan_radius
        self.name = name
        self.prev_v = np.zeros(2)         # commitment: bias toward last chosen velocity
        # per-step uncertainty: conformal quantile radii (r_k ~ 2.15 sigma for a 2D
        # isotropic Gaussian at alpha=0.1) when given, else a hand-set linear growth.
        self.conf = None if conformal_radii is None else np.asarray(conformal_radii) / 2.146
        self.rng = np.random.default_rng(seed)
        ang = np.linspace(-math.pi, math.pi, 25, endpoint=False)
        d = np.stack([np.cos(ang), np.sin(ang)], 1)
        sp = np.array([0.0, 0.3, 0.55, V_MAX])
        self.cands = (d[None] * sp[:, None, None]).reshape(-1, 2)      # (C,2)

    def plan(self, pos, goal, ped_pos, ped_vel, preds=None, futures=None):
        # futures: optional (K,N,T,2) array of SAMPLED pedestrian futures at DT spacing (e.g. from a
        # multimodal predictor, predictor_multimodal.py); when given they replace the Gaussian-around-prediction
        # Monte Carlo samples, so several plausible paths per person enter the collision risk.
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
        if futures is not None:
            S = np.asarray(futures, float)[:, :, :T]                    # (K,N,T,2) sampled futures
        else:
            if preds is None:
                preds = [predict_cv(p, v, self.h) for p, v in zip(ped_pos, ped_vel)]
            P = np.stack([q[:T] for q in preds])                        # (N,T,2)
            if self.conf is not None:   # interpolate conformal sigma_k (0.4 s grid) to DT
                sig = np.interp(t[:, 0], (np.arange(len(self.conf)) + 1) * 0.4, self.conf)
                sigma = (self.sscale * sig)[None, None, :, None]
            else:
                sigma = (self.sscale * (0.10 + 0.22 * t))[None, None]       # (1,1,T,1)
            S = P[None] + self.rng.normal(size=(self.K,) + P.shape) * sigma  # (K,N,T,2)
        extra = _plan_extra(self.cands, pos, ped_pos, ped_vel) if self.plan_radius else 0.15
        thresh = ROBOT_R + PED_R + extra
        # Same scores as a (C,K,N,T) broadcast, one person at a time. The full
        # array is ~C*K*N*T and was large enough to stall this 24-person compare.
        mode = 'union' if self.union else 'discount' if self.horizon_discount is not None else 'any'
        risk, min_dist = _collision_risk(traj, S, thresh, mode, self.horizon_discount)
        wr = self.wr
        if self.adapt_when is not None:
            moving = np.linalg.norm(self.cands, axis=1) > 0.05
            if moving.any() and float(risk[moving].min()) >= self.adapt_when:
                wr = self.wr * self.adapt_scale
        intr = np.maximum(0.0, PERSONAL_SPACE - min_dist).mean(axis=1)
        gdir = (goal - pos) / max(np.linalg.norm(goal - pos), 1e-6)
        cost = (self.wg * np.linalg.norm(goal - traj[:, -1], axis=1)
                + wr * risk + self.ws * 3.0 * intr + wall
                - self.wp * (self.cands @ gdir)                          # progress
                + self.wc * np.linalg.norm(self.cands - self.prev_v, axis=1))  # commitment
        self.last = dict(cost=cost, risk=risk)       # exposed for visualisation (render_social_compare.py)
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
    if hasattr(planner, 'prev_v'):
        planner.prev_v = np.zeros(2)
    if hasattr(planner, 'rng'):
        planner.rng = np.random.default_rng(seed)
    start_gap = float(np.linalg.norm(goal - pos))
    log = dict(collided=False, min_clear=1e9, intrusion_s=0.0, jerk=0.0,
               reached=False, t=max_t, path_m=0.0, stalled_s=0.0, speed_sum=0.0)
    for k in range(int(max_t / DT)):
        # pedestrians bounce off a 14x8 box (keeps them in play)
        for i in range(len(ps)):
            ps[i] += pv[i] * DT
            if abs(ps[i][1]) > 4: pv[i][1] *= -1
            if ps[i][0] < -1 or ps[i][0] > 11: pv[i][0] *= -1
        v = planner.plan(pos, goal, ps, pv)
        if use_filter:
            v = safety_filter(v, pos, ps)
        speed = float(np.linalg.norm(v))
        log['jerk'] += np.linalg.norm(v - prev_v) / DT
        log['path_m'] += speed * DT
        log['speed_sum'] += speed
        if speed < 0.05:
            log['stalled_s'] += DT
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
    steps = max(k + 1, 1)
    log['jerk'] /= steps
    log['mean_speed'] = log['speed_sum'] / steps
    log['progress'] = float(np.clip(1.0 - np.linalg.norm(pos - goal) / max(start_gap, 1e-6), 0.0, 1.0))
    log['timeout'] = not log['reached']
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


def risk_variants(samples=12, sigma_scale=0.5, seed=0):
    """Current risk, a horizon discount, an adaptive risk weight, and a time-ignored union mask."""
    common = dict(samples=samples, sigma_scale=sigma_scale, seed=seed)
    return [
        RiskPlanner(name='risk', **common),
        RiskPlanner(name='discount', horizon_discount=0.15, **common),
        RiskPlanner(name='adapt', adapt_when=0.4, adapt_scale=0.25, **common),
        RiskPlanner(name='union', union=True, **common),
    ]


def compare_risk(seeds=8, n_crowd=24, max_t=20.0, samples=12):
    """Same crowd seeds for each risk variant. Arrival, timeout, and stall are separate."""
    rows = []
    for pl in risk_variants(samples=samples):
        runs = [simulate(pl, 'crowd', s, max_t=max_t, n_crowd=n_crowd) for s in range(seeds)]
        ok = [r for r in runs if r['reached']]
        rows.append(dict(
            planner=pl.name, peds=n_crowd, seeds=seeds, seconds=max_t,
            collision_rate=float(np.mean([r['collided'] for r in runs])),
            success=float(np.mean([r['reached'] for r in runs])),
            timeout=float(np.mean([r['timeout'] for r in runs])),
            stalled_s=float(np.mean([r['stalled_s'] for r in runs])),
            progress=float(np.mean([r['progress'] for r in runs])),
            mean_speed=float(np.mean([r['mean_speed'] for r in runs])),
            path_m=float(np.mean([r['path_m'] for r in runs])),
            intrusion_s=float(np.mean([r['intrusion_s'] for r in runs])),
            min_clearance=float(np.mean([r['min_clear'] for r in runs])),
            time_success_s=float(np.mean([r['t'] for r in ok])) if ok else float('nan')))
        print(rows[-1]['planner'], 'coll', round(rows[-1]['collision_rate'], 2),
              'succ', round(rows[-1]['success'], 2),
              'stall', round(rows[-1]['stalled_s'], 1),
              'prog', round(rows[-1]['progress'], 2),
              'clear', round(rows[-1]['min_clearance'], 2), flush=True)
    return rows


if __name__ == '__main__':
    import sys
    if 'compare' in sys.argv:
        for r in compare_risk():
            print(r['planner'].ljust(9),
                  f"coll={r['collision_rate']:.2f} succ={r['success']:.2f}",
                  f"timeout={r['timeout']:.2f} stall={r['stalled_s']:.1f}s",
                  f"prog={r['progress']:.2f} v={r['mean_speed']:.2f}",
                  f"path={r['path_m']:.1f} intr={r['intrusion_s']:.1f}",
                  f"clear={r['min_clearance']:.2f} t_ok={r['time_success_s']:.1f}")
        sys.exit(0)
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
