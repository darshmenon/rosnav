#!/usr/bin/env python3
"""
eval_learned_planner.py — reactive vs constant-velocity-predictive vs
learned-predictor planner, in *social-force* crowds (the same dynamics the
learned model was trained on; pedestrians also repel from the robot).

  python3 eval_learned_planner.py --weights ../../../outputs/ped_predictor.pt
"""
import argparse
import os
import sys

import numpy as np
import torch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import social_nav_core as s            # noqa: E402
import train_ped_predictor as tp       # noqa: E402

BOX, SIM_DT = 10.0, s.DT


class Crowd:
    """Social-force pedestrians stepped at SIM_DT; the robot is a repeller."""

    def __init__(self, n, rng, robot_start):
        self.rng = rng
        self.pos = rng.uniform(0, BOX, (n, 2))
        for i in range(n):                       # keep the start area clear
            while np.linalg.norm(self.pos[i] - robot_start) < 1.5:
                self.pos[i] = rng.uniform(0, BOX, 2)
        self.goal = rng.uniform(0, BOX, (n, 2))
        self.vel = np.zeros((n, 2))
        self.vdes = rng.uniform(0.8, 1.4, n)

    def step(self, robot):
        n = len(self.pos)
        g = self.goal - self.pos
        gd = np.linalg.norm(g, axis=1, keepdims=True) + 1e-6
        f = (g / gd * self.vdes[:, None] - self.vel) / 0.5
        d = self.pos[:, None] - self.pos[None]
        dist = np.linalg.norm(d, axis=2) + np.eye(n) * 1e3
        f += (2.0 * np.exp((0.6 - dist) / 0.4)[..., None] * d / dist[..., None]).sum(1)
        dr = self.pos - robot
        dd = np.linalg.norm(dr, axis=1, keepdims=True) + 1e-6
        f += 2.0 * np.exp((0.6 - dd) / 0.4) * dr / dd        # react to robot
        self.vel += f * SIM_DT
        sp = np.linalg.norm(self.vel, axis=1, keepdims=True)
        self.vel *= np.minimum(1.0, 1.8 / np.maximum(sp, 1e-6))
        self.pos = np.clip(self.pos + self.vel * SIM_DT, 0, BOX)
        sw = np.linalg.norm(self.goal - self.pos, axis=1) < 0.6
        self.goal[sw] = self.rng.uniform(0, BOX, (sw.sum(), 2))


class LearnedPredictor:
    def __init__(self, weights, device='cpu'):
        self.net = tp.SocialTransformer().to(device)
        self.net.load_state_dict(torch.load(weights, map_location=device))
        self.net.eval()
        self.dev = device

    @torch.no_grad()
    def __call__(self, hist, horizon_s):
        """hist: (n, 8, 2) at 0.4 s -> list of (T,2) at SIM_DT spacing."""
        n = len(hist)
        x = torch.zeros(1, tp.NMAX, tp.OBS, 2)
        m = torch.zeros(1, tp.NMAX, dtype=torch.bool)
        k = min(n, tp.NMAX)
        x[0, :k] = torch.tensor(hist[:k], dtype=torch.float32)
        m[0, :k] = True
        y = self.net(x.to(self.dev), m.to(self.dev))[0].cpu().numpy()  # (N,12,2)
        t_src = np.arange(0, tp.PRED + 1) * tp.DT
        t_dst = np.arange(1, int(horizon_s / SIM_DT) + 1) * SIM_DT
        out = []
        for i in range(n):
            if i >= k:                                  # beyond NMAX: CV fallback
                out.append(None); continue
            path = np.vstack([hist[i, -1], y[i]])
            out.append(np.stack([np.interp(t_dst, t_src, path[:, c]) for c in (0, 1)], 1))
        return out


def run(planner, mode, n, seed, predictor=None, max_t=40.0):
    rng = np.random.default_rng(seed)
    pos, goal = np.array([0.5, 5.0]), np.array([9.5, 5.0])
    crowd = Crowd(n, rng, pos)
    hist = [crowd.pos.copy() for _ in range(tp.OBS * 4)]      # 0.1s ring buffer
    prev_v, intr, jerk, col, mn = np.zeros(2), 0.0, 0.0, False, 1e9
    k = 0
    for k in range(int(max_t / SIM_DT)):
        crowd.step(pos)
        hist.append(crowd.pos.copy()); hist.pop(0)
        preds = None
        if mode == 'learned':
            h = np.stack(hist[::-1][0::4][:tp.OBS][::-1], 1)
            preds = predictor(h, planner.h)
            preds = [p if p is not None else s.predict_cv(crowd.pos[i], crowd.vel[i], planner.h)
                     for i, p in enumerate(preds)]
        v = planner.plan(pos, goal, crowd.pos, crowd.vel, **({'preds': preds} if preds else {})) \
            if mode != 'reactive' else planner.plan(pos, goal, crowd.pos, crowd.vel)
        v = s.safety_filter(v, pos, crowd.pos)
        jerk += np.linalg.norm(v - prev_v) / SIM_DT; prev_v = v
        pos = pos + v * SIM_DT
        d = np.linalg.norm(crowd.pos - pos, axis=1).min()
        mn = min(mn, d - s.ROBOT_R - s.PED_R)
        col |= d < s.ROBOT_R + s.PED_R
        intr += SIM_DT * (d < s.PERSONAL_SPACE)
        if np.linalg.norm(pos - goal) < 0.3:
            return dict(col=col, ok=True, t=k * SIM_DT, intr=intr, mn=mn, jerk=jerk / max(k, 1))
    return dict(col=col, ok=False, t=max_t, intr=intr, mn=mn, jerk=jerk / max(k, 1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', default='outputs/ped_predictor.pt')
    ap.add_argument('--seeds', type=int, default=12)
    ap.add_argument('--only', nargs='*', default=[])
    ap.add_argument('--counts', type=int, nargs='+', default=[8, 16, 24])
    a = ap.parse_args()
    pred = LearnedPredictor(a.weights)
    import json
    cj = a.weights + '.conformal.json'
    conf = json.load(open(cj))['radii'] if os.path.exists(cj) else None
    setups = [('reactive', s.ReactivePlanner(), 'reactive'),
              ('cv-predictive', s.PredictivePlanner(), 'cv'),
              ('learned-predictive', s.PredictivePlanner(), 'learned'),
              ('cv-risk', s.RiskPlanner(conformal_radii=conf, sigma_scale=0.5), 'cv'),
              ('learned-risk', s.RiskPlanner(conformal_radii=conf, sigma_scale=0.5), 'learned')]
    if a.only:
        setups = [x for x in setups if x[0] in a.only]
    print(f"{'planner':<20}{'peds':>5}{'coll%':>7}{'succ%':>7}{'clear_m':>9}{'intr_s':>8}{'jerk':>7}{'t_s':>7}")
    for n in a.counts:
        for name, pl, mode in setups:
            r = [run(pl, mode, n, sd, pred) for sd in range(a.seeds)]
            f = lambda k: np.mean([x[k] for x in r])
            print(f"{name:<20}{n:>5}{100*f('col'):>7.0f}{100*f('ok'):>7.0f}{f('mn'):>9.2f}"
                  f"{f('intr'):>8.1f}{f('jerk'):>7.2f}{f('t'):>7.1f}", flush=True)


if __name__ == '__main__':
    main()
