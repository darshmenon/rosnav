#!/usr/bin/env python3
"""
render_social_compare.py — two matplotlib simulations of the social-nav core (no ROS / Gazebo / torch).

  split   : same crowd, same seed, three planners side by side (reactive | predictive | risk-aware),
            live collision counters, trails, 1.2 m personal-space rings -> GIF
  sweep   : collision rate / success / intrusion vs crowd size (4..24) per planner, several seeds -> PNG
  commit  : risk planner with and without the commitment term, same crowd -> PNG of trails + heading flips
  heatmap : the risk planner's 100 candidate velocities coloured by Monte Carlo collision risk and by total
            cost, with the chosen velocity, next to the world snapshot it was chosen in -> PNG

  python3 render_social_compare.py split   --out split.gif --seed 3 --seconds 30
  python3 render_social_compare.py heatmap --out heatmap.png --seed 3 --t 9.0
"""
import argparse, os, random, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import PillowWriter
from matplotlib.patches import Circle

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import social_nav_core as s

DT = s.DT


class Run:
    """One planner in the shared 'crowd' scenario (pedestrians bounce in a 12x8 m box)."""
    def __init__(self, planner, seed, n):
        rng = random.Random(seed)
        self.pos, self.goal, ps, pv = s.make_scenario('crowd', rng, n)
        self.pos = self.pos.copy()
        self.ps = [p.copy() for p in ps]
        self.pv = [v + np.array([rng.gauss(0, .05), rng.gauss(0, .05)]) for v in pv]
        self.pl, self.col, self.in_c, self.intr, self.done, self.t_goal = planner, 0, False, 0.0, False, None
        self.trail = [self.pos.copy()]
        self.min_clear = 1e9

    def step(self, k):
        for i in range(len(self.ps)):
            self.ps[i] += self.pv[i] * DT
            if abs(self.ps[i][1]) > 4: self.pv[i][1] *= -1
            if self.ps[i][0] < -1 or self.ps[i][0] > 11: self.pv[i][0] *= -1
        if self.done:
            return
        v = s.safety_filter(self.pl.plan(self.pos, self.goal, self.ps, self.pv), self.pos, self.ps)
        self.pos = self.pos + v * DT
        self.trail.append(self.pos.copy())
        d = min(np.linalg.norm(self.pos - p) for p in self.ps)
        self.min_clear = min(self.min_clear, d - s.ROBOT_R - s.PED_R)
        hit = d < s.ROBOT_R + s.PED_R
        self.col += int(hit and not self.in_c); self.in_c = hit
        self.intr += DT * (d < s.PERSONAL_SPACE)
        if np.linalg.norm(self.pos - self.goal) < 0.3:
            self.done, self.t_goal = True, k * DT


def draw_world(ax, r, cols, t):
    ax.add_patch(plt.Rectangle((-1, -4), 12, 8, fc='#f4f4f4', ec='k', lw=1))
    for i, p in enumerate(r.ps):
        ax.add_patch(Circle(p, s.PED_R, color=cols[i], zorder=5))
        ax.plot(*np.stack([p, p + r.pv[i] * 1.5]).T, '-', color=cols[i], lw=1, alpha=.6, zorder=4)
    ax.add_patch(Circle(r.pos, s.PERSONAL_SPACE, fill=False, ls='--', ec='tab:green', lw=.9, alpha=.7))
    ax.add_patch(Circle(r.pos, s.ROBOT_R, fc='tab:green', ec='k', zorder=6))
    ax.plot(*np.array(r.trail[-150:]).T, '-', color='tab:green', lw=1.4, alpha=.7)
    ax.plot(*r.goal, '*', color='gold', ms=14, mec='k', zorder=7)
    ax.set_xlim(-1.5, 11.5); ax.set_ylim(-4.5, 4.5); ax.set_aspect('equal'); ax.set_xticks([]); ax.set_yticks([])


def split(a):
    names = ['reactive', 'predictive', 'risk-aware']
    runs = [Run(p, a.seed, a.peds) for p in (s.ReactivePlanner(), s.PredictivePlanner(), s.RiskPlanner(sigma_scale=0.5))]
    cols = plt.cm.tab10(np.arange(a.peds) % 10)
    fig, axs = plt.subplots(1, 3, figsize=(15, 3.9), dpi=70)
    w = PillowWriter(fps=10)
    with w.saving(fig, a.out, dpi=70):
        for k in range(int(a.seconds / DT)):
            for r in runs:
                r.step(k)
            if k % 2:
                continue
            for ax, r, nm in zip(axs, runs, names):
                ax.clear(); draw_world(ax, r, cols, k * DT)
                state = f'reached goal {r.t_goal:.1f}s' if r.done else 'driving'
                ax.set_title(f'{nm}   collisions {r.col}   intrusion {r.intr:.1f}s\n{state}', fontsize=10)
            fig.suptitle(f'same crowd, same seed   t = {k*DT:4.1f} s   (green = robot, dashed ring = 1.2 m personal space)',
                         fontsize=10)
            w.grab_frame()
            if all(r.done for r in runs):
                break
    print('wrote', a.out, {n: (r.col, round(r.intr, 1), r.t_goal) for n, r in zip(names, runs)})


def heatmap(a):
    r = Run(s.RiskPlanner(sigma_scale=0.5, samples=48), a.seed, a.peds)
    for k in range(int(a.t / DT)):
        r.step(k)
    v = r.pl.plan(r.pos, r.goal, r.ps, r.pv)
    risk, cost, cands = r.pl.last['risk'], r.pl.last['cost'], r.pl.cands
    cols = plt.cm.tab10(np.arange(a.peds) % 10)
    fig, axs = plt.subplots(1, 3, figsize=(16, 4.8), dpi=100)
    draw_world(axs[0], r, cols, a.t)
    for p, pv in zip(r.ps, r.pv):
        axs[0].plot(*s.predict_cv(p, pv, 3.0).T, ':', color='tab:blue', lw=1.2)
    axs[0].arrow(*r.pos, *(v * 1.5), width=0.06, head_width=.3, fc='red', ec='k', zorder=9)
    axs[0].set_title(f'world at t={a.t:.1f}s  (blue dotted = predicted paths, red = chosen)', fontsize=10)
    for ax, val, ttl, cm in ((axs[1], risk, 'P(collision) per candidate velocity', 'RdYlGn_r'),
                             (axs[2], cost, 'total cost (lower = better)', 'viridis_r')):
        sc = ax.scatter(cands[:, 0], cands[:, 1], c=val, cmap=cm, s=60, edgecolor='k', lw=.3)
        ax.plot(*v, 'o', ms=16, mfc='none', mec='red', mew=2.5)
        g = (r.goal - r.pos) / np.linalg.norm(r.goal - r.pos)
        ax.annotate('', xy=g * .8, xytext=(0, 0), arrowprops=dict(arrowstyle='->', color='gold', lw=2))
        ax.set_aspect('equal'); ax.set_xlabel('vx (m/s)'); ax.set_ylabel('vy (m/s)')
        ax.set_title(ttl + '\n(gold = direction to goal, red ring = chosen)', fontsize=10)
        fig.colorbar(sc, ax=ax, fraction=.046)
    fig.tight_layout(); fig.savefig(a.out); print('wrote', a.out, 'chosen v =', np.round(v, 2))


def sweep(a):
    mk = {'reactive': lambda: s.ReactivePlanner(), 'predictive': lambda: s.PredictivePlanner(),
          'risk-aware': lambda: s.RiskPlanner(sigma_scale=0.5)}
    counts = (4, 8, 16, 24)
    res = {k: {n: [] for n in counts} for k in mk}
    for n in counts:
        for seed in range(a.seeds):
            for k, f in mk.items():
                r = Run(f(), seed, n)
                for i in range(int(a.seconds / DT)):
                    r.step(i)
                    if r.done:
                        break
                res[k][n].append((r.col > 0, r.done, r.intr))
        print('peds', n, 'done', flush=True)
    fig, axs = plt.subplots(1, 3, figsize=(14, 4), dpi=100)
    colr = {'reactive': 'tab:red', 'predictive': 'tab:blue', 'risk-aware': 'tab:green'}
    for j, (ttl, f) in enumerate((('collision rate', lambda x: np.mean([q[0] for q in x]) * 100),
                                  ('reached goal (%)', lambda x: np.mean([q[1] for q in x]) * 100),
                                  ('personal-space intrusion (s)', lambda x: np.mean([q[2] for q in x])))):
        for k in mk:
            axs[j].plot(counts, [f(res[k][n]) for n in counts], '-o', color=colr[k], label=k, lw=2)
        axs[j].set_xlabel('pedestrians'); axs[j].set_title(ttl + (' (%)' if j == 0 else '')); axs[j].grid(alpha=.3)
    axs[0].legend()
    fig.suptitle(f'crowd-density sweep, {a.seeds} seeds per cell, {a.seconds:.0f} s per run (offline social-nav core)')
    fig.tight_layout(); fig.savefig(a.out)
    print('wrote', a.out, {k: {n: [round(f(v), 1) for f in (lambda x: np.mean([q[0] for q in x]) * 100,)][0] for n, v in d.items()} for k, d in res.items()})


def commit(a):
    runs = [('without commitment (w_commit=0)', Run(s.RiskPlanner(sigma_scale=0.5, w_commit=0.0), a.seed, a.peds)),
            ('with commitment (w_commit=0.6)', Run(s.RiskPlanner(sigma_scale=0.5, w_commit=0.6), a.seed, a.peds))]
    heads = {n: [] for n, _ in runs}
    for k in range(int(a.seconds / DT)):
        for n, r in runs:
            before = r.pos.copy(); r.step(k)
            d = r.pos - before
            if np.linalg.norm(d) > 1e-4:
                heads[n].append(np.arctan2(d[1], d[0]))
    cols = plt.cm.tab10(np.arange(a.peds) % 10)
    fig, axs = plt.subplots(1, 2, figsize=(14, 4.2), dpi=100)
    for ax, (n, r) in zip(axs, runs):
        draw_world(ax, r, cols, a.seconds)
        ax.clear(); draw_world(ax, r, cols, a.seconds)
        tr = np.array(r.trail); ax.plot(*tr.T, '-', color='tab:green', lw=2)
        h = np.unwrap(np.array(heads[n]))
        flips = int((np.abs(np.diff(h)) > 0.5).sum()) if len(h) > 1 else 0
        path = float(np.linalg.norm(np.diff(tr, axis=0), axis=1).sum()) if len(tr) > 1 else 0.0
        state = f'goal at {r.t_goal:.1f}s' if r.done else f'NOT reached in {a.seconds:.0f}s'
        ax.set_title(f'{n}\n{state}, heading jumps >0.5 rad: {flips}, path {path:.1f} m, collisions {r.col}', fontsize=10)
        print(n, state, 'flips', flips, 'path', round(path, 1), 'col', r.col)
    fig.tight_layout(); fig.savefig(a.out); print('wrote', a.out)


ap = argparse.ArgumentParser()
ap.add_argument('mode', choices=['split', 'heatmap', 'sweep', 'commit'])
ap.add_argument('--out', default=None)
ap.add_argument('--seed', type=int, default=3)
ap.add_argument('--peds', type=int, default=8)
ap.add_argument('--seconds', type=float, default=30)
ap.add_argument('--t', type=float, default=9.0)
ap.add_argument('--seeds', type=int, default=8)
a = ap.parse_args()
a.out = a.out or ('social_split.gif' if a.mode == 'split' else f'social_{a.mode}.png')
dict(split=split, heatmap=heatmap, sweep=sweep, commit=commit)[a.mode](a)
