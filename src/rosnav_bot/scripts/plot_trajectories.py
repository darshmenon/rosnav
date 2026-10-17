#!/usr/bin/env python3
"""Overlay ground-truth trajectories (and the planned path) per goal for several controllers.

Usage: plot_trajectories.py OUT.png dwb=a_nav.json mppi=b_nav.json rpp=c_nav.json
Prints per-run gt path length, cross-track error vs the planned path, and total turning (smoothness).
"""
import json
import math
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

COLORS = {'dwb': '#1f77b4', 'mppi': '#d62728', 'rpp': '#2ca02c'}


def cross_track(track, plan):
    if not plan or len(plan) < 2 or not track:
        return None

    def seg_dist(p, a, b):
        vx, vy = b[0] - a[0], b[1] - a[1]
        n = vx * vx + vy * vy
        t = 0 if n == 0 else max(0, min(1, ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / n))
        return math.hypot(p[0] - a[0] - t * vx, p[1] - a[1] - t * vy)

    e = [min(seg_dist(p, plan[i], plan[i + 1]) for i in range(len(plan) - 1)) for p in track]
    return sum(e) / len(e), max(e)


def turning(track):
    h = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(track, track[1:])
         if math.hypot(b[0] - a[0], b[1] - a[1]) > 0.01]
    return sum(abs((y - x + math.pi) % (2 * math.pi) - math.pi) for x, y in zip(h, h[1:]))


def main():
    out = sys.argv[1]
    runs = {}
    for a in sys.argv[2:]:
        k, f = a.split('=', 1)
        runs[k] = json.load(open(f))['runs']
    ngoal = max(len(r) for r in runs.values())
    fig, axs = plt.subplots(1, ngoal, figsize=(5.5 * ngoal, 5), squeeze=False)
    print('ctrl  goal ok     gt_len planned xte_mean xte_max turn_rad')
    for gi in range(ngoal):
        ax = axs[0][gi]
        plan_drawn = False
        for k, r in runs.items():
            if gi >= len(r):
                continue
            g = r[gi]
            tr = [t[2] for t in g.get('track', []) if t[2]]
            if tr:
                ax.plot(*zip(*tr), '-', color=COLORS.get(k),
                        label=f"{k} ({'ok' if g['success'] else 'FAIL'}, {g['elapsed_sec']:.0f}s)")
            pl = g.get('planned_path')
            if pl and not plan_drawn:
                ax.plot(*zip(*pl), '--', color='gray', lw=1, label='planned')
                plan_drawn = True
            e = cross_track(tr, pl)
            print(f"{k:5} {gi + 1:4} {g['success']!s:6} {g.get('gt_path_len_m')} {g.get('planned_path_len_m')} "
                  + (f"{e[0]:.3f} {e[1]:.3f} " if e else '- - ')
                  + (f"{turning(tr):.2f}" if tr else '-'))
        gp = next(r[gi]['goal'] for r in runs.values() if gi < len(r))
        ax.plot(gp['x'], gp['y'], 'k*', ms=14)
        ax.set_title(f'goal {gi + 1}')
        ax.set_aspect('equal')
        ax.grid(alpha=.3)
        ax.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(out, dpi=120)
    print('wrote', out)


if __name__ == '__main__':
    main()
