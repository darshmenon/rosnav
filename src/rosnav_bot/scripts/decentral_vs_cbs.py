#!/usr/bin/env python3
"""Centralized MAPF (PP / ECBS, conflict-free by construction) vs decentralized
controllers (vo / mppi_prior) on IDENTICAL grid instances.

Instances come from mapf_compare.make_instance (4-connected grid, 1 cell = 1 m,
1 m/s). Centralized plans execute lock-step, so plan length == execution time.
Decentralized runs reuse decentral_nav_sim.simulate() by temporarily swapping its
make_scenario for one built from the same grid/starts/goals (x=col, y=row).
No ROS / Gazebo. Usage: nice -n 19 python3 decentral_vs_cbs.py [--seeds 8]
"""
import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mapf_compare as mc                      # noqa: E402
import decentral_nav_sim as sim                # noqa: E402

OUT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'images', 'decentral_nav'))
METHODS = ('pp', 'ecbs', 'vo', 'mppi_prior')
MAX_T = 90.0


def central(algo, free, starts, goals, timeout):
    hs = [mc.bfs_dist(free, g) for g in goals]
    r = mc.run(algo, free, starts, goals, hs, timeout)
    if not r['ok'] or r['paths'] is None:
        return dict(success=False, collisions=0, ms=None, soc=None)
    arr = []
    for p in r['paths']:                       # arrival = last time the agent is not yet parked at goal
        t = len(p) - 1
        while t > 0 and p[t - 1] == p[-1]:
            t -= 1
        arr.append(t)
    return dict(success=True, collisions=r['conflicts'], ms=float(max(arr)), soc=float(sum(arr)))


def decentral(ctrl, free, starts, goals, seed):
    world = sim.World(free)
    s = np.array([[c, r] for r, c in starts], float)
    g = np.array([[c, r] for r, c in goals], float)
    orig = sim.make_scenario
    sim.make_scenario = lambda kind, n, sd: (world, s, g, None)
    try:
        res = sim.simulate('custom', ctrl, n=len(starts), seed=seed, max_t=MAX_T)
    finally:
        sim.make_scenario = orig
    n = len(starts)
    # same ideal-time formula simulate() uses, to rebuild a sum-of-costs proxy
    ideal = np.array([world.dist_field(g[i])[int(s[i][1]), int(s[i][0])] for i in range(n)]) / sim.VMAX
    ideal = np.maximum(0.8 * ideal, np.linalg.norm(g - s, axis=1) / sim.VMAX)
    soc = float(np.nansum(ideal) + n * res['delay'])
    return dict(success=bool(res['success']), collisions=int(res['collisions']),
                ms=res['makespan'] if res['success'] else None, soc=soc if res['success'] else None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seeds', type=int, default=8)
    ap.add_argument('--size', type=int, default=24)
    ap.add_argument('--density', type=float, default=0.12)
    ap.add_argument('--ns', type=int, nargs='+', default=[4, 8, 12])
    ap.add_argument('--timeout', type=float, default=20.0)
    a = ap.parse_args()
    rows = []
    for n in a.ns:
        for seed in range(a.seeds):
            free, starts, goals = mc.make_instance(a.size, n, a.density, seed)
            for m in METHODS:
                t0 = time.time()
                r = central(m, free, starts, goals, a.timeout) if m in ('pp', 'ecbs') \
                    else decentral(m, free, starts, goals, seed)
                r.update(n=n, seed=seed, method=m)
                rows.append(r)
                print(f"n={n} seed={seed} {m:10s} ok={r['success']} ms={r['ms']} coll={r['collisions']} ({time.time()-t0:.1f}s)", flush=True)
    report(rows, a)


def agg(rows, m, n):
    rs = [r for r in rows if r['method'] == m and r['n'] == n]
    ok = [r for r in rs if r['success']]
    def f(k): return float(np.mean([r[k] for r in ok])) if ok else float('nan')
    return dict(succ=len(ok) / len(rs), ms=f('ms'), soc=f('soc'),
                coll=float(np.mean([r['collisions'] for r in rs])),
                collrate=float(np.mean([r['collisions'] > 0 for r in rs])), k=len(rs))


def report(rows, a):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    os.makedirs(OUT, exist_ok=True)
    col = dict(pp='#1f77b4', ecbs='#17becf', vo='#ff7f0e', mppi_prior='#d62728')
    # paired makespan on instances where all methods succeeded
    lines = ["# Decentralized vs centralized MAPF\n",
             f"{a.size}x{a.size} grid, obstacle density {a.density}, {a.seeds} seeds per n, 1 cell = 1 m, 1 m/s. "
             f"Centralized (PP, ECBS w=1.5) plans are conflict-free and executed lock-step (time = plan length; "
             f"planning time excluded, {a.timeout:.0f}s budget). Decentralized (vo, mppi_prior) run in the continuous "
             f"simulator (r=0.3 m, goal tol 0.35 m, {MAX_T:.0f}s limit) from the same obstacles/starts/goals. "
             f"Makespan/SoC averaged over successful runs only; decentral SoC is a proxy (ideal + n*mean delay).\n",
             "| n | method | success | makespan s | SoC s | collisions/run | runs w/ collision |", "|---|---|---|---|---|---|---|"]
    fig, ax = plt.subplots(1, 4, figsize=(17, 3.9))
    for n in a.ns:
        for m in METHODS:
            g = agg(rows, m, n)
            lines.append(f"| {n} | {m} | {g['succ']*100:.0f}% | {g['ms']:.1f} | {g['soc']:.1f} | {g['coll']:.2f} | {g['collrate']*100:.0f}% |")
    w = 0.2
    for k, (key, title) in enumerate([('succ', 'success rate'), ('ms', 'makespan (s, successes)'),
                                      ('soc', 'sum of costs (s, successes)'), ('coll', 'collision pairs / run')]):
        for j, m in enumerate(METHODS):
            ax[k].bar(np.arange(len(a.ns)) + (j - 1.5) * w, [agg(rows, m, n)[key] for n in a.ns], w, label=m, color=col[m])
        ax[k].set_xticks(range(len(a.ns)), [f'n={n}' for n in a.ns])
        ax[k].set_title(title, fontsize=10)
        ax[k].spines[['top', 'right']].set_visible(False)
    ax[0].legend(fontsize=8, frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'vs_cbs.png'), dpi=130)
    # paired comparison on commonly-solved instances
    lines += ["", "## Paired makespan ratio vs ECBS (instances both solved)", "", "| n | method | ratio (median) | paired runs |", "|---|---|---|---|"]
    for n in a.ns:
        for m in ('vo', 'mppi_prior'):
            rat = []
            for seed in range(a.seeds):
                e = [r for r in rows if r['n'] == n and r['seed'] == seed and r['method'] == 'ecbs'][0]
                d = [r for r in rows if r['n'] == n and r['seed'] == seed and r['method'] == m][0]
                if e['success'] and d['success']:
                    rat.append(d['ms'] / e['ms'])
            lines.append(f"| {n} | {m} | {np.median(rat):.2f} | {len(rat)} |" if rat else f"| {n} | {m} | n/a | 0 |")
    open(os.path.join(OUT, 'vs_cbs.md'), 'w').write('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
