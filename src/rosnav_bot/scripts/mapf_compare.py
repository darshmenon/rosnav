#!/usr/bin/env python3
"""
mapf_compare.py — standalone (no ROS) multi-agent path finding benchmark.

Large 4-connected grid, N diff-drive robots, each with its own colour-coded
start (circle) and goal (square). Solvers compared on identical instances:

  independent  A* per robot, ignores others (baseline; reports conflicts left)
  pp           Prioritized Planning (longest-trip-first, random-order restarts)
  cbs          Conflict-Based Search (optimal, vertex + edge conflicts)
  ecbs         Bounded-suboptimal CBS (focal search on #conflicts, weight w)

Outputs go to <repo>/images/mapf/: instance.png, solution_<algo>.png,
comparison.png, cbs.gif.

  python3 mapf_compare.py                      # 16 robots demo + scaling sweep
  python3 mapf_compare.py --size 40 --agents 16 --seed 3 --no-sweep
"""

import argparse
import heapq
import itertools
import os
import random
import time
from collections import deque

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import animation, colors as mcolors
import numpy as np

MOVES = ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1))
HORIZON = 150
OUT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'images', 'mapf'))
ALGOS = ('independent', 'pp', 'cbs', 'ecbs')


# ── Instance generation ──────────────────────────────────────────────────────

def bfs_dist(free, goal):
    rows, cols = free.shape
    d = np.full((rows, cols), 10**6, dtype=int)
    d[goal] = 0
    q = deque([goal])
    while q:
        r, c = q.popleft()
        for dr, dc in MOVES[1:]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and free[nr, nc] and d[nr, nc] > d[r, c] + 1:
                d[nr, nc] = d[r, c] + 1
                q.append((nr, nc))
    return d


def make_instance(size, n_agents, density, seed):
    rng = random.Random(seed)
    while True:
        free = np.ones((size, size), dtype=bool)
        for r in range(size):
            for c in range(size):
                if rng.random() < density:
                    free[r, c] = False
        cells = [(r, c) for r in range(size) for c in range(size) if free[r, c]]
        if len(cells) < 2 * n_agents:
            continue
        # keep only the largest connected component so every goal is reachable
        seed_cell = rng.choice(cells)
        comp = {tuple(x) for x in np.argwhere(bfs_dist(free, seed_cell) < 10**6)}
        if len(comp) < 2 * n_agents + 10:
            continue
        pool = sorted(comp)
        rng.shuffle(pool)
        starts, goals = pool[:n_agents], pool[n_agents:2 * n_agents]
        if all(abs(s[0] - g[0]) + abs(s[1] - g[1]) >= size // 3 for s, g in zip(starts, goals)):
            return free, starts, goals


# ── Low level: time-expanded A* ──────────────────────────────────────────────

def astar(free, start, goal, h, vertex_c, edge_c, other_paths=None, horizon=HORIZON):
    """State (r,c,t). Ties broken by fewer conflicts with other_paths (for ECBS focal)."""
    rows, cols = free.shape
    hold = max((t for (r, c, t) in vertex_c if (r, c) == goal), default=-1)
    tie = {}
    if other_paths:
        for p in other_paths:
            for t, cell in enumerate(p):
                tie.setdefault((cell, t), 0)
                tie[(cell, t)] += 1
    heap = [(h[start], 0, 0, start[0], start[1], 0)]
    parent, best, closed = {}, {(start[0], start[1], 0): 0}, set()
    while heap:
        _, _, g, r, c, t = heapq.heappop(heap)
        if (r, c, t) in closed:
            continue
        closed.add((r, c, t))
        if (r, c) == goal and t > hold:
            path, cur = [(r, c)], (r, c, t)
            while cur in parent:
                cur = parent[cur]
                path.append(cur[:2])
            return path[::-1]
        if t >= horizon:
            continue
        for dr, dc in MOVES:
            nr, nc, nt = r + dr, c + dc, t + 1
            if not (0 <= nr < rows and 0 <= nc < cols and free[nr, nc]):
                continue
            if (nr, nc, nt) in vertex_c or ((dr or dc) and (r, c, nr, nc, t) in edge_c):
                continue
            st = (nr, nc, nt)
            if g + 1 < best.get(st, 1 << 30):
                best[st] = g + 1
                parent[st] = (r, c, t)
                heapq.heappush(heap, (g + 1 + h[nr, nc], tie.get(((nr, nc), nt), 0), g + 1, nr, nc, nt))
    return None


# ── Conflicts ────────────────────────────────────────────────────────────────

def at(p, t):
    return p[t] if t < len(p) else p[-1]


def find_conflicts(paths, first_only=False):
    out, T = [], max(len(p) for p in paths)
    n = len(paths)
    for t in range(T):
        seen = {}
        for i in range(n):
            cell = at(paths[i], t)
            if cell in seen:
                out.append(('v', seen[cell], i, cell, t))
                if first_only:
                    return out
            else:
                seen[cell] = i
        for i in range(n):
            for j in range(i + 1, n):
                a0, a1, b0, b1 = at(paths[i], t), at(paths[i], t + 1), at(paths[j], t), at(paths[j], t + 1)
                if a0 == b1 and a1 == b0 and a0 != a1:
                    out.append(('e', i, j, (a0, a1), t))
                    if first_only:
                        return out
    return out


def soc(paths):
    return sum(len(p) - 1 for p in paths)


# ── Solvers ──────────────────────────────────────────────────────────────────

def solve_independent(free, starts, goals, hs, **_):
    paths = [astar(free, s, g, h, set(), set()) for s, g, h in zip(starts, goals, hs)]
    return paths, {}


def solve_pp(free, starts, goals, hs, deadline, seed=0, **_):
    n, rng = len(starts), random.Random(seed)
    order = sorted(range(n), key=lambda i: -hs[i][starts[i]])
    restarts = 0
    while time.time() < deadline:
        paths, vc, ec, ok = [None] * n, set(), set(), True
        for i in order:
            p = astar(free, starts[i], goals[i], hs[i], vc, ec)
            if p is None:
                ok = False
                break
            paths[i] = p
            for t in range(HORIZON + 1):
                vc.add((*at(p, t), t))
            for t in range(len(p) - 1):
                if p[t] != p[t + 1]:
                    ec.add((*p[t + 1], *p[t], t))
        if ok:
            return paths, {'restarts': restarts}
        restarts += 1
        rng.shuffle(order)
    return None, {'restarts': restarts}


def solve_cbs(free, starts, goals, hs, deadline, w=1.0, **_):
    """w=1 → CBS (best-first on cost). w>1 → ECBS-style focal search: among nodes
    with cost <= w*min_cost expand the one with fewest conflicts."""
    n, counter = len(starts), itertools.count()

    def plan(i, con, paths):
        others = [p for j, p in enumerate(paths) if j != i and p] if w > 1 else None
        return astar(free, starts[i], goals[i], hs[i], con[i][0], con[i][1], others)

    con = [(set(), set()) for _ in range(n)]
    paths = []
    for i in range(n):
        p = plan(i, con, paths + [None] * (n - i))
        if p is None:
            return None, {'nodes': 0}
        paths.append(p)
    root = (soc(paths), len(find_conflicts(paths)), next(counter), con, paths)
    open_l, nodes = [root], 0
    while open_l and time.time() < deadline:
        if w > 1:
            cmin = min(x[0] for x in open_l)
            focal = [x for x in open_l if x[0] <= w * cmin]
            node = min(focal, key=lambda x: (x[1], x[0], x[2]))
            open_l.remove(node)
        else:
            node = min(open_l, key=lambda x: (x[0], x[2]))
            open_l.remove(node)
        cost, _, _, con, paths = node
        nodes += 1
        conf = find_conflicts(paths, first_only=True)
        if not conf:
            return paths, {'nodes': nodes}
        kind, a, b, where, t = conf[0]
        for agent, c in ((a, where), (b, where)):
            if kind == 'v':
                add_v, add_e = {(*c, t)}, set()
            else:
                (c0, c1) = where
                mv = (c0, c1) if agent == a else (c1, c0)
                add_v, add_e = set(), {(*mv[0], *mv[1], t)}
            ncon = list(con)
            ncon[agent] = (con[agent][0] | add_v, con[agent][1] | add_e)
            p = plan(agent, ncon, paths)
            if p is None:
                continue
            npaths = list(paths)
            npaths[agent] = p
            open_l.append((soc(npaths), len(find_conflicts(npaths)), next(counter), ncon, npaths))
    return None, {'nodes': nodes}


def run(algo, free, starts, goals, hs, timeout, seed=0):
    t0 = time.time()
    deadline = t0 + timeout
    if algo == 'independent':
        paths, info = solve_independent(free, starts, goals, hs)
    elif algo == 'pp':
        paths, info = solve_pp(free, starts, goals, hs, deadline, seed=seed)
    elif algo == 'cbs':
        paths, info = solve_cbs(free, starts, goals, hs, deadline, w=1.0)
    else:
        paths, info = solve_cbs(free, starts, goals, hs, deadline, w=1.5)
    dt = time.time() - t0
    res = {'algo': algo, 'time': dt, 'paths': paths, 'info': info}
    if paths is None or any(p is None for p in paths):
        res.update(ok=False, soc=None, makespan=None, conflicts=None)
    else:
        nconf = len(find_conflicts(paths))
        res.update(ok=(nconf == 0) or algo == 'independent', soc=soc(paths),
                   makespan=max(len(p) - 1 for p in paths), conflicts=nconf)
    return res


# ── Plotting ─────────────────────────────────────────────────────────────────

def agent_colors(n):
    cmap = plt.get_cmap('tab20')
    return [cmap(i % 20) for i in range(n)]


def draw_grid(ax, free, starts, goals, cols, paths=None, title=''):
    ax.imshow(~free, cmap='Greys', origin='upper', vmin=0, vmax=1.6, interpolation='nearest')
    if paths:
        for p, c in zip(paths, cols):
            ys, xs = zip(*p)
            ax.plot(xs, ys, '-', color=c, lw=1.6, alpha=0.8)
    for i, (s, g, c) in enumerate(zip(starts, goals, cols)):
        ax.scatter(s[1], s[0], s=70, color=c, edgecolor='k', zorder=3)
        ax.scatter(g[1], g[0], s=80, marker='s', color=c, edgecolor='k', zorder=3)
        ax.text(s[1], s[0], str(i + 1), ha='center', va='center', fontsize=5, zorder=4)
    ax.set_xticks([]); ax.set_yticks([]); ax.set_title(title, fontsize=9)


def save_instance(free, starts, goals, cols, name='instance.png'):
    fig, ax = plt.subplots(figsize=(7, 7))
    draw_grid(ax, free, starts, goals, cols,
              title=f'{len(starts)} robots on {free.shape[0]}x{free.shape[1]} grid  (● start, ■ goal)')
    fig.tight_layout(); fig.savefig(os.path.join(OUT_DIR, name), dpi=130); plt.close(fig)


def save_solutions(free, starts, goals, cols, results):
    fig, axes = plt.subplots(1, len(results), figsize=(5.2 * len(results), 5.6))
    for ax, r in zip(np.atleast_1d(axes), results):
        if r['paths'] is None:
            draw_grid(ax, free, starts, goals, cols, title=f"{r['algo']}: FAILED ({r['time']:.1f}s)")
            continue
        extra = f", {r['conflicts']} conflicts" if r['conflicts'] else ''
        draw_grid(ax, free, starts, goals, cols, r['paths'],
                  f"{r['algo']}: SoC {r['soc']}, makespan {r['makespan']}{extra}, {r['time']:.2f}s")
    fig.tight_layout(); fig.savefig(os.path.join(OUT_DIR, 'solutions.png'), dpi=120); plt.close(fig)


def save_gif(free, starts, goals, cols, paths, name='cbs.gif'):
    T = max(len(p) for p in paths)
    fig, ax = plt.subplots(figsize=(6, 6))
    draw_grid(ax, free, starts, goals, cols)
    dots = [ax.scatter([], [], s=110, color=c, edgecolor='k', zorder=5) for c in cols]
    txt = ax.text(0.02, 0.98, '', transform=ax.transAxes, va='top', fontsize=9,
                  bbox=dict(fc='w', alpha=.7, lw=0))

    def upd(t):
        for d, p in zip(dots, paths):
            r, c = at(p, t)
            d.set_offsets([[c, r]])
        txt.set_text(f't = {t}/{T - 1}')
        return dots + [txt]
    ani = animation.FuncAnimation(fig, upd, frames=T, interval=150)
    ani.save(os.path.join(OUT_DIR, name), writer=animation.PillowWriter(fps=7))
    plt.close(fig)


def save_comparison(sweep, demo, n_demo):
    palette = {'independent': '#9aa0a6', 'pp': '#e8710a', 'cbs': '#1a73e8', 'ecbs': '#188038'}
    ns = sorted(sweep)
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.2))
    w = 0.2
    for k, a in enumerate(ALGOS[1:]):
        x = np.arange(len(ns)) + (k - 1) * w
        succ = [100 * np.mean([r['ok'] for r in sweep[n][a]]) for n in ns]
        axes[0].bar(x, succ, w, label=a, color=palette[a])
        sc = [np.mean([r['soc'] / sweep[n]['independent'][i]['soc'] for i, r in enumerate(sweep[n][a]) if r['ok']] or [np.nan])
              for n in ns]
        axes[1].bar(x, sc, w, label=a, color=palette[a])
        tm = [np.mean([r['time'] for r in sweep[n][a]]) for n in ns]
        axes[2].bar(x, tm, w, label=a, color=palette[a])
    for ax, t in zip(axes, ('Solved, % (conflict-free)', 'Sum of costs ÷ independent lower bound', 'Mean runtime, s (log)')):
        ax.set_xticks(np.arange(len(ns))); ax.set_xticklabels(ns)
        ax.set_xlabel('robots'); ax.set_title(t, fontsize=10)
    axes[1].set_ylim(1.0, None)
    axes[2].set_yscale('log'); axes[0].legend(fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(OUT_DIR, 'comparison.png'), dpi=130); plt.close(fig)


# ── Main ─────────────────────────────────────────────────────────────────────

def fmt(r):
    if not r['ok'] and r['paths'] is None:
        return f"{r['algo']:<12} FAILED/timeout  {r['time']:6.2f}s  {r['info']}"
    return (f"{r['algo']:<12} SoC={r['soc']:<5} makespan={r['makespan']:<4} conflicts={r['conflicts']:<3} "
            f"{r['time']:6.2f}s  {r['info']}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--size', type=int, default=32)
    ap.add_argument('--agents', type=int, default=16)
    ap.add_argument('--density', type=float, default=0.12)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--timeout', type=float, default=30.0)
    ap.add_argument('--sweep-seeds', type=int, default=5)
    ap.add_argument('--sweep-agents', type=int, nargs='+', default=[4, 8, 12, 16])
    ap.add_argument('--no-sweep', action='store_true')
    a = ap.parse_args()
    os.makedirs(OUT_DIR, exist_ok=True)

    free, starts, goals = make_instance(a.size, a.agents, a.density, a.seed)
    hs = [bfs_dist(free, g) for g in goals]
    cols = agent_colors(a.agents)
    save_instance(free, starts, goals, cols)

    print(f'== demo: {a.agents} robots, {a.size}x{a.size}, density {a.density}, seed {a.seed}')
    results = []
    for algo in ALGOS:
        r = run(algo, free, starts, goals, hs, a.timeout, seed=a.seed)
        print(fmt(r)); results.append(r)
    save_solutions(free, starts, goals, cols, results)
    best = next((r for r in results if r['algo'] == 'cbs' and r['ok']), None) or \
        next((r for r in results if r['algo'] in ('ecbs', 'pp') and r['ok']), None)
    if best:
        save_gif(free, starts, goals, cols, best['paths'], f"{best['algo']}.gif")
        print(f"wrote {best['algo']}.gif")

    if not a.no_sweep:
        sweep = {}
        for n in a.sweep_agents:
            sweep[n] = {al: [] for al in ALGOS}
            for s in range(a.sweep_seeds):
                f, st, go = make_instance(a.size, n, a.density, 100 + s)
                h = [bfs_dist(f, g) for g in go]
                for al in ALGOS:
                    sweep[n][al].append(run(al, f, st, go, h, a.timeout / 3, seed=s))
            print(f'n={n:<3} ' + '  '.join(
                f"{al}: {100 * np.mean([r['ok'] for r in sweep[n][al]]):.0f}% "
                f"{np.mean([r['time'] for r in sweep[n][al]]):.2f}s" for al in ALGOS))
        save_comparison(sweep, results, a.agents)
    print('outputs in', OUT_DIR)


if __name__ == '__main__':
    main()
