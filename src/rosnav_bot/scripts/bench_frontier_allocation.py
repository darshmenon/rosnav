#!/usr/bin/env python3
"""Offline benchmark: Hungarian vs greedy frontier assignment (FrontierCoordinator._hungarian_plan). No ROS graph, no robot.

WHAT IT IS NOT: a robot run. The score model is SYNTHETIC: score(robot, frontier) = value + 12 - 0.5 * distance, with value
uniform in [1, 10], not the production scorer. Scenarios are random (random.Random(seed), seeds 0..N-1), so every number is
reproducible from the command line.

Modes
  greedy               each idle robot in order takes its best frontier not within assign_radius (1 m) of goals already taken
  hungarian            IMITATES THE PRE-FIX coordinator: Hungarian plan, but a robot without a planned goal falls back to greedy
                       without the planned goals of later robots reserved, so it can duplicate one (inflated figure)
  hungarian_reserved   current code (commit 10779e4): planned goals are reserved before any fallback pick

Distributions (--dist)
  base       15x15 m, 2-6 robots, 3-12 frontiers      (the original 500-scenario run)
  large      40x40 m, 2-6 robots, 5-30 frontiers
  eight      15x15 m, exactly 8 robots, 8-16 frontiers
  fewer      15x15 m, 3-8 robots, 1 to (robots-1) frontiers (more robots than frontiers)
  clustered  15x15 m, 2-6 robots, 6-14 frontiers drawn in 2-3 tight clusters (sigma 1.2 m, clusters overlap assign_radius)

Scorers (--scorer)
  synthetic  the formula above (default)
  utility    the coordinator's own FrontierCoordinator._score_frontier with node-default parameters (scorer 'utility':
             1.0*size_m - 3.0*distance, minus the suspicious-frontier penalty)
  weighted   same, scorer 'weighted' (3.0*info_gain - 1.0*distance)
  In utility/weighted the frontier attributes are still random (size_m 0.5-6, clearance 0.2-2.0, info_gain 0-5) and distance is
  straight-line, not the path distance the node uses; _pick's filters (hard suspicious ratio, costmap, min distance) are not
  applied. Scores can be negative, so those modes report distance / frontier size / score difference, never a % of score.

Commands (from src/rosnav_bot/scripts):
  python3 bench_frontier_allocation.py 500 hungarian          # base, pre-fix imitation
  python3 bench_frontier_allocation.py 500 hungarian_reserved # base, current code
  python3 bench_frontier_allocation.py 500 hungarian_reserved --dist clustered
"""
import argparse
import importlib.util
import math
import os
import random
import statistics

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location('frontier_coordinator', os.path.join(_HERE, 'frontier_coordinator.py'))
fc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fc)

ASSIGN_R = 1.0
DISTS = ('base', 'large', 'eight', 'fewer', 'clustered')
SCORERS = ('synthetic', 'utility', 'weighted')


class _Log:
    def info(self, *a, **k):
        pass


def _bare(table):
    o = fc.FrontierCoordinator.__new__(fc.FrontierCoordinator)
    o.get_logger = _Log
    o._assign_r = ASSIGN_R

    def _pick(frontiers, pos, ns, collect=None):
        for score, xy in table[ns]:
            collect.append((score, xy, {'score': score, 'distance': 1.0, 'size_m': 1.0, 'info_gain': 0.0,
                                        'clearance': 0.0, 'suspicious_ratio': 0.0}))
        return None, {'n_total': 0}
    o._pick = _pick
    return o


def _near(xy, pts):
    return any(math.hypot(xy[0] - p[0], xy[1] - p[1]) < ASSIGN_R for p in pts)


def _greedy_pick(cands, taken):
    for s, xy in sorted(cands, key=lambda c: -c[0]):
        if not _near(xy, taken):
            return s, xy
    return None


def _production_scorer(name):
    o = fc.FrontierCoordinator.__new__(fc.FrontierCoordinator)
    o._scorer, o._assigned, o._sep_w, o._sep_r = name, {}, 0.0, 6.0
    o._suspect_ratio, o._suspect_penalty, o._suspect_buffer = 3.0, 8.0, 1.0
    o._gain_scale, o._potential_scale, o._info_w, o._dist_w = 1.0, 3.0, 3.0, 1.0
    o._hyst_r, o._hyst_gain = 2.0, 1.5
    return o


def run(table, robots, mode, goals_out=None):
    """-> (total score, conflicting goal pairs, robots assigned). goals_out (dict) receives robot -> goal xy."""
    plan = {}
    if mode in ('hungarian', 'hungarian_reserved') and len(robots) > 1:
        plan = _bare(table)._hungarian_plan(robots, [], {r: (0, 0) for r in robots})
    assigned, total = {}, 0.0
    reserved = {r: v[0] for r, v in plan.items()} if mode == 'hungarian_reserved' else {}
    for r in robots:
        if r in plan:
            xy = plan[r][0]
            s = next(sc for sc, p in table[r] if p == xy)
        else:
            g = _greedy_pick(table[r], [v[1] for k, v in assigned.items() if k != r] +
                             [xy_ for k, xy_ in reserved.items() if k != r])
            if g is None:
                continue
            s, xy = g
        assigned[r] = (s, xy)
        total += s
    if goals_out is not None:
        goals_out.update({r: v[1] for r, v in assigned.items()})
    goals = [v[1] for v in assigned.values()]
    conflicts = sum(1 for i in range(len(goals)) for j in range(i + 1, len(goals))
                    if math.hypot(goals[i][0] - goals[j][0], goals[i][1] - goals[j][1]) < ASSIGN_R)
    return total, conflicts, len(assigned)


def scenario(seed, dist='base', _geom=None):
    rng = random.Random(seed)
    size = 40.0 if dist == 'large' else 15.0
    if dist == 'eight':
        nr, nf = 8, rng.randint(8, 16)
    elif dist == 'fewer':
        nr = rng.randint(3, 8)
        nf = rng.randint(1, nr - 1)
    elif dist == 'large':
        nr, nf = rng.randint(2, 6), rng.randint(5, 30)
    elif dist == 'clustered':
        nr, nf = rng.randint(2, 6), rng.randint(6, 14)
    else:
        nr, nf = rng.randint(2, 6), rng.randint(3, 12)
    robots = [f'r{i + 1}' for i in range(nr)]
    pos = {r: (rng.uniform(0, size), rng.uniform(0, size)) for r in robots}
    if dist == 'clustered':
        centres = [(rng.uniform(2, size - 2), rng.uniform(2, size - 2)) for _ in range(rng.randint(2, 3))]
        fr = []
        for _ in range(nf):
            cx, cy = rng.choice(centres)
            fr.append((min(size, max(0, rng.gauss(cx, 1.2))), min(size, max(0, rng.gauss(cy, 1.2)))))
    else:
        fr = [(rng.uniform(0, size), rng.uniform(0, size)) for _ in range(nf)]
    if _geom is not None:
        _geom.append((pos, fr))
    val = [rng.uniform(1, 10) for _ in range(nf)]
    table = {r: [(val[j] + 12 - 0.5 * math.hypot(pos[r][0] - f[0], pos[r][1] - f[1]), f)
                 for j, f in enumerate(fr)] for r in robots}
    return table, robots, nr, nf


def _geometry(seed, dist):
    geom = []
    table = scenario(seed, dist, geom)[0]
    return table, geom[0][0], geom[0][1]


def production_scenario(seed, dist, scorer):
    """Same robots / frontier positions as `scenario`, scored by the coordinator's own code. -> (table, robots, pos, size)."""
    robots = scenario(seed, dist)[1]
    _, pos, fr = _geometry(seed, dist)
    rng = random.Random(10_000_000 + seed)
    attrs = [{'point': f, 'size_m': rng.uniform(0.5, 6.0), 'clearance': rng.uniform(0.2, 2.0),
              'info_gain': rng.uniform(0.0, 5.0)} for f in fr]
    o = _production_scorer(scorer)
    table = {r: [(o._score_frontier(a, math.hypot(pos[r][0] - a['point'][0], pos[r][1] - a['point'][1]), r), a['point'])
                 for a in attrs] for r in robots}
    return table, robots, pos, {a['point']: a['size_m'] for a in attrs}


def benchmark_production(n=500, dist='base', scorer='utility'):
    """Greedy vs current (reserved) Hungarian under the production scorer."""
    tot = {m: {'score': 0.0, 'dist': 0.0, 'size': 0.0, 'assigned': 0, 'conflicts': 0} for m in ('greedy', 'hungarian_reserved')}
    better = worse = 0
    for seed in range(n):
        table, robots, pos, size = production_scenario(seed, dist, scorer)
        scores = {}
        for m, t in tot.items():
            goals = {}
            total, conflicts, assigned = run(table, robots, m, goals)
            scores[m] = total
            t['score'] += total
            t['conflicts'] += conflicts
            t['assigned'] += assigned
            t['dist'] += sum(math.hypot(pos[r][0] - g[0], pos[r][1] - g[1]) for r, g in goals.items())
            t['size'] += sum(size[g] for g in goals.values())
        better += scores['hungarian_reserved'] > scores['greedy'] + 1e-9
        worse += scores['hungarian_reserved'] < scores['greedy'] - 1e-9
    g, h = tot['greedy'], tot['hungarian_reserved']
    return {'n': n, 'dist': dist, 'scorer': scorer, 'greedy': g, 'hungarian': h, 'better': better, 'worse': worse,
            'score_diff_per_robot': (h['score'] - g['score']) / h['assigned'],
            'distance_pct': (h['dist'] - g['dist']) / g['dist'] * 100, 'size_pct': (h['size'] - g['size']) / g['size'] * 100}


def report_production(r):
    n, g, h = r['n'], r['greedy'], r['hungarian']
    eq = n - r['better'] - r['worse']
    return '\n'.join([
        f"N={n} scenarios (seeds 0..{n - 1}), dist={r['dist']}, scorer={r['scorer']} (FrontierCoordinator._score_frontier), "
        "random frontier attributes, straight-line distance",
        f"total score: greedy {g['score']:.1f}  hungarian {h['score']:.1f}  difference {r['score_diff_per_robot']:+.3f} per assigned robot",
        f"total straight-line distance: greedy {g['dist']:.1f} m  hungarian {h['dist']:.1f} m  ({r['distance_pct']:+.2f}%)",
        f"total frontier size assigned: greedy {g['size']:.1f} m  hungarian {h['size']:.1f} m  ({r['size_pct']:+.2f}%)",
        f"hungarian better {r['better']} ({r['better'] / n:.1%})  equal {eq} ({eq / n:.1%})  worse {r['worse']} ({r['worse'] / n:.1%})",
        f"robots assigned: greedy {g['assigned']}  hungarian {h['assigned']}; conflicting pairs: greedy {g['conflicts']}  hungarian {h['conflicts']}"])


def benchmark(n=500, mode='hungarian_reserved', dist='base'):
    rows = []
    for seed in range(n):
        table, robots, nr, nf = scenario(seed, dist)
        rows.append((run(table, robots, 'greedy'), run(table, robots, mode), nr, nf))
    gt, ht = sum(r[0][0] for r in rows), sum(r[1][0] for r in rows)
    per = [(h[0] - g[0]) / g[0] * 100 for g, h, *_ in rows if g[0] > 0]
    return {
        'n': n, 'mode': mode, 'dist': dist, 'greedy_total': gt, 'hungarian_total': ht, 'gain_pct': (ht - gt) / gt * 100,
        'per_mean': statistics.mean(per), 'per_median': statistics.median(per), 'per_min': min(per), 'per_max': max(per),
        'better': sum(1 for g, h, *_ in rows if h[0] > g[0] + 1e-9),
        'worse': sum(1 for g, h, *_ in rows if h[0] < g[0] - 1e-9),
        'greedy_conflicts': sum(r[0][1] for r in rows), 'conflicts': sum(r[1][1] for r in rows),
        'conflict_scenarios': sum(1 for r in rows if r[1][1]),
        'greedy_assigned': sum(r[0][2] for r in rows), 'assigned': sum(r[1][2] for r in rows),
        'robots': sum(r[2] for r in rows),
        'by_robots': {k: statistics.mean((h[0] - g[0]) / g[0] * 100 for g, h, nr, _ in rows if nr == k and g[0] > 0)
                      for k in sorted({r[2] for r in rows})},
    }


def report(r):
    n = r['n']
    equal = n - r['better'] - r['worse']
    out = [f"N={n} scenarios (seeds 0..{n - 1}), dist={r['dist']}, mode={r['mode']}, assign_radius={ASSIGN_R}, "
           "score = value + 12 - 0.5*dist (synthetic)",
           f"total score: greedy {r['greedy_total']:.1f}  hungarian {r['hungarian_total']:.1f}  gain {r['gain_pct']:+.2f}% (sum-weighted)",
           f"per-scenario gain: mean {r['per_mean']:+.2f}%  median {r['per_median']:+.2f}%  min {r['per_min']:+.2f}%  max {r['per_max']:+.2f}%",
           f"hungarian strictly better {r['better']} ({r['better'] / n:.1%})  equal {equal} ({equal / n:.1%})  "
           f"worse {r['worse']} ({r['worse'] / n:.1%})",
           f"conflicting goal pairs (within assign_radius): greedy {r['greedy_conflicts']}  hungarian {r['conflicts']}; "
           f"scenarios with >=1 hungarian conflict: {r['conflict_scenarios']}",
           f"robots assigned: greedy {r['greedy_assigned']}  hungarian {r['assigned']}  (of {r['robots']})",
           'mean gain by robot count: ' + ', '.join(f'{k}: {v:+.2f}%' for k, v in r['by_robots'].items())]
    return '\n'.join(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('n', type=int, nargs='?', default=500)
    ap.add_argument('mode', nargs='?', default='hungarian_reserved', choices=('hungarian', 'hungarian_reserved'))
    ap.add_argument('--dist', default='base', choices=DISTS)
    ap.add_argument('--scorer', default='synthetic', choices=SCORERS)
    a = ap.parse_args()
    if a.scorer == 'synthetic':
        print(report(benchmark(a.n, a.mode, a.dist)))
    else:
        print(report_production(benchmark_production(a.n, a.dist, a.scorer)))


if __name__ == '__main__':
    main()
