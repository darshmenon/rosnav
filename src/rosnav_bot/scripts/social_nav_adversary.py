#!/usr/bin/env python3
"""
social_nav_adversary.py -- conformal risk budget + adversarial crowd search (pure Python, no ROS).

Imports (never edits) social_nav_core.py and online_conformal.py.

1. ConformalRiskBudget: online (1-alpha) quantile of the REALISED constant-velocity prediction
   error per horizon step (OnlineConformal pinball update). ConformalRiskPlanner is a RiskPlanner
   whose per-step pedestrian uncertainty (sigma_k = r_k / 2.146) is replaced each cycle by that
   quantile, so the risk margin follows the predictor's actual quality. Same plan() interface,
   so social_nav_core.simulate() runs it unchanged.
2. Adversarial crowds: a bounded CEM search over per-pedestrian (x, y, speed, heading) that
   maximises a planner's near-miss / intrusion score. Speeds are capped at plausible walking
   speeds. Fixed seeds throughout. A 'failure atlas' bins where near-misses happen.

  nice -n 19 python3 social_nav_adversary.py            # full run -> images/social_nav_adversary/
  nice -n 19 python3 social_nav_adversary.py --quick
"""
import contextlib
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import social_nav_core as snc  # noqa: E402
from online_conformal import OnlineConformal  # noqa: E402

DT, ROBOT_R, PED_R = snc.DT, snc.ROBOT_R, snc.PED_R
N_STEPS, STEP_S = 8, 0.4                   # conformal grid: 0.4 s .. 3.2 s
LAG = int(round(STEP_S / DT))              # sim steps per conformal step
Z90 = 2.146                                # r = Z90 * sigma for a 2D isotropic Gaussian at alpha=0.1


def nominal_radii():
    """Radii equivalent to RiskPlanner's hand-set sigma(t) = 0.10 + 0.22 t."""
    return Z90 * (0.10 + 0.22 * STEP_S * np.arange(1, N_STEPS + 1))


class ConformalRiskBudget:
    """Online conformal quantile of CV prediction error, fed by watching the crowd."""

    def __init__(self, alpha=0.1, eta=0.05, radii0=None, every=2):
        self.alpha, self.every = alpha, every
        self.conf = OnlineConformal(nominal_radii() if radii0 is None else radii0, alpha=alpha, eta=eta)
        self.reset()

    def reset(self):
        self.hist, self.tick, self.n_updates = [], 0, 0

    def observe(self, ped_pos, ped_vel):
        """Call once per control cycle with current pedestrian positions/velocities."""
        pos = np.asarray(ped_pos, float).reshape(-1, 2)
        vel = np.asarray(ped_vel, float).reshape(-1, 2)
        self.tick += 1
        if len(pos) and self.tick % self.every == 0:
            for k in range(N_STEPS):
                L = LAG * (k + 1)
                if len(self.hist) < L:
                    break
                p0, v0 = self.hist[-L]
                if len(p0) != len(pos):
                    continue
                err = np.linalg.norm(pos - (p0 + v0 * L * DT), axis=1)
                for e in err:
                    self.conf.update(k, float(e))
                    self.n_updates += 1
        self.hist.append((pos.copy(), vel.copy()))
        if len(self.hist) > LAG * N_STEPS + 1:
            self.hist.pop(0)

    @property
    def radii(self):
        return self.conf.radii

    @property
    def sigma(self):
        return self.conf.radii / Z90


class ConformalRiskPlanner(snc.RiskPlanner):
    """RiskPlanner with an online conformal risk budget (same plan() interface)."""

    def __init__(self, alpha=0.1, eta=0.05, name='conf-risk', **kw):
        self.budget = ConformalRiskBudget(alpha=alpha, eta=eta)
        super().__init__(conformal_radii=self.budget.radii, name=name, **kw)

    # simulate() reassigns planner.rng at every episode start -> use it as the reset hook
    @property
    def rng(self):
        return self._rng

    @rng.setter
    def rng(self, v):
        self._rng = v
        b = getattr(self, 'budget', None)
        if b is not None:
            b.reset()
            b.conf = OnlineConformal(nominal_radii(), alpha=b.alpha, eta=b.conf.eta)
            self.conf = b.radii / Z90

    def plan(self, pos, goal, ped_pos, ped_vel, preds=None, futures=None):
        self.budget.observe(ped_pos, ped_vel)
        self.conf = self.budget.sigma.copy()       # RiskPlanner stores sigma in .conf
        return super().plan(pos, goal, ped_pos, ped_vel, preds, futures)


def make_planners(seed=0):
    kw = dict(samples=12, seed=seed)
    return [snc.ReactivePlanner(), snc.PredictivePlanner(),
            snc.RiskPlanner(name='risk', **kw), ConformalRiskPlanner(**kw)]


# ---------------------------------------------------------------- scenarios
N_PEDS = 6
SPEED_RANGE = (0.3, 1.5)                    # m/s, plausible walking speeds
LO = np.array([2.0, -3.5, SPEED_RANGE[0], -np.pi])
HI = np.array([8.5, 3.5, SPEED_RANGE[1], np.pi])


def decode(theta):
    """theta in [0,1]^(N*4) -> (ped_starts, ped_vels) within the plausible bounds."""
    u = np.clip(np.asarray(theta, float).reshape(-1, 4), 0, 1)
    p = LO + u * (HI - LO)
    starts = [row[:2].copy() for row in p]
    vels = [row[2] * np.array([np.cos(row[3]), np.sin(row[3])]) for row in p]
    return starts, vels


def random_theta(rng, n=N_PEDS):
    return rng.uniform(0, 1, n * 4)


@contextlib.contextmanager
def fixed_scenario(theta):
    """Make social_nav_core.simulate(kind='fixed') use the pedestrians encoded by theta."""
    orig = snc.make_scenario
    starts, vels = decode(theta)

    def mk(kind, rng, n_crowd=8):
        return (np.array([0.0, 0.0]), np.array([10.0, 0.0]),
                [s.copy() for s in starts], [v.copy() for v in vels])
    snc.make_scenario = mk
    try:
        yield
    finally:
        snc.make_scenario = orig


def run(planner, theta, seed=0, max_t=15.0, record=None):
    """simulate() on the scenario theta. record: list that receives (x,y,clearance) per cycle."""
    orig_plan = planner.plan
    if record is not None:
        def spy(pos, goal, ped_pos, ped_vel, *a, **k):
            c = min(np.linalg.norm(pos - p) for p in ped_pos) - ROBOT_R - PED_R
            record.append((float(pos[0]), float(pos[1]), float(c)))
            return orig_plan(pos, goal, ped_pos, ped_vel, *a, **k)
        planner.plan = spy
    try:
        with fixed_scenario(theta):
            return snc.simulate(planner, 'fixed', seed=seed, max_t=max_t)
    finally:
        planner.plan = orig_plan


NEAR = 0.2   # m clearance (surface to surface) counted as a near-miss


def score(log):
    """Higher = worse for the robot."""
    return (log['intrusion_s'] + 10.0 * log['collided']
            + 4.0 * max(0.0, 0.3 - log['min_clear']) + 0.1 * log['stalled_s'])


def fitness(planner, theta, seeds=(0, 1), max_t=15.0):
    return float(np.mean([score(run(planner, theta, s, max_t)) for s in seeds]))


def cem_search(planner, pop=12, iters=4, elite_frac=0.25, seeds=(0, 1), max_t=15.0, seed=123, top=5):
    """Bounded CEM over crowd parameters. Returns (top thetas best-first, best-fitness history)."""
    rng = np.random.default_rng(seed)
    d = N_PEDS * 4
    mu, sd = np.full(d, 0.5), np.full(d, 0.3)
    seen, hist = [], []
    for _ in range(iters):
        X = np.clip(rng.normal(mu, sd, (pop, d)), 0, 1)
        f = np.array([fitness(planner, x, seeds, max_t) for x in X])
        seen += list(zip(f, [x for x in X]))
        ne = max(2, int(pop * elite_frac))
        E = X[np.argsort(-f)[:ne]]
        mu, sd = E.mean(0), np.maximum(E.std(0), 0.05)
        hist.append(float(f.max()))
    seen.sort(key=lambda t: -t[0])
    return [x for _, x in seen[:top]], hist


# ---------------------------------------------------------------- evaluation
def evaluate_set(planner, thetas, seeds=(10, 11), max_t=15.0, record=None):
    runs = [run(planner, th, s, max_t, record) for th in thetas for s in seeds]
    return dict(
        collision=float(np.mean([r['collided'] for r in runs])),
        near_miss=float(np.mean([r['min_clear'] < NEAR for r in runs])),
        intrusion_s=float(np.mean([r['intrusion_s'] for r in runs])),
        success=float(np.mean([r['reached'] for r in runs])),
        stalled_s=float(np.mean([r['stalled_s'] for r in runs])),
        n=len(runs))


def atlas(ax, rec, title):
    pts = np.array([(x, y) for x, y, c in rec if c < NEAR]).reshape(-1, 2)
    H, xe, ye = np.histogram2d(pts[:, 0], pts[:, 1], bins=[22, 14], range=[[-1, 11], [-4, 4]]) \
        if len(pts) else (np.zeros((22, 14)), np.linspace(-1, 11, 23), np.linspace(-4, 4, 15))
    im = ax.imshow(H.T, origin='lower', extent=[-1, 11, -4, 4], cmap='magma_r', aspect='equal')
    ax.plot([0, 10], [0, 0], 'c--', lw=0.8)
    ax.plot([0, 10], [0, 0], 'c*', ms=0)
    ax.set_title(f'{title}\n{len(pts)} near-miss cycles', fontsize=8)
    ax.tick_params(labelsize=6)
    return im


def main(quick=False):
    import json
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'images', 'social_nav_adversary')
    os.makedirs(out, exist_ok=True)
    pop, iters, n_rand, top = (6, 2, 3, 3) if quick else (12, 4, 6, 5)
    planners = make_planners()
    rng = np.random.default_rng(7)
    rand_set = [random_theta(rng) for _ in range(n_rand)]
    adv_set = {}
    for pl in planners:
        adv_set[pl.name], h = cem_search(pl, pop=pop, iters=iters, top=top)
        print(pl.name, 'cem best-fitness by iter', [round(x, 2) for x in h], flush=True)
    res, recs = {}, {}
    for pl in planners:
        rr, ra = [], []
        res[(pl.name, 'random')] = evaluate_set(pl, rand_set, record=rr)
        res[(pl.name, 'adversarial(own)')] = evaluate_set(pl, adv_set[pl.name], record=ra)
        recs[pl.name] = (rr, ra)
        print(pl.name, res[(pl.name, 'random')], res[(pl.name, 'adversarial(own)')], flush=True)
    transfer = {(a, b): evaluate_set(pb, adv_set[a], seeds=(10,))
                for a in adv_set for pb in planners for b in [pb.name]}
    # atlas
    for pl in planners:
        fig, axs = plt.subplots(1, 2, figsize=(8, 3))
        rr, ra = recs[pl.name]
        im = atlas(axs[0], rr, f'{pl.name}: random crowds')
        atlas(axs[1], ra, f'{pl.name}: adversarial crowds')
        fig.colorbar(im, ax=axs, shrink=0.8, label='near-miss cycles')
        fig.savefig(os.path.join(out, f'atlas_{pl.name}.png'), dpi=110)
        plt.close(fig)
    # results.md
    L = ['# Social-nav adversary results', '',
         f'{n_rand} random crowds vs top-{top} CEM-adversarial crowds per planner ({N_PEDS} peds, speed '
         f'{SPEED_RANGE[0]}-{SPEED_RANGE[1]} m/s, 15 s episodes, 2 noise seeds each, held-out from the search seeds). '
         f'Near-miss = surface clearance < {NEAR} m. CEM pop={pop}, iters={iters}.', '',
         '| planner | scenarios | collision | near-miss | intrusion s | success | stalled s |', '|---|---|---|---|---|---|---|']
    for (p, k), r in res.items():
        L.append(f"| {p} | {k} | {r['collision']:.2f} | {r['near_miss']:.2f} | {r['intrusion_s']:.1f} | "
                 f"{r['success']:.2f} | {r['stalled_s']:.1f} |")
    L += ['', '## Transfer: rows = planner the crowd was tuned against, columns = planner run (near-miss rate / intrusion s)', '',
          '| tuned against | ' + ' | '.join(p.name for p in planners) + ' |', '|---|' + '---|' * len(planners)]
    for a in adv_set:
        L.append(f'| {a} | ' + ' | '.join(
            f"{transfer[(a, p.name)]['near_miss']:.2f} / {transfer[(a, p.name)]['intrusion_s']:.1f}" for p in planners) + ' |')
    L += ['', 'Atlas images: `atlas_<planner>.png` (left random, right adversarial). Caveat: crowds are constant-velocity '
          'bouncing walkers and some adversarial layouts may be near-unavoidable; the conformal budget is judged against '
          'the fixed-sigma `risk` planner, see the numbers above rather than a claim here.']
    open(os.path.join(out, 'results.md'), 'w').write('\n'.join(L) + '\n')
    json.dump({'adv': {k: [list(map(float, t)) for t in v] for k, v in adv_set.items()}},
              open(os.path.join(out, 'adversarial_crowds.json'), 'w'))
    print('\n'.join(L))


# ---------------------------------------------------------------- benchmark
def _ci(x, rate=False):
    """95% CI half-width: Wilson for rates, normal for means."""
    x = np.asarray(x, float)
    x = x[~np.isnan(x)]
    n = len(x)
    if n < 2:
        return float('nan')
    m = x.mean()
    if rate:
        z = 1.96
        den = 1 + z * z / n
        return float(z * np.sqrt(m * (1 - m) / n + z * z / (4 * n * n)) / den)
    return float(1.96 * x.std(ddof=1) / np.sqrt(n))


def _summ(logs):
    g = lambda k: [float(r[k]) for r in logs]
    ok = [r['t'] for r in logs if r['reached']]
    out = {}
    for k, rate in (('collided', True), ('reached', True), ('min_clear', False),
                    ('intrusion_s', False), ('jerk', False)):
        out[k] = (float(np.mean(g(k))), _ci(g(k), rate))
    out['t_goal'] = (float(np.mean(ok)) if ok else float('nan'), _ci(ok))
    out['n'] = len(logs)
    return out


def bench(quick=False, seeds=15, max_t=25.0):
    """All planners x {crossing, head_on, crowd, density sweep, adversarial}. Writes md + PNGs."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'images', 'social_nav_adversary')
    os.makedirs(out, exist_ok=True)
    if quick:
        seeds, max_t = 3, 12.0
    counts = (4, 8, 16, 24)
    pop, iters = (6, 2) if quick else (12, 4)
    planners = make_planners()
    names = [p.name for p in planners]
    # adversarial pool: top-3 crowds found against each planner, plus equally many random ones
    pool = []
    for pl in planners:
        th, _ = cem_search(pl, pop=pop, iters=iters, top=3)
        pool += th
    rng = np.random.default_rng(99)
    rand_pool = [random_theta(rng) for _ in pool]
    R = {}
    for pl in planners:
        for kind in ('crossing', 'head_on', 'crowd'):
            R[(pl.name, kind)] = _summ([snc.simulate(pl, kind, s, max_t=max_t) for s in range(seeds)])
        for n in counts:
            R[(pl.name, f'crowd{n}')] = _summ([snc.simulate(pl, 'crowd', s, max_t=max_t, n_crowd=n) for s in range(seeds)])
        for tag, P in (('random6', rand_pool), ('adversarial6', pool)):
            R[(pl.name, tag)] = _summ([run(pl, th, s, max_t) for th in P for s in (20, 21)])
        print('bench done', pl.name, flush=True)
    scen = ['crossing', 'head_on', 'crowd'] + [f'crowd{n}' for n in counts] + ['random6', 'adversarial6']
    f = lambda t, d=2: 'nan' if np.isnan(t[0]) else f'{t[0]:.{d}f}+-{t[1]:.{d}f}'
    L = ['# Social-nav benchmark (mean +- 95% CI)', '',
         f'{seeds} seeds per base/density cell, max_t={max_t}s. random6/adversarial6: {len(pool)} crowds x 2 noise seeds '
         '(adversarial = top-3 CEM crowds found against each planner, pooled). t_goal = successes only. '
         'Collision/success CIs are Wilson; seeds are shared across planners.', '',
         '| scenario | planner | n | collision | success | min clear m | intrusion s | t_goal s | jerk |', '|---|---|---|---|---|---|---|---|---|']
    for sc in scen:
        for p in names:
            r = R[(p, sc)]
            L.append(f"| {sc} | {p} | {r['n']} | {f(r['collided'])} | {f(r['reached'])} | {f(r['min_clear'])} | "
                     f"{f(r['intrusion_s'], 1)} | {f(r['t_goal'], 1)} | {f(r['jerk'])} |")
    open(os.path.join(out, 'benchmark.md'), 'w').write('\n'.join(L) + '\n')
    cols = plt.rcParams['axes.prop_cycle'].by_key()['color']
    base = ['crossing', 'head_on', 'crowd', 'random6', 'adversarial6']
    fig, axs = plt.subplots(1, 3, figsize=(13, 3.6))
    for ax, (k, lab) in zip(axs, (('collided', 'collision rate'), ('intrusion_s', 'intrusion s'), ('reached', 'success rate'))):
        w = 0.8 / len(names)
        for i, p in enumerate(names):
            ax.bar(np.arange(len(base)) + i * w, [R[(p, s)][k][0] for s in base], w,
                   yerr=[R[(p, s)][k][1] for s in base], label=p, color=cols[i], capsize=2)
        ax.set_xticks(np.arange(len(base)) + 0.3)
        ax.set_xticklabels(base, rotation=20, fontsize=7)
        ax.set_title(lab, fontsize=9)
    axs[0].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'bench_bars.png'), dpi=110)
    plt.close(fig)
    fig, axs = plt.subplots(1, 3, figsize=(13, 3.4))
    for ax, (k, lab) in zip(axs, (('collided', 'collision rate'), ('intrusion_s', 'intrusion s'), ('reached', 'success rate'))):
        for i, p in enumerate(names):
            ax.errorbar(counts, [R[(p, f'crowd{n}')][k][0] for n in counts],
                        yerr=[R[(p, f'crowd{n}')][k][1] for n in counts], label=p, color=cols[i], marker='o', capsize=2)
        ax.set_xlabel('pedestrians')
        ax.set_title(lab, fontsize=9)
    axs[0].legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'bench_density.png'), dpi=110)
    plt.close(fig)
    print('\n'.join(L))


if __name__ == '__main__':
    if '--bench' in sys.argv:
        bench(quick='--quick' in sys.argv)
    else:
        main(quick='--quick' in sys.argv)
