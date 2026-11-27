#!/usr/bin/env python3
"""Offline paired benchmark of frontier selection policies on FrontierSelectEnv.

  nice -n 19 python3 src/rosnav_bot/scripts/bench_frontier_policies.py \\
      --episodes 12 --out images/frontier_bench

Same spawn seeds for every policy; single process, torch threads=1. Writes results.md,
results.json and chart.png. Metrics: final coverage, steps to 90% (censored at max_steps,
fraction reached reported), path length (sum of straight-line hops, env semantics).
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from rosnav_bot.rl.frontier_env import FrontierSelectEnv  # noqa: E402
from rosnav_bot.rl import frontier_policies as fp  # noqa: E402

MAPS = ['cafe', 'hospital', 'maze', 'obstacles']
HERE = os.path.dirname(os.path.abspath(__file__))


def run_episode(env, policy, seed, max_steps):
    obs, info = env.reset(seed=seed)
    k = env.k
    res = env.resolution
    prev = None
    failed = []
    cov_trace, path, steps_90, last_cov = [], 0.0, None, 0.0
    path_90 = None
    if hasattr(policy, 'reset'):
        policy.reset()
    for step in range(1, max_steps + 1):
        mask = info['action_mask']
        if not mask.any():
            break
        feat = obs.reshape(k, -1)
        goals = np.full((k, 2), 1e9)
        for i, c in enumerate(env._candidates):
            goals[i] = env.world_xy(*c['goal_rc'])
        pos = np.array(env.world_xy(*env._pos_rc))
        ctx = {'goals': goals, 'pos': pos, 'prev': prev, 'failed': failed}
        a = policy(feat, mask, ctx)
        d = float(feat[a, 0])
        obs, _, term, trunc, info = env.step(a)
        path += d
        cov = info.get('coverage', last_cov)
        if cov - last_cov < 1e-4:  # wasted hop -> treat like a failed goal (live cooldown)
            failed.append((goals[a, 0], goals[a, 1], fp.failed_radius(d)))
        last_cov = cov
        cov_trace.append(cov)
        prev = pos
        if steps_90 is None and cov >= 0.9:
            steps_90, path_90 = step, path
        if term or trunc:
            break
    return last_cov, steps_90, path, cov_trace, (path_90 if path_90 is not None else path)


def boot_ci(x, n=2000, seed=0):
    x = np.asarray(x, float)
    rng = np.random.default_rng(seed)
    m = rng.choice(x, (n, len(x))).mean(1)
    return float(x.mean()), float(np.percentile(m, 2.5)), float(np.percentile(m, 97.5))


def paired_ci(a, b, n=2000, seed=1):
    d = np.asarray(a, float) - np.asarray(b, float)
    rng = np.random.default_rng(seed)
    m = rng.choice(d, (n, len(d))).mean(1)
    return float(d.mean()), float(np.percentile(m, 2.5)), float(np.percentile(m, 97.5))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--episodes', type=int, default=12)
    ap.add_argument('--max-steps', type=int, default=80)
    ap.add_argument('--sensor-range', type=float, default=3.5)
    ap.add_argument('--tag', default='')
    ap.add_argument('--maps', nargs='+', default=MAPS)
    ap.add_argument('--model', default=os.path.join(
        HERE, '../../../runs/rl/ppo_frontier_v2/ppo_frontier.pt'))
    ap.add_argument('--out', default=os.path.join(HERE, '../../../images/frontier_bench'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    pols = fp.all_policies(os.path.abspath(a.model))
    print('policies:', list(pols), flush=True)
    data = {}
    for m in a.maps:
        env = FrontierSelectEnv(os.path.join(HERE, '..', 'maps', f'map_{m}.yaml'),
                                k_candidates=8, max_steps=a.max_steps, target_coverage=2.0,
                                sensor_range=a.sensor_range)
        data[m] = {}
        for n, p in pols.items():
            rs = [run_episode(env, p, 1000 + s, a.max_steps) for s in range(a.episodes)]
            data[m][n] = {
                'cov': [r[0] for r in rs],
                's90': [r[1] if r[1] is not None else a.max_steps + 1 for r in rs],
                'reached': [r[1] is not None for r in rs],
                'path': [r[2] for r in rs],
                'path90': [r[4] for r in rs],
                'trace': [r[3] + [r[3][-1] if r[3] else 0.0] * (a.max_steps - len(r[3]))
                          for r in rs],
            }
            c = np.mean(data[m][n]['cov'])
            print(f'{m:10s} {n:18s} cov={c:.3f} s90={np.mean(data[m][n]["s90"]):.1f}',
                  flush=True)
    with open(os.path.join(a.out, f'results{a.tag}.json'), 'w') as f:
        json.dump({'episodes': a.episodes, 'max_steps': a.max_steps,
                   'summary': {m: {n: {k: v for k, v in d.items() if k != 'trace'}
                                   for n, d in dm.items()} for m, dm in data.items()}}, f)
    write_md(a, data, pols)
    write_chart(a, data, pols)


def fmt(x):
    m, lo, hi = boot_ci(x)
    return m, lo, hi


def write_md(a, data, pols):
    L = [f'# Offline frontier-policy benchmark (sensor {a.sensor_range} m)',
         '',
         f'{a.episodes} paired episodes per map (spawn seeds 1000..{999 + a.episodes}, identical '
         f'across policies), `FrontierSelectEnv` k=8, max_steps={a.max_steps}, sensor {a.sensor_range} m. '
         'Cells are mean [95% bootstrap CI]. Steps-to-90% is censored at max_steps+1 for '
         'episodes that never reach 90% (reach rate in the last column). Path length = sum of '
         'straight-line hop lengths (env semantics), metres. Offline env: no Nav2, no path '
         'planning, no real failures; "failed cooldown" is emulated by blacklisting a goal '
         'whose hop revealed nothing.', '']
    for m, dm in data.items():
        L += [f'## {m}', '',
              '| policy | final coverage | steps to 90% | path to 90% (m) | total path (m) | reached 90% |',
              '|---|---|---|---|---|---|']
        for n, d in dm.items():
            c, s, p = fmt(d['cov']), fmt(d['s90']), fmt(d['path'])
            q = fmt(d['path90'])
            L.append(f'| {n} | {c[0]:.3f} [{c[1]:.3f}, {c[2]:.3f}] | '
                     f'{s[0]:.1f} [{s[1]:.1f}, {s[2]:.1f}] | '
                     f'{q[0]:.0f} [{q[1]:.0f}, {q[2]:.0f}] | '
                     f'{p[0]:.0f} [{p[1]:.0f}, {p[2]:.0f}] | '
                     f'{sum(d["reached"])}/{len(d["reached"])} |')
        L.append('')
        base = 'live'
        L += [f'Paired difference vs `{base}` (policy - live; 95% CI), steps / path-to-90% (m) / coverage:', '']
        for n, d in dm.items():
            if n == base:
                continue
            ds = paired_ci(d['s90'], dm[base]['s90'])
            dc = paired_ci(d['cov'], dm[base]['cov'])
            dp = paired_ci(d['path90'], dm[base]['path90'])
            sig_p = 'SIG' if dp[1] > 0 or dp[2] < 0 else 'ns'
            sig_s = 'SIG' if ds[1] > 0 or ds[2] < 0 else 'ns'
            sig_c = 'SIG' if dc[1] > 0 or dc[2] < 0 else 'ns'
            L.append(f'- {n}: steps {ds[0]:+.1f} [{ds[1]:+.1f}, {ds[2]:+.1f}] {sig_s}; '
                     f'path90 {dp[0]:+.1f} [{dp[1]:+.1f}, {dp[2]:+.1f}] {sig_p}; '
                     f'coverage {dc[0]:+.3f} [{dc[1]:+.3f}, {dc[2]:+.3f}] {sig_c}')
        L.append('')
    with open(os.path.join(a.out, f'results{a.tag}.md'), 'w') as f:
        f.write('\n'.join(L))


def write_chart(a, data, pols):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    names = list(pols)
    maps = list(data)
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.2))
    w = 0.8 / len(names)
    cmap = plt.get_cmap('tab10')
    for i, n in enumerate(names):
        ms, errs = [], []
        for m in maps:
            mu, lo, hi = boot_ci(data[m][n]['s90'])
            ms.append(mu)
            errs.append([mu - lo, hi - mu])
        ax[0].bar(np.arange(len(maps)) + i * w, ms, w, yerr=np.array(errs).T,
                  label=n, color=cmap(i), capsize=1.5, error_kw={'lw': 0.7})
    ax[0].set_xticks(np.arange(len(maps)) + 0.4 - w / 2)
    ax[0].set_xticklabels(maps)
    ax[0].set_ylabel('steps to 90% coverage (lower better)')
    ax[0].set_title('Steps to 90% (mean, 95% CI)')
    for i, n in enumerate(names):
        ms, errs = [], []
        for m in maps:
            mu, lo, hi = boot_ci(data[m][n]['path90'])
            ms.append(mu)
            errs.append([mu - lo, hi - mu])
        ax[1].bar(np.arange(len(maps)) + i * w, ms, w, yerr=np.array(errs).T,
                  color=cmap(i), capsize=1.5, error_kw={'lw': 0.7})
    ax[1].set_xticks(np.arange(len(maps)) + 0.4 - w / 2)
    ax[1].set_xticklabels(maps)
    ax[1].set_yscale('log')
    ax[1].set_ylabel('path to 90% (m, log)')
    ax[1].set_title('Path to 90% (mean, 95% CI)')
    for i, n in enumerate(names):
        tr = np.mean([np.mean(data[m][n]['trace'], 0) for m in maps], 0)
        ax[2].plot(np.arange(1, len(tr) + 1), tr, label=n, color=cmap(i), lw=1.4)
    ax[2].set_xlabel('step')
    ax[2].set_title('Coverage vs step (mean over maps)')
    ax[2].axhline(0.9, color='gray', lw=0.5, ls='--')
    ax[0].legend(fontsize=7, ncol=2)
    fig.tight_layout()
    fig.savefig(os.path.join(a.out, f'chart{a.tag}.png'), dpi=80)


if __name__ == '__main__':
    main()
