#!/usr/bin/env python3
"""eval_frontier_policy.py — learned frontier scorer vs heuristics on FrontierSelectEnv.

  python3 src/rosnav_bot/scripts/eval_frontier_policy.py \\
      --model runs/rl/ppo_frontier/ppo_frontier.pt --maps <map.yaml> ... --episodes 30

Reports mean final coverage and mean steps used per policy per map (same
spawn seeds for every policy, so the comparison is paired).
"""
import argparse

import numpy as np
import torch

from rosnav_bot.rl.frontier_env import FrontierSelectEnv
from rosnav_bot.rl.frontier_policy import FrontierScorer


def _heuristics(net):
    def nearest(obs, mask):
        d = np.where(mask, obs[:, 0], np.inf)
        return int(np.argmin(d))

    def max_gain(obs, mask):
        g = np.where(mask, obs[:, 2], -np.inf)
        return int(np.argmax(g))

    def gain_per_dist(obs, mask):
        g = np.where(mask, obs[:, 2] / (obs[:, 0] + 1.0), -np.inf)
        return int(np.argmax(g))

    rng = np.random.default_rng(0)

    def rand(obs, mask):
        return int(rng.choice(np.flatnonzero(mask)))

    pols = {'nearest': nearest, 'max_gain': max_gain,
            'gain/dist': gain_per_dist, 'random': rand}
    if net is not None:
        def learned(obs, mask):
            o = torch.as_tensor(obs, dtype=torch.float32).view(1, -1, obs.shape[-1])
            m = torch.as_tensor(mask).unsqueeze(0)
            with torch.no_grad():
                return int(net.act(o, m, deterministic=True)[0].item())
        pols['learned'] = learned
    return pols


def run(env, policy, seed):
    obs, info = env.reset(seed=seed)
    k = env.k
    cov, steps = 0.0, 0
    while True:
        mask = info['action_mask']
        if not mask.any():
            break
        a = policy(obs.reshape(k, -1), mask)
        obs, _, term, trunc, info = env.step(a)
        steps += 1
        cov = info.get('coverage', cov)
        if term or trunc:
            break
    return cov, steps


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--model', default=None)
    ap.add_argument('--maps', nargs='+', required=True)
    ap.add_argument('--episodes', type=int, default=30)
    ap.add_argument('--max-steps', type=int, default=60)
    ap.add_argument('--sensor-range', type=float, default=3.5)
    a = ap.parse_args()

    net = None
    k = 8
    if a.model:
        ck = torch.load(a.model, map_location='cpu')
        k = ck['k_candidates']
        net = FrontierScorer(ck['n_features'])
        net.load_state_dict(ck['state_dict'])
        net.eval()
    pols = _heuristics(net)
    tot = {n: [] for n in pols}
    for m in a.maps:
        env = FrontierSelectEnv(m, k_candidates=k, max_steps=a.max_steps,
                                sensor_range=a.sensor_range)
        row = []
        for n, p in pols.items():
            res = [run(env, p, s) for s in range(a.episodes)]
            c, st = np.mean([r[0] for r in res]), np.mean([r[1] for r in res])
            tot[n].append(c)
            row.append(f'{n}={c:.3f}/{st:.0f}st')
        print(f'{m.split("/")[-1]:32s} ' + '  '.join(row), flush=True)
    print('MEAN coverage: ' + '  '.join(f'{n}={np.mean(v):.3f}' for n, v in tot.items()))


if __name__ == '__main__':
    main()
