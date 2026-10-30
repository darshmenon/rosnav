#!/usr/bin/env python3
"""eval_scan_nav_policy.py — success / collision rate of ppo_scan_nav.pt on ScanNavEnv.

  python3 src/rosnav_bot/scripts/eval_scan_nav_policy.py \\
      --models runs/rl/ppo_scan_nav_maze2/ppo_scan_nav.pt --map <map.yaml>
"""
import argparse

import numpy as np
import torch

from rosnav_bot.rl.policy import ActorCritic
from rosnav_bot.rl.scan_nav_env import ScanNavEnv


def run(env, act, seed):
    obs, _ = env.reset(seed=seed)
    coll = 0
    while True:
        obs, _, term, trunc, info = env.step(act(obs))
        coll += int(info['collided'])
        if term or trunc:
            return term, coll, env._steps


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--models', nargs='*', default=[])
    ap.add_argument('--map', required=True)
    ap.add_argument('--episodes', type=int, default=50)
    ap.add_argument('--goal-max-dist', type=float, default=None)
    a = ap.parse_args()

    pols = {}
    for p in a.models:
        ck = torch.load(p, map_location='cpu')
        net = ActorCritic(ck['obs_dim'], ck['act_dim'])
        net.load_state_dict(ck['state_dict'])
        net.eval()
        def act(o, net=net):
            with torch.no_grad():
                return net.act(torch.as_tensor(o, dtype=torch.float32).unsqueeze(0),
                               deterministic=True)[0][0].numpy()
        pols[p] = (act, ck['n_beams'])
    env = ScanNavEnv(a.map, n_beams=next(iter(pols.values()))[1] if pols else 36,
                     goal_max_dist=a.goal_max_dist)
    pols['random'] = (lambda o: np.random.uniform(-1, 1, 2), None)
    for n, (act, _) in pols.items():
        res = [run(env, act, s) for s in range(a.episodes)]
        print(f'{n}: success={np.mean([r[0] for r in res]):.2f} '
              f'collisions/ep={np.mean([r[1] for r in res]):.1f} '
              f'steps={np.mean([r[2] for r in res]):.0f}')


if __name__ == '__main__':
    main()
