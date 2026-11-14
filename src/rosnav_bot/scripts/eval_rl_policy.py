#!/usr/bin/env python3
"""Offline goal-reaching eval on the ScanNavEnv: trained PPO policy vs LQR path tracking vs a naive go-to-goal P-controller.

  python3 scripts/eval_rl_policy.py --map maps/map_maze.yaml --model runs/rl/ppo_maze/ppo_scan_nav.pt --episodes 100

Same seeded start/goal/obstacles for every controller. LQR tracks the straight start->goal line (goal_max_dist keeps it
collision-free by construction), so it measures tracking only; PPO has to avoid the stamped obstacles from the scan.
Reports success rate, collision rate, mean steps-to-goal on successes.
"""
import argparse
import importlib.util
import math
import os
import sys

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
from rosnav_bot.rl.scan_nav_env import ScanNavEnv   # noqa: E402
from rosnav_bot.rl.policy import ActorCritic        # noqa: E402


def _load(name):
    s = importlib.util.spec_from_file_location(name, os.path.join(HERE, name + '.py'))
    m = importlib.util.module_from_spec(s)
    sys.modules[name] = m
    s.loader.exec_module(m)
    return m


def ppo_actor(path):
    ck = torch.load(path, map_location='cpu', weights_only=False)
    net = ActorCritic(int(ck.get('obs_dim', 39)), int(ck.get('act_dim', 2)))
    net.load_state_dict(ck['state_dict'])
    net.eval()

    def act(env, obs):
        with torch.no_grad():
            a = net.act(torch.as_tensor(obs, dtype=torch.float32).unsqueeze(0), deterministic=True)[0]
        return np.asarray(a).reshape(-1)[:2]
    return act


def p_actor():
    def act(env, obs):
        gx, gy = obs[-3], obs[-2]          # goal in robot frame
        ang = math.atan2(gy, gx)
        v = 0.8 if abs(ang) < 0.5 else 0.0
        return np.array([v, np.clip(2.0 * ang, -1, 1)])
    return act


def lqr_actor(lqr_core):
    st = {'ctl': None, 'v': 0.0, 'w': 0.0}

    def act(env, obs):
        if st['ctl'] is None or env._steps == 0:
            st.update(ctl=lqr_core.UnicycleLQR(lqr_core.LQRConfig(dt=env.dt, v_max=env.v_max, w_max=env.w_max)),
                      v=0.0, w=0.0, path=np.c_[np.linspace(env._xy[0], env._goal[0], 60),
                                               np.linspace(env._xy[1], env._goal[1], 60)])
        v, w, _ = st['ctl'].solve((env._xy[0], env._xy[1], env._yaw), st['v'], st['w'], st['path'])
        st['v'], st['w'] = v, w
        return np.array([v / env.v_max, w / env.w_max])
    return act


def run(actor, args, seed0):
    env = ScanNavEnv(args.map, goal_max_dist=args.goal_max_dist, seed=seed0)
    ok = col = 0
    steps = []
    for ep in range(args.episodes):
        obs, _ = env.reset(seed=seed0 + ep)
        hit = False
        while True:
            obs, _, term, trunc, info = env.step(actor(env, obs))
            hit |= info['collided']
            if term or trunc:
                break
        ok += int(term)
        col += int(hit)
        if term:
            steps.append(env._steps)
    n = args.episodes
    return 100.0 * ok / n, 100.0 * col / n, (float(np.mean(steps)) if steps else float('nan'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True)
    ap.add_argument('--model', default='')
    ap.add_argument('--episodes', type=int, default=100)
    ap.add_argument('--goal-max-dist', type=float, default=3.0)
    ap.add_argument('--seed', type=int, default=1000)
    args = ap.parse_args()
    _load('mpc_core')
    lqr = _load('lqr_core')
    arms = [('p-controller', p_actor()), ('lqr (straight line)', lqr_actor(lqr))]
    if args.model:
        arms.insert(0, ('ppo ' + os.path.basename(os.path.dirname(args.model)), ppo_actor(args.model)))
    print(f'{"controller":28} {"success%":>9} {"collided%":>10} {"mean steps":>11}   ({args.episodes} eps, goal<= {args.goal_max_dist} m)')
    for name, a in arms:
        s, c, m = run(a, args, args.seed)
        print(f'{name:28} {s:9.1f} {c:10.1f} {m:11.1f}')


if __name__ == '__main__':
    main()
