#!/usr/bin/env python3
"""train_social_ppo.py -- PPO for SocialNavMujocoEnv (kinematic MuJoCo crowd nav, diff-drive).

Reuses ActorCritic from rosnav_bot/rl/policy.py; the PPO loop is a minimal rewrite of
train_ppo.py with (a) several envs stepped sequentially in ONE process, (b) correct bootstrapping
at the rollout end and on timeouts.  Network outputs are in [-1, 1] and mapped to the env's
physical action: v = (a0+1)/2 * 0.8 m/s, w = a1 * 1.8 rad/s.

Run with /usr/bin/python3 (the only interpreter that has `mujoco`):
  OMP_NUM_THREADS=1 /usr/bin/python3 scripts/train_social_ppo.py --steps 400000 --out runs/social_ppo.pt
Writes <out> (checkpoint) and <out>.curve.json (per-update training curve).
"""
import argparse
import json
import os
import sys
import time

import numpy as np
import torch
import torch.nn as nn

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
sys.path.insert(0, ROOT)
from rosnav_bot.rl.policy import ActorCritic  # noqa: E402
from rosnav_bot.rl.social_nav_mujoco_env import SocialNavMujocoEnv, V_MAX, W_MAX  # noqa: E402


def to_physical(a):
    """Network action in [-1,1]^2 -> (v, w)."""
    a = np.clip(a, -1.0, 1.0)
    return np.array([(a[0] + 1.0) * 0.5 * V_MAX, a[1] * W_MAX], np.float32)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--steps', type=int, default=200_000)
    ap.add_argument('--out', default='runs/social_ppo.pt')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--n-envs', type=int, default=1)
    ap.add_argument('--rollout', type=int, default=1024, help='env steps per update (all envs together)')
    ap.add_argument('--epochs', type=int, default=4)
    ap.add_argument('--minibatch', type=int, default=256)
    ap.add_argument('--lr', type=float, default=3e-4)
    ap.add_argument('--gamma', type=float, default=0.99)
    ap.add_argument('--lam', type=float, default=0.95)
    ap.add_argument('--clip', type=float, default=0.2)
    ap.add_argument('--log-every', type=int, default=1, help='print every N updates')
    ap.add_argument('--map', default=None)
    a = ap.parse_args()

    torch.set_num_threads(1)
    torch.manual_seed(a.seed)
    np.random.seed(a.seed)
    kw = {'map_yaml': a.map} if a.map else {}
    envs = [SocialNavMujocoEnv(**kw) for _ in range(a.n_envs)]
    obs = np.stack([e.reset(seed=a.seed * 1000 + i)[0] for i, e in enumerate(envs)])
    obs_dim = envs[0].observation_space.shape[0]
    net = ActorCritic(obs_dim, 2)
    opt = torch.optim.Adam(net.parameters(), lr=a.lr)
    T = max(1, a.rollout // a.n_envs)
    ep_ret, ep_len = np.zeros(a.n_envs), np.zeros(a.n_envs, int)
    recent = []          # (return, len, success, collided)
    curve, step, upd, t0 = [], 0, 0, time.time()
    print(f'[train_social_ppo] steps={a.steps} n_envs={a.n_envs} obs_dim={obs_dim} '
          f'rollout={T}x{a.n_envs} seed={a.seed} -> {a.out}', flush=True)

    while step < a.steps:
        bo = np.zeros((T, a.n_envs, obs_dim), np.float32)
        ba = np.zeros((T, a.n_envs, 2), np.float32)
        blp, br, bv, bd = (np.zeros((T, a.n_envs), np.float32) for _ in range(4))
        for t in range(T):
            with torch.no_grad():
                act, val, logp = net.act(torch.as_tensor(obs, dtype=torch.float32))
            act, val, logp = act.numpy(), val.numpy(), logp.numpy()
            bo[t], ba[t], blp[t], bv[t] = obs, act, logp, val
            for i, e in enumerate(envs):
                o, r, term, trunc, info = e.step(to_physical(act[i]))
                ep_ret[i] += r
                ep_len[i] += 1
                if trunc:        # bootstrap through the time limit
                    with torch.no_grad():
                        r += a.gamma * net.forward(torch.as_tensor(o, dtype=torch.float32)[None])[2].item()
                br[t, i], bd[t, i] = r, float(term or trunc)
                if term or trunc:
                    recent.append((ep_ret[i], ep_len[i], bool(info['success']), bool(info['collided'])))
                    ep_ret[i], ep_len[i] = 0.0, 0
                    o, _ = e.reset()
                obs[i] = o
            step += a.n_envs
        with torch.no_grad():
            last_v = net.forward(torch.as_tensor(obs, dtype=torch.float32))[2].numpy()
        adv = np.zeros_like(br)
        last = np.zeros(a.n_envs, np.float32)
        for t in reversed(range(T)):
            nv = last_v if t == T - 1 else bv[t + 1]
            nt = 1.0 - bd[t]
            delta = br[t] + a.gamma * nv * nt - bv[t]
            last = delta + a.gamma * a.lam * nt * last
            adv[t] = last
        ret = adv + bv
        O = torch.as_tensor(bo.reshape(-1, obs_dim))
        A = torch.as_tensor(ba.reshape(-1, 2))
        LP = torch.as_tensor(blp.reshape(-1))
        AD = torch.as_tensor(adv.reshape(-1))
        AD = (AD - AD.mean()) / (AD.std() + 1e-8)
        RT = torch.as_tensor(ret.reshape(-1))
        idx = np.arange(len(O))
        for _ in range(a.epochs):
            np.random.shuffle(idx)
            for s in range(0, len(idx), a.minibatch):
                mb = idx[s:s + a.minibatch]
                logp, ent, v = net.evaluate(O[mb], A[mb])
                ratio = (logp - LP[mb]).exp()
                pl = -torch.min(ratio * AD[mb], torch.clamp(ratio, 1 - a.clip, 1 + a.clip) * AD[mb]).mean()
                loss = pl + 0.5 * nn.functional.mse_loss(v, RT[mb]) - 0.01 * ent.mean()
                opt.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(net.parameters(), 0.5)
                opt.step()
        upd += 1
        w = recent[-50:]
        row = {'update': upd, 'step': step, 'episodes': len(recent), 'time_s': round(time.time() - t0, 1),
               'ep_return': float(np.mean([x[0] for x in w])) if w else None,
               'ep_len': float(np.mean([x[1] for x in w])) if w else None,
               'success_rate': float(np.mean([x[2] for x in w])) if w else None,
               'collision_rate': float(np.mean([x[3] for x in w])) if w else None,
               'log_std': net.log_std.detach().numpy().round(3).tolist()}
        curve.append(row)
        if upd % a.log_every == 0:
            f = lambda x: 'n/a' if x is None else f'{x:.2f}'
            print(f"[train_social_ppo] upd={upd} step={step}/{a.steps} eps={len(recent)} "
                  f"ret={f(row['ep_return'])} len={f(row['ep_len'])} succ={f(row['success_rate'])} "
                  f"coll={f(row['collision_rate'])} std={row['log_std']} "
                  f"{step / (time.time() - t0):.0f} sps", flush=True)

    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    torch.save({'state_dict': net.state_dict(), 'obs_dim': obs_dim, 'act_dim': 2, 'v_max': V_MAX,
                'w_max': W_MAX, 'n_beams': 36, 'steps': step, 'seed': a.seed}, a.out)
    json.dump(curve, open(a.out + '.curve.json', 'w'), indent=1)
    print(f'[train_social_ppo] saved {a.out} (+ .curve.json); {step} steps in {time.time() - t0:.0f}s '
          f'({step / (time.time() - t0):.0f} steps/s)', flush=True)


if __name__ == '__main__':
    main()
