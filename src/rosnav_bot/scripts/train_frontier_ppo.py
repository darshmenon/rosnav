#!/usr/bin/env python3
"""
train_frontier_ppo.py — train a learned frontier-selection scorer (no Gazebo).

Trains rl/frontier_policy.py's FrontierScorer on rl/frontier_env.py's
FrontierSelectEnv, rotating across several saved maps for cross-layout
generalization. Mirrors train_ppo.py's pure-PyTorch PPO loop, adapted for a
masked-categorical action (pick 1 of K candidate frontiers) instead of a
continuous [v, w] action.

  # smoke (~seconds on CPU)
  python3 src/rosnav_bot/scripts/train_frontier_ppo.py --smoke

  # longer run across several maps
  python3 src/rosnav_bot/scripts/train_frontier_ppo.py \\
      --maps src/rosnav_bot/maps/map_house_builtin.yaml \\
             src/rosnav_bot/maps/map_maze.yaml \\
             src/rosnav_bot/maps/map_bench_room_small.yaml \\
             src/rosnav_bot/maps/map_corridor.yaml \\
      --timesteps 200000 --out runs/rl/ppo_frontier

Deploy:
  ros2 launch rosnav_bot slam_nav.launch.py world_name:=house explore:=true \\
      explorer:=frontier frontier_scorer:=learned \\
      learned_model_path:=runs/rl/ppo_frontier/ppo_frontier.pt
"""

from __future__ import annotations

import argparse
import glob
import os
import sys

import numpy as np
import torch
import torch.nn as nn


def _gae(rewards, values, dones, gamma=0.99, lam=0.95):
    adv = np.zeros_like(rewards, dtype=np.float32)
    last = 0.0
    for t in reversed(range(len(rewards))):
        next_v = 0.0 if t == len(rewards) - 1 else values[t + 1]
        next_nonterminal = 1.0 - float(dones[t])
        delta = rewards[t] + gamma * next_v * next_nonterminal - values[t]
        last = delta + gamma * lam * next_nonterminal * last
        adv[t] = last
    returns = adv + values
    return adv, returns


class _RotatingEnv:
    """Picks a new map from `map_yamls` each episode reset — same env API,
    keeps the network's input shape (k * n_features) fixed across maps."""

    def __init__(self, map_yamls, k_candidates, max_steps, seed):
        from rosnav_bot.rl.frontier_env import FrontierSelectEnv
        self._cls = FrontierSelectEnv
        self.map_yamls = map_yamls
        self.k = k_candidates
        self.max_steps = max_steps
        self._rng = np.random.default_rng(seed)
        self.env = self._cls(map_yamls[0], k_candidates=k_candidates, max_steps=max_steps)
        self.observation_space = self.env.observation_space
        self.action_space = self.env.action_space

    def reset(self, seed=None):
        map_yaml = self.map_yamls[self._rng.integers(0, len(self.map_yamls))]
        self.env = self._cls(map_yaml, k_candidates=self.k, max_steps=self.max_steps, seed=seed)
        return self.env.reset(seed=seed)

    def step(self, action):
        return self.env.step(action)


def train_torch(env, timesteps: int, out_dir: str, seed: int, device: str, k: int, n_features: int):
    from rosnav_bot.rl.frontier_policy import FrontierScorer

    net = FrontierScorer(n_features=n_features).to(device)
    opt = torch.optim.Adam(net.parameters(), lr=3e-4)

    rollout = 1024
    epochs = 4
    minibatch = 256
    clip = 0.2

    obs, info = env.reset(seed=seed)
    mask = info['action_mask']
    # Belt-and-suspenders on top of the env's own reset-retry (see
    # frontier_env.py): never let an all-invalid mask reach net.act(), which
    # would build Categorical(logits=all -inf) -> NaN.
    while not mask.any():
        obs, info = env.reset()
        mask = info['action_mask']
    ep_ret = 0.0
    ep_len = 0
    completed = []
    step = 0
    while step < timesteps:
        buf_o, buf_m, buf_a, buf_logp, buf_r, buf_v, buf_done = [], [], [], [], [], [], []
        for _ in range(rollout):
            ot = torch.as_tensor(obs, dtype=torch.float32, device=device).view(1, k, n_features)
            mt = torch.as_tensor(mask, dtype=torch.bool, device=device).unsqueeze(0)
            with torch.no_grad():
                action, value, logp = net.act(ot, mt, deterministic=False)
            a = int(action.item())
            next_obs, reward, terminated, truncated, next_info = env.step(a)
            done = terminated or truncated
            buf_o.append(obs)
            buf_m.append(mask)
            buf_a.append(a)
            buf_logp.append(float(logp.item()))
            buf_r.append(float(reward))
            buf_v.append(float(value.item()))
            buf_done.append(bool(done))
            ep_ret += reward
            ep_len += 1
            obs, mask = next_obs, next_info['action_mask']
            step += 1
            if done:
                completed.append((ep_ret, ep_len))
                ep_ret, ep_len = 0.0, 0
                obs, info = env.reset()
                mask = info['action_mask']
                while not mask.any():
                    obs, info = env.reset()
                    mask = info['action_mask']
            if step >= timesteps:
                break

        o = torch.as_tensor(np.asarray(buf_o), dtype=torch.float32, device=device).view(-1, k, n_features)
        m = torch.as_tensor(np.asarray(buf_m), dtype=torch.bool, device=device)
        a = torch.as_tensor(np.asarray(buf_a), dtype=torch.long, device=device)
        old_logp = torch.as_tensor(np.asarray(buf_logp), dtype=torch.float32, device=device)
        rewards = np.asarray(buf_r, dtype=np.float32)
        values = np.asarray(buf_v, dtype=np.float32)
        dones = np.asarray(buf_done, dtype=np.float32)
        adv, ret = _gae(rewards, values, dones)
        adv_t = torch.as_tensor(adv, dtype=torch.float32, device=device)
        ret_t = torch.as_tensor(ret, dtype=torch.float32, device=device)
        adv_t = (adv_t - adv_t.mean()) / (adv_t.std() + 1e-8)

        idx = np.arange(len(buf_o))
        for _ in range(epochs):
            np.random.shuffle(idx)
            for start in range(0, len(idx), minibatch):
                mb = idx[start:start + minibatch]
                logp, entropy, v = net.evaluate(o[mb], m[mb], a[mb])
                ratio = (logp - old_logp[mb]).exp()
                surr1 = ratio * adv_t[mb]
                surr2 = torch.clamp(ratio, 1 - clip, 1 + clip) * adv_t[mb]
                policy_loss = -torch.min(surr1, surr2).mean()
                value_loss = nn.functional.mse_loss(v, ret_t[mb])
                loss = policy_loss + 0.5 * value_loss - 0.01 * entropy.mean()
                opt.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(net.parameters(), 0.5)
                opt.step()

        if completed:
            rets = [r for r, _ in completed[-20:]]
            print(f'[train_frontier_ppo] step={step}/{timesteps} '
                  f'ep_ret_mean={np.mean(rets):.2f} last_len={completed[-1][1]}',
                  flush=True)

    os.makedirs(out_dir, exist_ok=True)
    save_path = os.path.join(out_dir, 'ppo_frontier.pt')
    torch.save({
        'state_dict': net.state_dict(),
        'n_features': n_features,
        'k_candidates': k,
    }, save_path)
    return save_path


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
    default_maps = [
        os.path.join(root, 'src/rosnav_bot/maps/map_maze.yaml'),
        os.path.join(root, 'src/rosnav_bot/maps/map_house_builtin.yaml'),
        os.path.join(root, 'src/rosnav_bot/maps/map_bench_room_small.yaml'),
        os.path.join(root, 'src/rosnav_bot/maps/map_corridor.yaml'),
    ]
    ap.add_argument('--maps', nargs='+', default=None,
                    help='Nav2 map yamls to rotate across (default: a handful of saved maps)')
    ap.add_argument('--timesteps', type=int, default=150_000)
    ap.add_argument('--out', default='runs/rl/ppo_frontier')
    ap.add_argument('--k-candidates', type=int, default=8)
    ap.add_argument('--max-steps', type=int, default=60)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--device', default='cpu')
    ap.add_argument('--smoke', action='store_true',
                    help='Short CPU run to verify the pipeline')
    args = ap.parse_args()

    pkg_root = os.path.join(root, 'src/rosnav_bot')
    if pkg_root not in sys.path:
        sys.path.insert(0, pkg_root)

    maps = args.maps or default_maps
    maps = [m for m in maps if os.path.isfile(m)]
    if not maps:
        # Fall back to any glob'd map yaml so --smoke still works on a
        # machine that hasn't saved these specific named maps yet.
        maps = sorted(glob.glob(os.path.join(root, 'src/rosnav_bot/maps/map_*.yaml')))
    if not maps:
        print('no map yaml files found under src/rosnav_bot/maps/', file=sys.stderr)
        sys.exit(2)
    print(f'[train_frontier_ppo] training maps: {maps}', flush=True)

    from rosnav_bot.rl.frontier_policy import N_FEATURES

    timesteps = args.timesteps
    out_dir = args.out
    if args.smoke:
        timesteps = 2048
        out_dir = 'runs/rl/_smoke_frontier'
        maps = maps[:1]
        print(f'[train_frontier_ppo] smoke: timesteps={timesteps} map={maps[0]}', flush=True)

    env = _RotatingEnv(maps, args.k_candidates, args.max_steps, args.seed)
    path = train_torch(env, timesteps, out_dir, args.seed, args.device,
                        args.k_candidates, N_FEATURES)
    print(f'\n[train_frontier_ppo] saved {path}')
    print('Deploy:')
    print('  ros2 launch rosnav_bot slam_nav.launch.py world_name:=<world> explore:=true \\')
    print('      explorer:=frontier frontier_scorer:=learned \\')
    print(f'      learned_model_path:={path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
