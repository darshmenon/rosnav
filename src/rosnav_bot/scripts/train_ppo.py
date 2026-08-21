#!/usr/bin/env python3
"""
train_ppo.py — train a local planner on ScanNavEnv (no Gazebo).

Uses a small pure-PyTorch PPO (stable-baselines3 segfaults on some
torch+CUDA stacks — keep --backend sb3 only if it works on your machine).

  # smoke (~seconds on CPU)
  python3 src/rosnav_bot/scripts/train_ppo.py --smoke

  # longer run on maze map
  python3 src/rosnav_bot/scripts/train_ppo.py \\
      --map src/rosnav_bot/maps/map_maze.yaml \\
      --timesteps 200000 \\
      --out runs/rl/ppo_maze

Deploy:
  ros2 run rosnav_bot rl_policy_node.py --ros-args \\
      -p model_path:=runs/rl/ppo_maze/ppo_scan_nav.pt
"""

from __future__ import annotations

import argparse
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


def train_torch(env, timesteps: int, out_dir: str, seed: int, device: str):
    from rosnav_bot.rl.policy import ActorCritic

    obs_dim = int(np.prod(env.observation_space.shape))
    act_dim = int(np.prod(env.action_space.shape))
    net = ActorCritic(obs_dim, act_dim).to(device)
    opt = torch.optim.Adam(net.parameters(), lr=3e-4)

    rollout = 1024
    epochs = 4
    minibatch = 256
    clip = 0.2

    obs, _ = env.reset(seed=seed)
    ep_ret = 0.0
    ep_len = 0
    completed = []
    step = 0
    while step < timesteps:
        buf_o, buf_a, buf_logp, buf_r, buf_v, buf_done = [], [], [], [], [], []
        for _ in range(rollout):
            ot = torch.as_tensor(obs, dtype=torch.float32, device=device).unsqueeze(0)
            with torch.no_grad():
                action, value, logp = net.act(ot, deterministic=False)
            a = action.squeeze(0).cpu().numpy()
            next_obs, reward, terminated, truncated, _ = env.step(a)
            done = terminated or truncated
            buf_o.append(obs)
            buf_a.append(a)
            buf_logp.append(float(logp.item()))
            buf_r.append(float(reward))
            buf_v.append(float(value.item()))
            buf_done.append(bool(done))
            ep_ret += reward
            ep_len += 1
            obs = next_obs
            step += 1
            if done:
                completed.append((ep_ret, ep_len))
                ep_ret, ep_len = 0.0, 0
                obs, _ = env.reset()
            if step >= timesteps:
                break

        o = torch.as_tensor(np.asarray(buf_o), dtype=torch.float32, device=device)
        a = torch.as_tensor(np.asarray(buf_a), dtype=torch.float32, device=device)
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
                logp, entropy, v = net.evaluate(o[mb], a[mb])
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
            print(f'[train_ppo] step={step}/{timesteps} '
                  f'ep_ret_mean={np.mean(rets):.2f} last_len={completed[-1][1]}',
                  flush=True)

    os.makedirs(out_dir, exist_ok=True)
    save_path = os.path.join(out_dir, 'ppo_scan_nav.pt')
    torch.save({
        'state_dict': net.state_dict(),
        'obs_dim': obs_dim,
        'act_dim': act_dim,
        'n_beams': getattr(env, 'n_beams', obs_dim - 3),
        'max_range': getattr(env, 'max_range', 8.0),
        'v_max': getattr(env, 'v_max', 0.5),
        'w_max': getattr(env, 'w_max', 1.0),
    }, save_path)
    return save_path


def train_sb3(env, timesteps: int, out_dir: str, seed: int):
    from stable_baselines3 import PPO
    from stable_baselines3.common.monitor import Monitor

    env = Monitor(env)
    os.makedirs(out_dir, exist_ok=True)
    model = PPO(
        'MlpPolicy', env, verbose=1, seed=seed,
        n_steps=min(1024, timesteps), batch_size=64,
        learning_rate=3e-4, gamma=0.99, tensorboard_log=None, device='cpu',
    )
    model.learn(total_timesteps=timesteps, progress_bar=False)
    save_path = os.path.join(out_dir, 'ppo_scan_nav')
    model.save(save_path)
    return save_path + '.zip'


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
    default_map = os.path.join(root, 'src/rosnav_bot/maps/map_maze.yaml')
    ap.add_argument('--map', default=default_map, help='Nav2 map yaml (PGM sibling)')
    ap.add_argument('--timesteps', type=int, default=100_000)
    ap.add_argument('--out', default='runs/rl/ppo_scan_nav')
    ap.add_argument('--n-beams', type=int, default=36)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--device', default='cpu')
    ap.add_argument('--backend', choices=('torch', 'sb3'), default='torch',
                    help='torch=pure PyTorch PPO (default); sb3=stable-baselines3')
    ap.add_argument('--smoke', action='store_true',
                    help='Short CPU run to verify the pipeline')
    args = ap.parse_args()

    pkg_root = os.path.join(root, 'src/rosnav_bot')
    if pkg_root not in sys.path:
        sys.path.insert(0, pkg_root)

    from rosnav_bot.rl.scan_nav_env import ScanNavEnv

    map_yaml = args.map
    timesteps = args.timesteps
    out_dir = args.out
    if args.smoke:
        timesteps = 2048
        out_dir = 'runs/rl/_smoke'
        print(f'[train_ppo] smoke: timesteps={timesteps} map={map_yaml} '
              f'backend={args.backend}', flush=True)

    if not os.path.isfile(map_yaml):
        print(f'map not found: {map_yaml}', file=sys.stderr)
        sys.exit(2)

    env = ScanNavEnv(map_yaml, n_beams=args.n_beams, seed=args.seed)
    if args.backend == 'sb3':
        path = train_sb3(env, timesteps, out_dir, args.seed)
    else:
        path = train_torch(env, timesteps, out_dir, args.seed, args.device)
    env.close()
    print(f'\n[train_ppo] saved {path}')
    print('Deploy:')
    print(f'  ros2 run rosnav_bot rl_policy_node.py --ros-args -p model_path:={path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
