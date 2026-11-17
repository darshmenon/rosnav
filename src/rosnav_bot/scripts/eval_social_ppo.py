#!/usr/bin/env python3
"""eval_social_ppo.py -- seeded comparison on SocialNavMujocoEnv scenes.

Methods (identical seeds => identical start/goal/crowd for all three):
  ppo   trained ActorCritic (deterministic mean action), lidar + goal + nearest-ped only
  risk  social_nav_core.RiskPlanner (+ safety_filter) picks a holonomic velocity; a proportional
        heading controller turns it into (v, w), copied from render_social_demo.py. It uses the
        ground-truth pedestrian states, the map distance field and an A* route lookahead, so it is
        a privileged baseline (the policy sees none of that).
  goal  plain go-to-goal: turn toward the goal, drive straight; no obstacle avoidance at all.

HONESTY: this is a KINEMATIC simulation (unicycle pose update, MuJoCo for contacts/ray casts,
social-force pedestrians), NOT Gazebo: no wheel slip, motor lag or sensor noise. Treat the numbers
as a relative comparison inside this environment, not as real-robot performance.

  /usr/bin/python3 scripts/eval_social_ppo.py --ckpt runs/social_ppo.pt --episodes 30
"""
import argparse
import os
import sys

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, HERE)
from rosnav_bot.rl.policy import ActorCritic  # noqa: E402
from rosnav_bot.rl.social_nav_mujoco_env import SocialNavMujocoEnv, V_MAX, W_MAX  # noqa: E402
import social_nav_core as snc  # noqa: E402


def heading_ctrl(v, theta):
    """Holonomic velocity -> (lin, ang) for a diff-drive (render_social_demo.py)."""
    sp = float(np.linalg.norm(v))
    if sp <= 0.02:
        return 0.0, 0.0
    err = (np.arctan2(v[1], v[0]) - theta + np.pi) % (2 * np.pi) - np.pi
    ang = float(np.clip(3.0 * err, -W_MAX, W_MAX))
    lin = sp * max(0.0, np.cos(err)) if abs(err) < 1.1 else 0.0
    return min(lin, V_MAX), ang


class GoalPolicy:
    def reset(self, env):
        pass

    def act(self, env, obs):
        return heading_ctrl(np.clip(env.goal - env.robot_xy, -V_MAX, V_MAX) * 1.0
                            if np.linalg.norm(env.goal - env.robot_xy) < V_MAX
                            else (env.goal - env.robot_xy) / np.linalg.norm(env.goal - env.robot_xy) * V_MAX,
                            env.theta)


class RiskPolicy:
    def reset(self, env):
        self.planner = snc.RiskPlanner(sigma_scale=0.5, clear_fn=env.world.clearance_many)
        self.route = list(env.route)

    def act(self, env, obs):
        robot, n = env.robot_xy, env.n_peds
        while len(self.route) > 1 and np.linalg.norm(np.array(self.route[0]) - robot) < 1.0:
            self.route.pop(0)
        tgt = np.array(self.route[min(2, len(self.route) - 1)]) if self.route else env.goal
        pos, vel = env.crowd.pos[:n], env.crowd.vel[:n]
        v = self.planner.plan(robot, tgt, pos, vel)
        v = snc.safety_filter(v, robot, pos)
        return heading_ctrl(v, env.theta)


class PPOPolicy:
    def __init__(self, ckpt):
        c = torch.load(ckpt, map_location='cpu')
        self.net = ActorCritic(c['obs_dim'], c['act_dim'])
        self.net.load_state_dict(c['state_dict'])
        self.net.eval()

    def reset(self, env):
        pass

    def act(self, env, obs):
        with torch.no_grad():
            a, _, _ = self.net.act(torch.as_tensor(obs, dtype=torch.float32)[None], deterministic=True)
        a = np.clip(a[0].numpy(), -1, 1)
        return (a[0] + 1) * 0.5 * V_MAX, a[1] * W_MAX


def run(policy, env, seeds):
    rows = []
    for s in seeds:
        obs, info = env.reset(seed=s)
        policy.reset(env)
        steps, done = 0, False
        while not done:
            obs, r, term, trunc, info = env.step(policy.act(env, obs))
            steps += 1
            done = term or trunc
        rows.append((info['success'], info['collided'], info['min_clearance'], info['intrusion_s'], steps))
    return np.array(rows, float)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ckpt', default=None, help='trained checkpoint (omit to skip the ppo row)')
    ap.add_argument('--episodes', type=int, default=30)
    ap.add_argument('--seed', type=int, default=1000)
    a = ap.parse_args()
    torch.set_num_threads(1)
    env = SocialNavMujocoEnv()
    seeds = [a.seed + i for i in range(a.episodes)]
    methods = [('goal', GoalPolicy()), ('risk', RiskPolicy())]
    if a.ckpt:
        methods.insert(0, ('ppo', PPOPolicy(a.ckpt)))
    print(f'{a.episodes} episodes, seeds {seeds[0]}..{seeds[-1]} (kinematic MuJoCo sim, not Gazebo)')
    print(f"{'method':8s} {'succ%':>6s} {'coll%':>6s} {'min_clr[m]':>10s} {'intr[s]':>8s} {'steps':>7s}")
    for name, pol in methods:
        m = run(pol, env, seeds)
        print(f'{name:8s} {100 * m[:, 0].mean():6.1f} {100 * m[:, 1].mean():6.1f} '
              f'{m[:, 2].mean():10.2f} {m[:, 3].mean():8.2f} {m[:, 4].mean():7.1f}', flush=True)


if __name__ == '__main__':
    main()
