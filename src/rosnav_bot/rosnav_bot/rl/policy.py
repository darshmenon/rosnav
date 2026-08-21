"""Small continuous-action Actor–Critic used by train_ppo.py / rl_policy_node.py."""

from __future__ import annotations

import torch
import torch.nn as nn
from torch.distributions import Normal


class ActorCritic(nn.Module):
    def __init__(self, obs_dim: int, act_dim: int = 2, hidden: int = 128):
        super().__init__()
        self.shared = nn.Sequential(
            nn.Linear(obs_dim, hidden),
            nn.Tanh(),
            nn.Linear(hidden, hidden),
            nn.Tanh(),
        )
        self.mu = nn.Linear(hidden, act_dim)
        self.log_std = nn.Parameter(torch.zeros(act_dim))
        self.value = nn.Linear(hidden, 1)

    def forward(self, obs: torch.Tensor):
        h = self.shared(obs)
        mu = torch.tanh(self.mu(h))
        std = self.log_std.exp().expand_as(mu).clamp(1e-3, 1.0)
        return mu, std, self.value(h).squeeze(-1)

    def act(self, obs: torch.Tensor, deterministic: bool = False):
        mu, std, value = self.forward(obs)
        if deterministic:
            return mu.detach(), value.detach(), None
        dist = Normal(mu, std)
        action = dist.sample().clamp(-1.0, 1.0)
        logp = dist.log_prob(action).sum(-1)
        return action.detach(), value.detach(), logp.detach()

    def evaluate(self, obs: torch.Tensor, action: torch.Tensor):
        mu, std, value = self.forward(obs)
        dist = Normal(mu, std)
        logp = dist.log_prob(action.clamp(-1.0, 1.0)).sum(-1)
        entropy = dist.entropy().sum(-1)
        return logp, entropy, value
