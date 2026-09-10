"""Per-candidate frontier scorer used by train_frontier_ppo.py / frontier_explorer.py.

Deep-Sets style: each candidate's feature vector is scored independently by
the same small MLP (no cross-candidate attention). This is deliberate — it
lets frontier_explorer.py call the exact same per-candidate forward pass
inside its existing _score_frontier(), one candidate at a time, with no
restructuring: argmax over independent logits reproduces this policy's
greedy (deterministic) action exactly, since softmax is monotonic per-logit.
"""

from __future__ import annotations

import torch
import torch.nn as nn
from torch.distributions import Categorical

N_FEATURES = 6


class FrontierScorer(nn.Module):
    def __init__(self, n_features: int = N_FEATURES, hidden: int = 64):
        super().__init__()
        self.encoder = nn.Sequential(
            nn.Linear(n_features, hidden),
            nn.Tanh(),
            nn.Linear(hidden, hidden),
            nn.Tanh(),
        )
        self.score_head = nn.Linear(hidden, 1)
        self.value_head = nn.Sequential(
            nn.Linear(hidden, hidden),
            nn.Tanh(),
            nn.Linear(hidden, 1),
        )

    def score_one(self, features: torch.Tensor) -> torch.Tensor:
        """features: (..., n_features) -> scalar logit per candidate."""
        return self.score_head(self.encoder(features)).squeeze(-1)

    def forward(self, obs: torch.Tensor, mask: torch.Tensor):
        """obs: (batch, k, n_features), mask: (batch, k) bool.

        Returns (logits, value) — logits masked to -inf where mask is False.
        """
        h = self.encoder(obs)
        logits = self.score_head(h).squeeze(-1)
        logits = logits.masked_fill(~mask, float('-inf'))
        pooled = (h * mask.unsqueeze(-1)).sum(1) / mask.sum(1, keepdim=True).clamp(min=1)
        value = self.value_head(pooled).squeeze(-1)
        return logits, value

    def act(self, obs: torch.Tensor, mask: torch.Tensor, deterministic: bool = False):
        logits, value = self.forward(obs, mask)
        if deterministic:
            action = logits.argmax(-1)
            return action.detach(), value.detach(), None
        dist = Categorical(logits=logits)
        action = dist.sample()
        logp = dist.log_prob(action)
        return action.detach(), value.detach(), logp.detach()

    def evaluate(self, obs: torch.Tensor, mask: torch.Tensor, action: torch.Tensor):
        logits, value = self.forward(obs, mask)
        dist = Categorical(logits=logits)
        logp = dist.log_prob(action)
        entropy = dist.entropy()
        return logp, entropy, value
