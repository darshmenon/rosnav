"""Pure-numpy frontier selection policies for the offline FrontierSelectEnv.

Every policy has the signature ``policy(feat, mask, ctx=None) -> int`` where ``feat`` is the
env's (k, 6) feature matrix (distance, size_m, info_gain, clearance, suspicious_ratio,
hysteresis), ``mask`` the (k,) valid-slot mask and ``ctx`` an optional dict with the extra
geometry some policies need:

  ctx['goals']  (k, 2) candidate goal positions in metres
  ctx['pos']    (2,)   robot position (m)
  ctx['prev']   (2,) or None  previous robot position (m), for turn cost
  ctx['failed'] list of (x, y, radius_m) recently failed goals (live node's cooldown list)

Live-node defaults mirror scripts/frontier_explorer.py (utility scorer).
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np

# frontier_explorer.py defaults
GAIN_SCALE = 1.0
POTENTIAL_SCALE = 3.0
HYST_GAIN = 1.5
SUSPECT_RATIO = 3.0
SUSPECT_PENALTY = 8.0
SUSPECT_BUFFER = 1.0
SUSPECT_HARD_RATIO = 6.0
MIN_GOAL_DIST = 0.35
FAILED_MIN_R, FAILED_MAX_R, FAILED_R_SCALE = 0.75, 3.0, 0.35


def _argmax_masked(score: np.ndarray, mask: np.ndarray) -> int:
    s = np.where(mask, score, -np.inf)
    return int(np.argmax(s))


def nearest(feat, mask, ctx=None) -> int:
    return _argmax_masked(-feat[:, 0], mask)


def max_gain(feat, mask, ctx=None) -> int:
    return _argmax_masked(feat[:, 2], mask)


def make_gain_per_cost(dist_w: float = 1.0, offset: float = 1.0):
    """utility = info_gain / (offset + dist_w * distance)."""
    def policy(feat, mask, ctx=None) -> int:
        return _argmax_masked(feat[:, 2] / (offset + dist_w * feat[:, 0]), mask)
    return policy


def failed_radius(distance_m: float) -> float:
    """Live node's failed-goal blacklist radius (clamp(scale*d, min, max))."""
    return float(min(FAILED_MAX_R, max(FAILED_MIN_R, FAILED_R_SCALE * distance_m)))


def _failed_filter(mask, ctx):
    if not ctx or not ctx.get('failed') or 'goals' not in ctx:
        return mask
    goals = np.asarray(ctx['goals'], dtype=float)
    keep = mask.copy()
    for fx, fy, r in ctx['failed']:
        keep &= np.hypot(goals[:, 0] - fx, goals[:, 1] - fy) > r
    return keep if keep.any() else mask  # never empty the pool (live node forgives too)


def live_scores(feat, hyst_gain=HYST_GAIN, suspect_penalty=SUSPECT_PENALTY,
                gain_scale=GAIN_SCALE, potential_scale=POTENTIAL_SCALE) -> np.ndarray:
    """Per-candidate score of frontier_explorer._score_frontier(), scorer='utility'."""
    d, size, clr, hyst = feat[:, 0], feat[:, 1], feat[:, 3], feat[:, 5]
    ratio = size / np.maximum(clr + SUSPECT_BUFFER, 1e-3)  # live buffer is 1.0, env's is 0.5
    pen = np.where(ratio > SUSPECT_RATIO, (ratio - SUSPECT_RATIO) * suspect_penalty, 0.0)
    return gain_scale * size - potential_scale * d + hyst_gain * (hyst > 0.5) - pen


def make_live(hyst_gain=HYST_GAIN, suspect_penalty=SUSPECT_PENALTY, use_failed=True,
              hard_filter=True):
    """The live node's utility heuristic: hysteresis + suspicious penalty/hard reject +
    min goal distance + failed-goal cooldown (clearance enters via the suspicious ratio)."""
    def policy(feat, mask, ctx=None) -> int:
        m = mask & (feat[:, 0] >= MIN_GOAL_DIST)
        if not m.any():
            m = mask
        if use_failed:
            m = _failed_filter(m, ctx)
        if hard_filter:
            ratio = feat[:, 1] / np.maximum(feat[:, 3] + SUSPECT_BUFFER, 1e-3)
            ok = m & (ratio <= SUSPECT_HARD_RATIO)
            if ok.any():
                m = ok
        return _argmax_masked(live_scores(feat, hyst_gain, suspect_penalty), m)
    return policy


def turn_angle(pos, prev, goal) -> float:
    """Absolute heading change (rad, 0..pi) needed to go from the last move to ``goal``."""
    if prev is None:
        return 0.0
    h = np.asarray(pos, float) - np.asarray(prev, float)
    g = np.asarray(goal, float) - np.asarray(pos, float)
    nh, ng = float(np.hypot(*h)), float(np.hypot(*g))
    if nh < 1e-6 or ng < 1e-6:
        return 0.0
    c = float(np.clip(np.dot(h, g) / (nh * ng), -1.0, 1.0))
    return math.acos(c)


def make_cost_aware(turn_w: float = 1.5, backtrack_w: float = 3.0, offset: float = 1.0):
    """gain / (offset + dist + turn_w*|turn| + backtrack_w*[turn > 90 deg]).

    turn_w is metres of cost per radian of heading change, backtrack_w the extra metres
    charged for reversing direction (a diff-drive spins in place and re-covers ground).
    Needs ctx (goals/pos/prev); without it degrades to gain/(offset+dist).
    """
    def policy(feat, mask, ctx=None) -> int:
        cost = offset + feat[:, 0]
        if ctx is not None and 'goals' in ctx:
            goals = np.asarray(ctx['goals'], float)
            ang = np.array([turn_angle(ctx['pos'], ctx.get('prev'), g) for g in goals])
            cost = cost + turn_w * ang + backtrack_w * (ang > math.pi / 2)
        return _argmax_masked(feat[:, 2] / cost, mask)
    return policy


def make_random(seed: int = 0):
    rng = np.random.default_rng(seed)

    def policy(feat, mask, ctx=None) -> int:
        return int(rng.choice(np.flatnonzero(mask)))
    return policy


def make_learned(model_path: str):
    """Greedy learned scorer; imports torch lazily (1 thread). None if file missing."""
    import os
    if not os.path.isfile(model_path):
        return None
    import torch
    torch.set_num_threads(1)
    from rosnav_bot.rl.frontier_policy import FrontierScorer
    ck = torch.load(model_path, map_location='cpu')
    net = FrontierScorer(ck.get('n_features', 6))
    net.load_state_dict(ck['state_dict'])
    net.eval()

    def policy(feat, mask, ctx=None) -> int:
        with torch.no_grad():
            lg = net.score_one(torch.as_tensor(feat, dtype=torch.float32)).numpy()
        return _argmax_masked(lg, mask)
    return policy


def all_policies(model_path: Optional[str] = None, seed: int = 0) -> dict:
    pols = {
        'nearest': nearest,
        'max_gain': max_gain,
        'gain/dist': make_gain_per_cost(1.0),
        'gain/dist(w=0.5)': make_gain_per_cost(0.5),
        'gain/dist(w=2)': make_gain_per_cost(2.0),
        'live': make_live(),
        'live-no-hyst': make_live(hyst_gain=0.0),
        'cost_aware': make_cost_aware(),
        'random': make_random(seed),
    }
    if model_path:
        learned = make_learned(model_path)
        if learned is not None:
            pols['learned'] = learned
    return pols
