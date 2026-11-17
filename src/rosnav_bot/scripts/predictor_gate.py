#!/usr/bin/env python3
"""
predictor_gate.py — a per-agent gate that falls back to constant velocity (CV) when the learned
pedestrian predictor is unsure.

Why: on ETH/UCY `hotel` the learned social transformer is WORSE than constant velocity (0.35 vs 0.25 m ADE)
while it wins on four other scenes. A deployable system should not trust a model that is unsure, so we blend

    pred = CV + g * (learned - CV),     g in [0, 1] per agent

with g computed from signals available at run time, and ALL gate parameters fitted on the held-out scene's
VAL split only (test is never touched for fitting or selection):

  scene   one constant g (grid search on val)                 - "how much do I trust the model on this scene"
  mc      g = clip(1 - u / tau, 0, 1), u = MC-dropout spread   - uncertain agent -> CV
  logit   logistic regression on [u, |learned-CV| at the end, speed, log crowd size] predicting
          "learned is better than CV for this agent"; g = P(learned better)
  online  scene-level trust from REALISED outcomes: once a window's 12 future steps have happened (12 windows
          later) the learned-vs-CV error of that window is known; a decayed running comparison of the two sets g.
          Causal (g[t] only uses windows <= t-12), needs no labels beyond what the robot sees itself.

NOTE (verified from the data layout): the 'val' split of the leave-one-scene-out folders contains the OTHER four
scenes' val files, never the held-out scene. So val-fitted gates (scene/mc/logit) cannot see the held-out scene's
regime; only `online` adapts to it at run time.

`auto` picks the gate (or no gate / CV) with the best VAL ADE. Leave-one-scene-out over ETH/UCY with the
loo_val_<scene>.pt weights (each trained without its scene).

  OMP_NUM_THREADS=2 /usr/bin/python3 predictor_gate.py --weights_dir ../../../outputs
"""
import argparse
import math
import os
import sys

import numpy as np
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import train_ped_predictor as tp  # noqa: E402

SCENES = ('eth', 'hotel', 'univ', 'zara1', 'zara2')
DEFAULT_ROOT = os.path.expanduser('~/datasets/trajectron_pp/repo/experiments/pedestrians/raw')


def per_agent_ade(pred, y):
    """(B,N,T,2) x2 -> (B,N) mean displacement error over the horizon."""
    return (pred - y).norm(dim=-1).mean(-1)


def mc_dropout(net, x, m, samples=16, seed=0):
    """Learned prediction (eval mode) + MC-dropout spread u (B,N): mean over horizon of the std norm."""
    net.eval()
    with torch.no_grad():
        mean = net(x, m)
    torch.manual_seed(seed)
    net.train()
    with torch.no_grad():
        S = torch.stack([net(x, m) for _ in range(samples)])
    net.eval()
    return mean, S.std(0).norm(dim=-1).mean(-1)


def features(x, m, learned, cv, u):
    """Per-agent gate features (B,N,4): MC spread, end-point disagreement with CV, speed, log crowd size."""
    dend = (learned[..., -1, :] - cv[..., -1, :]).norm(dim=-1)
    speed = ((x[..., -1, :] - x[..., -4, :]) / (3 * tp.DT)).norm(dim=-1)
    n = m.sum(1, keepdim=True).float().expand_as(speed)
    return torch.stack([u, dend, speed, torch.log1p(n)], -1)


def window_errors(cv, learned, y, m):
    """Per-window mean agent ADE of CV and of the learned model, in stream order -> two (B,) arrays."""
    n = m.sum(1).clamp(min=1).float()
    el = (per_agent_ade(learned, y) * m).sum(1) / n
    ec = (per_agent_ade(cv, y) * m).sum(1) / n
    return el.numpy().astype(float), ec.numpy().astype(float)


def online_g(e_l, e_c, kappa, decay, prior=1.0, lag=tp.PRED, shrink=5.0):
    """Causal online trust. g[t] uses only windows <= t-lag (their 12-step futures have happened by then).
    g = sigmoid(-kappa * conf * (decayed mean e_l - decayed mean e_c)); conf shrinks to 0 with little evidence,
    so g starts at 0.5 (kappa irrelevant) unless `prior` says otherwise for t < lag."""
    B = len(e_l)
    g = np.full(B, float(prior))
    sl = sc = ws = 0.0
    for t in range(B):
        j = t - lag
        if j >= 0:
            sl, sc, ws = decay * sl + e_l[j], decay * sc + e_c[j], decay * ws + 1.0
            d = (sl - sc) / ws
            conf = ws / (ws + shrink)
            z = max(-30.0, min(30.0, kappa * conf * d))
            g[t] = 1.0 / (1.0 + math.exp(z))
    return g


def blend(cv, learned, g):
    return cv + g[..., None, None] * (learned - cv)


class Gate:
    """Fitted on VAL. `kind` in {'cv', 'learned', 'scene', 'mc', 'logit', 'online'}; call `.g(feats)` -> (…,) in [0,1]."""

    def __init__(self, kind, **p):
        self.kind, self.p = kind, p

    def g(self, f, ctx=None):
        if self.kind == 'online':
            el, ec = window_errors(ctx['cv'], ctx['learned'], ctx['y'], ctx['m'])
            gw = torch.tensor(online_g(el, ec, **self.p), dtype=torch.float32)
            return gw[:, None].expand(f.shape[:-1])
        if self.kind == 'cv':
            return torch.zeros(f.shape[:-1])
        if self.kind == 'learned':
            return torch.ones(f.shape[:-1])
        if self.kind == 'scene':
            return torch.full(f.shape[:-1], float(self.p['g']))
        if self.kind == 'mc':
            return (1 - f[..., 0] / self.p['tau']).clamp(0, 1)
        mu, sd, w, b = self.p['mu'], self.p['sd'], self.p['w'], self.p['b']
        return torch.sigmoid(((f - mu) / sd) @ w + b)


def fit_gates(cv, learned, feats, y, m):
    """Fit every gate on one (val) set. Returns {name: Gate} plus each gate's val ADE."""
    F, e_c, e_l = feats[m], per_agent_ade(cv, y)[m], per_agent_ade(learned, y)[m]
    cvp, lp, yp = cv[m], learned[m], y[m]                 # (A,T,2)

    def val_ade(g):
        return (cvp + g[:, None, None] * (lp - cvp) - yp).norm(dim=-1).mean().item()

    gates = {'cv': Gate('cv'), 'learned': Gate('learned')}
    best = min(((val_ade(torch.full((len(F),), g)), g) for g in np.linspace(0, 1, 21)))
    gates['scene'] = Gate('scene', g=best[1])
    us = F[:, 0]
    taus = [float(np.quantile(us.numpy(), q)) for q in np.linspace(0.3, 1.0, 15)] + [1e9]
    gates['mc'] = Gate('mc', tau=min(taus, key=lambda t: val_ade((1 - us / t).clamp(0, 1))))
    # logistic regression, standardised features, tiny L2 so it stays near "scene" when uninformative
    mu, sd = F.mean(0), F.std(0).clamp(min=1e-6)
    Z, lab = (F - mu) / sd, (e_l < e_c).float()
    w = torch.zeros(F.shape[1], requires_grad=True)
    b = torch.tensor(float(torch.logit(lab.mean().clamp(0.02, 0.98))), requires_grad=True)
    opt = torch.optim.LBFGS([w, b], max_iter=100)

    def closure():
        opt.zero_grad()
        loss = torch.nn.functional.binary_cross_entropy_with_logits(Z @ w + b, lab) + 1e-2 * (w ** 2).sum()
        loss.backward()
        return loss
    opt.step(closure)
    gates['logit'] = Gate('logit', mu=mu, sd=sd, w=w.detach(), b=b.detach())
    # online trust gate: grid over (kappa, decay, prior) on the val stream (windows are in time order per file)
    el, ec = window_errors(cv, learned, y, m)
    from itertools import product
    cand = [(online_g(el, ec, k, d, pr), dict(kappa=k, decay=d, prior=pr))
            for k, d, pr in product((5.0, 20.0, 60.0), (0.9, 0.97, 0.995), (0.5, 1.0))]
    def stream_ade(gw):
        gp = torch.tensor(gw, dtype=torch.float32)[:, None].expand(m.shape)[m]
        return val_ade(gp)
    gates['online'] = Gate('online', **min(cand, key=lambda c: stream_ade(c[0]))[1])
    ctx = dict(cv=cv, learned=learned, y=y, m=m)
    ades = {k: val_ade(g.g(feats, ctx)[m]) for k, g in gates.items()}
    return gates, ades


def evaluate(gate, cv, learned, feats, y, m):
    p = blend(cv, learned, gate.g(feats, dict(cv=cv, learned=learned, y=y, m=m)))
    return (p - y).norm(dim=-1)[m].mean().item()


def run_scene(scene, root, weights, samples=16):
    net = tp.SocialTransformer(dropout=0.2)
    net.load_state_dict(torch.load(weights, map_location='cpu'))
    out = {}
    for split in ('val', 'test'):
        x, y, m = tp.load_real(root, scene, [split])
        cv = tp.cv_pred(x)
        learned, u = mc_dropout(net, x, m, samples)
        out[split] = (cv, learned, features(x, m, learned, cv, u), y, m)
    gates, vades = fit_gates(*out['val'])
    cv, learned, f, y, m = out['test']
    tests = {k: evaluate(g, cv, learned, f, y, m) for k, g in gates.items()}
    auto = min(vades, key=vades.get)
    return dict(scene=scene, val=vades, test=tests, auto=auto, n_val=int(out['val'][4].sum()),
                n_test=int(m.sum()), gates=gates,
                g_mean=float(gates[auto].g(f, dict(cv=cv, learned=learned, y=y, m=m))[m].mean()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', default=DEFAULT_ROOT)
    ap.add_argument('--weights_dir', default=os.path.join(HERE, '..', '..', '..', 'outputs'))
    ap.add_argument('--samples', type=int, default=16)
    a = ap.parse_args()
    torch.set_num_threads(2)
    rows = []
    for sc in SCENES:
        rows.append(run_scene(sc, a.root, os.path.join(a.weights_dir, f'loo_val_{sc}.pt'), a.samples))
        r = rows[-1]
        print(f"{sc:6s} n_test={r['n_test']:5d}  VAL ADE " + ' '.join(f'{k}={v:.3f}' for k, v in r['val'].items())
              + f"  -> auto={r['auto']} (mean g {r['g_mean']:.2f})", flush=True)
    print('\nTEST ADE (m), leave-one-scene-out, gates fitted on VAL only')
    print(f"{'scene':7s}{'CV':>8s}{'learned':>9s}{'scene-g':>9s}{'mc':>8s}{'logit':>8s}{'online':>8s}{'auto':>8s}  picked")
    tot = {k: 0.0 for k in ('cv', 'learned', 'scene', 'mc', 'logit', 'online', 'auto')}
    ntot = 0
    for r in rows:
        t = r['test']
        print(f"{r['scene']:7s}{t['cv']:8.3f}{t['learned']:9.3f}{t['scene']:9.3f}{t['mc']:8.3f}{t['logit']:8.3f}"
              f"{t['online']:8.3f}{t[r['auto']]:8.3f}  {r['auto']}")
        for k in tot:
            tot[k] += (t[r['auto']] if k == 'auto' else t[k]) * r['n_test']
        ntot += r['n_test']
    print(f"{'ALL':7s}" + ''.join(f"{tot[k] / ntot:{w}.3f}" for k, w in (('cv', 8), ('learned', 9), ('scene', 9),
                                                                       ('mc', 8), ('logit', 8), ('online', 8), ('auto', 8))))


if __name__ == '__main__':
    main()
