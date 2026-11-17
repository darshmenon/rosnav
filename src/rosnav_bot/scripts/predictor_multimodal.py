#!/usr/bin/env python3
"""
predictor_multimodal.py — best-of-K (multimodal) pedestrian trajectory prediction on top of the social transformer.

People have several plausible futures (stop, turn, keep going). The single-mode model predicts the mean of them, which
is a path nobody walks. Here the transformer's last layer emits K residual paths over constant velocity (K extra output
heads on the shared social encoder), trained with a relaxed winner-takes-all loss:

    loss = (1 - eps) * min_k ADE_k + eps * mean_k ADE_k        (per agent; eps keeps losing heads alive)

Heads are fine-tuned from the single-mode leave-one-scene-out weights (trained WITHOUT the held-out scene) with
distinct random velocity offsets on the last-layer bias to break symmetry. The epoch is chosen on VAL minADE@K.

Reported (held-out scene test split): minADE/minFDE@K for the multimodal heads against
  * CV (constant velocity, K=1),
  * the single-mode learned model (K=1),
  * "CV-spray": K constant-velocity paths with Gaussian velocity noise (sigma chosen on val) — the fair baseline
    that shows whether the learned heads add anything beyond just adding spread.

  OMP_NUM_THREADS=2 nice -n 10 /usr/bin/python3 predictor_multimodal.py train --scene zara1 --out ../../../outputs/mm_zara1.pt
  /usr/bin/python3 predictor_multimodal.py figure --scene zara1 --weights ../../../outputs/mm_zara1.pt
Planner use: MultimodalPredictor.futures(hist, horizon_s, K) -> (K,N,T,2) for RiskPlanner.plan(futures=...).
"""
import argparse
import math
import os
import sys
import time

import numpy as np
import torch
import torch.nn as nn

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import train_ped_predictor as tp  # noqa: E402

DEFAULT_ROOT = os.path.expanduser('~/datasets/trajectron_pp/repo/experiments/pedestrians/raw')
OUTDIR = os.path.join(HERE, '..', '..', '..', 'outputs')
ASSETS = os.path.join(HERE, '..', '..', '..', 'assets', 'social_nav')


class SocialTransformerMM(tp.SocialTransformer):
    """Same encoder; head emits K residual paths. forward -> (B,N,K,PRED,2)."""

    def __init__(self, K=20, d=96, heads=4, layers=3, dropout=0.1):
        super().__init__(d, heads, layers, dropout)
        self.K = K
        self.head = nn.Sequential(nn.Linear(d, d), nn.ReLU(), nn.Linear(d, K * tp.PRED * 2))

    def forward(self, x, mask):
        last = x[..., -1, :]
        rel = (x - last[..., None, :]).flatten(-2)
        cen = (last * mask[..., None]).sum(1, keepdim=True) / mask.sum(1, keepdim=True)[..., None].clamp(min=1)
        vel = (x[..., -1, :] - x[..., -4, :]) / (3 * tp.DT)
        h = self.enc(self.emb(torch.cat([rel, last - cen, vel], -1)), src_key_padding_mask=~mask)
        res = self.head(h).view(*x.shape[:2], self.K, tp.PRED, 2)
        return tp.cv_pred(x)[:, :, None] + res


def from_single(path, K=20, dropout=0.1, spread=0.25, seed=0):
    """Multimodal net initialised from single-mode weights: encoder + first head layer copied, last layer tiled K times
    with per-head random velocity offsets (m/s, std `spread`) added to the bias so the heads start distinct."""
    net = SocialTransformerMM(K, dropout=dropout)
    sd = torch.load(path, map_location='cpu')
    new = net.state_dict()
    for k, v in sd.items():
        if not k.startswith('head.2'):
            new[k] = v
    g = torch.Generator().manual_seed(seed)
    w, b = sd['head.2.weight'], sd['head.2.bias']                        # (PRED*2, d), (PRED*2,)
    new['head.2.weight'] = w.repeat(K, 1) + 0.01 * torch.randn(K * tp.PRED * 2, w.shape[1], generator=g)
    t = (torch.arange(1, tp.PRED + 1).float() * tp.DT)[:, None]          # (PRED,1)
    off = torch.randn(K, 1, 2, generator=g) * spread * t[None]           # (K,PRED,2) straight-line deviations
    new['head.2.bias'] = b.repeat(K) + off.reshape(-1)
    net.load_state_dict(new)
    return net


# ---------------------------------------------------------------- metrics
def min_at_k(P, y, m, k):
    """P (B,N,K,T,2) -> minADE, minFDE over the first k heads (per agent best head), masked mean."""
    d = (P[:, :, :k] - y[:, :, None]).norm(dim=-1)                        # (B,N,k,T)
    ade = d.mean(-1).min(-1).values[m]
    fde = d[..., -1].min(-1).values[m]
    return ade.mean().item(), fde.mean().item()


def cv_spray(x, k, sigma, seed=0):
    """K constant-velocity paths with Gaussian velocity noise -> (B,N,k,T,2); head 0 is plain CV."""
    g = torch.Generator().manual_seed(seed)
    v = (x[..., -1, :] - x[..., -4, :]) / (3 * tp.DT)
    noise = torch.randn(*v.shape[:2], k, 2, generator=g) * sigma
    noise[:, :, 0] = 0
    t = torch.arange(1, tp.PRED + 1).float().view(1, 1, 1, tp.PRED, 1) * tp.DT
    return x[:, :, -1][:, :, None, None] + t * (v[:, :, None] + noise)[:, :, :, None]


def endpoint_diversity(P, m):
    """Mean pairwise endpoint distance between heads (m) — near 0 means the heads collapsed."""
    e = P[:, :, :, -1]                                                    # (B,N,K,2)
    d = (e[:, :, :, None] - e[:, :, None]).norm(dim=-1).mean((-1, -2))
    return d[m].mean().item()


# ---------------------------------------------------------------- training
def wta_loss(P, y, m, eps=0.05):
    ade = (P - y[:, :, None]).norm(dim=-1).mean(-1)                       # (B,N,K)
    return ((1 - eps) * ade.min(-1).values + eps * ade.mean(-1))[m].mean()


def train(a):
    torch.set_num_threads(int(os.environ.get('OMP_NUM_THREADS', 2)))
    t0 = time.time()
    Xtr, Ytr, Mtr = tp.load_real(a.root, a.scene, ['train'])
    Xva, Yva, Mva = tp.load_real(a.root, a.scene, ['val'])
    Xte, Yte, Mte = tp.load_real(a.root, a.scene, ['test'])
    print(f'{a.scene}: {len(Xtr)} train / {len(Xva)} val / {len(Xte)} test windows ({time.time() - t0:.0f}s)', flush=True)
    base = os.path.join(a.weights_dir, f'loo_val_{a.scene}.pt')
    net = from_single(base, a.K, dropout=a.dropout)
    single = tp.SocialTransformer(dropout=a.dropout)
    single.load_state_dict(torch.load(base, map_location='cpu'))
    single.eval()
    opt = torch.optim.AdamW(net.parameters(), a.lr, weight_decay=a.wd)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, a.epochs)
    best, best_state, best_ep = 1e9, None, 0
    for ep in range(a.epochs):
        net.train()
        perm = torch.randperm(len(Xtr))
        for i in range(0, len(perm), 128):
            idx = perm[i:i + 128]
            x, y, m = Xtr[idx], Ytr[idx], Mtr[idx]
            th = torch.rand(len(idx)) * 2 * math.pi
            R = torch.stack([torch.cos(th), -torch.sin(th), torch.sin(th), torch.cos(th)], -1).view(-1, 1, 1, 2, 2)
            x, y = (R @ x[..., None]).squeeze(-1), (R @ y[..., None]).squeeze(-1)
            loss = wta_loss(net(x, m), y, m, a.eps)
            opt.zero_grad(); loss.backward()
            nn.utils.clip_grad_norm_(net.parameters(), 1.0)
            opt.step()
        sched.step()
        net.eval()
        with torch.no_grad():
            v = min_at_k(net(Xva, Mva), Yva, Mva, a.K)[0]
        if v < best:
            best, best_ep = v, ep + 1
            best_state = {k: t.detach().clone() for k, t in net.state_dict().items()}
        if ep % 5 == 4 or ep == a.epochs - 1:
            print(f'epoch {ep + 1:2d} loss {loss.item():.3f}  val minADE@{a.K} {v:.3f}', flush=True)
    net.load_state_dict(best_state); net.eval()
    torch.save(net.state_dict(), a.out)
    print(f'saved {a.out} (best val epoch {best_ep}, val minADE@{a.K} {best:.3f}, {time.time() - t0:.0f}s)')
    report(net, single, (Xva, Yva, Mva), (Xte, Yte, Mte), a.K)


def report(net, single, val, test, K):
    Xva, Yva, Mva = val
    Xte, Yte, Mte = test
    with torch.no_grad():
        Pva, Pte = net(Xva, Mva), net(Xte, Mte)
        Ste = single(Xte, Mte)
    # spray sigma on val, at K
    sigmas = (0.05, 0.1, 0.2, 0.3, 0.5, 0.8)
    sig = min(sigmas, key=lambda s: min_at_k(cv_spray(Xva, K, s), Yva, Mva, K)[0])
    print(f'\nTEST minADE / minFDE (m) on the held-out scene   (CV-spray sigma={sig} m/s chosen on val)')
    print(f"{'method':32s}" + ''.join(f'{"K=" + str(k):>14s}' for k in (1, 5, 10, K) if k <= K))
    ks = [k for k in (1, 5, 10, K) if k <= K]
    ks = list(dict.fromkeys(ks))
    rows = {
        'CV': [min_at_k(tp.cv_pred(Xte)[:, :, None], Yte, Mte, 1)] * len(ks),
        'single-mode learned': [min_at_k(Ste[:, :, None], Yte, Mte, 1)] * len(ks),
        'CV-spray (noise baseline)': [min_at_k(cv_spray(Xte, k, sig), Yte, Mte, k) for k in ks],
        'multimodal heads (learned)': [min_at_k(Pte, Yte, Mte, k) for k in ks],
    }
    for n, r in rows.items():
        print(f'{n:32s}' + ''.join(f'{a:8.3f}/{f:5.3f}' for a, f in r))
    print(f'endpoint diversity across heads: {endpoint_diversity(Pte, Mte):.2f} m '
          f'(CV-spray {endpoint_diversity(cv_spray(Xte, K, sig), Mte):.2f} m)')


# ---------------------------------------------------------------- planner interface
class MultimodalPredictor:
    """Drop-in beside eval_learned_planner.LearnedPredictor: hist (n,8,2) at 0.4 s."""

    def __init__(self, weights, K=20, device='cpu'):
        self.net = SocialTransformerMM(K)
        self.net.load_state_dict(torch.load(weights, map_location=device))
        self.net.eval()
        self.K = K

    @torch.no_grad()
    def paths(self, hist):
        """(n,K,12,2) heads' absolute paths on the 0.4 s grid (agents beyond NMAX are dropped)."""
        n = min(len(hist), tp.NMAX)
        x = torch.zeros(1, tp.NMAX, tp.OBS, 2)
        m = torch.zeros(1, tp.NMAX, dtype=torch.bool)
        x[0, :n] = torch.tensor(np.asarray(hist)[:n], dtype=torch.float32)
        m[0, :n] = True
        return self.net(x, m)[0, :n].numpy()

    def futures(self, hist, horizon_s=3.0, k=None, dt=0.1, rng=None):
        """(k,N,T,2) sampled joint futures at `dt`: sample j takes a random head independently for each agent
        (marginal best-of-K heads are not jointly consistent), then interpolates the 0.4 s grid to `dt`."""
        rng = rng or np.random.default_rng(0)
        P = self.paths(hist)                                              # (n,K,12,2)
        n = len(P)
        k = k or self.K
        last = np.asarray(hist)[:n, -1]
        t_src = np.arange(0, tp.PRED + 1) * tp.DT
        t_dst = np.arange(1, int(horizon_s / dt) + 1) * dt
        out = np.zeros((k, n, len(t_dst), 2))
        for j in range(k):
            heads = rng.integers(0, self.K, n)
            for i in range(n):
                path = np.vstack([last[i], P[i, heads[i]]])
                out[j, i] = np.stack([np.interp(t_dst, t_src, path[:, c]) for c in (0, 1)], 1)
        return out


# ---------------------------------------------------------------- figure
def figure(a):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    torch.set_num_threads(2)
    X, Y, M = tp.load_real(a.root, a.scene, ['test'])
    net = SocialTransformerMM(a.K)
    net.load_state_dict(torch.load(a.weights, map_location='cpu')); net.eval()
    single = tp.SocialTransformer(dropout=0.2)
    single.load_state_dict(torch.load(os.path.join(a.weights_dir, f'loo_val_{a.scene}.pt'), map_location='cpu'))
    single.eval()
    with torch.no_grad():
        P, S = net(X, M), single(X, M)
    C = tp.cv_pred(X)
    moving = np.where((M.sum(1) >= 2) & ((X[:, :, -1] - X[:, :, 0]).norm(dim=-1).max(1).values > 1.5).numpy())[0]
    rng = np.random.default_rng(a.seed)
    pick = rng.choice(moving, a.panels, replace=False)
    ncol = (a.panels + 1) // 2
    fig, axs = plt.subplots(2, ncol, figsize=(5.2 * ncol, 9), dpi=90)
    for ax, w in zip(axs.ravel(), pick):
        sp = (X[w, :, -1] - X[w, :, -4]).norm(dim=-1) * M[w]
        agents = torch.argsort(sp, descending=True)[:3].tolist()
        for i in agents:
            l = X[w, i, -1].numpy()
            ax.plot(*X[w, i].numpy().T, '-o', color='0.55', ms=2.5, lw=1.5)
            for k in range(a.K):
                ax.plot(*np.vstack([l, P[w, i, k].numpy()]).T, '-', color='tab:orange', lw=.8, alpha=.45)
            ax.plot(*np.vstack([l, Y[w, i].numpy()]).T, '-', color='k', lw=2.2)
            ax.plot(*np.vstack([l, C[w, i].numpy()]).T, ':', color='tab:blue', lw=2)
            ax.plot(*np.vstack([l, S[w, i].numpy()]).T, '--', color='tab:green', lw=2)
        best = (P[w, agents] - Y[w, agents][:, None]).norm(dim=-1).mean(-1).min(-1).values.mean().item()
        ssm = (S[w, agents] - Y[w, agents]).norm(dim=-1).mean().item()
        ax.set_aspect('equal'); ax.grid(alpha=.2)
        ax.set_title(f'window {w}: minADE@{a.K} {best:.2f} m  vs single-mode {ssm:.2f} m', fontsize=10)
    for ax in axs.ravel()[len(pick):]:
        ax.axis('off')
    fig.suptitle(f'{a.scene} (held-out scene): {a.K} sampled futures per pedestrian (orange) vs truth (black), '
                 f'constant velocity (blue dotted), single-mode model (green dashed); grey = observed',
                 fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, .96))
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    fig.savefig(a.out)
    print('wrote', a.out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', choices=['train', 'figure'])
    ap.add_argument('--scene', default='zara1')
    ap.add_argument('--root', default=DEFAULT_ROOT)
    ap.add_argument('--weights_dir', default=OUTDIR)
    ap.add_argument('--weights', default='')
    ap.add_argument('--out', default='')
    ap.add_argument('--K', type=int, default=20)
    ap.add_argument('--epochs', type=int, default=25)
    ap.add_argument('--lr', type=float, default=5e-4)
    ap.add_argument('--wd', type=float, default=1e-3)
    ap.add_argument('--dropout', type=float, default=0.2)
    ap.add_argument('--eps', type=float, default=0.05)
    ap.add_argument('--panels', type=int, default=6)
    ap.add_argument('--seed', type=int, default=1)
    a = ap.parse_args()
    if a.mode == 'train':
        a.out = a.out or os.path.join(OUTDIR, f'mm_{a.scene}.pt')
        train(a)
    else:
        a.weights = a.weights or os.path.join(OUTDIR, f'mm_{a.scene}.pt')
        a.out = a.out or os.path.join(ASSETS, f'multimodal_{a.scene}.png')
        figure(a)


if __name__ == '__main__':
    main()
