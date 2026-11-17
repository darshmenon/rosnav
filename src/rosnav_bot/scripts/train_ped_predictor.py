#!/usr/bin/env python3
"""
train_ped_predictor.py — train a social-attention transformer pedestrian
trajectory predictor and compare it with constant velocity (CV).

Data: synthetic social-force crowds (goal-seeking + Helbing repulsion +
random goal switches), in the standard ETH/UCY window format: 8 observed
steps -> 12 predicted steps at 2.5 FPS (dt=0.4s). Synthetic only: swap
`make_dataset()` for an ETH/UCY/TrajNet++ loader returning the same arrays
to train on real data (see todo).

Model: per-agent temporal MLP embedding -> transformer encoder across the
agents of a scene (social attention) -> residual over the CV prediction.

  python3 train_ped_predictor.py --epochs 25 --out /tmp/ped_pred.pt
"""
import argparse
import math
import time

import numpy as np
import torch
import torch.nn as nn

OBS, PRED, DT, NMAX = 8, 12, 0.4, 24


# ---------------------------------------------------------------- data
def simulate_scene(rng, n, steps=OBS + PRED + 24, box=None):
    box = box or max(10.0, 2.2 * math.sqrt(n) * 2)   # keep density sane for big crowds
    pos = rng.uniform(0, box, (n, 2))
    goal = rng.uniform(0, box, (n, 2))
    vel = np.zeros((n, 2))
    vdes = rng.uniform(0.8, 1.5, n)
    out = np.zeros((steps, n, 2))
    sub = 4
    h = DT / sub
    for t in range(steps):
        out[t] = pos
        for _ in range(sub):
            g = goal - pos
            gd = np.linalg.norm(g, axis=1, keepdims=True) + 1e-6
            f = (g / gd * vdes[:, None] - vel) / 0.5          # drive to goal
            d = pos[:, None] - pos[None]                       # repulsion
            dist = np.linalg.norm(d, axis=2) + np.eye(n) * 1e3
            f += (2.0 * np.exp((0.6 - dist) / 0.4)[..., None] * d
                  / dist[..., None]).sum(1)
            vel += f * h
            sp = np.linalg.norm(vel, axis=1, keepdims=True)
            vel *= np.minimum(1.0, 1.8 / np.maximum(sp, 1e-6))
            pos = pos + vel * h
        sw = (np.linalg.norm(goal - pos, axis=1) < 0.6) | (rng.random(n) < 0.02)
        goal[sw] = rng.uniform(0, box, (sw.sum(), 2))          # intent changes
    return out


def make_dataset(n_scenes, seed):
    rng = np.random.default_rng(seed)
    X = np.zeros((0, NMAX, OBS, 2), np.float32)
    Y = np.zeros((0, NMAX, PRED, 2), np.float32)
    M = np.zeros((0, NMAX), bool)
    xs, ys, ms = [], [], []
    for _ in range(n_scenes):
        n = int(rng.integers(3, NMAX + 1))   # up to NMAX agents per scene
        tr = simulate_scene(rng, n)
        for s in range(0, tr.shape[0] - OBS - PRED + 1, 4):
            w = tr[s:s + OBS + PRED].transpose(1, 0, 2)        # (n, 20, 2)
            x = np.zeros((NMAX, OBS, 2), np.float32)
            y = np.zeros((NMAX, PRED, 2), np.float32)
            m = np.zeros(NMAX, bool)
            x[:n], y[:n], m[:n] = w[:, :OBS], w[:, OBS:], True
            xs.append(x); ys.append(y); ms.append(m)
    return (torch.tensor(np.stack(xs)), torch.tensor(np.stack(ys)),
            torch.tensor(np.stack(ms)))


def load_real(root, heldout, splits):
    """ETH/UCY (Trajectron++ raw layout): root/<heldout>/<split>/*.txt with
    columns frame, id, x, y; frames step 10 = 0.4 s. Windows of OBS+PRED
    consecutive frames; agents present in all of them, nearest NMAX kept;
    each window is centred on its last-observation centroid."""
    import glob
    xs, ys, ms = [], [], []
    for sp in splits:
        for f in sorted(glob.glob(f'{root}/{heldout}/{sp}/*.txt')):
            d = np.loadtxt(f)
            if d.size == 0:
                continue
            frames = np.unique(d[:, 0])
            by = {}
            for fr, i, x, y in d:
                by.setdefault(fr, {})[i] = (x, y)
            fset = set(frames.tolist())
            for f0 in frames:
                seq = [f0 + 10 * k for k in range(OBS + PRED)]
                if not all(q in fset for q in seq):
                    continue
                ids = set(by[seq[0]]).intersection(*[set(by[q]) for q in seq[1:]])
                if not ids:
                    continue
                w = np.array([[by[q][i] for q in seq] for i in sorted(ids)], np.float32)
                cen = w[:, OBS - 1].mean(0)
                w = w - cen
                order = np.argsort(np.linalg.norm(w[:, OBS - 1], axis=1))[:NMAX]
                w = w[order]
                n = len(w)
                x = np.zeros((NMAX, OBS, 2), np.float32)
                y = np.zeros((NMAX, PRED, 2), np.float32)
                m = np.zeros(NMAX, bool)
                x[:n], y[:n], m[:n] = w[:, :OBS], w[:, OBS:], True
                xs.append(x); ys.append(y); ms.append(m)
    return (torch.tensor(np.stack(xs)), torch.tensor(np.stack(ys)),
            torch.tensor(np.stack(ms)))


def cv_pred(x):
    """Constant velocity from the mean of the last 3 observed velocities."""
    v = (x[..., -1, :] - x[..., -4, :]) / (3 * DT)
    t = torch.arange(1, PRED + 1, device=x.device).view(1, 1, PRED, 1) * DT
    return x[..., -1:, :] + t * v[..., None, :]


# --------------------------------------------------------------- model
class SocialTransformer(nn.Module):
    def __init__(self, d=96, heads=4, layers=3, dropout=0.1):
        super().__init__()
        self.emb = nn.Sequential(nn.Linear(OBS * 2 + 2 + 2, d), nn.ReLU(),
                                 nn.Linear(d, d))
        enc = nn.TransformerEncoderLayer(d, heads, 4 * d, dropout, batch_first=True)
        self.enc = nn.TransformerEncoder(enc, layers)
        self.head = nn.Sequential(nn.Linear(d, d), nn.ReLU(),
                                  nn.Linear(d, PRED * 2))

    def forward(self, x, mask):
        last = x[..., -1, :]                                   # (B,N,2)
        rel = (x - last[..., None, :]).flatten(-2)             # shape of motion
        cen = (last * mask[..., None]).sum(1, keepdim=True) / mask.sum(
            1, keepdim=True)[..., None].clamp(min=1)
        vel = (x[..., -1, :] - x[..., -4, :]) / (3 * DT)
        tok = self.emb(torch.cat([rel, last - cen, vel], -1))
        h = self.enc(tok, src_key_padding_mask=~mask)
        res = self.head(h).view(*x.shape[:2], PRED, 2)
        return cv_pred(x) + res                                # residual on CV


def ade_fde(pred, y, m):
    d = (pred - y).norm(dim=-1)[m]                             # (K, PRED)
    return d.mean().item(), d[:, -1].mean().item()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--epochs', type=int, default=25)
    ap.add_argument('--scenes', type=int, default=2500)
    ap.add_argument('--out', default='ped_predictor.pt')
    ap.add_argument('--real_root', default='', help='.../experiments/pedestrians/raw')
    ap.add_argument('--heldout', default='eth', help='eth|hotel|univ|zara1|zara2')
    ap.add_argument('--init', default='', help='weights to fine-tune from')
    ap.add_argument('--lr', type=float, default=1e-3)
    ap.add_argument('--wd', type=float, default=1e-4, help='AdamW weight decay (more = more regularisation)')
    ap.add_argument('--dropout', type=float, default=0.1)
    ap.add_argument('--select_on_val', action='store_true',
                    help='real data: train on train only, keep the epoch with the best VAL ADE, report test')
    ap.add_argument('--all_scenes', action='store_true',
                    help='real data: train on all five scenes (no held-out scene) for a deployable model')
    a = ap.parse_args()
    dev = 'cuda' if torch.cuda.is_available() else 'cpu'
    t0 = time.time()
    Xva = None
    if a.real_root and a.all_scenes:
        Xtr, Ytr, Mtr = load_real(a.real_root, 'eth', ['train', 'val', 'test'])   # eth folder = 4 scenes + eth test
        Xte, Yte, Mte = load_real(a.real_root, 'eth', ['test'])
        print('REAL ETH/UCY, ALL five scenes in training: the printed test numbers are IN-SAMPLE (no held-out scene)')
    elif a.real_root and a.select_on_val:
        Xtr, Ytr, Mtr = load_real(a.real_root, a.heldout, ['train'])
        Xva = load_real(a.real_root, a.heldout, ['val'])
        Xte, Yte, Mte = load_real(a.real_root, a.heldout, ['test'])
        print(f'REAL ETH/UCY, held-out scene: {a.heldout}; epoch chosen on val ({len(Xva[0])} windows)')
    elif a.real_root:
        Xtr, Ytr, Mtr = load_real(a.real_root, a.heldout, ['train', 'val'])
        Xte, Yte, Mte = load_real(a.real_root, a.heldout, ['test'])
        print(f'REAL ETH/UCY, held-out scene: {a.heldout}')
    else:
        Xtr, Ytr, Mtr = make_dataset(a.scenes, seed=1)
        Xte, Yte, Mte = make_dataset(a.scenes // 5, seed=999)     # unseen scenes
    print(f'data {len(Xtr)} train / {len(Xte)} test windows in {time.time()-t0:.0f}s')
    Xte, Yte, Mte = Xte.to(dev), Yte.to(dev), Mte.to(dev)
    print('CV baseline  ADE/FDE = %.3f / %.3f m' %
          ade_fde(cv_pred(Xte), Yte, Mte))
    net = SocialTransformer(dropout=a.dropout).to(dev)
    if a.init:
        net.load_state_dict(torch.load(a.init, map_location=dev))
        net.eval()
        with torch.no_grad():
            print('init weights, zero-shot ADE/FDE = %.3f / %.3f m' % ade_fde(net(Xte, Mte), Yte, Mte))
    opt = torch.optim.AdamW(net.parameters(), a.lr, weight_decay=a.wd)
    best_val, best_state = 1e9, None
    if Xva is not None:
        Xva = tuple(v.to(dev) for v in Xva)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, a.epochs)
    bs = 128
    for ep in range(a.epochs):
        net.train()
        perm = torch.randperm(len(Xtr))
        for i in range(0, len(perm), bs):
            idx = perm[i:i + bs]
            x, y, m = Xtr[idx].to(dev), Ytr[idx].to(dev), Mtr[idx].to(dev)
            # augmentation: random rotation of the whole scene
            th = torch.rand(len(idx), device=dev) * 2 * math.pi
            R = torch.stack([torch.cos(th), -torch.sin(th),
                             torch.sin(th), torch.cos(th)], -1).view(-1, 1, 1, 2, 2)
            x = (R @ x[..., None]).squeeze(-1)
            y = (R @ y[..., None]).squeeze(-1)
            loss = ((net(x, m) - y).norm(dim=-1)[m]).mean()
            opt.zero_grad(); loss.backward()
            nn.utils.clip_grad_norm_(net.parameters(), 1.0)
            opt.step()
        sched.step()
        if Xva is not None:
            net.eval()
            with torch.no_grad():
                v_ade = ade_fde(net(Xva[0], Xva[2]), Xva[1], Xva[2])[0]
            if v_ade < best_val:
                best_val, best_ep = v_ade, ep + 1
                best_state = {k: v.detach().clone() for k, v in net.state_dict().items()}
        if ep % 5 == 4 or ep == a.epochs - 1:
            net.eval()
            with torch.no_grad():
                print('epoch %2d  loss %.3f  test ADE/FDE = %.3f / %.3f m' %
                      ((ep + 1, loss.item()) + ade_fde(net(Xte, Mte), Yte, Mte)))
    if best_state is not None:
        net.load_state_dict(best_state); net.eval()
        with torch.no_grad():
            print('best val epoch %d (val ADE %.3f): test ADE/FDE = %.3f / %.3f m' %
                  ((best_ep, best_val) + ade_fde(net(Xte, Mte), Yte, Mte)))
    torch.save(net.state_dict(), a.out)
    print('saved', a.out, f'({time.time()-t0:.0f}s total)')


if __name__ == '__main__':
    main()
