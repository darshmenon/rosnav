#!/usr/bin/env python3
"""
render_predictor_real.py — the learned social-attention predictor on REAL ETH/UCY pedestrians (matplotlib PNG).

Weights should come from a leave-one-scene-out run (trained WITHOUT the shown scene), so every panel is honest
held-out data. Each panel is one scene snapshot: observed 8 steps (grey), true next 12 steps (black), constant-velocity
prediction (blue dotted), learned prediction (orange), discs = empirical 90% error radius per horizon step (computed on
that scene's windows). Titles give ADE (m) for CV vs learned in that window, so losses are shown as well as wins.

  python3 render_predictor_real.py --weights loo_val_zara1.pt --scene zara1 --out predictor_real_zara1.png
"""
import argparse, os, sys
import numpy as np
import torch
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import train_ped_predictor as tp

ap = argparse.ArgumentParser()
ap.add_argument('--weights', required=True)
ap.add_argument('--scene', default='zara1')
ap.add_argument('--root', default=os.path.expanduser('~/datasets/trajectron_pp/repo/experiments/pedestrians/raw'))
ap.add_argument('--out', default='predictor_real.png')
ap.add_argument('--panels', type=int, default=6)
ap.add_argument('--seed', type=int, default=0)
a = ap.parse_args()

torch.set_num_threads(1)
X, Y, M = tp.load_real(a.root, a.scene, ['test'])
net = tp.SocialTransformer(); net.load_state_dict(torch.load(a.weights, map_location='cpu')); net.eval()
with torch.no_grad():
    P = net(X, M)                                    # (W, N, 12, 2)
C = tp.cv_pred(X)
err_l = (P - Y).norm(dim=-1)                         # (W,N,12)
err_c = (C - Y).norm(dim=-1)
mask = M[..., None].expand_as(err_l)
radii = np.percentile(err_l[mask].numpy().reshape(-1, 12) if False else
                      np.stack([err_l[..., k][M].numpy() for k in range(12)], 1), 90, axis=0)
ade_l = ((err_l * M[..., None]).sum((1, 2)) / (M.sum(1) * 12).clamp(min=1)).numpy()
ade_c = ((err_c * M[..., None]).sum((1, 2)) / (M.sum(1) * 12).clamp(min=1)).numpy()
print(f'{a.scene}: windows={len(X)}  ADE CV={ade_c.mean():.3f}  learned={ade_l.mean():.3f}  r90={np.round(radii[[0,5,11]],2)}')

rng = np.random.default_rng(a.seed)
moving = np.where((M.sum(1) >= 3) & (np.linalg.norm((X[:, :, -1] - X[:, :, 0]).numpy(), axis=-1).max(1) > 1.5))[0]
pick = rng.choice(moving, a.panels, replace=False)
fig, axs = plt.subplots(2, (a.panels + 1) // 2, figsize=(5.2 * ((a.panels + 1) // 2), 9), dpi=90)
for ax, w in zip(axs.ravel(), pick):
    m = M[w].numpy()
    for i in np.where(m)[0]:
        ax.plot(*X[w, i].numpy().T, '-o', color='0.55', ms=2.5, lw=1.5)
        ax.plot(*np.vstack([X[w, i, -1].numpy(), Y[w, i].numpy()]).T, '-', color='k', lw=1.8)
        ax.plot(*np.vstack([X[w, i, -1].numpy(), C[w, i].numpy()]).T, ':', color='tab:blue', lw=2)
        pl = np.vstack([X[w, i, -1].numpy(), P[w, i].numpy()])
        ax.plot(*pl.T, '-', color='tab:orange', lw=2)
        for k in (3, 7, 11):
            ax.add_patch(Circle(pl[k + 1], radii[k], fill=False, ec='tab:orange', lw=.8, alpha=.6))
    ax.set_aspect('equal'); ax.grid(alpha=.2)
    ax.set_title(f'window {w}: ADE CV {ade_c[w]:.2f} m  |  learned {ade_l[w]:.2f} m', fontsize=10,
                 color='tab:green' if ade_l[w] < ade_c[w] else 'tab:red')
for ax in axs.ravel()[len(pick):]:
    ax.axis('off')
fig.suptitle(f'{a.scene} (held-out scene)   grey = observed 3.2 s, black = truth next 4.8 s, blue dotted = constant velocity, '
             f'orange = learned (discs = empirical 90% radius)\nscene mean ADE: CV {ade_c.mean():.2f} m, learned {ade_l.mean():.2f} m', fontsize=11)
fig.tight_layout(rect=(0, 0, 1, .95)); fig.savefig(a.out)
print('wrote', a.out)
