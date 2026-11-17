#!/usr/bin/env python3
"""
conformal_calibrate.py — split-conformal uncertainty radii for the learned
pedestrian predictor (idea from GenSafeNav, CoRL'25, scaled down).

For each horizon step k, r_k = the (1-alpha) quantile of ||pred_k - true_k|| on
held-out calibration windows. With exchangeable data the true position lies
within r_k of the prediction with prob >= 1-alpha. The planner/costmap use r_k
instead of a hand-set "sigma grows with time" guess.

  python3 conformal_calibrate.py --weights outputs/ped_predictor_n24.pt --alpha 0.1
  -> outputs/ped_predictor_n24.pt.conformal.json   {"alpha":0.1,"radii":[12 floats]}
Real-data calibration: add --real_root <raw> --heldout <scene> (calibrates on that scene's val split).
"""
import argparse, json
import numpy as np
import torch
import train_ped_predictor as tp

ap = argparse.ArgumentParser()
ap.add_argument('--weights', required=True)
ap.add_argument('--alpha', type=float, default=0.1)
ap.add_argument('--real_root', default='')
ap.add_argument('--heldout', default='eth')
a = ap.parse_args()
net = tp.SocialTransformer(); net.load_state_dict(torch.load(a.weights, map_location='cpu')); net.eval()
if a.real_root:
    X, Y, M = tp.load_real(a.real_root, a.heldout, ['val'])
else:
    X, Y, M = tp.make_dataset(400, seed=4242)       # calibration seed != train(1) / test(999)
with torch.no_grad():
    err = (net(X, M) - Y).norm(dim=-1)              # (B,N,PRED)
radii = []
for k in range(tp.PRED):
    e = err[:, :, k][M].numpy()
    n = len(e)
    q = min(1.0, np.ceil((n + 1) * (1 - a.alpha)) / n)  # finite-sample conformal correction
    radii.append(float(np.quantile(e, q)))
out = a.weights + '.conformal.json'
json.dump({'alpha': a.alpha, 'dt': tp.DT, 'radii': radii}, open(out, 'w'))
print('conformal radii (m) per 0.4 s step:', [round(r, 2) for r in radii])
print('saved', out)
