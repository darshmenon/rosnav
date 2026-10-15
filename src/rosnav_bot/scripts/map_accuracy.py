#!/usr/bin/env python3
"""
map_accuracy.py — score a candidate 2D occupancy map (e.g. ORB-SLAM3's 3D-depth ->
2D grid from orb_slam3_occupancy_bridge.py, or an RTAB-Map projection) against a
reference map (default: the lidar slam_toolbox map of the same world).

The candidate is first ICP-aligned to the reference (rigid 2D, occupied cells only),
because SLAM maps are anchored at the robot's start pose and may be shifted/rotated.
Reported (tol = --tol metres, default 0.10):
  precision  - fraction of candidate occupied cells within tol of a reference wall
  recall     - fraction of reference walls (inside the candidate's known area) within tol
  f1, mean_wall_err_m (candidate->ref nearest distance, occupied cells)
  known_m2   - mapped area (free+occupied), free_agree - candidate-free cells that are
               not occupied in the reference (false-free check)
  icp_dx/dy/dyaw - the alignment that had to be applied (map-frame consistency)

usage: map_accuracy.py candidate.yaml reference.yaml [--tol 0.10] [--json out.json]
"""
import argparse
import json
import math

import numpy as np
from scipy.spatial import cKDTree


def load(yaml_path):
    import yaml, os
    meta = yaml.safe_load(open(yaml_path))
    img_path = os.path.join(os.path.dirname(yaml_path), meta['image'])
    with open(img_path, 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split()); f.readline()
        im = np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)[::-1]   # row 0 = bottom (y min)
    res, (ox, oy, _) = meta['resolution'], meta['origin']
    occ_t, free_t = meta.get('occupied_thresh', 0.65), meta.get('free_thresh', 0.25)
    p = (255 - im) / 255.0 if not meta.get('negate', 0) else im / 255.0
    occ, free = p > occ_t, (p < free_t)
    ys, xs = np.nonzero(occ)
    fy, fx = np.nonzero(free)
    pts = lambda yy, xx: np.c_[ox + (xx + 0.5) * res, oy + (yy + 0.5) * res]
    return pts(ys, xs), pts(fy, fx), res


def icp(src, dst_tree, dst, iters=60, max_d=0.6):
    R, t = np.eye(2), np.zeros(2)
    for _ in range(iters):
        cur = src @ R.T + t
        d, i = dst_tree.query(cur)
        m = d < max_d
        if m.sum() < 20:
            break
        a, b = cur[m], dst[i[m]]
        ca, cb = a.mean(0), b.mean(0)
        U, _, Vt = np.linalg.svd((a - ca).T @ (b - cb))
        dR = Vt.T @ U.T
        if np.linalg.det(dR) < 0:
            Vt[-1] *= -1; dR = Vt.T @ U.T
        dt = cb - dR @ ca
        R, t = dR @ R, dR @ t + dt
        if np.linalg.norm(dt) < 1e-5:
            break
    return R, t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('candidate'); ap.add_argument('reference')
    ap.add_argument('--tol', type=float, default=0.10); ap.add_argument('--json')
    a = ap.parse_args()
    c_occ, c_free, cres = load(a.candidate)
    r_occ, r_free, rres = load(a.reference)
    rt = cKDTree(r_occ)
    R, t = icp(c_occ, rt, r_occ)
    tf = lambda p: p @ R.T + t
    c_occ, c_free = tf(c_occ), tf(c_free)
    ct = cKDTree(c_occ)
    d_c, _ = rt.query(c_occ)
    # recall only over reference walls the candidate could have seen (near any known cell)
    known = cKDTree(np.vstack([c_occ, c_free]))
    seen = known.query(r_occ)[0] < 2 * cres
    d_r, _ = ct.query(r_occ[seen])
    prec, rec = float((d_c < a.tol).mean()), float((d_r < a.tol).mean()) if seen.any() else 0.0
    false_free = float((rt.query(c_free)[0] < a.tol).mean()) if len(c_free) else 0.0
    out = dict(precision=round(prec, 3), recall=round(rec, 3),
               f1=round(2 * prec * rec / (prec + rec), 3) if prec + rec else 0.0,
               mean_wall_err_m=round(float(d_c.mean()), 3),
               known_m2=round((len(c_occ) + len(c_free)) * cres ** 2, 1),
               false_free_frac=round(false_free, 4),
               icp_dx=round(float(t[0]), 2), icp_dy=round(float(t[1]), 2),
               icp_dyaw_deg=round(math.degrees(math.atan2(R[1, 0], R[0, 0])), 2))
    print(json.dumps(out))
    if a.json:
        open(a.json, 'w').write(json.dumps(out))


if __name__ == '__main__':
    main()
