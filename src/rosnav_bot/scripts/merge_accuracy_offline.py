#!/usr/bin/env python3
"""
merge_accuracy_offline.py - how much pose error can the known-pose map merge take?

Splits a saved map into two overlapping robot maps, perturbs robot B's pose by a known error,
merges with map_merge_known.merge_grids and scores the result against the original with the same
ICP-aligned metrics as map_accuracy.py. No ROS, no simulator.

usage: merge_accuracy_offline.py [map.yaml] [--json out.json]
"""
import argparse
import importlib.util
import json
import math
import os
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def _load(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(HERE, name + '.py'))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def read_pgm(path):
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        return np.frombuffer(f.read(), dtype=np.uint8).reshape(h, w)[::-1].copy()   # row 0 = y min


def write_map(grid, ox, oy, res, stem):
    """grid int8 FREE/OCC/UNKNOWN, row 0 = y min -> stem.pgm + stem.yaml."""
    img = np.full(grid.shape, 205, dtype=np.uint8)
    img[grid == 0], img[grid == 100] = 254, 0
    with open(stem + '.pgm', 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (grid.shape[1], grid.shape[0]))
        f.write(img[::-1].tobytes())
    with open(stem + '.yaml', 'w') as f:
        f.write(f'image: {os.path.basename(stem)}.pgm\nresolution: {res}\norigin: [{ox}, {oy}, 0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return stem + '.yaml'


def run(map_yaml, errors):
    import yaml
    mm, acc = _load('map_merge_known'), _load('map_accuracy')
    meta = yaml.safe_load(open(map_yaml))
    res = meta['resolution']
    ox, oy = meta['origin'][0], meta['origin'][1]
    im = read_pgm(os.path.join(os.path.dirname(map_yaml), meta['image']))
    gt = np.full(im.shape, -1, dtype=np.int8)
    gt[im >= 191], gt[im <= 90] = 0, 100
    w = gt.shape[1]
    a, b = gt.copy(), gt.copy()
    a[:, int(w * 0.6):] = -1
    b[:, :int(w * 0.4)] = -1
    rows = []
    with tempfile.TemporaryDirectory() as td:
        ref = write_map(gt, ox, oy, res, os.path.join(td, 'ref'))
        for dxy, dyaw in errors:
            pb = dict(x=dxy, y=0.0, yaw=math.radians(dyaw))
            # rotate about the map origin: that is how a wrong spawn yaw acts on the local frame
            merged = mm.merge_grids([(a, ox, oy, dict(x=0, y=0, yaw=0)), (b, ox, oy, pb)], res)
            grid, min_c, min_r = merged
            cand = write_map(grid, min_c * res, min_r * res, res, os.path.join(td, 'cand'))
            c_occ, c_free, cres = acc.load(cand)
            r_occ, _, _ = acc.load(ref)
            from scipy.spatial import cKDTree
            rt = cKDTree(r_occ)
            R, t = acc.icp(c_occ, rt, r_occ)
            c_occ = c_occ @ R.T + t
            d_c, _ = rt.query(c_occ)
            seen = cKDTree(c_occ).query(r_occ)[0] < 2 * cres
            d_r, _ = cKDTree(c_occ).query(r_occ[seen])
            prec, rec = float((d_c < 0.10).mean()), float((d_r < 0.10).mean())
            rows.append(dict(pose_err_m=dxy, yaw_err_deg=dyaw, precision=round(prec, 3), recall=round(rec, 3),
                             f1=round(2 * prec * rec / (prec + rec), 3) if prec + rec else 0.0,
                             mean_wall_err_m=round(float(d_c.mean()), 3)))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map', nargs='?', default=os.path.join(HERE, '..', 'maps', 'map_cafe.yaml'))
    ap.add_argument('--json')
    a = ap.parse_args()
    errors = [(0.0, 0.0), (0.05, 0.0), (0.10, 0.0), (0.20, 0.0), (0.40, 0.0),
              (0.0, 1.0), (0.0, 2.0), (0.0, 5.0), (0.10, 2.0)]
    rows = run(a.map, errors)
    print(f"{'pose err':>9} {'yaw err':>8} {'prec':>6} {'recall':>7} {'F1':>6} {'wall err':>9}")
    for r in rows:
        print(f"{r['pose_err_m']:>8.2f}m {r['yaw_err_deg']:>7.1f}d {r['precision']:>6.3f} {r['recall']:>7.3f} "
              f"{r['f1']:>6.3f} {r['mean_wall_err_m']:>8.3f}m")
    if a.json:
        json.dump(rows, open(a.json, 'w'), indent=1)


if __name__ == '__main__':
    main()
