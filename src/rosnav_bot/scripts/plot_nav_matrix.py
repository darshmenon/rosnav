#!/usr/bin/env python3
"""Overlay every planner+controller combo's driven path on the map.

Usage: plot_nav_matrix.py MAP.yaml OUT.png label=a_nav.json [label=b_nav.json ...]
Solid = driven ground-truth track, dashed gray = planned path, star = goal.
"""
import json
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml


def load_map(yaml_path):
    m = yaml.safe_load(open(yaml_path))
    img = plt.imread(os.path.join(os.path.dirname(yaml_path), m['image']))
    if img.ndim == 3:
        img = img[..., 0]
    h, w = img.shape
    ox, oy = m['origin'][:2]
    r = m['resolution']
    return img, (ox, ox + w * r, oy, oy + h * r)


def main():
    yaml_path, out = sys.argv[1], sys.argv[2]
    runs = {a.split('=', 1)[0]: json.load(open(a.split('=', 1)[1]))['runs'] for a in sys.argv[3:]}
    img, ext = load_map(yaml_path)
    cmap = plt.get_cmap('tab10')
    fig, ax = plt.subplots(figsize=(10, 7))
    ax.imshow(img, cmap='gray', extent=ext, origin='upper', alpha=.5)
    pts = []
    planned_drawn = set()
    for ci, (label, runs_) in enumerate(runs.items()):
        n_ok = sum(1 for g in runs_ if g['success'])
        first = True
        for g in runs_:
            tr = [t[2] for t in g.get('track', []) if t[2]]
            pl = g.get('planned_path')
            if pl and g['goal_index'] not in planned_drawn:
                planned_drawn.add(g['goal_index'])
                ax.plot(*zip(*pl), '--', color='gray', lw=1)
            if tr:
                ax.plot(*zip(*tr), '-', color=cmap(ci), lw=2,
                        label=f'{label}  ({n_ok}/{len(runs_)} ok)' if first else None)
                first = False
                pts += tr
            ax.plot(g['goal']['x'], g['goal']['y'], 'k*', ms=12, zorder=5)
            pts.append((g['goal']['x'], g['goal']['y']))
    xs, ys = zip(*pts)
    ax.set_xlim(min(xs) - 1, max(xs) + 1)
    ax.set_ylim(min(ys) - 1, max(ys) + 1)
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Nav2 planner x controller: driven paths (cafe, fixed goals)')
    ax.grid(alpha=.3)
    ax.legend(loc='upper left', bbox_to_anchor=(1.01, 1), fontsize=9)
    plt.tight_layout()
    plt.savefig(out, dpi=130)
    print('wrote', out)


if __name__ == '__main__':
    main()
