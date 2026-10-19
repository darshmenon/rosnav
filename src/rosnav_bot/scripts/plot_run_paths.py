#!/usr/bin/env python3
"""plot_run_paths.py — overlay robot trajectories + visit-density heatmap on a saved map.

Reads a nav_msgs/Odometry (or tf-derived) trajectory CSV (t,x,y) per robot and a Nav2
map (.yaml + .pgm). Not a ROS node — matplotlib only, so it runs offline on logged data.

Record a trajectory while a run is live:
  ros2 topic echo --csv /odom --field pose.pose.position > robot1.csv   # columns x,y,z
Then:
  plot_run_paths.py map_maze.yaml robot1.csv [robot2.csv ...] -o paths.png

For a map-frame trajectory pass --csv-cols x y (default: first two columns).
"""
import argparse
import os

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import yaml


def load_map(yaml_path):
    meta = yaml.safe_load(open(yaml_path))
    img = plt.imread(os.path.join(os.path.dirname(yaml_path), meta['image']))
    if img.ndim == 3:
        img = img[..., 0]
    return img, float(meta['resolution']), meta['origin'][:2]


def load_csv(path):
    arr = np.genfromtxt(path, delimiter=',', comments='#')
    return arr[:, :2] if arr.ndim == 2 else arr[None, :2]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('map_yaml')
    ap.add_argument('trajectories', nargs='+')
    ap.add_argument('-o', '--out', default='run_paths.png')
    ap.add_argument('--bins', type=float, default=0.25, help='heatmap cell size (m)')
    a = ap.parse_args()

    img, res, (ox, oy) = load_map(a.map_yaml)
    h, w = img.shape
    extent = [ox, ox + w * res, oy, oy + h * res]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    for ax in (ax1, ax2):
        ax.imshow(img, cmap='gray', origin='upper', extent=extent)
        ax.set_aspect('equal')
        ax.set_xlabel('x (m)')
    allp = []
    for i, t in enumerate(a.trajectories):
        xy = load_csv(t)
        allp.append(xy)
        ax1.plot(xy[:, 0], xy[:, 1], lw=1.4, label=os.path.splitext(os.path.basename(t))[0])
        ax1.plot(*xy[0], 'o', ms=6)
        ax1.plot(*xy[-1], 's', ms=6)
    ax1.set_title('Trajectories (o start, ■ end)')
    ax1.legend(loc='upper right', fontsize=8)
    xy = np.vstack(allp)
    xb = np.arange(extent[0], extent[1] + a.bins, a.bins)
    yb = np.arange(extent[2], extent[3] + a.bins, a.bins)
    H, _, _ = np.histogram2d(xy[:, 0], xy[:, 1], bins=[xb, yb])
    Hm = np.ma.masked_where(H.T == 0, H.T)
    im = ax2.imshow(Hm, origin='lower', extent=[xb[0], xb[-1], yb[0], yb[-1]], cmap='inferno', alpha=0.85)
    fig.colorbar(im, ax=ax2, label='samples / cell')
    ax2.set_title(f'Visit density ({a.bins:.2f} m cells) — revisits show as hot spots')
    fig.tight_layout()
    fig.savefig(a.out, dpi=130)
    print(f'wrote {a.out}')


if __name__ == '__main__':
    main()
