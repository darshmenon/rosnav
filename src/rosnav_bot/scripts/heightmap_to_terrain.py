#!/usr/bin/env python3
"""heightmap_to_terrain.py — exact terrain model from a Gazebo heightmap PNG.

The outdoor worlds use Fuel's "Heightmap Bowl" (129x129 px, <size>129 129 10</size>, centred on the
origin). The heightmap IS the ground truth, so no SLAM is needed to know the terrain: this builds
  <out>.ply                point cloud (x y z nx ny nz slope_deg) on a regular grid
  <out>_elevation.png      grey-scaled height, for viewing
  <out>_speed.{yaml,pgm}   Nav2 SpeedFilter mask (mode: scale; 0 = full speed, 100 = near-stop)
  <out>_keepout.{yaml,pgm} Nav2 KeepoutFilter mask (occupied where slope > --block-deg)
and prints slope statistics. A diff-drive base uses them as slope-graded speed limits and a keepout
on too-steep ground, instead of treating hills as walls (see terrain_traversability.py for the live,
sensor-based version of the same rule).

  python3 src/rosnav_bot/scripts/heightmap_to_terrain.py \\
      --heightmap "$HOME/.gz/fuel/fuel.gazebosim.org/chapulina/models/heightmap bowl/1/materials/textures/heightmap_bowl.png" \\
      --out src/rosnav_bot/maps/terrain_outdoor
Wire the masks with the existing gs_speed_mask:= / gs_keepout_mask:= launch args.
"""
import argparse
import math

import numpy as np
from PIL import Image


def bilinear(h, xs, ys):
    """h: HxW, xs/ys in pixel coords."""
    x0 = np.clip(np.floor(xs).astype(int), 0, h.shape[1] - 2)
    y0 = np.clip(np.floor(ys).astype(int), 0, h.shape[0] - 2)
    fx, fy = xs - x0, ys - y0
    return (h[y0, x0] * (1 - fx) * (1 - fy) + h[y0, x0 + 1] * fx * (1 - fy)
            + h[y0 + 1, x0] * (1 - fx) * fy + h[y0 + 1, x0 + 1] * fx * fy)


def write_mask(path_base, img, res, ox, oy, mode):
    Image.fromarray(img).save(path_base + '.pgm')
    base = path_base.split('/')[-1]
    thr = ('occupied_thresh: 0.65\nfree_thresh: 0.25\n' if mode == 'trinary'
           else 'occupied_thresh: 1.0\nfree_thresh: 0.0\n')
    with open(path_base + '.yaml', 'w') as f:
        f.write(f'image: {base}.pgm\nmode: {mode}\nresolution: {res}\n'
                f'origin: [{ox:.3f}, {oy:.3f}, 0]\nnegate: 0\n{thr}')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--heightmap', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--size', type=float, nargs=3, default=[129.0, 129.0, 10.0], metavar=('X', 'Y', 'Z'))
    ap.add_argument('--pos', type=float, nargs=3, default=[0.0, 0.0, 0.0], metavar=('X', 'Y', 'Z'))
    ap.add_argument('--res', type=float, default=0.25, help='output grid cell (m)')
    ap.add_argument('--max-deg', type=float, default=20.0, help='slope mapped to near-stop speed')
    ap.add_argument('--block-deg', type=float, default=25.0, help='slope above which keepout is set')
    ap.add_argument('--min-speed', type=float, default=0.2, help='speed fraction at --max-deg')
    a = ap.parse_args()

    im = np.asarray(Image.open(a.heightmap).convert('L'), dtype=np.float64) / 255.0
    H, W = im.shape
    sx, sy, sz = a.size
    # Gazebo spreads the W x H samples across the size; image row 0 is +y (north).
    xs = np.arange(-sx / 2, sx / 2 + 1e-9, a.res) + a.pos[0]
    ys = np.arange(-sy / 2, sy / 2 + 1e-9, a.res) + a.pos[1]
    px = (xs - a.pos[0] + sx / 2) / sx * (W - 1)
    py = (sy / 2 - (ys - a.pos[1])) / sy * (H - 1)
    PX, PY = np.meshgrid(px, py)
    z = bilinear(im, PX, PY) * sz + a.pos[2]          # rows follow ys (ascending y)
    gy, gx = np.gradient(z, a.res)
    slope = np.degrees(np.arctan(np.hypot(gx, gy)))
    nrm = np.stack([-gx, -gy, np.ones_like(z)], -1)
    nrm /= np.linalg.norm(nrm, axis=-1, keepdims=True)

    X, Y = np.meshgrid(xs, ys)
    pts = np.c_[X.ravel(), Y.ravel(), z.ravel(), nrm.reshape(-1, 3), slope.ravel()].astype('<f4')
    with open(a.out + '.ply', 'wb') as f:
        f.write(('ply\nformat binary_little_endian 1.0\n'
                 f'element vertex {len(pts)}\n'
                 'property float x\nproperty float y\nproperty float z\n'
                 'property float nx\nproperty float ny\nproperty float nz\n'
                 'property float slope_deg\nend_header\n').encode())
        f.write(pts.tobytes())

    e = ((z - z.min()) / max(np.ptp(z), 1e-9) * 255).astype(np.uint8)[::-1]
    Image.fromarray(e).resize((e.shape[1] * 4, e.shape[0] * 4), Image.NEAREST).save(a.out + '_elevation.png')

    ox, oy = xs[0] - a.res / 2, ys[0] - a.res / 2
    # SpeedFilter (scale mode): value = 100 * (1 - speed fraction); 0 = full speed
    frac = 1.0 - (1.0 - a.min_speed) * np.clip(slope / a.max_deg, 0, 1)
    speed = np.round((1.0 - frac) * 255).astype(np.uint8)   # 0 .. (1-min_speed)*255
    write_mask(a.out + '_speed', speed[::-1], a.res, ox, oy, 'scale')
    keep = np.where(slope > a.block_deg, 0, 254).astype(np.uint8)
    write_mask(a.out + '_keepout', keep[::-1], a.res, ox, oy, 'trinary')

    print(f'terrain {len(pts)} pts  x[{xs[0]:.1f},{xs[-1]:.1f}] y[{ys[0]:.1f},{ys[-1]:.1f}]  '
          f'z[{z.min():.2f},{z.max():.2f}] m')
    print(f'slope deg: mean={slope.mean():.1f} p95={np.percentile(slope, 95):.1f} max={slope.max():.1f}  '
          f'>{a.max_deg:.0f} deg: {100 * (slope > a.max_deg).mean():.2f}%  >{a.block_deg:.0f} deg: {100 * (slope > a.block_deg).mean():.2f}%')
    sx0, sy0 = 40.0, 0.0
    c, r = int((sx0 - xs[0]) / a.res), int((sy0 - ys[0]) / a.res)
    print(f'spawn (40,0): z={z[r, c]:.2f} m slope={slope[r, c]:.1f} deg')


if __name__ == '__main__':
    main()
