#!/usr/bin/env python3
"""
gen_privacy_zone_mask.py — rasterize a rectangular map-frame zone into a Nav2
costmap-filter-mask for a BinaryFilter (type: 3) layer. Same mask-generation
shape as gen_terrain_speed_mask.py (concepts.md §36) but binary instead of
graded: mask value 100 inside the zone, 0 outside — matching
gs_binary_filter.yaml's base=0/multiplier=1 so BinaryFilter's flip_threshold
(50.0, in nav2_params.yaml) cleanly separates the two.

Not a ROS node — plain numpy/Pillow/PyYAML, run in the normal ROS env:

  ros2 run rosnav_bot gen_privacy_zone_mask.py \\
      --align-to src/rosnav_bot/maps/map_house.yaml \\
      --center -1.0 2.5 --size 2.0 2.0 \\
      --out src/rosnav_bot/maps/privacy_zone_house.yaml

Nav2 wiring: slam_nav.launch.py gs_privacy_mask:=<path> — see concepts.md §38.
"""
import argparse
import os

import numpy as np
import yaml
from PIL import Image


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', required=True, help='output mask yaml path (pgm written alongside, same basename)')
    ap.add_argument('--center', type=float, nargs=2, required=True, metavar=('X', 'Y'), help='zone center, map frame meters')
    ap.add_argument('--size', type=float, nargs=2, required=True, metavar=('SX', 'SY'), help='zone width/height, meters')
    ap.add_argument('--yaw', type=float, default=0.0, help='zone rotation, radians')
    ap.add_argument('--resolution', type=float, default=0.05, help='meters/pixel, ignored with --align-to')
    ap.add_argument('--margin', type=float, default=1.0, help='meters of padding around the zone, ignored with --align-to')
    ap.add_argument('--align-to', default='', help='existing map yaml to inherit resolution/origin/size from, for pixel-exact overlay')
    args = ap.parse_args()

    cx0, cy0 = args.center
    sx, sy = args.size

    if args.align_to:
        with open(args.align_to) as f:
            m = yaml.safe_load(f)
        resolution = float(m['resolution'])
        origin = m['origin']
        map_dir = os.path.dirname(os.path.abspath(args.align_to))
        ref_img = Image.open(os.path.join(map_dir, m['image']))
        width, height = ref_img.size
        print(f'Aligned to {args.align_to}: resolution={resolution}, origin={origin}, size={width}x{height}', flush=True)
    else:
        pad = max(sx, sy) / 2 + args.margin
        origin_x = cx0 - pad
        origin_y = cy0 - pad
        resolution = args.resolution
        width = max(1, int(np.ceil(2 * pad / resolution)))
        height = max(1, int(np.ceil(2 * pad / resolution)))
        origin = [origin_x, origin_y, 0.0]
        print(f'Auto-fit bbox: origin={origin}, size={width}x{height} @ {resolution} m/px', flush=True)

    cols = np.arange(width)
    rows = np.arange(height)
    px = origin[0] + (cols + 0.5) * resolution
    py = origin[1] + (height - 1 - rows + 0.5) * resolution
    gx, gy = np.meshgrid(px, py)  # shape (height, width), row 0 = top = max y

    dx = gx - cx0
    dy = gy - cy0
    cyaw, syaw = np.cos(-args.yaw), np.sin(-args.yaw)
    lx = dx * cyaw - dy * syaw
    ly = dx * syaw + dy * cyaw
    inside = (np.abs(lx) <= sx / 2) & (np.abs(ly) <= sy / 2)

    value = np.where(inside, 100.0, 0.0)
    print(f'{int(inside.sum())}/{inside.size} cells inside the privacy zone '
          f'({100*inside.sum()/inside.size:.1f}%)', flush=True)

    img = np.round(value).astype(np.uint8)
    out_yaml = args.out
    out_pgm = os.path.splitext(out_yaml)[0] + '.pgm'
    Image.fromarray(img, mode='L').save(out_pgm)
    with open(out_yaml, 'w') as f:
        yaml.safe_dump({
            'image': os.path.basename(out_pgm),
            'mode': 'scale',
            'resolution': resolution,
            'origin': list(origin),
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25,
        }, f, default_flow_style=None, sort_keys=False)
    print(f'Wrote {out_pgm} and {out_yaml}', flush=True)


if __name__ == '__main__':
    main()
