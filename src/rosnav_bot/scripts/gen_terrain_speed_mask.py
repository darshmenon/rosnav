#!/usr/bin/env python3
"""
gen_terrain_speed_mask.py — rasterize the terrain_* friction zones defined in
a Gazebo world SDF (see worlds/multi_terrain_robot_diff.world) into a Nav2
costmap-filter-mask for a SpeedFilter (type: 1) layer. The "static,
ground-truth" sibling of gs_speed_mask_from_splat.py (concepts.md §30/§36),
driven by SDF friction (<surface><friction><ode><mu>) instead of Gaussian
density — no splat capture needed, the ground truth is already in the world
file.

Same mask conventions as gs_speed_mask_from_splat.py so gs_speed_filter.yaml's
base=100/multiplier=-1 works unchanged: mask value 0 = full speed (100%),
value 100 = near-stop (0%). mu >= --full-speed-mu maps to 0, mu <=
--min-speed-mu maps to 100, everything between is linear. Cells outside every
terrain_* zone are left at 0 (full speed) — this mask only ever slows the
robot down, never blocks it outright (that's gs_keepout_mask / KeepoutFilter).

Not a ROS node — plain re/numpy/Pillow/PyYAML, run in the normal ROS env:

  ros2 run rosnav_bot gen_terrain_speed_mask.py \\
      --world src/rosnav_bot/worlds/multi_terrain_robot_diff.world \\
      --align-to src/rosnav_bot/maps/map_multi_terrain_robot_diff.yaml \\
      --out src/rosnav_bot/maps/terrain_speed_multi_terrain_robot_diff.yaml

Nav2 wiring: reuses the *existing* gs_speed_mask:=<path> launch arg on
slam_nav.launch.py / multi_robot.launch.py — no new launch arg, no new Nav2
plugin, no new costmap_filter_info_server needed. See concepts.md §36.
"""
import argparse
import os
import re

import numpy as np
import yaml
from PIL import Image

MODEL_RE = re.compile(r'<model name="(terrain_[^"]+)">(.*?)</model>', re.S)
POSE_RE = re.compile(r'<pose>([^<]+)</pose>')
BOX_SIZE_RE = re.compile(r'<box>\s*<size>([^<]+)</size>')
MU_RE = re.compile(r'<mu>([^<]+)</mu>')


def parse_terrain_zones(world_path):
    with open(world_path) as f:
        text = f.read()
    zones = []
    for name, body in MODEL_RE.findall(text):
        pose_m = POSE_RE.search(body)
        size_m = BOX_SIZE_RE.search(body)
        mu_m = MU_RE.search(body)
        if not (pose_m and size_m and mu_m):
            print(f'WARNING: skipping {name} — missing pose/box-size/mu', flush=True)
            continue
        px, py, pz, rr, rp, ry = (float(v) for v in pose_m.group(1).split())
        sx, sy, sz = (float(v) for v in size_m.group(1).split())
        mu = float(mu_m.group(1))
        zones.append({'name': name, 'x': px, 'y': py, 'yaw': ry, 'sx': sx, 'sy': sy, 'mu': mu})
        print(f'{name}: pose=({px:.2f},{py:.2f}) yaw={ry:.3f} size=({sx:.2f}x{sy:.2f}) mu={mu}', flush=True)
    return zones


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', required=True, help='world SDF file with terrain_* zone models')
    ap.add_argument('--out', required=True, help='output mask yaml path (pgm written alongside, same basename)')
    ap.add_argument('--resolution', type=float, default=0.05, help='meters/pixel, ignored with --align-to')
    ap.add_argument('--margin', type=float, default=1.0, help='meters of padding around the zone bbox, ignored with --align-to')
    ap.add_argument('--align-to', default='', help='existing map yaml to inherit resolution/origin/size from, for pixel-exact overlay')
    ap.add_argument('--full-speed-mu', type=float, default=1.2, help='friction at/above which mask value is 0 (100% speed)')
    ap.add_argument('--min-speed-mu', type=float, default=0.25, help='friction at/below which mask value is 100 (near-stop)')
    args = ap.parse_args()

    if args.full_speed_mu <= args.min_speed_mu:
        raise SystemExit('--full-speed-mu must be greater than --min-speed-mu')

    zones = parse_terrain_zones(args.world)
    if not zones:
        raise SystemExit(f'No terrain_* models found in {args.world}')

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
        pad = [max(z['sx'], z['sy']) for z in zones]
        origin_x = min(z['x'] - p / 2 - args.margin for z, p in zip(zones, pad))
        origin_y = min(z['y'] - p / 2 - args.margin for z, p in zip(zones, pad))
        max_x = max(z['x'] + p / 2 + args.margin for z, p in zip(zones, pad))
        max_y = max(z['y'] + p / 2 + args.margin for z, p in zip(zones, pad))
        resolution = args.resolution
        width = max(1, int(np.ceil((max_x - origin_x) / resolution)))
        height = max(1, int(np.ceil((max_y - origin_y) / resolution)))
        origin = [origin_x, origin_y, 0.0]
        print(f'Auto-fit bbox: origin={origin}, size={width}x{height} @ {resolution} m/px', flush=True)

    # map-frame cell centers — row 0 = top = max y, matches gs_speed_mask_from_splat.py.
    cols = np.arange(width)
    rows = np.arange(height)
    cx = origin[0] + (cols + 0.5) * resolution
    cy = origin[1] + (height - 1 - rows + 0.5) * resolution
    gx, gy = np.meshgrid(cx, cy)  # shape (height, width)

    value = np.zeros((height, width), dtype=np.float64)
    covered = np.zeros((height, width), dtype=bool)
    for z in zones:
        mu = min(max(z['mu'], args.min_speed_mu), args.full_speed_mu)
        speed_pct = (mu - args.min_speed_mu) / (args.full_speed_mu - args.min_speed_mu)
        mask_value = (1.0 - speed_pct) * 100.0  # matches gs_speed_filter.yaml's base=100,multiplier=-1
        dx = gx - z['x']
        dy = gy - z['y']
        cyaw, syaw = np.cos(-z['yaw']), np.sin(-z['yaw'])
        lx = dx * cyaw - dy * syaw
        ly = dx * syaw + dy * cyaw
        inside = (np.abs(lx) <= z['sx'] / 2) & (np.abs(ly) <= z['sy'] / 2)
        value[inside] = mask_value
        covered |= inside
        print(f"{z['name']}: mu={z['mu']} -> mask value {mask_value:.1f} ({speed_pct*100:.0f}% speed), "
              f"{int(inside.sum())} cells", flush=True)

    print(f'{int(covered.sum())}/{covered.size} cells inside a terrain zone '
          f'({100*covered.sum()/covered.size:.1f}%); rest left at 0 (full speed)', flush=True)

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
