#!/usr/bin/env python3
"""
gs_speed_mask_from_splat.py — rasterize a converted splat_points.npz (see
gs_splat_to_pointcloud.py) into a Nav2 costmap-filter-mask for a SpeedFilter
(type: 1) layer, graded by Gaussian density instead of the binary occupied/
free split gs_mask_from_splat.py's KeepoutFilter mask uses (see concepts.md
§28/§30). A KeepoutFilter is all-or-nothing; a SpeedFilter lets the robot
slow down through sparse/hazy splat regions (low confidence, or real open-air
haze — see gs_mask_from_splat.py's docstring on why presence-only
rasterization overmarks) instead of just refusing to enter, while still
forcing a near-stop through the densest (most likely solid) regions.

Same density-count-per-cell approach as gs_mask_from_splat.py (binned grid,
per-cell Gaussian COUNT, not mere presence), but instead of thresholding to
occupied/free, count is linearly mapped to a speed percentage: empty cells
-> 100% (unrestricted), cells at or above --dense-percentile -> --min-speed
(near-stop), everything between interpolated.

Not a ROS node — plain numpy/Pillow, run in the normal ROS environment:

  ros2 run rosnav_bot gs_speed_mask_from_splat.py \\
      --npz /home/asimov/gs_data/cafe_points.npz \\
      --out src/rosnav_bot/maps/gs_speed_cafe.yaml \\
      --z-min 0.05 --z-max 2.0

Nav2 wiring: the output mask uses `mode: scale` (map_server publishes the
raw 0-255 pixel value linearly as a 0-100 OccupancyGrid value, unlike
gs_mask_from_splat.py's `mode: trinary`) — see gs_speed_filter.yaml, whose
costmap_filter_info_server `base`/`multiplier` convert that 0-100 value into
an actual speed-limit percentage via `speed_pct = base + multiplier * value`.
This script writes value=0 for empty cells and value=100 for max-density
cells, so gs_speed_filter.yaml uses base=100, multiplier=-1 (speed_pct =
100 - value) to get empty->100% speed, max-density->0% speed.
"""
import argparse
import os

import numpy as np
import yaml
from PIL import Image


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--npz', required=True, help='splat_points.npz from gs_splat_to_pointcloud.py')
    ap.add_argument('--out', required=True, help='output mask yaml path (pgm written alongside, same basename)')
    ap.add_argument('--resolution', type=float, default=0.05, help='meters/pixel, ignored with --align-to')
    ap.add_argument('--margin', type=float, default=1.0, help='meters of padding around the point bbox, ignored with --align-to')
    ap.add_argument('--z-min', type=float, default=0.05, help='obstacle band floor (matches nav2_params.yaml VoxelLayer min_obstacle_height)')
    ap.add_argument('--z-max', type=float, default=2.0, help='obstacle band ceiling (matches nav2_params.yaml VoxelLayer max_obstacle_height)')
    ap.add_argument('--dense-percentile', type=float, default=90.0,
                     help='Gaussian count at/above this percentile of nonzero cells maps to full '
                          'restriction (mask value 100); count 0 always maps to 0 (unrestricted). '
                          'Same "densest cells are real surfaces" logic as gs_mask_from_splat.py.')
    ap.add_argument('--align-to', default='', help='existing map yaml to inherit resolution/origin/size from, for pixel-exact overlay')
    ap.add_argument('--xy-crop', type=float, nargs=4, default=None, metavar=('XMIN', 'YMIN', 'XMAX', 'YMAX'),
                     help='drop points outside this XY box before rasterizing (see gs_mask_from_splat.py '
                          'for why — orbit-capture floater noise otherwise blows up the auto-fit bbox)')
    args = ap.parse_args()

    d = np.load(args.npz)
    xyz = d['xyz']
    n_total = xyz.shape[0]
    if args.xy_crop:
        xmin, ymin, xmax, ymax = args.xy_crop
        crop = (xyz[:, 0] >= xmin) & (xyz[:, 0] <= xmax) & (xyz[:, 1] >= ymin) & (xyz[:, 1] <= ymax)
        xyz = xyz[crop]
        print(f'{xyz.shape[0]}/{n_total} Gaussians survive --xy-crop {args.xy_crop}', flush=True)
    band = (xyz[:, 2] >= args.z_min) & (xyz[:, 2] <= args.z_max)
    xyz = xyz[band]
    print(f'{xyz.shape[0]}/{n_total} Gaussians in height band [{args.z_min}, {args.z_max}]', flush=True)
    if xyz.shape[0] == 0:
        raise SystemExit('No Gaussians survived the height-band filter — check --z-min/--z-max')

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
        origin_x = float(xyz[:, 0].min()) - args.margin
        origin_y = float(xyz[:, 1].min()) - args.margin
        max_x = float(xyz[:, 0].max()) + args.margin
        max_y = float(xyz[:, 1].max()) + args.margin
        resolution = args.resolution
        width = max(1, int(np.ceil((max_x - origin_x) / resolution)))
        height = max(1, int(np.ceil((max_y - origin_y) / resolution)))
        origin = [origin_x, origin_y, 0.0]
        print(f'Auto-fit bbox: origin={origin}, size={width}x{height} @ {resolution} m/px', flush=True)

    cols = np.floor((xyz[:, 0] - origin[0]) / resolution).astype(np.int64)
    rows = (height - 1) - np.floor((xyz[:, 1] - origin[1]) / resolution).astype(np.int64)
    in_bounds = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    dropped = int((~in_bounds).sum())
    if dropped:
        print(f'{dropped} points fell outside the mask bounds (only relevant with --align-to) — dropped', flush=True)

    count = np.zeros((height, width), dtype=np.float64)
    np.add.at(count, (rows[in_bounds], cols[in_bounds]), 1.0)
    nonzero = count[count > 0]
    if nonzero.size == 0:
        raise SystemExit('No cells received any Gaussians — nothing to rasterize')
    dense_ref = max(np.percentile(nonzero, args.dense_percentile), 1.0)
    print(f'Gaussian count per cell: median(nonzero)={np.median(nonzero):.1f}, '
          f'p{args.dense_percentile:g}={dense_ref:.1f} (-> mask value 100), max={nonzero.max():.0f}', flush=True)

    # 0 (empty) -> 0, dense_ref and above -> 100, linear in between.
    value = np.clip(count / dense_ref * 100.0, 0.0, 100.0)
    img = np.round(value).astype(np.uint8)
    print(f'Mask value stats: mean={value.mean():.1f}, '
          f'{(value >= 100).sum()}/{value.size} cells fully restricted ({100*(value>=100).sum()/value.size:.1f}%)', flush=True)

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
