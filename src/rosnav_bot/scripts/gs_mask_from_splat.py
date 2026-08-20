#!/usr/bin/env python3
"""
gs_mask_from_splat.py — rasterize a converted splat_points.npz (see
gs_splat_to_pointcloud.py) into a Nav2 costmap-filter-mask PGM+YAML pair, so
a trained Gaussian Splat can enrich a costmap as a KeepoutFilter layer
without touching the planner/controller (see concepts.md §28).

Not a ROS node — plain numpy/Pillow, run in the normal ROS environment (no
nerfstudio venv needed, unlike gs_splat_to_pointcloud.py):

  ros2 run rosnav_bot gs_mask_from_splat.py \\
      --npz /home/asimov/gs_data/cafe_points.npz \\
      --out src/rosnav_bot/maps/gs_keepout_cafe.yaml \\
      --z-min 0.05 --z-max 2.0

Gaussian centers in [z_min, z_max] are height-filtered (same obstacle band
convention as the local/global costmap's VoxelLayer, see nav2_params.yaml)
and rasterized to occupied cells, then dilated a couple cells to absorb
splat noise. Pass --align-to an existing map yaml (e.g. maps/map_hospital.yaml)
to pixel-align the mask to that map's resolution/origin/size instead of
auto-fitting a bounding box — required for the mask to overlay correctly
with a real SLAM map's static_layer.

Caveat: splat.ply positions are only as good as the underlying capture +
training (see concepts.md §27 gotcha on gs_capture.py's coordinate
convention) — this script does no coordinate-frame correction, it rasterizes
whatever xyz the npz carries, same as gs_view_pointcloud.py.
"""
import argparse
import os

import numpy as np
import yaml
from PIL import Image

OCCUPIED = 0    # nav2 map convention: 0 = occupied/black, 254 = free/white
FREE = 254
UNKNOWN = 205


def dilate(occ: np.ndarray, cells: int) -> np.ndarray:
    """Grow the occupied mask by `cells` pixels in each of 4 directions, per pass."""
    out = occ.copy()
    for _ in range(cells):
        grown = out.copy()
        grown[1:, :] |= out[:-1, :]
        grown[:-1, :] |= out[1:, :]
        grown[:, 1:] |= out[:, :-1]
        grown[:, :-1] |= out[:, 1:]
        out = grown
    return out


def load_align_to(map_yaml_path: str):
    with open(map_yaml_path) as f:
        m = yaml.safe_load(f)
    resolution = float(m['resolution'])
    origin = m['origin']
    map_dir = os.path.dirname(os.path.abspath(map_yaml_path))
    ref_img = Image.open(os.path.join(map_dir, m['image']))
    width, height = ref_img.size
    return resolution, origin, width, height


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--npz', required=True, help='splat_points.npz from gs_splat_to_pointcloud.py')
    ap.add_argument('--out', required=True, help='output mask yaml path (pgm written alongside, same basename)')
    ap.add_argument('--resolution', type=float, default=0.05, help='meters/pixel, ignored with --align-to')
    ap.add_argument('--margin', type=float, default=1.0, help='meters of padding around the point bbox, ignored with --align-to')
    ap.add_argument('--z-min', type=float, default=0.05, help='obstacle band floor (matches nav2_params.yaml VoxelLayer min_obstacle_height)')
    ap.add_argument('--z-max', type=float, default=2.0, help='obstacle band ceiling (matches nav2_params.yaml VoxelLayer max_obstacle_height)')
    ap.add_argument('--dilate', type=int, default=2, help='cells to grow occupied regions by, absorbs splat noise')
    ap.add_argument('--align-to', default='', help='existing map yaml to inherit resolution/origin/size from, for pixel-exact overlay')
    ap.add_argument('--xy-crop', type=float, nargs=4, default=None, metavar=('XMIN', 'YMIN', 'XMAX', 'YMAX'),
                     help='drop points outside this XY box before rasterizing. An orbit-style capture '
                          '(gs_capture.py) under-constrains depth away from the rig, so splatfacto grows '
                          'large "floater" Gaussians streaking radially outward from the capture center — '
                          'confirmed on a real converged splat to otherwise roughly double the occupied '
                          'fraction and blow up the auto-fit bbox well past the real scene. Without '
                          '--align-to (which bounds things to a real map instead), crop to the known '
                          'capture footprint (e.g. gs_capture.py centers_x/y +/- a couple meters).')
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
        resolution, origin, width, height = load_align_to(args.align_to)
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

    occ = np.zeros((height, width), dtype=bool)
    cols = np.floor((xyz[:, 0] - origin[0]) / resolution).astype(np.int64)
    # image row 0 is the top (max y), matching the map_saver/map_server convention.
    rows = (height - 1) - np.floor((xyz[:, 1] - origin[1]) / resolution).astype(np.int64)
    in_bounds = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    dropped = int((~in_bounds).sum())
    if dropped:
        print(f'{dropped} points fell outside the mask bounds (only relevant with --align-to) — dropped', flush=True)
    occ[rows[in_bounds], cols[in_bounds]] = True

    if args.dilate:
        occ = dilate(occ, args.dilate)

    img = np.full((height, width), FREE, dtype=np.uint8)
    img[occ] = OCCUPIED
    print(f'{int(occ.sum())}/{occ.size} cells marked occupied ({100 * occ.sum() / occ.size:.1f}%)', flush=True)

    out_yaml = args.out
    out_pgm = os.path.splitext(out_yaml)[0] + '.pgm'
    Image.fromarray(img, mode='L').save(out_pgm)
    with open(out_yaml, 'w') as f:
        yaml.safe_dump({
            'image': os.path.basename(out_pgm),
            'mode': 'trinary',
            'resolution': resolution,
            'origin': list(origin),
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25,
        }, f, default_flow_style=None, sort_keys=False)
    print(f'Wrote {out_pgm} and {out_yaml}', flush=True)


if __name__ == '__main__':
    main()
