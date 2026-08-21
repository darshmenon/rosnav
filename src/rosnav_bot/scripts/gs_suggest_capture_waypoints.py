#!/usr/bin/env python3
"""
gs_suggest_capture_waypoints.py — close the loop the other direction from
the rest of the GS-navigation work (§28-32): instead of using a splat to
improve navigation, use what navigation (SLAM) has already explored to
suggest where gs_capture.py's next orbit sweep should point, so the splat
grows to cover the areas the robot has actually mapped.

Not a ROS node — plain numpy/Pillow, run in the normal ROS environment.
Reads a saved Nav2 map (e.g. maps/map_cafe.yaml from `ros2 run nav2_map_server
map_saver_cli` after a SLAM session) and, optionally, an existing splat's
converted npz (gs_splat_to_pointcloud.py) to know what's already covered.
Free cells NOT already covered are grid-binned into clusters and each
cluster's centroid becomes a suggested capture center.

  ros2 run rosnav_bot gs_suggest_capture_waypoints.py \\
      --map src/rosnav_bot/maps/map_cafe.yaml \\
      --covered-npz /home/asimov/gs_data/cafe_points_clean2.npz \\
      --cell-size 2.0 --top-n 6

Known limitation: gs_capture.py's centers_x/centers_y parameters form a
CARTESIAN GRID (`[(x, y) for x in centers_x for y in centers_y]`), not an
arbitrary waypoint list — so scattered cluster centroids from an irregular
uncovered region don't collapse cleanly into that (centers_x, centers_y)
parameterization. This script prints the suggested (x, y) points directly;
turning each into an actual capture pass currently means either running
gs_capture.py once per point (single-element centers_x/centers_y each time)
or extending gs_capture.py to accept an explicit waypoint list — left as
follow-up, not attempted here (see concepts.md §33).
"""
import argparse
import os

import numpy as np
import yaml
from PIL import Image


def load_map(map_yaml_path: str):
    with open(map_yaml_path) as f:
        m = yaml.safe_load(f)
    resolution = float(m['resolution'])
    origin = m['origin']
    map_dir = os.path.dirname(os.path.abspath(map_yaml_path))
    img = np.array(Image.open(os.path.join(map_dir, m['image'])).convert('L'))
    negate = int(m.get('negate', 0))
    if negate:
        img = 255 - img
    # Trinary PGM convention this repo's tools use (gs_mask_from_splat.py,
    # standard map_saver_cli output): 254=free, 0=occupied, ~205=unknown.
    # Only the free band counts — occupied and unknown are both "not safe
    # to suggest a capture waypoint here".
    free_mask = img >= 250
    height, width = img.shape
    return free_mask, resolution, origin, width, height


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', required=True, help='Nav2 map yaml (e.g. from map_saver_cli after SLAM)')
    ap.add_argument('--covered-npz', default='', help='existing splat_points.npz — cells within '
                     '--cover-radius of any of its points are treated as already covered')
    ap.add_argument('--cover-radius', type=float, default=1.0,
                     help='meters — how close a covered-npz point must be to a free cell to count '
                          'it as already covered (grid-approximated, not an exact radius query)')
    ap.add_argument('--cell-size', type=float, default=2.0,
                     help='clustering bin size in meters for grouping uncovered free cells')
    ap.add_argument('--top-n', type=int, default=6, help='max suggested waypoints, largest clusters first')
    args = ap.parse_args()

    free_mask, resolution, origin, width, height = load_map(args.map)
    rows, cols = np.nonzero(free_mask)
    # inverse of gs_mask_from_splat.py's world->pixel convention (row 0 = top/max-y).
    xs = origin[0] + (cols + 0.5) * resolution
    ys = origin[1] + (height - 1 - rows + 0.5) * resolution
    print(f'{len(xs)} free cells in {args.map}', flush=True)

    if args.covered_npz:
        d = np.load(args.covered_npz)
        covered_xyz = d['xyz']
        # Grid-approximate coverage: bin both the covered splat points and the
        # free-space query points at --cover-radius resolution and keep only
        # free cells whose bin (or an adjacent bin) contains a covered point —
        # avoids an O(n*m) distance query against a large splat point cloud.
        r = args.cover_radius
        covered_bins = set(zip(np.floor(covered_xyz[:, 0] / r).astype(int),
                                np.floor(covered_xyz[:, 1] / r).astype(int)))
        query_bins_x = np.floor(xs / r).astype(int)
        query_bins_y = np.floor(ys / r).astype(int)
        uncovered = np.zeros(len(xs), dtype=bool)
        for i in range(len(xs)):
            bx, by = query_bins_x[i], query_bins_y[i]
            near = any((bx + dx, by + dy) in covered_bins for dx in (-1, 0, 1) for dy in (-1, 0, 1))
            uncovered[i] = not near
        xs, ys = xs[uncovered], ys[uncovered]
        print(f'{len(xs)}/{len(uncovered)} free cells are NOT within {r}m of an existing '
              f'covered ({args.covered_npz}) point', flush=True)

    if len(xs) == 0:
        print('Nothing uncovered — no waypoints to suggest.', flush=True)
        return

    bin_x = np.floor(xs / args.cell_size).astype(int)
    bin_y = np.floor(ys / args.cell_size).astype(int)
    clusters = {}
    for i in range(len(xs)):
        key = (int(bin_x[i]), int(bin_y[i]))
        clusters.setdefault(key, []).append(i)

    ranked = sorted(clusters.items(), key=lambda kv: len(kv[1]), reverse=True)[:args.top_n]
    print(f'\nTop {len(ranked)} suggested capture centers (largest uncovered area first):', flush=True)
    for key, idxs in ranked:
        cx = float(xs[idxs].mean())
        cy = float(ys[idxs].mean())
        print(f'  centers_x:=[{cx:.2f}] centers_y:=[{cy:.2f}]  '
              f'({len(idxs)} uncovered free cells, ~{len(idxs) * resolution * resolution:.1f}m^2)', flush=True)


if __name__ == '__main__':
    main()
