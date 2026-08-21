#!/usr/bin/env python3
"""
gs_merge_splats.py — concatenate multiple splat_points.npz files (see
gs_splat_to_pointcloud.py) into one merged npz, so several robots' or
sessions' partial GS captures of the same facility can feed a single
gs_mask_from_splat.py / gs_speed_mask_from_splat.py / gs_semantic_fusion.py
downstream, instead of one splat per robot — same "one shared filter topic
covers every robotN namespace" idea multi_robot.launch.py's gs_keepout_mask
already uses, extended upstream to the splat data itself (see concepts.md
§32).

Not a ROS node — plain numpy, run in the normal ROS environment.

Common case (no --transform needed): multiple gs_capture.py runs against the
SAME Gazebo world already share one absolute coordinate frame — gs_capture.py
writes rig poses in that world frame (see concepts.md §27), so two robots
capturing different rooms of the same building can just be concatenated
directly:

  ros2 run rosnav_bot gs_merge_splats.py \\
      --npz robot1_lobby.npz robot2_kitchen.npz \\
      --out merged_facility.npz

If a capture used its own independent coordinate frame (e.g. a standalone
nerfstudio run with auto scene-centering/scaling, not gs_capture.py's
convention), pass a per-file rigid transform (translation + yaw about Z) to
register it into the first file's frame before merging:

  ros2 run rosnav_bot gs_merge_splats.py \\
      --npz robot1_lobby.npz --transform 0 0 0 0 \\
      --npz robot2_kitchen.npz --transform 8.5 -2.0 0 90 \\
      --out merged_facility.npz

Duplicate/overlapping Gaussians where two captures see the same physical
area are NOT deduplicated — downstream density-threshold rasterization
(gs_mask_from_splat.py / gs_speed_mask_from_splat.py) already tolerates
overlapping points fine (it counts per-cell, so overlap just adds count),
but a raw merged point count in an overlap region is not meaningful on its
own. Left as a known limitation, not attempted here.
"""
import argparse

import numpy as np


def yaw_matrix(deg: float) -> np.ndarray:
    rad = np.radians(deg)
    c, s = np.cos(rad), np.sin(rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class NpzTransformAction(argparse.Action):
    """Pairs each --npz with the --transform that immediately follows it,
    in command-line order, without needing a rigid --npz N --transform N
    per-index API."""

    def __call__(self, parser, namespace, values, option_string=None):
        items = getattr(namespace, 'items', None) or []
        if option_string == '--npz':
            items.append({'npz': values, 'transform': (0.0, 0.0, 0.0, 0.0)})
        else:  # --transform applies to the most recently added --npz
            if not items:
                parser.error('--transform must follow an --npz')
            items[-1]['transform'] = tuple(float(v) for v in values)
        namespace.items = items


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--npz', action=NpzTransformAction, required=True,
                     help='a splat_points.npz to merge in (repeatable)')
    ap.add_argument('--transform', action=NpzTransformAction, nargs=4, metavar=('X', 'Y', 'Z', 'YAW_DEG'),
                     help='rigid transform (translation + yaw about Z, degrees) applied to the '
                          'PRECEDING --npz before merging; omit for captures already sharing a '
                          'common world frame (identity)')
    ap.add_argument('--out', required=True, help='merged output npz path')
    args = ap.parse_args()

    all_xyz, all_rgb, all_opacity = [], [], []
    for item in args.items:
        d = np.load(item['npz'])
        xyz = d['xyz'].astype(np.float64)
        tx, ty, tz, yaw = item['transform']
        if (tx, ty, tz, yaw) != (0.0, 0.0, 0.0, 0.0):
            xyz = xyz @ yaw_matrix(yaw).T + np.array([tx, ty, tz])
            print(f'{item["npz"]}: {xyz.shape[0]} points, transform=({tx}, {ty}, {tz}, yaw={yaw}deg)', flush=True)
        else:
            print(f'{item["npz"]}: {xyz.shape[0]} points, no transform (assumed shared world frame)', flush=True)
        all_xyz.append(xyz.astype(np.float32))
        all_rgb.append(d['rgb'])
        all_opacity.append(d['opacity'])

    merged_xyz = np.concatenate(all_xyz, axis=0)
    merged_rgb = np.concatenate(all_rgb, axis=0)
    merged_opacity = np.concatenate(all_opacity, axis=0)
    print(f'Merged {len(args.items)} splats -> {merged_xyz.shape[0]} total points, '
          f'XY bbox=({merged_xyz[:, 0].min():.2f}, {merged_xyz[:, 1].min():.2f}) to '
          f'({merged_xyz[:, 0].max():.2f}, {merged_xyz[:, 1].max():.2f})', flush=True)

    np.savez(args.out, xyz=merged_xyz, rgb=merged_rgb, opacity=merged_opacity)
    print(f'Wrote {args.out}', flush=True)


if __name__ == '__main__':
    main()
