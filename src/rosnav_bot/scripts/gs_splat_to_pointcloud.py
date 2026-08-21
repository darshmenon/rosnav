#!/usr/bin/env python3
"""
gs_splat_to_pointcloud.py — convert a nerfstudio gaussian-splat .ply export
into a plain (xyz, rgb, opacity) .npz for gs_view_pointcloud.py to publish.

Not a ROS node — run inside the nerfstudio venv (needs plyfile), separate
from the ROS 2 environment gs_view_pointcloud.py runs in:

  source ~/venvs/nerfstudio/bin/activate
  ns-export gaussian-splat --load-config <config.yml> --output-dir splat_export/
  python3 scripts/gs_splat_to_pointcloud.py splat_export/splat.ply splat_points.npz

Gaussian color is stored as spherical-harmonics DC term (f_dc_0/1/2), not
plain RGB — converted here via the degree-0 SH basis constant.

Also drops "floater" Gaussians by scale: an orbit-style capture (gs_capture.py
sweeps a rig around a small fixed area, see concepts.md §27) under-constrains
depth for anything far from that area, and splatfacto compensates with large,
blurry Gaussians streaking radially outward from the capture center instead of
correctly modeling background — confirmed empirically on a real 30000-step
converged cafe splat: median max-axis scale is ~0.009m within 2m of center vs.
~0.078m beyond 6m, an 8x jump, and left unfiltered these floaters roughly
double the "occupied" fraction gs_mask_from_splat.py rasterizes. Opacity
thresholding alone does NOT catch this (floaters are often high-opacity, just
oversized) — scale is the actual discriminator.

Undoes nerfstudio's training-time coordinate normalization. `ns-train` calls
`auto_orient_and_center_poses` then auto-scales camera positions so the scene
roughly fits a unit-ish cube (`dataparser_transforms.json`: `X_train = scale *
(R @ X_world + t)`) — and confirmed by reading nerfstudio's own exporter
source (nerfstudio/scripts/exporter.py: `positions = model.means...`),
`ns-export gaussian-splat` writes those raw TRAINING-space positions straight
to the .ply with no inverse applied. Left uncorrected, every exported splat
is off by `scale` (e.g. 0.22 observed on a real capture — the whole scene
comes out ~4.5x too small versus true Gazebo-world meters), which silently
undersized every GS-derived Nav2 artifact this repo builds (keepout/speed
masks, semantic-fusion 3D positions, relocalization checks) relative to the
robot/map TF frames — caught by eyeballing the splat next to the robot model
in RViz and looking implausibly small, not by any of this pipeline's own
automated checks. Pass --dataparser-transform to correct it; omit only if
you've already re-exported through a nerfstudio version/flag that undoes this
itself (verify by comparing bbox extent against a known real-world distance
before trusting an uncorrected npz).
"""
import argparse
import json
import sys

import numpy as np
from plyfile import PlyData

SH_C0 = 0.28209479177387814


def load_dataparser_transform(path):
    with open(path) as f:
        d = json.load(f)
    transform = np.array(d['transform'], dtype=np.float64)  # 3x4: [R | t]
    r, t = transform[:, :3], transform[:, 3]
    scale = float(d['scale'])
    return r, t, scale


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('ply_path')
    ap.add_argument('out_path')
    ap.add_argument('opacity_thresh', nargs='?', type=float, default=0.3)
    ap.add_argument('max_scale', nargs='?', type=float, default=0.05)
    ap.add_argument('--dataparser-transform', default='',
                     help='path to dataparser_transforms.json from the training output dir '
                          '(outputs/.../splatfacto/<timestamp>/dataparser_transforms.json) — '
                          'undoes nerfstudio\'s training-time scale/center normalization so xyz '
                          'comes out in true Gazebo-world meters. Strongly recommended; see the '
                          'module docstring for why the .ply is wrong without it.')
    args = ap.parse_args()

    ply_path, out_path = args.ply_path, args.out_path
    opacity_thresh, max_scale = args.opacity_thresh, args.max_scale

    ply = PlyData.read(ply_path)
    v = ply['vertex']
    xyz = np.stack([v['x'], v['y'], v['z']], axis=1).astype(np.float64)

    if args.dataparser_transform:
        r, t, scale = load_dataparser_transform(args.dataparser_transform)
        # Forward: X_train = scale * (R @ X_world + t)  =>  X_world = R^T @ (X_train / scale - t)
        xyz = (xyz / scale - t) @ r  # R^T applied via right-multiply by R for row vectors
        max_scale = max_scale / scale  # Gaussian extent scales the same way (no translation term)
        print(f'Applied inverse dataparser transform (scale={scale}) — '
              f'max_scale threshold corrected to {max_scale:.4f}m in training space', file=sys.stderr)
    else:
        print('WARNING: no --dataparser-transform given — xyz stays in nerfstudio\'s '
              'training-normalized space, NOT true world meters (see module docstring)', file=sys.stderr)
    xyz = xyz.astype(np.float32)

    opacity_logit = np.array(v['opacity'], dtype=np.float32)
    opacity = 1.0 / (1.0 + np.exp(-opacity_logit))

    dc = np.stack([v['f_dc_0'], v['f_dc_1'], v['f_dc_2']], axis=1).astype(np.float32)
    rgb = np.clip(0.5 + SH_C0 * dc, 0.0, 1.0)
    rgb_u8 = (rgb * 255).astype(np.uint8)

    # scale_0/1/2 are stored log-scale (splatfacto convention); take the
    # largest axis per-Gaussian as the floater signal.
    scale = np.exp(np.stack([v['scale_0'], v['scale_1'], v['scale_2']], axis=1)).max(axis=1)

    keep = (opacity > opacity_thresh) & (scale < max_scale)
    xyz, rgb_u8, opacity = xyz[keep], rgb_u8[keep], opacity[keep]

    print(f'{keep.sum()}/{len(keep)} Gaussians kept '
          f'(opacity > {opacity_thresh}, scale < {max_scale}m)', file=sys.stderr)
    np.savez(out_path, xyz=xyz, rgb=rgb_u8, opacity=opacity)
    print(f'Wrote {out_path}', file=sys.stderr)


if __name__ == '__main__':
    main()
