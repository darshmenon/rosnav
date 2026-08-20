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
"""
import sys

import numpy as np
from plyfile import PlyData

SH_C0 = 0.28209479177387814


def main():
    if len(sys.argv) < 3:
        print(f'Usage: {sys.argv[0]} <splat.ply> <out.npz> '
              f'[opacity_thresh=0.3] [max_scale=0.05]', file=sys.stderr)
        sys.exit(1)

    ply_path, out_path = sys.argv[1], sys.argv[2]
    opacity_thresh = float(sys.argv[3]) if len(sys.argv) > 3 else 0.3
    max_scale = float(sys.argv[4]) if len(sys.argv) > 4 else 0.05

    ply = PlyData.read(ply_path)
    v = ply['vertex']
    xyz = np.stack([v['x'], v['y'], v['z']], axis=1).astype(np.float32)

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
