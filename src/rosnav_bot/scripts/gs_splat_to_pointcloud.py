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
"""
import sys

import numpy as np
from plyfile import PlyData

SH_C0 = 0.28209479177387814


def main():
    if len(sys.argv) < 3:
        print(f'Usage: {sys.argv[0]} <splat.ply> <out.npz> [opacity_thresh=0.3]', file=sys.stderr)
        sys.exit(1)

    ply_path, out_path = sys.argv[1], sys.argv[2]
    opacity_thresh = float(sys.argv[3]) if len(sys.argv) > 3 else 0.3

    ply = PlyData.read(ply_path)
    v = ply['vertex']
    xyz = np.stack([v['x'], v['y'], v['z']], axis=1).astype(np.float32)

    opacity_logit = np.array(v['opacity'], dtype=np.float32)
    opacity = 1.0 / (1.0 + np.exp(-opacity_logit))

    dc = np.stack([v['f_dc_0'], v['f_dc_1'], v['f_dc_2']], axis=1).astype(np.float32)
    rgb = np.clip(0.5 + SH_C0 * dc, 0.0, 1.0)
    rgb_u8 = (rgb * 255).astype(np.uint8)

    keep = opacity > opacity_thresh
    xyz, rgb_u8, opacity = xyz[keep], rgb_u8[keep], opacity[keep]

    print(f'{keep.sum()}/{len(keep)} Gaussians kept (opacity > {opacity_thresh})', file=sys.stderr)
    np.savez(out_path, xyz=xyz, rgb=rgb_u8, opacity=opacity)
    print(f'Wrote {out_path}', file=sys.stderr)


if __name__ == '__main__':
    main()
