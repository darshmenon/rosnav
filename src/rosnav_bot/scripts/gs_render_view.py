#!/usr/bin/env python3
"""
gs_render_view.py - render a trained Gaussian splat (`ns-export gaussian-splat` .ply) to PNG with the real
gsplat rasterizer (anisotropic Gaussians, opacity blending) - not the point-cloud approximation RViz shows.
Needs torch + CUDA + gsplat + plyfile; not a ROS node.

Usage:
  python3 gs_render_view.py splat.ply --out render.png                    # orbit camera around the scene centre
  python3 gs_render_view.py splat.ply --out r.png --yaw 90 --pitch 25 --dist 6
  python3 gs_render_view.py splat.ply --out strip.png --orbit 4           # N views side by side
  python3 gs_render_view.py splat.ply --out f0.png --transforms cafe/transforms.json \\
      --dataparser outputs/.../dataparser_transforms.json --frame 0        # exact training camera pose
Coordinates are whatever the .ply holds (nerfstudio normalised space unless it was exported with a transform).
"""
import argparse
import math

import numpy as np


def load_splat(path, max_scale=None):
    from plyfile import PlyData
    v = PlyData.read(path)['vertex']
    xyz = np.stack([v['x'], v['y'], v['z']], 1).astype(np.float32)
    dc = np.stack([v['f_dc_0'], v['f_dc_1'], v['f_dc_2']], 1).astype(np.float32)
    rgb = np.clip(dc * 0.28209479177387814 + 0.5, 0.0, 1.0)            # SH degree-0 -> colour
    opacity = 1.0 / (1.0 + np.exp(-np.asarray(v['opacity'], np.float32)))
    scales = np.exp(np.stack([v['scale_0'], v['scale_1'], v['scale_2']], 1)).astype(np.float32)
    quats = np.stack([v['rot_0'], v['rot_1'], v['rot_2'], v['rot_3']], 1).astype(np.float32)
    quats /= np.linalg.norm(quats, axis=1, keepdims=True) + 1e-9
    keep = np.ones(len(xyz), bool) if max_scale is None else scales.max(1) < max_scale
    return xyz[keep], rgb[keep], opacity[keep], scales[keep], quats[keep]


def look_at(eye, target, up=(0, 0, 1)):
    f = np.asarray(target, np.float64) - eye
    f /= np.linalg.norm(f)
    r = np.cross(f, up); r /= np.linalg.norm(r)
    d = np.cross(f, r)                                                  # camera y points down in the image
    c2w = np.eye(4); c2w[:3, 0], c2w[:3, 1], c2w[:3, 2], c2w[:3, 3] = r, d, f, eye
    return np.linalg.inv(c2w).astype(np.float32)


def render(splat, view, width, height, fov_deg, device='cuda'):
    import torch
    from gsplat import rasterization
    xyz, rgb, op, sc, q = (torch.from_numpy(a).to(device) for a in splat)
    f = 0.5 * width / math.tan(math.radians(fov_deg) / 2)
    K = torch.tensor([[f, 0, width / 2], [0, f, height / 2], [0, 0, 1]], dtype=torch.float32, device=device)
    img, alpha, _ = rasterization(xyz, q, sc, op, rgb, torch.from_numpy(view)[None].to(device), K[None], width, height)
    return (img[0].clamp(0, 1).cpu().numpy() * 255).astype(np.uint8), float(alpha.mean())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('ply'); ap.add_argument('--out', required=True)
    ap.add_argument('--width', type=int, default=960); ap.add_argument('--height', type=int, default=600)
    ap.add_argument('--fov', type=float, default=70.0)
    ap.add_argument('--yaw', type=float, default=35.0); ap.add_argument('--pitch', type=float, default=20.0)
    ap.add_argument('--dist', type=float, default=0.0, help='camera distance from the centre (default: scene size)')
    ap.add_argument('--transforms', help='nerfstudio transforms.json: render from one of its training cameras')
    ap.add_argument('--dataparser', help='dataparser_transforms.json of the training run (maps poses into splat space)')
    ap.add_argument('--frame', type=int, default=0)
    ap.add_argument('--walkthrough', type=int, default=0, metavar='STEP',
                    help='with --transforms: render every STEP-th training camera and write an animated GIF to --out')
    ap.add_argument('--side-by-side', action='store_true', help='walkthrough: put the training image beside the render')
    ap.add_argument('--orbit', type=int, default=0, help='render N yaw steps side by side')
    ap.add_argument('--max-scale', type=float, default=None, help='drop Gaussians larger than this (floaters)')
    a = ap.parse_args()

    from PIL import Image
    splat = load_splat(a.ply, a.max_scale)
    if a.transforms:
        import json
        import os
        tj = json.load(open(a.transforms))
        dp = json.load(open(a.dataparser)) if a.dataparser else None
        w, h = int(tj['w']), int(tj['h'])
        fov = math.degrees(2 * math.atan(0.5 * w / float(tj['fl_x'])))

        def view_of(i):
            c2w = np.array(tj['frames'][i]['transform_matrix'], np.float64)    # nerfstudio/OpenGL: camera looks down -z
            if dp:
                c2w = np.vstack([np.array(dp['transform'], np.float64), [0, 0, 0, 1]]) @ c2w
                c2w[:3, 3] *= float(dp['scale'])
            c2w = c2w @ np.diag([1.0, -1.0, -1.0, 1.0])                          # -> OpenCV: x right, y down, z forward
            return np.linalg.inv(c2w).astype(np.float32)
        if a.walkthrough:
            idx = list(range(0, len(tj['frames']), a.walkthrough))
            ims = []
            for i in idx:
                img, _ = render(splat, view_of(i), w, h, fov)
                if a.side_by_side:
                    gt = np.asarray(Image.open(os.path.join(os.path.dirname(a.transforms), tj['frames'][i]['file_path'])).convert('RGB'))
                    img = np.concatenate([gt, img], 1)
                ims.append(Image.fromarray(img).resize((img.shape[1] // 2, img.shape[0] // 2)))
            ims[0].save(a.out, save_all=True, append_images=ims[1:], duration=100, loop=0, optimize=True)
            print(f'{len(ims)} frames -> {a.out}')
            return
        img, cover = render(splat, view_of(a.frame), w, h, fov)
        Image.fromarray(img).save(a.out)
        print(f'training frame {a.frame}: coverage={cover:.2f} -> {a.out}')
        return
    xyz = splat[0]
    lo, hi = np.percentile(xyz, 2, axis=0), np.percentile(xyz, 98, axis=0)
    centre, dist = (lo + hi) / 2, a.dist or float(np.linalg.norm(hi - lo)) * 0.6
    frames = []
    for i in range(max(1, a.orbit)):
        yaw = math.radians(a.yaw + (360.0 / a.orbit) * i if a.orbit else a.yaw)
        pitch = math.radians(a.pitch)
        eye = centre + dist * np.array([math.cos(yaw) * math.cos(pitch), math.sin(yaw) * math.cos(pitch), math.sin(pitch)])
        img, cover = render(splat, look_at(eye, centre), a.width, a.height, a.fov)
        print(f'view {i}: yaw={math.degrees(yaw):.0f} coverage={cover:.2f}')
        frames.append(img)
    Image.fromarray(np.concatenate(frames, 1) if len(frames) > 1 else frames[0]).save(a.out)
    print(f'{len(xyz)} Gaussians -> {a.out}')


if __name__ == '__main__':
    main()
