#!/usr/bin/env python3
"""ply_to_traversability.py — saved 3D cloud (.ply) -> elevation + traversability Nav2 map.

Same rule as terrain_traversability.py (blocked if neighbour slope > max-slope-deg or
in-cell height spread > step), applied offline to a whole cloud. Not a ROS node.

  python3 src/rosnav_bot/scripts/ply_to_traversability.py \\
      --ply src/rosnav_bot/maps/map_outdoor_rocks_3d.ply \\
      --out src/rosnav_bot/maps/map_outdoor_rocks_trav
Writes <out>.pgm/.yaml (occupancy), <out>_elevation.png (height, for viewing).
"""
import argparse
import math

import numpy as np


def read_ply(path):
    with open(path, 'rb') as f:
        n, props = 0, []
        while True:
            l = f.readline()
            if l.startswith(b'element vertex'):
                n = int(l.split()[2])
            elif l.startswith(b'property float') and len(props) < 64 and n and b'element' not in l:
                props.append(l.split()[2].decode())
            elif l.startswith(b'element') and n and not l.startswith(b'element vertex'):
                pass
            if l.startswith(b'end_header'):
                break
        props = props[:next((i for i, p in enumerate(props) if p == 'view_px'), len(props))]
        return np.fromfile(f, dtype='<f4', count=n * len(props)).reshape(n, len(props))[:, :3]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--ply', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--res', type=float, default=0.05)
    ap.add_argument('--cell', type=float, default=0.25, help='height-grid cell (m)')
    ap.add_argument('--max-slope-deg', type=float, default=25.0)
    ap.add_argument('--min-points', type=int, default=5, help='ignore cells with fewer points')
    ap.add_argument('--step', type=float, default=0.25, help='max in-cell height spread (m)')
    a = ap.parse_args()

    p = read_ply(a.ply)
    p = p[np.isfinite(p).all(1)]
    x0, y0 = p[:, 0].min(), p[:, 1].min()
    w = int(math.ceil((p[:, 0].max() - x0) / a.cell)) + 1
    h = int(math.ceil((p[:, 1].max() - y0) / a.cell)) + 1
    ix = ((p[:, 0] - x0) / a.cell).astype(int)
    iy = ((p[:, 1] - y0) / a.cell).astype(int)
    zmin = np.full((h, w), np.inf)
    zmax = np.full((h, w), -np.inf)
    np.minimum.at(zmin, (iy, ix), p[:, 2])
    np.maximum.at(zmax, (iy, ix), p[:, 2])
    count = np.zeros((h, w), int)
    np.add.at(count, (iy, ix), 1)
    seen = count >= a.min_points
    zmin = np.where(seen, zmin, np.inf)
    zmax = np.where(seen, zmax, -np.inf)
    ground = np.where(seen, zmin, np.nan)

    blocked = seen & ((zmax - zmin) > a.step)
    tan = math.tan(math.radians(a.max_slope_deg))
    for dy, dx in ((0, 1), (1, 0), (1, 1), (1, -1)):
        g2 = np.roll(ground, (-dy, -dx), (0, 1))
        dz = np.abs(ground - g2) / (a.cell * math.hypot(dy, dx))
        steep = np.nan_to_num(dz, nan=0.0) > tan
        # mark only the higher cell of a steep pair (the uphill wall)
        higher = np.where(np.nan_to_num(ground, nan=-9) > np.nan_to_num(g2, nan=-9), True, False)
        blocked |= steep & higher
        blocked |= np.roll(steep & ~higher, (dy, dx), (0, 1))

    # rasterise cells -> map pixels (row 0 = max y)
    W, H = int(w * a.cell / a.res), int(h * a.cell / a.res)
    r = int(round(a.cell / a.res))
    occ = np.repeat(np.repeat(blocked, r, 0), r, 1)[:H, :W]
    unk = np.repeat(np.repeat(~seen, r, 0), r, 1)[:H, :W]
    img = np.full(occ.shape, 254, np.uint8)
    img[unk] = 205
    img[occ] = 0
    img = img[::-1]

    import cv2
    cv2.imwrite(a.out + '.pgm', img)
    base = a.out.split('/')[-1]
    with open(a.out + '.yaml', 'w') as f:
        f.write(f'image: {base}.pgm\nmode: trinary\nresolution: {a.res}\n'
                f'origin: [{x0:.3f}, {y0:.3f}, 0]\nnegate: 0\n'
                'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
    e = np.where(seen, ground, np.nanmin(ground))
    e = ((e - e.min()) / max(np.ptp(e), 1e-6) * 255).astype(np.uint8)[::-1]
    cv2.imwrite(a.out + '_elevation.png', cv2.resize(e, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
    print(f'cells={seen.sum()} blocked={blocked.sum()} ({100 * blocked.sum() / seen.sum():.1f}% of seen) '
          f'z={p[:, 2].min():.2f}..{p[:, 2].max():.2f} -> {a.out}.pgm')


if __name__ == '__main__':
    main()
