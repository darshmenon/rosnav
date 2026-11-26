#!/usr/bin/env python3
"""ply_to_layered_costmap.py — terrain .ply -> graded, layered Nav2 cost masks.

Idea borrowed from naturerobots/mesh_navigation: instead of one binary "blocked" test, keep
several terrain-cost LAYERS (steepness, roughness, step, ridge, border,
clearance, uncertainty) and combine them into one
graded traversability cost. Here the layers live on a 2D height grid so plain Nav2 can use them
(no Move Base Flex): the result is a SpeedFilter mask (graded slow-down) + KeepoutFilter mask
(only truly lethal cells). Not a ROS node.

Twist beyond MeshNav: every output cell also records WHICH layer dominated its cost
(<out>_why.png, colour-coded) so a surprising detour can be explained at a glance.

  python3 scripts/ply_to_layered_costmap.py --ply maps/terrain_outdoor.ply --out maps/terrain_outdoor_layered

Writes <out>_speed.pgm/.yaml (mode scale, white=full speed, darker=slower), <out>_keepout.pgm/.yaml
(trinary), <out>_<layer>.png per layer, <out>_cost.png and <out>_why.png.
"""
import argparse
import math

import numpy as np

LAYERS = ('steepness', 'roughness', 'step', 'ridge', 'border', 'clearance', 'uncertainty')
# default weights: cost of a layer at its own "saturation" value, as a fraction of 1.0
DEFAULT_WEIGHTS = {'steepness': 1.0, 'roughness': 0.7, 'step': 1.0, 'ridge': 0.5, 'border': 0.4, 'clearance': 1.0, 'uncertainty': 0.5}


# Starting-point limits per drive type (guesses to tune, not measured). Ackermann can't turn on the
# spot so it gets stricter step/ridge limits; mecanum slips sideways on slopes so a lower slope limit.
DRIVE_LIMITS = {
    'diff': dict(max_slope_deg=25.0, max_step=0.25, max_rough=0.08, max_ridge=0.15),
    'mecanum': dict(max_slope_deg=18.0, max_step=0.20, max_rough=0.06, max_ridge=0.12),
    'ackermann': dict(max_slope_deg=22.0, max_step=0.20, max_rough=0.07, max_ridge=0.10),
}


def resolve_limits(drive_type, **overrides):
    """Preset limits for a drive type; any non-None override wins."""
    out = dict(DRIVE_LIMITS[drive_type])
    out.update({k: v for k, v in overrides.items() if v is not None})
    return out


def speed_pixels(cost, seen, min_speed=0.8):
    """Cost 0..1 -> PGM pixels for map_server 'scale' mode (black=100, white=0).

    Decoded mask value v gives speed (100 - v)%; v = 0 is full speed. v is capped at
    (1 - min_speed) * 100 because Nav2 treats a 0% limit as 'no limit'."""
    value = np.where(seen, np.clip(cost, 0.0, 1.0) * (1.0 - min_speed) * 100.0, 0.0)
    return np.round(255 * (1 - value / 100.0)).astype(np.uint8)


def read_ply(path):
    """All float32 vertex properties -> (names, Nx k array). Binary little-endian only."""
    names, n, in_vertex = [], 0, False
    with open(path, 'rb') as f:
        while True:
            line = f.readline()
            if line.startswith(b'element'):
                in_vertex = line.startswith(b'element vertex')
                if in_vertex:
                    n = int(line.split()[2])
            elif line.startswith(b'property float') and in_vertex:
                names.append(line.split()[2].decode())
            elif line.startswith(b'end_header'):
                break
        data = np.fromfile(f, dtype='<f4', count=n * len(names)).reshape(n, len(names))
    return names, data


def height_grid(p, cell, min_points=1, robot_height=None):
    """Bin Nx3 points -> (ground, zmax, seen, x0, y0). ground = per-cell min z (NaN if unseen).

    With robot_height, zmax only counts points the robot could hit (below ground + robot_height),
    so a canopy or bridge deck overhead is not mistaken for a wall."""
    p = p[np.isfinite(p).all(1)]
    x0, y0 = p[:, 0].min(), p[:, 1].min()
    w = int(math.ceil((p[:, 0].max() - x0) / cell)) + 1
    h = int(math.ceil((p[:, 1].max() - y0) / cell)) + 1
    ix = ((p[:, 0] - x0) / cell).astype(int)
    iy = ((p[:, 1] - y0) / cell).astype(int)
    zmin = np.full((h, w), np.inf)
    zmax = np.full((h, w), -np.inf)
    cnt = np.zeros((h, w), int)
    np.minimum.at(zmin, (iy, ix), p[:, 2])
    hit = np.ones(len(p), bool) if robot_height is None else p[:, 2] - zmin[iy, ix] < robot_height
    np.maximum.at(zmax, (iy[hit], ix[hit]), p[hit, 2])
    np.add.at(cnt, (iy, ix), 1)
    seen = cnt >= min_points
    return np.where(seen, zmin, np.nan), np.where(seen, zmax, np.nan), seen, x0, y0


def overhead_gap(p, ground, cell, x0, y0, max_step=0.25):
    """Free height above ground per cell: lowest point higher than max_step over it (inf if none)."""
    p = p[np.isfinite(p).all(1)]
    ix = ((p[:, 0] - x0) / cell).astype(int)
    iy = ((p[:, 1] - y0) / cell).astype(int)
    h = p[:, 2] - ground[iy, ix]
    above = h > max_step                     # NaN (unseen cell) compares False
    gap = np.full(ground.shape, np.inf)
    np.minimum.at(gap, (iy[above], ix[above]), h[above])
    return gap


def point_uncertainty(p, seen, cell, x0, y0, confident_points=None):
    """0..1 per cell from point support: 0 at `confident_points` or more (default: median of seen cells).

    A count-based stand-in for elevation uncertainty (idea from ProTrav, arXiv 2610.30839); it is not
    a probabilistic model. A uniform mesh (same count everywhere) scores 0."""
    p = p[np.isfinite(p).all(1)]
    count = np.zeros(seen.shape)
    np.add.at(count, (((p[:, 1] - y0) / cell).astype(int), ((p[:, 0] - x0) / cell).astype(int)), 1)
    if not seen.any():
        return np.zeros(seen.shape)
    need = confident_points or np.median(count[seen])
    return np.where(seen, np.clip(1.0 - count / need, 0.0, 1.0), 0.0)


def _filled(ground, seen):
    """Ground with unseen cells filled from the nearest seen cell (so gradients stay finite)."""
    from scipy import ndimage
    idx = ndimage.distance_transform_edt(~seen, return_distances=False, return_indices=True)
    return ground[tuple(idx)]


def layer_costs(ground, zmax, seen, cell, max_slope_deg=25.0, max_step=0.25,
                max_rough=0.08, max_ridge=0.15, border_m=0.75,
                gap=None, robot_height=0.5, headroom=0.3, uncertainty=None):
    """Per-layer cost in [0, 1] (1 = at/over that layer's limit). Returns {name: HxW array}."""
    from scipy import ndimage
    g = _filled(ground, seen)
    gy, gx = np.gradient(g, cell)
    slope = np.degrees(np.arctan(np.hypot(gx, gy)))
    steep = slope / max_slope_deg

    # roughness: std of the ground residual to its 3x3 mean (bumpiness independent of slope)
    mean = ndimage.uniform_filter(g, 3)
    rough = np.sqrt(np.maximum(ndimage.uniform_filter(g * g, 3) - mean * mean, 0.0)) / max_rough

    # step: tallest in-cell spread (walls/rocks) or biggest drop to a 4-neighbour
    spread = np.nan_to_num(zmax - ground, nan=0.0)
    nb = np.zeros_like(g)
    for ax in (0, 1):
        for sh in (1, -1):
            nb = np.maximum(nb, np.abs(g - np.roll(g, sh, ax)))
    step = np.maximum(spread, nb) / max_step

    # ridge: |discrete Laplacian| — crests and ditches the wheels can high-centre on
    lap = np.abs(ndimage.laplace(g)) / (cell * cell) * cell  # ~ curvature * cell, metres
    ridge = lap / max_ridge

    # border: nearer than border_m to unmapped space is distrusted
    dist = ndimage.distance_transform_edt(seen) * cell if not seen.all() else np.full(seen.shape, np.inf)
    border = np.clip(1.0 - dist / border_m, 0.0, 1.0)

    # clearance: lethal if the free height is under the robot, fading out over `headroom` above it
    clear = np.zeros_like(g) if gap is None else (robot_height + headroom - gap) / headroom

    out = {'steepness': steep, 'roughness': rough, 'step': step, 'ridge': ridge, 'border': border,
           'clearance': clear,
           'uncertainty': np.zeros_like(g) if uncertainty is None else uncertainty}
    return {k: np.where(seen | (k == 'border'), np.clip(v, 0.0, 1.0), 0.0) for k, v in out.items()}


def combine(layers, weights=None, inflate_m=0.0, cell=0.25):
    """Weighted-max combine -> (cost 0..1, dominant layer index). Optional inflation of the cost."""
    from scipy import ndimage
    w = dict(DEFAULT_WEIGHTS, **(weights or {}))
    stack = np.stack([np.clip(layers[k] * w[k], 0.0, 1.0) for k in LAYERS])
    cost = stack.max(0)
    why = stack.argmax(0)
    if inflate_m > 0 and (cost >= 0.99).any():   # edt of an all-True grid returns garbage distances
        # spread cost to neighbours with linear decay so hazards get a safety margin
        d = ndimage.distance_transform_edt(cost < 0.99, return_indices=False) * cell
        halo = np.clip(1.0 - d / inflate_m, 0.0, 1.0) * 0.99
        cost = np.maximum(cost, halo)
    return cost, why


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--ply', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--drive-type', choices=sorted(DRIVE_LIMITS), default='diff',
                    help='preset slope/step/roughness/ridge limits; explicit --max-* flags override')
    ap.add_argument('--cell', type=float, default=0.25, help='grid cell = mask resolution (m)')
    ap.add_argument('--max-slope-deg', type=float, default=None)
    ap.add_argument('--max-step', type=float, default=None)
    ap.add_argument('--max-rough', type=float, default=None)
    ap.add_argument('--max-ridge', type=float, default=None)
    ap.add_argument('--min-points', type=int, default=1, help='points/cell to count as seen (1 for meshes, ~3 for noisy clouds)')
    ap.add_argument('--min-speed', type=float, default=0.8,
                    help='speed fraction at cost 1.0. DWB scales max but not min speeds, so keep it at or above '
                         'both min_speed_xy/max_speed_xy and min_speed_theta/max_vel_theta (0.75 here)')
    ap.add_argument('--robot-height', type=float, default=0.5, help='clearance needed above ground (m)')
    ap.add_argument('--confident-points', type=float, default=None,
                    help='points/cell for zero uncertainty (default: median of seen cells)')
    ap.add_argument('--inflate', type=float, default=0.5, help='hazard safety margin (m)')
    ap.add_argument('--lethal', type=float, default=0.9, help='cost >= this goes in the keepout mask')
    ap.add_argument('--weight', action='append', default=[], metavar='LAYER=W',
                    help='override a layer weight, e.g. --weight roughness=0.3')
    a = ap.parse_args()

    names, data = read_ply(a.ply)
    ground, zmax, seen, x0, y0 = height_grid(data[:, :3], a.cell, a.min_points, a.robot_height)
    lim = resolve_limits(a.drive_type, max_slope_deg=a.max_slope_deg, max_step=a.max_step,
                         max_rough=a.max_rough, max_ridge=a.max_ridge)
    print(f'drive_type={a.drive_type} limits={lim}')
    gap = overhead_gap(data[:, :3], ground, a.cell, x0, y0, lim['max_step'])
    unc = point_uncertainty(data[:, :3], seen, a.cell, x0, y0, a.confident_points)
    layers = layer_costs(ground, zmax, seen, a.cell, gap=gap, robot_height=a.robot_height, uncertainty=unc, **lim)
    weights = {k: float(v) for k, v in (s.split('=') for s in a.weight)}
    cost, why = combine(layers, weights, a.inflate, a.cell)

    import cv2
    flip = lambda m: m[::-1]  # image row 0 = max y
    base = a.out.split('/')[-1]
    speed = speed_pixels(cost, seen, a.min_speed)
    cv2.imwrite(a.out + '_speed.pgm', flip(speed))
    keep = np.full(cost.shape, 254, np.uint8)
    keep[~seen] = 205
    keep[cost >= a.lethal] = 0
    cv2.imwrite(a.out + '_keepout.pgm', flip(keep))
    for suffix, mode, extra in (('speed', 'scale', 'occupied_thresh: 1.0\nfree_thresh: 0.0\n'),
                                ('keepout', 'trinary', 'occupied_thresh: 0.65\nfree_thresh: 0.25\n')):
        with open(f'{a.out}_{suffix}.yaml', 'w') as f:
            f.write(f'image: {base}_{suffix}.pgm\nmode: {mode}\nresolution: {a.cell}\n'
                    f'origin: [{x0:.3f}, {y0:.3f}, 0]\nnegate: 0\n{extra}')

    up = lambda m: cv2.resize(flip(m), None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST)
    for k, v in layers.items():
        cv2.imwrite(f'{a.out}_{k}.png', up(cv2.applyColorMap((v * 255).astype(np.uint8), cv2.COLORMAP_INFERNO)))
    cv2.imwrite(a.out + '_cost.png', up(cv2.applyColorMap((cost * 255).astype(np.uint8), cv2.COLORMAP_INFERNO)))
    palette = np.array([[60, 60, 255], [60, 200, 255], [255, 120, 60], [60, 220, 60], [200, 60, 200], [255, 220, 40], [20, 20, 20]], np.uint8)
    wimg = palette[why]
    wimg[cost < 0.2] = 255  # negligible cost -> white
    wimg[~seen] = 128
    cv2.imwrite(a.out + '_why.png', up(wimg))

    n = seen.sum()
    share = {k: 100 * ((why == i) & (cost >= 0.3) & seen).sum() / max(n, 1) for i, k in enumerate(LAYERS)}
    print(f'cells={n} mean_cost={cost[seen].mean():.3f} lethal={100 * (cost >= a.lethal)[seen].mean():.1f}% '
          f'| dominant layer share (cost>=0.3): ' + ' '.join(f'{k}={v:.1f}%' for k, v in share.items()))
    print(f'-> {a.out}_speed.yaml / _keepout.yaml (+ per-layer PNGs, _cost.png, _why.png)')


if __name__ == '__main__':
    main()
