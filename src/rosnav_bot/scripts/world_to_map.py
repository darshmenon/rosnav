#!/usr/bin/env python3
"""world_to_map.py - build a Nav2 occupancy map straight from a Gazebo .world file (no sim, no explorer).

Takes every static box / cylinder collision (world models plus <include>d models found under models/), keeps the ones that
cross the lidar plane (z band), rasterizes them in the robot START frame (a SLAM map's frame; AMCL starts at 0,0,0 there),
then flood-fills the free space outward from the spawn. Anything the fill cannot reach stays unknown, like a real scan.
Exact walls, no scan plateau, no window leakage, no dependence on how well an explorer drove.

  world_to_map.py worlds/cafe.world --spawn 0 -2 -1.57 --out maps/map_cafe_raycast
  -> map_cafe_raycast.pgm / .yaml / .png

Limits: boxes and cylinders only (mesh collisions are skipped with a warning), yaw-only rotations (roll/pitch ignored).
"""
import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET
from collections import deque

FREE, UNKNOWN, OCC = 254, 205, 0


def parse_pose(text):
    v = [float(x) for x in (text or '0 0 0 0 0 0').split()] + [0.0] * 6
    return tuple(v[:6])


def compose(a, b):
    """Pose b expressed in the frame of pose a (yaw only; roll/pitch dropped). Poses are (x, y, z, r, p, yaw)."""
    c, s = math.cos(a[5]), math.sin(a[5])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2], 0.0, 0.0, a[5] + b[5])


def find_model_sdf(uri, model_dirs):
    name = uri.replace('model://', '').strip('/')
    for root in model_dirs:
        for dp, _dn, fn in os.walk(root):
            if os.path.basename(dp) == name and 'model.sdf' in fn:
                return os.path.join(dp, 'model.sdf')
    return None


def _shapes_of_model(model, base, shapes, warns):
    mpose = compose(base, parse_pose((model.findtext('pose') or '').strip() or None))
    for link in model.findall('link'):
        lpose = compose(mpose, parse_pose(link.findtext('pose')))
        for col in link.findall('collision'):
            cpose = compose(lpose, parse_pose(col.findtext('pose')))
            geom = col.find('geometry')
            if geom is None:
                continue
            if geom.find('box') is not None:
                sx, sy, sz = (float(x) for x in geom.find('box').findtext('size').split())
                shapes.append(('box', cpose, (sx, sy, sz)))
            elif geom.find('cylinder') is not None:
                cy = geom.find('cylinder')
                shapes.append(('cyl', cpose, (float(cy.findtext('radius')), float(cy.findtext('length')))))
            elif geom.find('mesh') is not None:
                warns.append(f"mesh collision '{col.get('name')}' in model '{model.get('name')}' skipped")
    for sub in model.findall('model'):
        _shapes_of_model(sub, mpose, shapes, warns)


def load_shapes(world_path, model_dirs):
    """-> (shapes, warnings). shape = ('box', pose, (sx, sy, sz)) | ('cyl', pose, (r, length))."""
    tree = ET.parse(world_path)
    world = tree.getroot().find('world')
    shapes, warns = [], []
    origin = (0.0,) * 6
    for model in world.findall('model'):
        _shapes_of_model(model, origin, shapes, warns)
    for inc in world.findall('include'):
        uri = (inc.findtext('uri') or '').strip()
        path = find_model_sdf(uri, model_dirs)
        if not path:
            warns.append(f'include {uri} not found in {model_dirs}')
            continue
        ipose = parse_pose(inc.findtext('pose'))
        root = ET.parse(path).getroot()
        m = root.find('model')
        if m is not None:
            _shapes_of_model(m, ipose, shapes, warns)
    return shapes, warns


def to_start_frame(pose, spawn):
    """World pose -> robot-start frame (x, y, yaw)."""
    dx, dy = pose[0] - spawn[0], pose[1] - spawn[1]
    c, s = math.cos(-spawn[2]), math.sin(-spawn[2])
    return (c * dx - s * dy, s * dx + c * dy, pose[5] - spawn[2])


def in_band(shape, z_lo, z_hi):
    kind, pose, dims = shape
    h = dims[2] if kind == 'box' else dims[1]
    return pose[2] + h / 2 >= z_lo and pose[2] - h / 2 <= z_hi


def rasterize(shapes, spawn, res=0.05, z_lo=0.2, z_hi=0.4, margin=1.0):
    """-> (occ, w, h, ox, oy, n_used). occ is a bytearray (row 0 = lowest y), True where a lidar-plane obstacle is."""
    used = [s for s in shapes if in_band(s, z_lo, z_hi)]
    if not used:
        raise ValueError('no collision crosses the lidar plane z=[%.2f, %.2f]' % (z_lo, z_hi))
    local = []
    xs, ys = [], []
    for kind, pose, dims in used:
        x, y, yaw = to_start_frame(pose, spawn)
        r = math.hypot(dims[0] / 2, dims[1] / 2) if kind == 'box' else dims[0]
        xs += [x - r, x + r]
        ys += [y - r, y + r]
        local.append((kind, x, y, yaw, dims))
    xs += [0.0]
    ys += [0.0]
    ox, oy = math.floor((min(xs) - margin) / res) * res, math.floor((min(ys) - margin) / res) * res
    w, h = int(math.ceil((max(xs) + margin - ox) / res)), int(math.ceil((max(ys) + margin - oy) / res))
    occ = bytearray(w * h)
    for kind, x, y, yaw, dims in local:
        c, s = math.cos(yaw), math.sin(yaw)
        r = math.hypot(dims[0] / 2, dims[1] / 2) if kind == 'box' else dims[0]
        i0, i1 = max(0, int((x - r - ox) / res)), min(w - 1, int((x + r - ox) / res))
        j0, j1 = max(0, int((y - r - oy) / res)), min(h - 1, int((y + r - oy) / res))
        for j in range(j0, j1 + 1):
            py = oy + (j + 0.5) * res - y
            for i in range(i0, i1 + 1):
                px = ox + (i + 0.5) * res - x
                if kind == 'box':
                    u, v = c * px + s * py, -s * px + c * py
                    hit = abs(u) <= dims[0] / 2 + res / 2 * 0.5 and abs(v) <= dims[1] / 2 + res / 2 * 0.5
                else:
                    hit = px * px + py * py <= (dims[0] + res / 4) ** 2
                if hit:
                    occ[j * w + i] = 1
    return occ, w, h, ox, oy, len(used)


def flood_free(occ, w, h, start):
    """4-connected flood fill over non-occupied cells. -> (free bytearray, leaked: bool reached the grid border)."""
    free = bytearray(w * h)
    si, sj = start
    if not (0 <= si < w and 0 <= sj < h) or occ[sj * w + si]:
        raise ValueError('spawn cell is outside the grid or inside an obstacle')
    q = deque([(si, sj)])
    free[sj * w + si] = 1
    leaked = False
    while q:
        i, j = q.popleft()
        if i in (0, w - 1) or j in (0, h - 1):
            leaked = True
        for ni, nj in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if 0 <= ni < w and 0 <= nj < h and not free[nj * w + ni] and not occ[nj * w + ni]:
                free[nj * w + ni] = 1
                q.append((ni, nj))
    return free, leaked


def to_image(occ, free, w, h):
    """Image rows top->bottom (PGM convention): occupied 0, free 254, unknown 205. Unknown = not reachable and not a wall."""
    rows = bytearray()
    for j in range(h - 1, -1, -1):
        for i in range(w):
            k = j * w + i
            rows.append(OCC if occ[k] else FREE if free[k] else UNKNOWN)
    return bytes(rows)


def crop(occ, free, w, h, ox, oy, res, pad=15):
    """Crop to the bbox of the reachable free cells (+pad, enough to keep the walls and a margin of unknown)."""
    ii = [k % w for k in range(w * h) if free[k]]
    jj = [k // w for k in range(w * h) if free[k]]
    i0, i1, j0, j1 = max(0, min(ii) - pad), min(w - 1, max(ii) + pad), max(0, min(jj) - pad), min(h - 1, max(jj) + pad)
    nw, nh = i1 - i0 + 1, j1 - j0 + 1
    o2, f2 = bytearray(nw * nh), bytearray(nw * nh)
    for j in range(nh):
        for i in range(nw):
            o2[j * nw + i] = occ[(j + j0) * w + i + i0]
            f2[j * nw + i] = free[(j + j0) * w + i + i0]
    return o2, f2, nw, nh, ox + i0 * res, oy + j0 * res


def write_map(prefix, img, w, h, ox, oy, res):
    with open(prefix + '.pgm', 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))        # no comment line: simple PGM readers (tests, tools) choke on it
        f.write(img)
    with open(prefix + '.yaml', 'w') as f:
        f.write(f'image: {os.path.basename(prefix)}.pgm\nmode: trinary\nresolution: {res}\n'
                f'origin: [{ox:.3f}, {oy:.3f}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    try:
        from PIL import Image
        Image.frombytes('L', (w, h), img).resize((w * 4, h * 4), Image.NEAREST).save(prefix + '.png')
    except ImportError:
        pass


def main(argv=None):
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('world')
    ap.add_argument('--spawn', type=float, nargs=3, default=[0.0, 0.0, 0.0], metavar=('X', 'Y', 'YAW'),
                    help='robot spawn in the world frame; the map frame is the robot start frame (default 0 0 0)')
    ap.add_argument('--out', required=True, help='output prefix (writes .pgm .yaml .png)')
    ap.add_argument('--res', type=float, default=0.05)
    ap.add_argument('--z-band', type=float, nargs=2, default=[0.2, 0.4], metavar=('LO', 'HI'), help='lidar plane height range, m')
    ap.add_argument('--models', nargs='*', default=[os.path.join(here, '..', 'models')], help='model search roots for <include>')
    a = ap.parse_args(argv)
    shapes, warns = load_shapes(a.world, a.models)
    for wmsg in warns:
        print('warning:', wmsg, file=sys.stderr)
    occ, w, h, ox, oy, n = rasterize(shapes, tuple(a.spawn), a.res, a.z_band[0], a.z_band[1])
    free, leaked = flood_free(occ, w, h, (int((0 - ox) / a.res), int((0 - oy) / a.res)))
    occ, free, w, h, ox, oy = crop(occ, free, w, h, ox, oy, a.res)
    write_map(a.out, to_image(occ, free, w, h), w, h, ox, oy, a.res)
    nocc, nfree = sum(occ), sum(free)
    print(f'{a.out}.yaml: {w}x{h} cells ({w * a.res:.1f} x {h * a.res:.1f} m), {n}/{len(shapes)} shapes in the lidar band, '
          f'{nocc} occupied, {nfree} free, origin ({ox:.2f}, {oy:.2f})')
    if leaked:
        print('LEAK: free space reaches the grid border, so the world has an opening (or a mesh wall was skipped)', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
