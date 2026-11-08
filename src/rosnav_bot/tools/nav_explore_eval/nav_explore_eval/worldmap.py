"""Exact ground-truth occupancy map from a Gazebo/SDF world: rasterise box and cylinder collisions."""
import math
import os
import re
import sys
import xml.etree.ElementTree as ET
from collections import deque

import numpy as np

from .mapio import Grid


def _pose(el):
    """(x, y, yaw) from a <pose> child; roll/pitch are ignored (walls are upright)."""
    p = el.find('pose') if el is not None else None
    v = [float(t) for t in (p.text or '').split()] if p is not None and p.text else []
    v += [0.0] * (6 - len(v))
    return v[0], v[1], v[5], v[2]


def _compose(a, b):
    (ax, ay, ayaw), (bx, by, byaw) = a, b
    c, s = math.cos(ayaw), math.sin(ayaw)
    return ax + c * bx - s * by, ay + s * bx + c * by, ayaw + byaw


def _scale(geo_el):
    t = geo_el.findtext('scale')
    v = [float(x) for x in t.split()] if t else [1.0, 1.0, 1.0]
    return (v + [1.0] * 3)[:3]


def _model_dirs(model_paths):
    dirs = [d for d in model_paths if d]
    for var in ('GAZEBO_MODEL_PATH', 'GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH', 'SDF_PATH'):
        dirs += [d for d in os.environ.get(var, '').split(os.pathsep) if d]
    dirs += [os.path.expanduser('~/.gazebo/models'), os.path.expanduser('~/.gz/models')]
    return dirs


def _resolve_uri(uri, model_paths, base_dir):
    """model://name[/rest], file:///abs, or a relative path -> existing local path, else None."""
    uri = uri.strip()
    if uri.startswith('model://'):
        rel = uri[len('model://'):]
        for d in _model_dirs(model_paths) + ([base_dir] if base_dir else []):
            cand = os.path.join(d, rel)
            if os.path.exists(cand):
                return cand
        return None
    if uri.startswith('file://'):
        uri = uri[len('file://'):]
    for cand in ([uri] if os.path.isabs(uri) else [os.path.join(base_dir or '.', uri)]):
        if os.path.exists(cand):
            return cand
    return None


def mesh_bounds(path):
    """Axis-aligned bounds ((x0,y0,z0),(x1,y1,z1)) of an .obj, .stl or .dae mesh, in metres, Z up."""
    ext = os.path.splitext(path)[1].lower()
    if ext == '.obj':
        pts = []
        for line in open(path, errors='ignore'):
            if line.startswith('v '):
                pts.append([float(t) for t in line.split()[1:4]])
        v = np.array(pts)
    elif ext == '.stl':
        raw = open(path, 'rb').read()
        n = int.from_bytes(raw[80:84], 'little') if len(raw) >= 84 else -1
        if len(raw) == 84 + 50 * n:                                    # binary
            rec = np.frombuffer(raw[84:], dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
            v = rec['v'].reshape(-1, 3).astype(float)
        else:                                                         # ascii
            v = np.array([[float(t) for t in ln.split()[1:4]] for ln in raw.decode(errors='ignore').splitlines()
                          if ln.strip().startswith('vertex')])
    elif ext == '.dae':
        root = ET.fromstring(re.sub(rb'\sxmlns="[^"]+"', b'', open(path, 'rb').read(), count=1))
        arrs = [a for a in root.iter('float_array') if 'position' in (a.get('id') or '').lower()]
        if not arrs:
            raise ValueError(f'{path}: no vertex positions found')
        v = np.vstack([np.array(a.text.split(), dtype=float).reshape(-1, 3) for a in arrs])
        unit = root.find('asset/unit')
        if unit is not None and unit.get('meter'):
            v = v * float(unit.get('meter'))
        if (root.findtext('asset/up_axis') or 'Z_UP').strip() == 'Y_UP':
            v = np.stack([v[:, 0], -v[:, 2], v[:, 1]], axis=1)
    else:
        raise ValueError(f'{path}: unsupported mesh format {ext}')
    if v.size == 0:
        raise ValueError(f'{path}: empty mesh')
    return v.min(axis=0), v.max(axis=0)


def shapes(world_text, z=0.2, model_paths=(), base_dir=None, warn=None):
    """[(kind, cx, cy, yaw, dims)] for collisions that intersect the horizontal plane at height `z`.
    Handles nested models, <include> (model:// from `model_paths`, $GAZEBO_MODEL_PATH, ...), box, cylinder
    and mesh (.obj/.stl/.dae, as its bounding box) collisions. `warn(msg)` reports what was skipped."""
    warn = warn or (lambda m: print(f'warning: {m}', file=sys.stderr))
    root = ET.fromstring(re.sub(r'<\?xml[^>]*\?>', '', world_text).strip())
    out = []

    def collision(col, frame, zc0, model_dir):
        cx, cy, cyaw, cz = _pose(col)
        geo = col.find('geometry')
        if geo is None:
            return
        x, y, yaw = _compose(frame, (cx, cy, cyaw))
        zc = zc0 + cz
        box, cyl, mesh = geo.find('box/size'), geo.find('cylinder'), geo.find('mesh')
        if box is not None:
            sx, sy, sz = (float(t) for t in box.text.split())
            if abs(z - zc) <= sz / 2:
                out.append(('box', x, y, yaw, (sx, sy)))
        elif cyl is not None:
            r, h = float(cyl.findtext('radius')), float(cyl.findtext('length'))
            if abs(z - zc) <= h / 2:
                out.append(('cyl', x, y, yaw, (r,)))
        elif mesh is not None:
            uri = mesh.findtext('uri')
            path = _resolve_uri(uri, model_paths, model_dir or base_dir) if uri else None
            if path is None:
                warn(f'mesh not found, skipped: {uri}')
                return
            try:
                lo, hi = mesh_bounds(path)
            except (ValueError, OSError, ET.ParseError) as e:
                warn(f'mesh skipped: {e}')
                return
            sc = np.array(_scale(mesh))
            lo, hi = lo * sc, hi * sc
            if not (zc + lo[2] <= z <= zc + hi[2]):
                return
            mx, my, _ = _compose((x, y, yaw), ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, 0.0))
            out.append(('box', mx, my, yaw, (float(hi[0] - lo[0]), float(hi[1] - lo[1]))))
        elif geo.find('heightmap') is not None:
            warn('heightmap collisions are not supported, skipped')

    def model(el, parent, zp, model_dir, depth=0):
        if depth > 8:
            return
        mx, my, myaw, mz = _pose(el)
        frame, zb = _compose(parent, (mx, my, myaw)), zp + mz
        for link in el.findall('link'):
            lx, ly, lyaw, lz = _pose(link)
            lf = _compose(frame, (lx, ly, lyaw))
            for col in link.findall('collision'):
                collision(col, lf, zb + lz, model_dir)
        for sub in el.findall('model'):
            model(sub, frame, zb, model_dir, depth + 1)
        for inc in el.findall('include'):
            include(inc, frame, zb, depth + 1)

    def include(inc, parent, zp, depth):
        uri = inc.findtext('uri')
        path = _resolve_uri(uri, model_paths, base_dir) if uri else None
        if path is None:
            warn(f'include not found, skipped: {uri}')
            return
        mdir = path if os.path.isdir(path) else os.path.dirname(path)
        sdf = path
        if os.path.isdir(path):
            sdf = os.path.join(path, 'model.sdf')
            if not os.path.exists(sdf):
                found = sorted(f for f in os.listdir(path) if f.endswith('.sdf'))
                sdf = os.path.join(path, found[0]) if found else None
        if not sdf:
            warn(f'no .sdf in {path}, skipped')
            return
        sub = ET.fromstring(re.sub(r'<\?xml[^>]*\?>', '', open(sdf).read()).strip())
        el = sub if sub.tag == 'model' else sub.find('.//model')
        if el is None:
            return
        ix, iy, iyaw, iz = _pose(inc)
        f = _compose(parent, (ix, iy, iyaw))
        model(el, f, zp + iz, mdir, depth)

    world = root.find('world') if root.tag != 'world' else root
    holder = world if world is not None else root
    for m in holder.findall('model'):
        model(m, (0.0, 0.0, 0.0), 0.0, base_dir)
    for inc in holder.findall('include'):
        include(inc, (0.0, 0.0, 0.0), 0.0, 0)
    return out


def rasterize(shape_list, res=0.05, margin=1.0, seed=(0.0, 0.0)):
    """Grid: 100 walls, 0 free space reachable from `seed`, -1 everything else (outside the walls)."""
    if not shape_list:
        raise ValueError('no collision shapes at that height - check --z or the world file')
    xs_, ys_ = [], []
    for kind, x, y, yaw, d in shape_list:
        if kind == 'box':
            hx = abs(math.cos(yaw)) * d[0] / 2 + abs(math.sin(yaw)) * d[1] / 2
            hy = abs(math.sin(yaw)) * d[0] / 2 + abs(math.cos(yaw)) * d[1] / 2
        else:
            hx = hy = d[0]
        xs_ += [x - hx, x + hx]; ys_ += [y - hy, y + hy]
    x0, x1, y0, y1 = min(xs_) - margin, max(xs_) + margin, min(ys_) - margin, max(ys_) + margin
    w, h = int(math.ceil((x1 - x0) / res)), int(math.ceil((y1 - y0) / res))
    xs = x0 + (np.arange(w) + 0.5) * res
    ys = y0 + (np.arange(h) + 0.5) * res
    X, Y = np.meshgrid(xs, ys)
    occ = np.zeros((h, w), dtype=bool)
    for kind, cx, cy, yaw, d in shape_list:
        dx, dy = X - cx, Y - cy
        c, s = math.cos(-yaw), math.sin(-yaw)
        u, v = c * dx - s * dy, s * dx + c * dy
        occ |= (np.abs(u) <= d[0] / 2) & (np.abs(v) <= d[1] / 2) if kind == 'box' else (dx * dx + dy * dy <= d[0] ** 2)
    grid = np.full((h, w), -1, dtype=np.int8)
    si, sj = int((seed[1] - y0) / res), int((seed[0] - x0) / res)
    if not (0 <= si < h and 0 <= sj < w) or occ[si, sj]:
        raise ValueError(f'seed {seed} is outside the world or inside a wall - pass --seed X Y in free space')
    seen = np.zeros_like(occ)
    q = deque([(si, sj)]); seen[si, sj] = True
    while q:
        i, j = q.popleft()
        for ni, nj in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if 0 <= ni < h and 0 <= nj < w and not seen[ni, nj] and not occ[ni, nj]:
                seen[ni, nj] = True
                q.append((ni, nj))
    grid[seen] = 0
    walls = occ & _touches(seen)                     # keep only walls that bound the explorable space
    grid[walls] = 100
    return Grid(grid, res, x0, y0)


def _touches(m):
    out = m.copy()
    for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)):
        out |= np.roll(np.roll(m, di, 0), dj, 1)
    return out


def write_map(grid, prefix):
    """prefix.pgm + prefix.yaml in map_server format (free 254, occupied 0, unknown 205)."""
    img = np.where(grid.data == 100, 0, np.where(grid.data == 0, 254, 205)).astype(np.uint8)[::-1]
    with open(prefix + '.pgm', 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (img.shape[1], img.shape[0]) + img.tobytes())
    name = prefix.rsplit('/', 1)[-1]
    with open(prefix + '.yaml', 'w') as f:
        f.write(f'image: {name}.pgm\nresolution: {grid.resolution}\norigin: [{grid.ox:.3f}, {grid.oy:.3f}, 0.0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
