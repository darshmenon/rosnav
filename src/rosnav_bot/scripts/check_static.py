#!/usr/bin/env python3
"""check_static - fast checks that catch a broken robot/sim before any simulator starts.

  src/rosnav_bot/scripts/check_static.py          # all checks
  src/rosnav_bot/scripts/check_static.py --only urdf yaml

Checks (each prints OK/FAIL/SKIP, exit code 1 on any FAIL):
  python  every script and launch file compiles
  yaml    every config/*.yaml and maps/*.yaml parses
  urdf    every urdf/*.urdf.xacro renders with xacro, is well-formed, and every joint
          references links that exist, with exactly one root link
  worlds  every worlds/*.world and *.sdf is well-formed XML
  launch  every launch file imports and builds a LaunchDescription (skipped without launch_ros)
  closure every box-built world is enclosed: the spawn point cannot reach the outside (mesh/include worlds skip)
"""
import argparse
import importlib.util
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
results = []


def record(group, name, status, detail=''):
    results.append((group, name, status, detail))
    mark = {'OK': 'ok  ', 'FAIL': 'FAIL', 'SKIP': 'skip'}[status]
    if status != 'OK' or os.environ.get('VERBOSE'):
        print(f'[{mark}] {group}: {name} {detail}')


def files(sub, *exts):
    d = os.path.join(PKG, sub)
    if not os.path.isdir(d):
        return []
    return sorted(os.path.join(d, f) for f in os.listdir(d) if f.endswith(exts))


def check_python():
    for path in files('scripts', '.py') + files('launch', '.py'):
        try:
            with open(path, 'rb') as f:
                compile(f.read(), path, 'exec')      # syntax check only, writes no .pyc
            record('python', os.path.basename(path), 'OK')
        except SyntaxError as e:
            record('python', os.path.basename(path), 'FAIL', f'line {e.lineno}: {e.msg}')


def check_yaml():
    try:
        import yaml
    except ImportError:
        record('yaml', 'pyyaml', 'SKIP', 'python3-yaml not installed')
        return
    for path in files('config', '.yaml', '.yml') + files('maps', '.yaml'):
        try:
            with open(path) as f:
                yaml.safe_load(f)
            record('yaml', os.path.basename(path), 'OK')
        except Exception as e:  # noqa: BLE001
            record('yaml', os.path.basename(path), 'FAIL', str(e).splitlines()[0][:120])


def check_urdf():
    for path in files('urdf', '.urdf.xacro'):
        name = os.path.basename(path)
        try:
            out = subprocess.run(['xacro', path], capture_output=True, text=True, timeout=60)
        except FileNotFoundError:
            record('urdf', name, 'SKIP', 'xacro not installed')
            continue
        if out.returncode != 0:
            record('urdf', name, 'FAIL', out.stderr.strip().splitlines()[-1][:120] if out.stderr.strip() else 'xacro failed')
            continue
        try:
            root = ET.fromstring(out.stdout)
        except ET.ParseError as e:
            record('urdf', name, 'FAIL', f'not well-formed: {e}')
            continue
        links = {l.get('name') for l in root.iter('link')}
        problems, children = [], set()
        for j in root.iter('joint'):
            p, c = j.find('parent'), j.find('child')
            if p is None or c is None:
                problems.append(f'joint {j.get("name")} missing parent/child')
                continue
            for end in (p.get('link'), c.get('link')):
                if end not in links:
                    problems.append(f'joint {j.get("name")} references missing link {end}')
            children.add(c.get('link'))
        roots = links - children
        if len(roots) != 1:
            problems.append(f'expected 1 root link, found {sorted(roots)}')
        if problems:
            record('urdf', name, 'FAIL', '; '.join(problems)[:200])
        else:
            record('urdf', name, 'OK', f'({len(links)} links)')


def check_worlds():
    for path in files('worlds', '.world', '.sdf'):
        try:
            ET.parse(path)
            record('worlds', os.path.basename(path), 'OK')
        except ET.ParseError as e:
            record('worlds', os.path.basename(path), 'FAIL', str(e))


def check_launch():
    try:
        import launch_ros  # noqa: F401
    except ImportError:
        record('launch', 'launch_ros', 'SKIP', 'ros-<distro>-launch-ros not installed')
        return
    sys.path.insert(0, os.path.join(PKG, 'launch'))
    for path in files('launch', '.py'):
        name = os.path.basename(path)
        if name.startswith('_'):
            continue
        try:
            spec = importlib.util.spec_from_file_location('chk_' + name.replace('.', '_'), path)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            gld = getattr(mod, 'generate_launch_description', None)
            if gld is None:
                record('launch', name, 'FAIL', 'no generate_launch_description()')
                continue
            gld()
            record('launch', name, 'OK')
        except Exception as e:  # noqa: BLE001
            msg = f'{type(e).__name__}: {str(e)[:100]}'
            # a dependency that is simply not installed (slam_toolbox, nav2_common, ...) is an
            # environment gap, not a launch-file bug: skip it, but fail on every other error
            if type(e).__name__ == 'PackageNotFoundError' or (
                    isinstance(e, ModuleNotFoundError) and not str(getattr(e, 'name', '')).startswith('rosnav')):
                record('launch', name, 'SKIP', f'missing dependency ({msg})')
            else:
                record('launch', name, 'FAIL', msg)


# worlds that are open on purpose (no walls to leak through)
OPEN_WORLDS = {'empty', 'outdoor', 'outdoor_arena', 'outdoor_rocks', 'rough_terrain', 'garden', 'lake_house',
               'multi_terrain', 'multi_terrain_robot_diff'}


def _pose6(text):
    v = [float(x) for x in (text or '0 0 0 0 0 0').split()] + [0.0] * 6
    return v[:6]


def world_wall_boxes(path, z_lo=0.1, z_hi=0.5):
    """(cx, cy, sx, sy, yaw) of every static box collision that blocks a 2D lidar plane (z in [z_lo, z_hi])."""
    import math
    root = ET.parse(path).getroot()
    out = []

    def walk(el, x, y, z, yaw):
        for ch in el:
            if ch.tag in ('model', 'link', 'collision'):
                px, py, pz, r, pi, ya = _pose6(ch.findtext('pose'))
                c, s = math.cos(yaw), math.sin(yaw)
                nx, ny, nz, nyaw = x + c * px - s * py, y + s * px + c * py, z + pz, yaw + ya
                if ch.tag == 'collision':
                    box = ch.find('geometry/box/size')
                    if box is not None and abs(r) < 0.05 and abs(pi) < 0.05:
                        sx, sy, sz = [float(v) for v in box.text.split()]
                        if nz - sz / 2 <= z_hi and nz + sz / 2 >= z_lo:
                            out.append((nx, ny, sx, sy, nyaw))
                else:
                    walk(ch, nx, ny, nz, nyaw)
    world = root.find('world') if root.find('world') is not None else root
    walk(world, 0.0, 0.0, 0.0, 0.0)
    return out


def world_leaks(boxes, start=(0.0, 0.0), res=0.05, pad=2.0):
    """True when `start` can reach the outside of the wall boxes (4-connected flood fill)."""
    import numpy as np
    from collections import deque
    xs = [b[0] + sgn * (b[2] / 2 + b[3] / 2) for b in boxes for sgn in (-1, 1)]
    ys = [b[1] + sgn * (b[2] / 2 + b[3] / 2) for b in boxes for sgn in (-1, 1)]
    x0, x1, y0, y1 = min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad
    W, H = int((x1 - x0) / res) + 1, int((y1 - y0) / res) + 1
    occ = np.zeros((H, W), dtype=bool)
    gx = x0 + (np.arange(W) + 0.5) * res
    gy = y0 + (np.arange(H) + 0.5) * res
    for cx, cy, sx, sy, yaw in boxes:
        import math
        r = math.hypot(sx, sy) / 2 + res
        c0, c1 = max(0, int((cx - r - x0) / res)), min(W, int((cx + r - x0) / res) + 1)
        r0, r1 = max(0, int((cy - r - y0) / res)), min(H, int((cy + r - y0) / res) + 1)
        X, Y = np.meshgrid(gx[c0:c1], gy[r0:r1])
        dx, dy = X - cx, Y - cy
        u = dx * math.cos(yaw) + dy * math.sin(yaw)
        v = -dx * math.sin(yaw) + dy * math.cos(yaw)
        occ[r0:r1, c0:c1] |= (np.abs(u) <= sx / 2 + res / 2) & (np.abs(v) <= sy / 2 + res / 2)
    sr, sc = int((start[1] - y0) / res), int((start[0] - x0) / res)
    if not (0 <= sr < H and 0 <= sc < W) or occ[sr, sc]:
        # start inside a wall/furniture: use the nearest free cell
        free = np.argwhere(~occ[max(sr - 40, 0):sr + 40, max(sc - 40, 0):sc + 40])
        if not len(free):
            return None
        d = np.abs(free - np.array([min(sr, 40), min(sc, 40)])).sum(axis=1)
        sr, sc = free[d.argmin()] + np.array([max(sr - 40, 0), max(sc - 40, 0)])
    seen = np.zeros_like(occ)
    q = deque([(sr, sc)])
    seen[sr, sc] = True
    while q:
        r, c = q.popleft()
        if r in (0, H - 1) or c in (0, W - 1):
            return True
        for nr, nc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
            if 0 <= nr < H and 0 <= nc < W and not occ[nr, nc] and not seen[nr, nc]:
                seen[nr, nc] = True
                q.append((nr, nc))
    return False


def check_closure():
    for path in files('worlds', '.world', '.sdf'):
        name = os.path.splitext(os.path.basename(path))[0]
        if name in OPEN_WORLDS:
            record('closure', name, 'SKIP', 'open by design')
            continue
        try:
            boxes = world_wall_boxes(path)
        except Exception as e:  # noqa: BLE001
            record('closure', name, 'FAIL', f'cannot parse geometry: {e}')
            continue
        if len(boxes) < 6:
            record('closure', name, 'SKIP', f'only {len(boxes)} wall boxes (mesh/include world)')
            continue
        leak = world_leaks(boxes)
        if leak is None:
            record('closure', name, 'SKIP', 'no free cell near the origin')
        elif leak:
            record('closure', name, 'FAIL', 'the spawn area is connected to the outside (gap in the walls)')
        else:
            record('closure', name, 'OK', f'({len(boxes)} boxes)')


CHECKS = {'python': check_python, 'yaml': check_yaml, 'urdf': check_urdf,
          'worlds': check_worlds, 'closure': check_closure, 'launch': check_launch}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--only', nargs='*', choices=sorted(CHECKS), help='run only these groups')
    args = ap.parse_args()
    for g in (args.only or CHECKS):
        CHECKS[g]()
    for g in (args.only or CHECKS):
        sel = [r for r in results if r[0] == g]
        print(f'{g:7s} {sum(r[2] == "OK" for r in sel):3d} ok  {sum(r[2] == "FAIL" for r in sel):3d} fail  '
              f'{sum(r[2] == "SKIP" for r in sel):3d} skip')
    failed = [r for r in results if r[2] == 'FAIL']
    print('STATIC CHECKS PASSED' if not failed else f'STATIC CHECKS FAILED ({len(failed)})')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
