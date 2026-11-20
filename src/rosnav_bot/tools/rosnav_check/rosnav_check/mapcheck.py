"""ROS-free map guard: is this saved map consistent with how the stack will use it?

The map frame of a SLAM-built map is the robot's START frame (odom origin = spawn pose) and AMCL is started at
(0, 0, 0) in it, so a map is only valid for the spawn it was built from. A map from another spawn yaw comes out
rotated against the world, and a plateaued mapping run comes out small. This catches both from the yaml+pgm alone.
"""
import os
import re

from .core import Item

ASPECT_ROT = 1.5        # |log-ratio| of w/h vs the reference beyond this factor flipping orientation = rotated
SMALL_FRAC = 0.6        # area below this fraction of the reference = incomplete
MIN_KNOWN_FRAC = 0.05   # known (free/occupied) cells below this fraction of the grid


def read_pgm(path):
    """(width, height, maxval, pixels bytes-like of len w*h) for P5 or P2."""
    with open(path, 'rb') as fh:
        raw = fh.read()
    m = re.match(rb'(P[25])\s+(?:#[^\n]*\n\s*)*(\d+)\s+(?:#[^\n]*\n\s*)*(\d+)\s+(?:#[^\n]*\n\s*)*(\d+)\s', raw)
    if not m:
        raise ValueError(f'{path}: not a PGM')
    kind, w, h, mx = m.group(1), int(m.group(2)), int(m.group(3)), int(m.group(4))
    body = raw[m.end():]
    px = bytes(body[:w * h]) if kind == b'P5' else bytes(int(t) for t in body.split()[:w * h])
    if len(px) != w * h:
        raise ValueError(f'{path}: truncated PGM ({len(px)} of {w * h} pixels)')
    return w, h, mx, px


def read_map(yaml_path):
    """Parse map yaml (+ its image) into a dict. No PyYAML dependency."""
    kv = {}
    with open(yaml_path) as fh:
        for line in fh:
            if ':' in line and not line.lstrip().startswith('#'):
                k, v = line.split(':', 1)
                kv[k.strip()] = v.strip()
    img = kv['image']
    img = img if os.path.isabs(img) else os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img)
    w, h, mx, px = read_pgm(img)
    ox, oy = (float(t) for t in kv['origin'].strip('[]').split(',')[:2])
    return dict(w=w, h=h, mx=mx, px=px, res=float(kv['resolution']), ox=ox, oy=oy, negate=int(kv.get('negate', 0)),
                free=float(kv.get('free_thresh', 0.25)), occ=float(kv.get('occupied_thresh', 0.65)), path=yaml_path)


def cell_state(mp, x, y):
    """'free' | 'occupied' | 'unknown' | 'outside' for a point in the map frame."""
    cx, cy = int((x - mp['ox']) / mp['res']), int((y - mp['oy']) / mp['res'])
    if not (0 <= cx < mp['w'] and 0 <= cy < mp['h']):
        return 'outside'
    p = mp['px'][(mp['h'] - 1 - cy) * mp['w'] + cx] / mp['mx']
    occ = p if mp['negate'] else 1.0 - p
    return 'occupied' if occ > mp['occ'] else 'free' if occ < mp['free'] else 'unknown'


def known_fraction(mp):
    n = len(mp['px'])
    unk = sum(1 for p in mp['px'] if mp['free'] < (p / mp['mx'] if mp['negate'] else 1.0 - p / mp['mx']) < mp['occ'])
    return 1.0 - unk / n if n else 0.0


def check_map(yaml_path, initial_pose=(0.0, 0.0), ref_yaml=None):
    """[Item] for the map guard; `ref_yaml` = a known-good map of the same world (e.g. the committed one)."""
    T = 'rosnav_check'
    try:
        mp = read_map(yaml_path)
    except (OSError, ValueError, KeyError) as e:
        return [Item(T, 'MAP_UNREADABLE', 'ERROR', f'{yaml_path}: {e}')]
    w_m, h_m = mp['w'] * mp['res'], mp['h'] * mp['res']
    items = [Item(T, 'MAP_INFO', 'INFO', f'{w_m:.1f} x {h_m:.1f} m ({mp["w"]}x{mp["h"]} cells), origin ({mp["ox"]}, {mp["oy"]}), '
                                         f'x {mp["ox"]:.1f}..{mp["ox"] + w_m:.1f}, y {mp["oy"]:.1f}..{mp["oy"] + h_m:.1f}')]
    st = cell_state(mp, *initial_pose)
    if st in ('outside', 'occupied'):
        items.append(Item(T, 'MAP_START_BAD', 'ERROR', f'AMCL start {initial_pose} is {st} in this map: the map was built from a different spawn pose',
                          'remap from the current spawn, or set the AMCL initial pose to the spawn expressed in this map\'s frame'))
    elif st == 'unknown':
        items.append(Item(T, 'MAP_START_UNKNOWN', 'WARN', f'AMCL start {initial_pose} is an unknown cell: mapping did not cover the spawn',
                          'remap, or drive/spin at the spawn so it is mapped'))
    kf = known_fraction(mp)
    if kf < MIN_KNOWN_FRAC:
        items.append(Item(T, 'MAP_MOSTLY_UNKNOWN', 'WARN', f'only {kf * 100:.1f}% of the grid is known'))
    if ref_yaml:
        try:
            rf = read_map(ref_yaml)
        except (OSError, ValueError, KeyError) as e:
            return items + [Item(T, 'MAP_REF_UNREADABLE', 'WARN', f'{ref_yaml}: {e}')]
        a_new, a_ref = w_m * h_m, rf['w'] * rf['res'] * rf['h'] * rf['res']
        land_new, land_ref = mp['w'] >= mp['h'], rf['w'] >= rf['h']
        if land_new != land_ref and max(mp['w'], mp['h']) / min(mp['w'], mp['h']) > 1.2 and max(rf['w'], rf['h']) / min(rf['w'], rf['h']) > 1.2:
            items.append(Item(T, 'MAP_ROTATED', 'WARN',
                              f'long axis is {"x" if land_new else "y"} here but {"x" if land_ref else "y"} in the reference: '
                              f'map looks rotated ~90 deg (built from a different spawn yaw)',
                              'rebuild from the spawn the reference used, or keep the frame convention and just remap fully'))
        if a_new < SMALL_FRAC * a_ref:
            items.append(Item(T, 'MAP_SMALL', 'WARN', f'area {a_new:.0f} m2 is {100 * a_new / a_ref:.0f}% of the reference ({a_ref:.0f} m2): '
                                                      f'mapping likely stopped early', 'continue exploring until coverage plateaus'))
    if not any(i.severity in ('ERROR', 'WARN') for i in items):
        items.append(Item(T, 'MAP_OK', 'OK', 'map is consistent with the AMCL start and the reference'))
    return items
