#!/usr/bin/env python3
"""
movingai_to_map.py — turn a MovingAI grid map (.map) into a Nav2 map (PGM + YAML) and, optionally, a Gazebo world.

  python3 movingai_to_map.py warehouse-10-20-10-2-1.map --out maps/ --cell 0.25 --world
Cells '@' 'O' 'T' are obstacles (anything else is free). `--cell` is the cell size in metres (default 0.25), the map
origin is the grid's lower-left corner. `--world` also writes <name>.world: obstacle runs merged into boxes (2 m high)
on a ground plane, same plugins as worlds/empty_room.world. Source maps: https://movingai.com/benchmarks/mapf/
"""
import argparse
import os

import numpy as np

OBST = set('@OT')


def read_map(path):
    with open(path) as f:
        lines = [l.rstrip('\n') for l in f]
    h = int([l for l in lines if l.startswith('height')][0].split()[1])
    w = int([l for l in lines if l.startswith('width')][0].split()[1])
    rows = lines[lines.index('map') + 1:][:h]
    return np.array([[c in OBST for c in r.ljust(w)[:w]] for r in rows], dtype=bool)   # True = obstacle, row 0 = top


def boxes(grid):
    """Merge horizontal runs, then stack identical runs vertically: [(x0, x1, y0, y1)] in cell units (row 0 = top)."""
    runs = {}
    for y, row in enumerate(grid):
        x = 0
        while x < len(row):
            if row[x]:
                x0 = x
                while x < len(row) and row[x]:
                    x += 1
                runs.setdefault((x0, x), []).append(y)
            else:
                x += 1
    out = []
    for (x0, x1), ys in runs.items():
        ys.sort(); s = p = ys[0]
        for y in ys[1:] + [None]:
            if y is not None and y == p + 1:
                p = y; continue
            out.append((x0, x1, s, p + 1))
            if y is not None:
                s = p = y
    return out


def write(grid, name, out, cell, world):
    os.makedirs(out, exist_ok=True)
    h, w = grid.shape
    from PIL import Image
    Image.fromarray(np.where(grid, 0, 254).astype(np.uint8)).save(f'{out}/{name}.pgm')
    open(f'{out}/{name}.yaml', 'w').write(
        f'image: {name}.pgm\nmode: trinary\nresolution: {cell}\norigin: [0.0, 0.0, 0]\nnegate: 0\n'
        'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
    if world:
        base = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'worlds', 'empty_room.world')).read()
        head = base[:base.index('<!-- ===== SINGLE')].replace("world name='empty_room'", f"world name='{name}'")
        bx = []
        for i, (x0, x1, y0, y1) in enumerate(boxes(grid)):
            sx, sy = (x1 - x0) * cell, (y1 - y0) * cell
            cx, cy = (x0 + x1) / 2 * cell, (h - (y0 + y1) / 2) * cell
            bx.append(f"<model name='o{i}'><static>true</static><pose>{cx:.3f} {cy:.3f} 1 0 0 0</pose><link name='l'>"
                      f"<collision name='c'><geometry><box><size>{sx:.3f} {sy:.3f} 2</size></box></geometry></collision>"
                      f"<visual name='v'><geometry><box><size>{sx:.3f} {sy:.3f} 2</size></box></geometry></visual></link></model>")
        open(f'{out}/{name}.world', 'w').write(head + '\n'.join(bx) + '\n  </world>\n</sdf>\n')
        return len(bx)
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map'); ap.add_argument('--out', default='.'); ap.add_argument('--cell', type=float, default=0.25)
    ap.add_argument('--world', action='store_true')
    a = ap.parse_args()
    g = read_map(a.map)
    n = write(g, os.path.splitext(os.path.basename(a.map))[0].replace('-', '_'), a.out, a.cell, a.world)
    print(f'{g.shape[1]}x{g.shape[0]} cells, {g.sum()} obstacle cells, {n} world boxes, written to {a.out}')


if __name__ == '__main__':
    main()
