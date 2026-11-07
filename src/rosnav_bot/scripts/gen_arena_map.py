#!/usr/bin/env python3
"""Render the exact occupancy map of worlds/arena_square.world (walls + markers) — no SLAM streaks.

  python3 scripts/gen_arena_map.py            # writes maps/map_arena_square.{pgm,yaml,png}
Free = 254, occupied = 0, unknown (outside the walls) = 205. Geometry must match the world file.
"""
import os
import subprocess

RES, HALF, WALL, MARGIN = 0.05, 15.0, 0.2, 1.0
MARKERS = [(9.0, 6.0, 0.5), (-7.5, -10.0, 0.5)]   # x, y, side — same as as_marker_a / as_marker_b
here = os.path.dirname(os.path.abspath(__file__))
out = os.path.join(here, '..', 'maps')
ext = HALF + WALL / 2 + MARGIN
n = int(round(2 * ext / RES))
org = -ext
img = [[205] * n for _ in range(n)]


def rect(x0, y0, x1, y1, v):
    for j in range(max(0, int((y0 - org) / RES)), min(n, int((y1 - org) / RES) + 1)):
        row = img[n - 1 - j]
        for i in range(max(0, int((x0 - org) / RES)), min(n, int((x1 - org) / RES) + 1)):
            row[i] = v


rect(-HALF, -HALF, HALF, HALF, 254)                                   # free interior
t = WALL / 2
rect(-HALF - t, HALF - t, HALF + t, HALF + t, 0)                      # north
rect(-HALF - t, -HALF - t, HALF + t, -HALF + t, 0)                    # south
rect(HALF - t, -HALF - t, HALF + t, HALF + t, 0)                      # east
rect(-HALF - t, -HALF - t, -HALF + t, HALF + t, 0)                    # west
for x, y, s in MARKERS:
    rect(x - s / 2, y - s / 2, x + s / 2, y + s / 2, 0)
with open(os.path.join(out, 'map_arena_square.pgm'), 'wb') as f:
    f.write(f'P5\n{n} {n}\n255\n'.encode())
    for row in img:
        f.write(bytes(row))
open(os.path.join(out, 'map_arena_square.yaml'), 'w').write(
    f'image: map_arena_square.pgm\nmode: trinary\nresolution: {RES}\norigin: [{org:.2f}, {org:.2f}, 0]\n'
    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
try:
    subprocess.run(['convert', os.path.join(out, 'map_arena_square.pgm'), '-resize', '100%',
                    os.path.join(out, 'map_arena_square.png')], check=True)
except Exception:  # noqa: BLE001
    pass
print(f'map_arena_square: {n}x{n} px, {2 * ext:.1f} m square')
