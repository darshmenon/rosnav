#!/usr/bin/env python3
"""
gen_rough_terrain.py — procedural all-rough-terrain Gazebo world ("rough_terrain").

Writes models/rough_terrain/ (heightmap PNG + textures + model.sdf) and worlds/rough_terrain.world.
The WHOLE world is uneven: rolling hills + bumpy micro-relief everywhere, a gully, a rock-field
band, with static rocks/trees dropped on the surface. Only a small gently-graded spawn pad exists.
Slope is clamped so a diff-drive robot can still traverse it (--max-slope-deg).

usage: gen_rough_terrain.py [--seed 7] [--size 100] [--relief 3.0] [--max-slope-deg 22]
Prints the spawn height to put in slam_nav.launch.py's _WORLD_SPAWN_DEFAULTS.
"""
import argparse
import math
from pathlib import Path

import numpy as np
from PIL import Image

N = 257   # heightmap grid must be 2^n + 1


def value_noise(rng, cells, n=N):
    g = rng.random((cells + 1, cells + 1))
    xs = np.linspace(0, cells, n)
    i = np.minimum(xs.astype(int), cells - 1)
    f = xs - i
    f = f * f * (3 - 2 * f)
    rows = g[i][:, i] * (1 - f)[None, :] * (1 - f)[:, None] + g[i][:, i + 1] * f[None, :] * (1 - f)[:, None] \
        + g[i + 1][:, i] * (1 - f)[None, :] * f[:, None] + g[i + 1][:, i + 1] * f[None, :] * f[:, None]
    return rows


def make_height(rng, size, relief, max_slope_deg):
    cell = size / (N - 1)
    h = np.zeros((N, N))
    for cells, amp in ((4, 1.0), (8, 0.55), (16, 0.28), (32, 0.14), (64, 0.07)):   # rolling -> bumpy
        h += amp * (value_noise(rng, cells) - 0.5)
    yy, xx = np.mgrid[0:N, 0:N] * cell - size / 2
    h += 0.35 * np.exp(-((yy - 0.25 * xx - 8) / 3.0) ** 2) * -1.0                 # diagonal gully
    band = np.exp(-((xx + 15) / 10.0) ** 2)                                        # rock-field band
    h += band * 0.25 * (value_noise(rng, 128) - 0.5)
    h = (h - h.min()) / (h.max() - h.min()) * relief
    pad = np.exp(-((xx ** 2 + yy ** 2) / 4.0 ** 2) ** 2)                           # gentle spawn pad r~4m
    h = h * (1 - pad) + h[N // 2, N // 2] * pad
    max_dz = math.tan(math.radians(max_slope_deg)) * cell                          # clamp slope by relaxation
    for _ in range(300):
        worst = 0.0
        for ax in (0, 1):
            d = np.diff(h, axis=ax)
            over = np.abs(d) - max_dz
            worst = max(worst, over.max())
            adj = np.where(over > 0, np.sign(d) * over * 0.5, 0.0)
            if ax == 0:
                h[:-1] += adj; h[1:] -= adj
            else:
                h[:, :-1] += adj; h[:, 1:] -= adj
        if worst < 1e-3:
            break
    h -= h.min()
    return h


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, default=7); ap.add_argument('--size', type=float, default=100.0)
    ap.add_argument('--relief', type=float, default=3.0); ap.add_argument('--max-slope-deg', type=float, default=22.0)
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    root = Path(__file__).resolve().parents[1]
    h = make_height(rng, a.size, a.relief, a.max_slope_deg)
    zrange = float(h.max())
    cell = a.size / (N - 1)
    gy, gx = np.gradient(h, cell)
    slope = np.degrees(np.arctan(np.hypot(gx, gy)))
    print(f'relief {zrange:.2f} m, slope mean {slope.mean():.1f} deg, p95 {np.percentile(slope, 95):.1f}, max {slope.max():.1f}')

    tex = root / 'models/rough_terrain/materials/textures'
    Image.fromarray((h / zrange * 255).astype(np.uint8), 'L').save(tex / 'height.png')
    # colour by height + slope: grass low, dirt/rock on slopes and high ground
    t = h / zrange
    rock = np.clip((slope - 8) / 14, 0, 1)
    base = np.stack([60 + 90 * t, 110 - 25 * t, 45 + 30 * t], -1)
    col = base * (1 - rock[..., None]) + np.array([120, 105, 90]) * rock[..., None]
    col += (rng.random((N, N, 1)) - 0.5) * 18
    Image.fromarray(np.clip(col, 0, 255).astype(np.uint8)).resize((1024, 1024), Image.BILINEAR).save(tex / 'diffuse.png')
    Image.new('RGB', (4, 4), (128, 128, 255)).save(tex / 'normal.png')

    size_xy = a.size
    (root / 'models/rough_terrain/model.config').write_text(
        '<?xml version="1.0"?>\n<model><name>rough_terrain</name><version>1.0</version>'
        '<sdf version="1.6">model.sdf</sdf><description>Procedural all-rough terrain (gen_rough_terrain.py)</description></model>\n')
    (root / 'models/rough_terrain/model.sdf').write_text(f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="rough_terrain">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><heightmap>
          <uri>model://rough_terrain/materials/textures/height.png</uri>
          <size>{size_xy} {size_xy} {zrange:.3f}</size><pos>0 0 0</pos>
        </heightmap></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>
      <visual name="visual">
        <geometry><heightmap>
          <use_terrain_paging>false</use_terrain_paging>
          <texture><size>{size_xy}</size>
            <diffuse>model://rough_terrain/materials/textures/diffuse.png</diffuse>
            <normal>model://rough_terrain/materials/textures/normal.png</normal></texture>
          <uri>model://rough_terrain/materials/textures/height.png</uri>
          <size>{size_xy} {size_xy} {zrange:.3f}</size><pos>0 0 0</pos>
        </heightmap></geometry>
      </visual>
    </link>
  </model>
</sdf>
''')

    def z_at(x, y):
        i = int(round((y + a.size / 2) / cell)); j = int(round((x + a.size / 2) / cell))
        return float(h[min(max(i, 0), N - 1), min(max(j, 0), N - 1)])

    props = []
    half = a.size / 2 - 4
    k = 0
    while len(props) < 70 and k < 2000:
        k += 1
        x, y = rng.uniform(-half, half, 2)
        if math.hypot(x, y) < 5.0:
            continue
        z = z_at(x, y)
        if len(props) < 45:      # rocks: half-buried ellipsoids (spheres scaled via box+sphere mix)
            r = rng.uniform(0.15, 0.5)
            props.append(f'''    <model name="rock_{len(props)}"><static>true</static><pose>{x:.2f} {y:.2f} {z + r * 0.3:.2f} 0 0 {rng.uniform(0, 3.14):.2f}</pose>
      <link name="l"><collision name="c"><geometry><sphere><radius>{r:.2f}</radius></sphere></geometry></collision>
      <visual name="v"><geometry><sphere><radius>{r:.2f}</radius></sphere></geometry>
      <material><ambient>0.4 0.38 0.35 1</ambient><diffuse>0.45 0.42 0.4 1</diffuse></material></visual></link></model>''')
        else:                    # trees: trunk collision + canopy visual only
            hh = rng.uniform(2.0, 4.0)
            props.append(f'''    <model name="tree_{len(props)}"><static>true</static><pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose>
      <link name="l"><collision name="c"><pose>0 0 {hh / 2:.2f} 0 0 0</pose><geometry><cylinder><radius>0.15</radius><length>{hh:.2f}</length></cylinder></geometry></collision>
      <visual name="t"><pose>0 0 {hh / 2:.2f} 0 0 0</pose><geometry><cylinder><radius>0.15</radius><length>{hh:.2f}</length></cylinder></geometry>
      <material><ambient>0.3 0.2 0.1 1</ambient><diffuse>0.35 0.25 0.12 1</diffuse></material></visual>
      <visual name="canopy"><pose>0 0 {hh + 0.6:.2f} 0 0 0</pose><geometry><sphere><radius>1.2</radius></sphere></geometry>
      <material><ambient>0.1 0.3 0.1 1</ambient><diffuse>0.15 0.4 0.15 1</diffuse></material></visual></link></model>''')

    base = (root / 'worlds/outdoor.world').read_text()
    head = base[base.index('<sdf'):base.index('<include>')]
    head = head.replace('world name="outdoor"', 'world name="rough_terrain"')
    cam = base[base.index('<!-- Overview camera'):]
    cam = cam.replace('48.0 8.0 15.0 0 0.55 -2.356', f'{a.size / 2 + 6:.1f} 0 {zrange + 25:.1f} 0 0.75 3.14159')
    (root / 'worlds/rough_terrain.world').write_text(
        '<?xml version="1.0" ?>\n<!-- Generated by scripts/gen_rough_terrain.py: whole world is rough terrain. '
        'Local heightmap (models/rough_terrain), no Fuel download. -->\n' + head +
        '<include><uri>model://rough_terrain</uri></include>\n\n' + '\n'.join(props) + '\n\n    ' + cam)
    print(f'spawn pad height z={z_at(0, 0):.2f} -> spawn z={z_at(0, 0) + 0.3:.2f}; {len(props)} props')


if __name__ == '__main__':
    main()
