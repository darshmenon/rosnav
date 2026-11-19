#!/usr/bin/env python3
"""Gazebo/Nav2 versions of the decentral_nav_sim scenarios: swap, corridor, crowd (no ROS in here).

  decentral_gz_scenarios.py gen            # write worlds/decentral_{swap,corridor}.world + maps/map_decentral_*.{pgm,yaml}
  decentral_gz_scenarios.py cmd corridor 4 # print the launch + goal-runner commands for a scenario

World + map are generated from the same box list, so the occupancy map matches the world exactly.
Geometry mirrors decentral_nav_sim: swap = robots on a circle going to the antipodal point; corridor =
two 6x6 m rooms joined by a 1.5 m x 10 m corridor, two groups swap sides; crowd = worlds/crowd.world
(walking people via moving_obstacles_gz.py), robots cross the hall left -> right.
"""
from __future__ import annotations

import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.realpath(__file__))       # realpath: via `ros2 run` this is an install/ symlink
PKG = os.path.normpath(os.path.join(HERE, '..'))
if not os.path.isdir(os.path.join(PKG, 'maps')):           # non-symlink install: use the package share dir
    try:
        from ament_index_python.packages import get_package_share_directory
        PKG = get_package_share_directory('rosnav_bot')
    except Exception:
        pass

# ---- boxes: (cx, cy, sx, sy) in metres, wall height 2 -------------------------------------------------
T = 0.2


def swap_boxes(half=7.0):
    return [(0, half + T / 2, 2 * half + 2 * T, T), (0, -half - T / 2, 2 * half + 2 * T, T),
            (half + T / 2, 0, T, 2 * half), (-half - T / 2, 0, T, 2 * half)]


def corridor_boxes(rw=6.0, hh=3.0, length=10.0, cw=1.5):
    x0 = length / 2
    xo = x0 + rw
    b = [(0, hh + T / 2, 2 * xo + 2 * T, T), (0, -hh - T / 2, 2 * xo + 2 * T, T),   # outer N / S
         (xo + T / 2, 0, T, 2 * hh), (-xo - T / 2, 0, T, 2 * hh)]                      # outer E / W
    blk = hh - cw / 2                                                                  # solid between corridor and wall
    for s in (1, -1):
        b.append((0, s * (cw / 2 + blk / 2), length, blk))
    return b


# ---- scenarios: starts / goals / yaws in the map frame -------------------------------------------------
def swap(n: int, radius=5.0, seed=0):
    rng = np.random.default_rng(seed)
    ang = 2 * math.pi * np.arange(n) / n + rng.uniform(0, 0.2)
    s = radius * np.stack([np.cos(ang), np.sin(ang)], 1)
    return s, -s


def corridor(n: int, rw=6.0, length=10.0, hh=3.0):
    """n robots, half start in the left room and half in the right; goals are in the opposite room."""
    xo = length / 2 + rw
    na, nb = (n + 1) // 2, n // 2

    def room(k, side):
        ys = np.linspace(-hh + 0.9, hh - 0.9, max(k, 1))[:k]
        xs = side * (xo - 1.0 - 1.2 * (np.arange(k) % 2))
        return np.stack([xs, ys], 1)
    a, b = room(na, -1), room(nb, +1)
    ga, gb = room(na, +1), room(nb, -1)
    return np.vstack([a, b]), np.vstack([ga, gb])


def _load_free(map_yaml):
    """(free bool grid with row0 = max y, resolution, origin) from a map yaml (trinary PGM; free = 254)."""
    import yaml
    with open(map_yaml) as f:
        meta = yaml.safe_load(f)
    with open(os.path.join(os.path.dirname(map_yaml), meta['image']), 'rb') as f:
        f.readline()
        w, h = map(int, f.readline().split())
        f.readline()
        img = np.frombuffer(f.read(), np.uint8).reshape(h, w)
    return img > 250, float(meta['resolution']), meta['origin']


def _snap(pt, free, res, org, clear=0.5):
    """Nearest cell centre to pt (map frame) whose clear-radius disc is entirely free."""
    from scipy import ndimage
    dist = ndimage.distance_transform_edt(free) * res
    rr, cc = np.nonzero(dist >= clear)
    x = org[0] + (cc + 0.5) * res
    y = org[1] + (free.shape[0] - rr - 0.5) * res
    k = int(np.argmin((x - pt[0]) ** 2 + (y - pt[1]) ** 2))
    return float(x[k]), float(y[k])


def crowd(n: int, w=18.0, h=12.0, seed=0):
    """Robots cross the crowd.world hall left -> right; nominal points are snapped to free cells with clearance."""
    rng = np.random.default_rng(seed)
    free, res, org = _load_free(os.path.join(PKG, 'maps', 'map_crowd.yaml'))
    ys = np.linspace(-h / 2 + 1.2, h / 2 - 1.2, n)
    s = np.array([_snap((-w / 2 + 0.8, y), free, res, org) for y in ys])
    g = np.array([_snap((w / 2 - 0.8, y), free, res, org) for y in rng.permutation(ys)])
    return s, g


SCEN = {
    'swap': dict(world='decentral_swap', map='map_decentral_swap.yaml', fn=swap),
    'corridor': dict(world='decentral_corridor', map='map_decentral_corridor.yaml', fn=corridor),
    'crowd': dict(world='crowd', map='map_crowd.yaml', fn=crowd),
}


def build(kind: str, n: int):
    sc = SCEN[kind]
    starts, goals = sc['fn'](n)
    yaw = np.arctan2(goals[:, 1] - starts[:, 1], goals[:, 0] - starts[:, 0])
    robots = [dict(name=f'robot{i + 1}', x=round(float(s[0]), 3), y=round(float(s[1]), 3), yaw=round(float(a), 3))
              for i, (s, a) in enumerate(zip(starts, yaw))]
    return dict(kind=kind, world=sc['world'], map=sc['map'], robots=robots, goals=[[round(float(g[0]), 3), round(float(g[1]), 3)] for g in goals])


# ---- world + map generation ----------------------------------------------------------------------------
_HEAD = """<?xml version="1.0" ?>
<!-- GENERATED by scripts/decentral_gz_scenarios.py gen - do not edit by hand. -->
<sdf version='1.8'>
  <world name='{name}'>
    <physics name='4ms' type='ignored'><max_step_size>0.004</max_step_size><real_time_factor>1</real_time_factor></physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="ignition-gazebo-contact-system" name="ignition::gazebo::systems::Contact"/>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>
    <light name='sun' type='directional'><pose>0 0 15 0 0 0</pose><cast_shadows>true</cast_shadows><intensity>1</intensity>
      <direction>-0.4 0.2 -0.9</direction><diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular></light>
    <model name='ground_plane'><static>true</static><link name='link'>
      <collision name='c'><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
      <visual name='v'><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        <material><ambient>0.75 0.75 0.75 1</ambient><diffuse>0.75 0.75 0.75 1</diffuse></material></visual></link></model>
"""
_BOX = """    <model name='{n}'><static>true</static><pose>{x} {y} 1 0 0 0</pose><link name='l'>
      <collision name='c'><geometry><box><size>{sx} {sy} 2</size></box></geometry></collision>
      <visual name='v'><geometry><box><size>{sx} {sy} 2</size></box></geometry>
        <material><ambient>0.5 0.5 0.6 1</ambient><diffuse>0.5 0.5 0.6 1</diffuse></material></visual></link></model>
"""


def world_sdf(name, boxes):
    body = ''.join(_BOX.format(n=f'wall_{i}', x=x, y=y, sx=sx, sy=sy) for i, (x, y, sx, sy) in enumerate(boxes))
    return _HEAD.format(name=name) + body + "  </world>\n</sdf>\n"


def raster(boxes, res=0.05, margin=1.0):
    xs = [(x - sx / 2, x + sx / 2) for x, _, sx, _ in boxes]
    ys = [(y - sy / 2, y + sy / 2) for _, y, _, sy in boxes]
    x0, x1 = min(a for a, _ in xs) - margin, max(b for _, b in xs) + margin
    y0, y1 = min(a for a, _ in ys) - margin, max(b for _, b in ys) + margin
    w, h = int(round((x1 - x0) / res)), int(round((y1 - y0) / res))
    img = np.full((h, w), 254, np.uint8)               # row 0 = top (max y)
    for (cx, cy, sx, sy) in boxes:
        c0, c1 = int((cx - sx / 2 - x0) / res), int(math.ceil((cx + sx / 2 - x0) / res))
        r0, r1 = int((y1 - (cy + sy / 2)) / res), int(math.ceil((y1 - (cy - sy / 2)) / res))
        img[max(r0, 0):r1, max(c0, 0):c1] = 0
    return img, (x0, y0)


def write_map(path_prefix, img, origin, res=0.05):
    with open(path_prefix + '.pgm', 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (img.shape[1], img.shape[0]))
        f.write(img.tobytes())
    with open(path_prefix + '.yaml', 'w') as f:
        f.write(f'image: {os.path.basename(path_prefix)}.pgm\nresolution: {res}\n'
                f'origin: [{origin[0]:.3f}, {origin[1]:.3f}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')


def gen():
    for kind, boxes in (('swap', swap_boxes()), ('corridor', corridor_boxes())):
        name = SCEN[kind]['world']
        with open(os.path.join(PKG, 'worlds', name + '.world'), 'w') as f:
            f.write(world_sdf(name, boxes))
        img, org = raster(boxes)
        write_map(os.path.join(PKG, 'maps', 'map_' + name.replace('decentral_', 'decentral_')), img, org)
        print(f'wrote worlds/{name}.world + maps/map_{name}.pgm/.yaml ({img.shape[1]}x{img.shape[0]})')


def cmd(kind, n, ctrl='vo'):
    sc = build(kind, n)
    rj = json.dumps(sc['robots'], separators=(',', ':'))
    print(f"ros2 launch rosnav_bot multi_robot.launch.py world:={sc['world']} explore:=false prebuilt_localization:=static controller:=decentral "
          f"decentral_ctrl:={ctrl} headless:=true rviz:=false robots_json:='{rj}'")
    if kind == 'crowd':
        print('python3 src/rosnav_bot/scripts/moving_obstacles_gz.py --world crowd --speed 0.7   # same GZ_PARTITION')
    print(f"ros2 run rosnav_bot decentral_gz_run.py --ros-args -p scenario:={kind} -p n:={n}")


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'gen':
        gen()
    elif len(sys.argv) > 3 and sys.argv[1] == 'cmd':
        cmd(sys.argv[2], int(sys.argv[3]), sys.argv[4] if len(sys.argv) > 4 else 'vo')
    else:
        print(__doc__)
