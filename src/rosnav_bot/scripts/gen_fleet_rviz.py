#!/usr/bin/env python3
"""Write an RViz config for N robots (model + scan + plan each, top-down view): gen_fleet_rviz.py N [out.rviz].

Starts from rviz/multi_robot.rviz (map, TF, grid), clones its Robot1 model per robot, and adds a coloured /plan
Path and /scan LaserScan per robot. Used by the decentral Gazebo demos so every robot is visible."""
import copy
import os
import sys

import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COLORS = ['255; 85; 0', '0; 170; 255', '85; 255; 0', '255; 0; 200', '255; 220; 0', '0; 255; 200', '170; 85; 255', '255; 255; 255']


def build(n: int, base: str = os.path.join(PKG, 'rviz', 'multi_robot.rviz'), view_z: float = 22.0) -> dict:
    sys.path.insert(0, os.path.join(PKG, 'launch'))
    import _common  # shared with multi_robot.launch.py, which now does this at launch time
    return _common.fleet_rviz_config([f'robot{i}' for i in range(1, n + 1)], base)


if __name__ == '__main__':
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 4
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(PKG, 'rviz', f'fleet_{n}.rviz')
    with open(out, 'w') as f:
        yaml.safe_dump(build(n), f, sort_keys=False)
    print(f'wrote {out} ({n} robots)')
