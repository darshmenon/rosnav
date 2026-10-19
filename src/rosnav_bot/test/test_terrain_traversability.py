"""Offline tests for terrain_traversability.classify() — no ROS, no sim."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from terrain_traversability import classify  # noqa: E402

CELL, STEP, MAX_SLOPE = 0.25, 0.25, 25.0
TAN = math.tan(math.radians(MAX_SLOPE))


def grid(x0=0.5, x1=5.0, y=2.5, d=0.05):
    xs, ys = np.meshgrid(np.arange(x0, x1, d), np.arange(-y, y, d))
    return xs.ravel(), ys.ravel()


def ramp(deg):
    x, y = grid()
    z = np.tan(math.radians(deg)) * np.clip(x - 2.0, 0, None)  # flat, then a ramp from x=2 m
    return np.column_stack([x, y, z - 0.25])


def blocked(p, R=np.eye(3)):
    return int(classify(p, R, CELL, TAN, STEP)[0].sum())


def rot(roll_deg=0.0, pitch_deg=0.0):
    r, p = math.radians(roll_deg), math.radians(pitch_deg)
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    return Ry @ Rx


def test_flat_ground_not_blocked():
    assert blocked(ramp(0)) == 0


def test_10deg_ramp_not_blocked():
    assert blocked(ramp(10)) == 0


def test_35deg_ramp_blocked():
    assert blocked(ramp(35)) > 0


def test_box_on_flat_ground_blocked():
    p = ramp(0)
    x, y = np.meshgrid(np.arange(3.0, 3.4, 0.05), np.arange(-0.2, 0.2, 0.05))
    box = np.column_stack([x.ravel(), y.ravel(), np.full(x.size, 0.4 - 0.25)])  # 0.4 m tall top face
    side = np.column_stack([np.full(8, 3.0), np.linspace(-0.2, 0.2, 8), np.linspace(-0.25, 0.15, 8)])
    assert blocked(np.vstack([p, box, side])) > 0


def test_tilted_robot_on_flat_ground_not_blocked():
    # robot pitched/rolled 5 deg: sensor sees the flat floor tilted; R_level (from IMU) must undo it
    R_level = rot(roll_deg=5, pitch_deg=5)          # base -> level
    floor_level = ramp(0)
    p_base = floor_level @ R_level                 # inverse rotation: level -> base
    assert blocked(p_base, R_level) == 0


def test_step_marks_uphill_side():
    x, y = grid(0.5, 4.0, 1.0)
    p = np.column_stack([x, y, np.where(x >= 2.0, 0.2, 0.0)])
    mask = classify(p, np.eye(3), CELL, TAN, STEP)[0]
    assert mask.any() and x[mask].min() >= 2.0 - CELL


if __name__ == '__main__':
    for n, f in list(globals().items()):
        if n.startswith('test_'):
            f()
            print('PASS', n)
