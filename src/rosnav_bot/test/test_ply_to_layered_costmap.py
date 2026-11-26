"""Offline tests for ply_to_layered_costmap layers/combine — no ROS, no sim."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from ply_to_layered_costmap import LAYERS, combine, height_grid, layer_costs, overhead_gap, point_uncertainty, resolve_limits, speed_pixels  # noqa: E402

CELL = 0.25


def cloud(zfun, n=40):
    xs, ys = np.meshgrid(np.arange(0, n * CELL, 0.05) + 0.01, np.arange(0, n * CELL, 0.05) + 0.01)
    return np.column_stack([xs.ravel(), ys.ravel(), zfun(xs.ravel(), ys.ravel())])


def costs(zfun):
    g, zmax, seen, _, _ = height_grid(cloud(zfun), CELL)
    return layer_costs(g, zmax, seen, CELL), seen


def test_flat_ground_is_free():
    layers, seen = costs(lambda x, y: 0 * x)
    cost, _ = combine(layers, cell=CELL)
    assert cost[5:-5, 5:-5].max() < 0.05
    assert layers['steepness'].max() == 0 and layers['step'].max() == 0


def test_slope_scales_steepness_not_step():
    flat, _ = costs(lambda x, y: 0 * x)
    ramp, _ = costs(lambda x, y: math.tan(math.radians(12.5)) * x)
    mid = ramp['steepness'][20, 20]
    assert 0.4 < mid < 0.6          # 12.5 deg of 25 deg limit
    assert ramp['step'][20, 20] < 0.5 * 1.0 and flat['steepness'][20, 20] == 0


def test_wall_is_lethal_via_step():
    layers, _ = costs(lambda x, y: np.where(x > 5.0, 1.0, 0.0))
    cost, why = combine(layers, cell=CELL)
    edge = (slice(15, 25), 20)
    assert cost[edge].max() >= 0.9
    assert LAYERS[why[edge][cost[edge].argmax()]] in ('step', 'steepness')


def test_weight_override_changes_dominance():
    layers, _ = costs(lambda x, y: 0.15 * np.sin(8 * x))   # bumpy
    base, _ = combine(layers, cell=CELL)
    damp, _ = combine(layers, {'roughness': 0.0, 'ridge': 0.0, 'steepness': 0.0, 'step': 0.0}, cell=CELL)
    assert damp.max() <= base.max()


def test_inflation_adds_halo_next_to_hazard():
    layers, _ = costs(lambda x, y: np.where(x > 5.0, 1.0, 0.0))
    plain, _ = combine(layers, cell=CELL)
    infl, _ = combine(layers, inflate_m=0.75, cell=CELL)
    assert infl.sum() > plain.sum()


def test_drive_type_presets_and_override():
    assert resolve_limits('mecanum')['max_slope_deg'] < resolve_limits('diff')['max_slope_deg']
    assert resolve_limits('ackermann', max_step=0.5)['max_step'] == 0.5
    assert resolve_limits('diff', max_step=None)['max_step'] == 0.25


def test_speed_pixels_match_map_server_scale_decoding():
    cost = np.array([0.0, 0.5, 1.0, 1.0])
    px = speed_pixels(cost, np.array([True, True, True, False]), min_speed=0.2)
    value = np.rint(100 * (1 - px / 255.0))        # what map_server publishes
    assert list(value) == [0, 40, 80, 0]           # never 100, which Nav2 reads as no limit
    assert np.rint(100 * (1 - speed_pixels(cost, cost >= 0) / 255.0)).max() == 20   # default floor 80%


def test_low_overhang_is_blocked_but_high_canopy_is_free():
    floor = cloud(lambda x, y: 0 * x)
    for z, blocked in ((0.35, True), (2.0, False)):
        roof = floor[(floor[:, 0] > 3) & (floor[:, 0] < 5)] + [0, 0, z]
        p = np.vstack([floor, roof])
        g, zmax, seen, x0, y0 = height_grid(p, CELL, robot_height=0.5)
        gap = overhead_gap(p, g, CELL, x0, y0)
        layers = layer_costs(g, zmax, seen, CELL, gap=gap, robot_height=0.5)
        cost, _ = combine(layers, cell=CELL)
        under = cost[10:30, 16]
        assert (under.min() >= 0.99) if blocked else (under.max() < 0.05)


def test_sparse_cells_are_uncertain_and_uniform_mesh_is_not():
    p = cloud(lambda x, y: 0 * x)
    g, _, seen, x0, y0 = height_grid(p, CELL)
    inner = point_uncertainty(p, seen, CELL, x0, y0)[2:-3, 2:-3]
    assert inner.max() < 0.4                        # evenly sampled (16..25 points per cell)
    left = p[:, 0] < 5.0
    sparse = np.vstack([p[left], p[~left][::25]])
    g, _, seen, x0, y0 = height_grid(sparse, CELL)
    u = point_uncertainty(sparse, seen, CELL, x0, y0, confident_points=40)
    assert u[seen & (np.arange(u.shape[1]) > 22)[None, :]].mean() > 0.8


def test_inflation_without_any_hazard_adds_nothing():
    cost, _ = combine({k: np.zeros((20, 20)) for k in LAYERS}, inflate_m=0.5, cell=CELL)
    assert cost.max() == 0
