import json, math, os, subprocess, sys
import numpy as np
import pytest
ROOT = os.path.join(os.path.dirname(__file__), '..')
sys.path.insert(0, ROOT)
from nav_explore_eval.mapio import load_yaml_map
from nav_explore_eval.worldmap import rasterize, shapes, write_map

BOX = "<model name='{n}'><static>true</static><link name='l'><pose>{x} {y} {z} 0 0 {yaw}</pose><collision name='c'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision></link></model>"


def world(*models, extra=''):
    return "<?xml version='1.0'?><sdf version='1.6'><world name='w'>" + ''.join(models) + extra + '</world></sdf>'


def room(side=4.0, t=0.2, h=2.0):
    s = side / 2
    return [BOX.format(n='n', x=0, y=s, z=1, yaw=0, sx=side + t, sy=t, sz=h), BOX.format(n='s', x=0, y=-s, z=1, yaw=0, sx=side + t, sy=t, sz=h),
            BOX.format(n='e', x=s, y=0, z=1, yaw=0, sx=t, sy=side, sz=h), BOX.format(n='w', x=-s, y=0, z=1, yaw=0, sx=t, sy=side, sz=h)]


def area(g, v=0):
    return (g.data == v).sum() * g.resolution ** 2


def test_room_interior_area():
    g = rasterize(shapes(world(*room())), res=0.05)
    assert area(g) == pytest.approx(3.8 * 3.8, rel=0.05)        # interior = side - wall thickness
    assert g.data[g.shape[0] // 2, g.shape[1] // 2] == 0


def test_outside_is_unknown_not_free():
    g = rasterize(shapes(world(*room())), res=0.05, margin=1.0)
    assert g.data[0, 0] == -1 and g.data[-1, -1] == -1


def test_obstacle_removes_free_space():
    obs = BOX.format(n='o', x=0, y=0, z=1, yaw=0, sx=1, sy=1, sz=2)
    free_empty = area(rasterize(shapes(world(*room())), 0.05))
    free_obs = area(rasterize(shapes(world(*room(), obs), ), 0.05, seed=(1.5, 1.5)))
    assert free_empty - free_obs == pytest.approx(1.0, abs=0.15)


def test_rotated_box_is_rotated():
    sh = shapes(world(BOX.format(n='d', x=0, y=0, z=1, yaw=math.pi / 4, sx=2, sy=0.2, sz=2), *room(6)))
    g = rasterize(sh, 0.05, seed=(2.0, 0.0))
    ys, xs = np.nonzero(g.data == 100)
    # diagonal bar: occupied cells near the centre must spread along both axes, not a flat line
    centre = (np.abs(xs * g.resolution + g.ox) < 0.8) & (np.abs(ys * g.resolution + g.oy) < 0.8)
    assert np.ptp(xs[centre]) > 20 and np.ptp(ys[centre]) > 20


def test_short_obstacle_below_lidar_is_ignored():
    low = BOX.format(n='low', x=0, y=0, z=0.05, yaw=0, sx=1, sy=1, sz=0.1)        # top at 0.1 m
    assert len(shapes(world(*room(), low), z=0.2)) == 4
    assert len(shapes(world(*room(), low), z=0.05)) == 5


def test_cylinder_and_ground_plane():
    cyl = "<model name='c'><link name='l'><pose>0 0 1 0 0 0</pose><collision name='c'><geometry><cylinder><radius>0.5</radius><length>2</length></cylinder></geometry></collision></link></model>"
    plane = "<model name='g'><link name='l'><collision name='c'><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision></link></model>"
    sh = shapes(world(*room(), cyl, plane))
    assert [s[0] for s in sh].count('cyl') == 1 and len(sh) == 5                  # plane never a wall
    g = rasterize(sh, 0.05, seed=(1.5, 1.5))
    assert area(rasterize(shapes(world(*room())), 0.05)) - area(g) == pytest.approx(math.pi * 0.25, abs=0.12)


def test_seed_inside_wall_or_outside_raises():
    sh = shapes(world(*room()))
    with pytest.raises(ValueError, match='seed'):
        rasterize(sh, 0.05, seed=(2.0, 0.0))            # inside the east wall
    with pytest.raises(ValueError, match='seed'):
        rasterize(sh, 0.05, seed=(50, 50))


def test_no_shapes_raises():
    with pytest.raises(ValueError, match='no collision'):
        rasterize(shapes(world(extra='')), 0.05)


def test_sealed_off_region_is_not_free():
    divider = BOX.format(n='d', x=0, y=0, z=1, yaw=0, sx=0.2, sy=4.0, sz=2)       # splits the room in two
    g = rasterize(shapes(world(*room(), divider)), 0.05, seed=(1.0, 0.0))
    assert area(g) == pytest.approx(1.9 * 3.8, rel=0.08)                            # only the seed's half


def test_write_and_reload_roundtrip(tmp_path):
    g = rasterize(shapes(world(*room())), 0.05)
    write_map(g, str(tmp_path / 'm'))
    b = load_yaml_map(str(tmp_path / 'm.yaml'))
    assert (b.data == g.data).all() and b.resolution == g.resolution and b.ox == pytest.approx(g.ox, abs=1e-3)


def test_cli_truth_from_world(tmp_path):
    w = tmp_path / 'w.world'; w.write_text(world(*room()))
    p = subprocess.run([sys.executable, '-m', 'nav_explore_eval.cli', 'truth-from-world', str(w), '--out', str(tmp_path / 'out')],
                       capture_output=True, text=True, cwd=ROOT)
    assert p.returncode == 0 and 'explorable area' in p.stdout and (tmp_path / 'out.yaml').exists()
    bad = subprocess.run([sys.executable, '-m', 'nav_explore_eval.cli', 'truth-from-world', str(w), '--out', str(tmp_path / 'x'), '--seed', '9', '9'],
                         capture_output=True, text=True, cwd=ROOT)
    assert bad.returncode == 2 and 'seed' in bad.stderr


def test_scoring_a_perfect_run_against_world_truth(tmp_path):
    from nav_explore_eval.metrics import score
    g = rasterize(shapes(world(*room())), 0.05)
    s = score(g, g)
    assert s['coverage_pct'] == 100 and s['wall_f1_pct'] == 100
