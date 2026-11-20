import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
import world_to_map as w  # noqa: E402

BOX = ("<sdf version='1.6'><world name='w'>%s</world></sdf>")


def _wall(name, x, y, sx, sy, z=0.5):
    return (f"<model name='{name}'><static>true</static><pose>{x} {y} {z} 0 0 0</pose><link name='l'><collision name='c'>"
            f"<geometry><box><size>{sx} {sy} 1.0</size></box></geometry></collision></link></model>")


def _room(tmp_path, gap=False):
    parts = [_wall('n', 0, 2, 4.2, 0.1), _wall('e', 2, 0, 0.1, 4.2), _wall('s', 0, -2, 4.2, 0.1),
             _wall('w', -2, 0, 0.1, 4.2 if not gap else 1.0, 0.5),
             _wall('floor', 0, 0, 9, 9, z=0.0)]            # z-range -0.5..0.5 would hit the band; keep it flat instead
    parts[-1] = parts[-1].replace('<size>9 9 1.0</size>', '<size>9 9 0.1</size>')
    p = tmp_path / 'room.world'
    p.write_text(BOX % ''.join(parts))
    return str(p)


def test_closed_room_fills_and_floor_is_ignored(tmp_path):
    shapes, warns = w.load_shapes(_room(tmp_path), [])
    assert not warns and len(shapes) == 5
    occ, gw, gh, ox, oy, n = w.rasterize(shapes, (0.0, 0.0, 0.0), res=0.1)
    assert n == 4                                           # floor (top at 0.05) is below the 0.2 m band
    free, leaked = w.flood_free(occ, gw, gh, (int(-ox / 0.1), int(-oy / 0.1)))
    assert not leaked
    assert abs(sum(free) * 0.01 - 3.9 * 3.9) < 0.8          # inside area, up to wall thickness


def test_gap_in_wall_leaks(tmp_path):
    shapes, _ = w.load_shapes(_room(tmp_path, gap=True), [])
    occ, gw, gh, ox, oy, _n = w.rasterize(shapes, (0.0, 0.0, 0.0), res=0.1)
    _free, leaked = w.flood_free(occ, gw, gh, (int(-ox / 0.1), int(-oy / 0.1)))
    assert leaked


def test_map_is_in_the_start_frame():
    # a spawn at (1, 0) facing +y(90deg): a world point 2 m ahead of the robot (1, 2) is (2, 0) in the start frame
    x, y, yaw = w.to_start_frame((1.0, 2.0, 0, 0, 0, 1.5708), (1.0, 0.0, 1.5708))
    assert abs(x - 2.0) < 1e-3 and abs(y) < 1e-3 and abs(yaw) < 1e-3


def test_mesh_collision_warns(tmp_path):
    p = tmp_path / 'm.world'
    p.write_text(BOX % "<model name='m'><link name='l'><collision name='c'><geometry><mesh><uri>x.dae</uri></mesh></geometry></collision></link></model>")
    _s, warns = w.load_shapes(str(p), [])
    assert warns and 'mesh' in warns[0]
