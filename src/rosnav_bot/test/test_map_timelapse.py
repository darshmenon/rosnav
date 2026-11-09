import importlib.util
import os
import types

import numpy as np
import pytest

pytest.importorskip('rclpy')
_spec = importlib.util.spec_from_file_location(
    'map_timelapse', os.path.join(os.path.dirname(__file__), '..', 'scripts', 'map_timelapse.py'))
mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mod)


def _info(w, h):
    pos = types.SimpleNamespace(x=-1.0, y=-1.0)
    return types.SimpleNamespace(resolution=0.05, origin=types.SimpleNamespace(position=pos), width=w, height=h)


def test_render_returns_rgb_frame_with_known_free_and_occupied():
    grid = np.full((60, 80), -1, dtype=np.int8)
    grid[10:50, 10:70] = 0
    grid[10, 10:70] = 100
    f = mod.render(grid, _info(80, 60), {'robot1': [(0.0, 0.0), (0.5, 0.2)], 'robot2': []}, 12.0)
    assert f.ndim == 3 and f.shape[2] == 3 and f.dtype == np.uint8
    assert f.shape[0] > 60 and f.shape[1] >= 80


def test_frames_differ_as_map_grows():
    g1 = np.full((60, 80), -1, dtype=np.int8); g1[25:35, 35:45] = 0
    g2 = g1.copy(); g2[10:50, 10:70] = 0
    a = mod.render(g1, _info(80, 60), {'': [(0, 0)]}, 4.0)
    b = mod.render(g2, _info(80, 60), {'': [(0, 0), (1, 1)]}, 8.0)
    assert a.shape == b.shape and not np.array_equal(a, b)


def test_pad_frames_makes_all_frames_the_same_size():
    a = np.zeros((40, 50, 3), np.uint8)
    b = np.zeros((60, 30, 3), np.uint8)
    out = mod.pad_frames([a, b])
    assert all(f.shape == (60, 50, 3) for f in out)
    assert out[0][45, 10, 0] == 255 and out[0][10, 10, 0] == 0   # padded area white, original kept


def test_cropped_frames_can_be_written_as_gif(tmp_path):
    imageio = pytest.importorskip('imageio.v2')   # inside the test: a module-level skip can abort the session
    g1 = np.full((60, 80), -1, dtype=np.int8); g1[25:35, 35:45] = 0
    g2 = np.full((60, 80), -1, dtype=np.int8); g2[10:50, 10:70] = 0
    frames = mod.pad_frames([mod.render(g, _info(80, 60), {'': [(0, 0)]}, t) for g, t in ((g1, 1.0), (g2, 5.0))])
    out = tmp_path / 't.gif'
    imageio.mimsave(out, frames, duration=0.2, loop=0)
    assert out.stat().st_size > 0


def test_trail_outside_map_does_not_stretch_the_frame():
    grid = np.full((60, 80), -1, dtype=np.int8)
    grid[10:50, 10:70] = 0
    near = mod.render(grid, _info(80, 60), {'robot1': [(0.0, 0.0)]}, 1.0)
    far = mod.render(grid, _info(80, 60), {'robot1': [(0.0, 0.0)], 'robot2': [(40.0, 40.0)]}, 1.0)
    assert far.shape == near.shape


def test_rotate_view_moves_cells_and_trails_consistently():
    grid = np.zeros((4, 6), dtype=np.int8)
    grid[1, 4] = 100                                   # cell at x = -1 + 4*0.05, y = -1 + 1*0.05
    info = _info(6, 4)
    x, y = -1.0 + 4.5 * 0.05, -1.0 + 1.5 * 0.05      # centre of cell (1, 4)
    for k in range(4):
        g, ext, tr = mod.rotate_view(grid, info, {'r': [(x, y)]}, k)
        (nx, ny), = tr['r']
        r, c = np.argwhere(g == 100)[0]
        # the marked cell, placed by the new extent, must land exactly on the transformed point
        cx = ext[0] + (c + 0.5) * 0.05
        cy = ext[2] + (r + 0.5) * 0.05
        assert abs(cx - nx) < 1e-9 and abs(cy - ny) < 1e-9, (k, cx, cy, nx, ny)
        assert g.shape == ((4, 6) if k % 2 == 0 else (6, 4))


def test_render_rotated_frame_is_landscape_for_a_tall_map():
    grid = np.full((120, 40), -1, dtype=np.int8)
    grid[10:110, 5:35] = 0
    info = _info(40, 120)
    tall = mod.render(grid, info, {'robot1': [(0.0, 0.0)]}, 1.0)
    wide = mod.render(grid, info, {'robot1': [(0.0, 0.0)]}, 1.0, rotate=1)
    assert tall.shape[0] > tall.shape[1] and wide.shape[1] > wide.shape[0]
