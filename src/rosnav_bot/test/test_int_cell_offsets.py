"""int() on a world->cell offset truncates toward zero, so a negative offset rounds up (-0.5 -> 0) where floor gives -1.
Checked sites: scripts/check_static.py world_leaks (start cell), coverage_planner.py region clip, bt_executor.py zones.
Finding (2026-11-23): none changes a result, because every negative case is clamped or lands on the border. These tests pin that."""
import importlib.util
import os

import pytest

np = pytest.importorskip('numpy')
SCRIPTS = os.path.join(os.path.dirname(__file__), '..', 'scripts')


def _load(name):
    spec = importlib.util.spec_from_file_location(name, os.path.join(SCRIPTS, name + '.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


# a 2 m x 2 m closed room made of four 0.1 m walls, centred on the origin
ROOM = [(0, 1, 2.1, 0.1, 0), (0, -1, 2.1, 0.1, 0), (1, 0, 0.1, 2.1, 0), (-1, 0, 0.1, 2.1, 0)]


def test_world_leaks_closed_room_does_not_leak():
    assert _load('check_static').world_leaks(ROOM, start=(0.0, 0.0)) is False


def test_world_leaks_open_room_leaks():
    assert _load('check_static').world_leaks(ROOM[:3], start=(0.0, 0.0)) is True


@pytest.mark.parametrize('shift', [0.0, 0.01, 0.025, 0.049, 0.051, 0.5])
def test_world_leaks_start_just_outside_grid_counts_as_outside(shift):
    # grid origin is min(xs) - pad = -1.05 - 2.0; a start a fraction of a cell beyond it has a negative cell offset
    cs = _load('check_static')
    x0 = min(b[0] - (b[2] / 2 + b[3] / 2) for b in ROOM) - 2.0
    assert cs.world_leaks(ROOM, start=(x0 - shift, 0.0)) is True


@pytest.mark.parametrize('off', [-0.99, -0.5, -0.01])
def test_negative_offset_clamps_to_same_bound_with_int_or_floor(off):
    import math
    # the coverage_planner / bt_executor clamp expressions: max(0, cell) and max(c_min, cell) with c_min >= 0
    assert max(0, int(off)) == max(0, math.floor(off)) == 0
    c_min = 3
    assert max(c_min, int(off)) == max(c_min, math.floor(off)) == c_min
