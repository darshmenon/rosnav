"""Run scripts/check_static.py (compile, YAML, URDF, worlds, closure) as part of the normal test suite."""
import os
import subprocess
import sys

SCRIPT = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'check_static.py')


def test_static_checks_pass():
    # launch files need the built workspace on the ROS path, so CI runs that group separately
    out = subprocess.run([sys.executable, SCRIPT, '--only', 'python', 'yaml', 'urdf', 'worlds', 'closure'],
                         capture_output=True, text=True, timeout=240)
    assert out.returncode == 0, out.stdout[-2000:]


def _closure_module():
    import importlib.util
    spec = importlib.util.spec_from_file_location('check_static_mod', SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _room(gap):
    """4 x 4 m room of 0.2 m walls; the east wall has a 1 m doorway when gap=True."""
    walls = [(0, 2, 4, 0.2, 0), (0, -2, 4, 0.2, 0), (-2, 0, 0.2, 4, 0)]
    if gap:
        walls += [(2, 1.25, 0.2, 1.5, 0), (2, -1.25, 0.2, 1.5, 0)]
    else:
        walls += [(2, 0, 0.2, 4, 0)]
    return walls


def test_world_closure_detects_a_doorway():
    mod = _closure_module()
    assert mod.world_leaks(_room(gap=False)) is False
    assert mod.world_leaks(_room(gap=True)) is True


def test_world_closure_rotated_walls():
    # the same closed room rotated 30 degrees about the origin stays closed
    import math
    c, s = math.cos(math.radians(30)), math.sin(math.radians(30))
    rot = [(x * c - y * s, x * s + y * c, sx, sy, yaw + math.radians(30)) for x, y, sx, sy, yaw in _room(False)]
    assert _closure_module().world_leaks(rot) is False
