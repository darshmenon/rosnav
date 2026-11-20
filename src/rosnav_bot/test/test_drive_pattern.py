import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
sys.modules.setdefault('rclpy', type(sys)('rclpy'))
for m in ('geometry_msgs', 'geometry_msgs.msg', 'rclpy.node'):
    sys.modules.setdefault(m, type(sys)(m))
sys.modules['geometry_msgs.msg'].Twist = object
sys.modules['rclpy.node'].Node = object
import drive_pattern as d  # noqa: E402


def test_fig8_flips_each_half_circle():
    half = math.pi / 0.4
    assert d.command('fig8', 1.0) == (0.4, 0.4)
    assert d.command('fig8', half + 1.0) == (0.4, -0.4)
    assert d.command('fig8', 2 * half + 1.0) == (0.4, 0.4)


def test_weave_and_circle():
    assert d.command('weave', 1.0) == (0.5, 0.8) and d.command('weave', 5.0) == (0.5, -0.8)
    assert d.command('circle', 99.0) == (0.2, 0.1)


def test_unknown_pattern_raises():
    try:
        d.command('zigzag', 0.0)
    except ValueError:
        return
    raise AssertionError('expected ValueError')


def test_avoid_hysteresis():
    assert d.avoid(0.4, 0.4, None, False) == (0.4, 0.4, False)            # no scan yet: follow the pattern
    assert d.avoid(0.4, 0.4, 2.0, False) == (0.4, 0.4, False)
    assert d.avoid(0.4, 0.4, 0.6, False) == (0.0, 0.6, True)              # too close: turn in place
    assert d.avoid(0.4, 0.4, 0.85, True) == (0.0, 0.6, True)              # still inside the clear band
    assert d.avoid(0.4, 0.4, 1.2, True) == (0.4, 0.4, False)              # clear: resume
