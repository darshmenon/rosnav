import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
# drive_pattern only needs `Twist` and `Node` as base-class placeholders. Stub them ONLY while it is imported and then
# restore sys.modules and the real attributes: overwriting geometry_msgs.msg.Twist / rclpy.node.Node on the real modules
# (as this file used to) broke every later test that builds a real Twist (collision monitor, nudge, rosbag replay).
_MISSING = object()
_saved_mods = {m: sys.modules.get(m, _MISSING) for m in ('rclpy', 'geometry_msgs', 'geometry_msgs.msg', 'rclpy.node')}
_saved_attrs = {}
for _m in _saved_mods:
    sys.modules.setdefault(_m, type(sys)(_m))
for _m, _a in (('geometry_msgs.msg', 'Twist'), ('rclpy.node', 'Node')):
    _saved_attrs[(_m, _a)] = getattr(sys.modules[_m], _a, _MISSING)
    setattr(sys.modules[_m], _a, object)
try:
    import drive_pattern as d  # noqa: E402
finally:
    for (_m, _a), _v in _saved_attrs.items():
        if _v is _MISSING:
            delattr(sys.modules[_m], _a)
        else:
            setattr(sys.modules[_m], _a, _v)
    for _m, _v in _saved_mods.items():
        if _v is _MISSING:
            sys.modules.pop(_m, None)


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
