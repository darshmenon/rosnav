"""person_distances: robot-to-pedestrian minimum distance and hit pairs used by decentral_gz_run."""
import importlib.util
import math
import os
import sys
import types

_STUBS = ('rclpy', 'rclpy.action', 'rclpy.node', 'rclpy.time', 'geometry_msgs', 'geometry_msgs.msg',
          'nav2_msgs', 'nav2_msgs.action', 'std_msgs', 'std_msgs.msg', 'tf2_ros')
_ATTRS = (('rclpy.action', 'ActionClient'), ('rclpy.node', 'Node'), ('geometry_msgs.msg', 'PoseStamped'),
          ('nav2_msgs.action', 'NavigateToPose'), ('std_msgs.msg', 'String'), ('tf2_ros', 'Buffer'),
          ('tf2_ros', 'TransformListener'))


def _load():
    """Import decentral_gz_run with ROS modules stubbed ONLY while it loads, then put sys.modules back, so the stubs
    never leak into other test files that need the real rclpy / launch_ros."""
    saved = {m: sys.modules.get(m) for m in _STUBS}
    try:
        for m in _STUBS:
            sys.modules[m] = types.ModuleType(m)
        for mod, attr in _ATTRS:
            setattr(sys.modules[mod], attr, type(attr, (), {}))
        d = os.path.join(os.path.dirname(__file__), '..', 'scripts')
        sys.path.insert(0, d)
        spec = importlib.util.spec_from_file_location('dgr', os.path.join(d, 'decentral_gz_run.py'))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    finally:
        for m, old in saved.items():
            if old is None:
                sys.modules.pop(m, None)
            else:
                sys.modules[m] = old


dgr = _load()


def test_min_and_hits():
    d, hits = dgr.person_distances({'r1': (0.0, 0.0), 'r2': (5.0, 0.0)}, [(1, 0.3, 0.4), (2, 5.0, 2.0)], 0.6)
    assert math.isclose(d, 0.5)
    assert hits == {('r1', 1)}


def test_no_peds_or_unknown_pose():
    d, hits = dgr.person_distances({'r1': (0.0, 0.0)}, [], 0.5)
    assert d == float('inf') and not hits
    d, hits = dgr.person_distances({'r1': None}, [(1, 0.0, 0.0)], 0.5)
    assert d == float('inf') and not hits
