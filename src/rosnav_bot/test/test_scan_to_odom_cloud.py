"""scan_to_odom_cloud: scans stamped just ahead of the newest odom TF must still
produce a cloud (latest-TF fallback); scans far ahead must be skipped."""
import importlib.util
import os

import pytest

pytest.importorskip('rclpy')
import tf2_ros  # noqa: E402
from builtin_interfaces.msg import Time as TimeMsg  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402

_SCRIPT = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'scan_to_odom_cloud.py')
_spec = importlib.util.spec_from_file_location('scan_to_odom_cloud', _SCRIPT)
mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mod)


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, m):
        self.sent.append(m)


def _tf(parent, child, sec, nsec=0, static=False):
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = parent, child
    t.header.stamp = TimeMsg(sec=sec, nanosec=nsec)
    t.transform.rotation.w = 1.0
    return t


def _node():
    n = mod.ScanToOdomCloud.__new__(mod.ScanToOdomCloud)
    n._target, n._timeout, n._z, n._max_lag = 'odom', Duration(seconds=0.01), 0.1, 0.25
    n._n = n._fail = n._n_fallback = 0
    n._last_err = ''
    n._tf = tf2_ros.Buffer()
    n._pub = _Pub()
    n._tf.set_transform(_tf('odom', 'base_link', 10, 0), 'test')
    n._tf.set_transform(_tf('base_link', 'laser_frame', 0), 'test')
    n._tf.set_transform_static(_tf('base_link', 'laser_frame', 0), 'test')
    return n


def _scan(sec, nsec):
    m = LaserScan()
    m.header.frame_id = 'laser_frame'
    m.header.stamp = TimeMsg(sec=sec, nanosec=nsec)
    m.angle_min, m.angle_increment, m.range_min, m.range_max = 0.0, 0.1, 0.1, 10.0
    m.ranges = [1.0] * 10
    return m


def test_scan_slightly_ahead_uses_latest_tf():
    n = _node()
    n._on_scan(_scan(10, 8_000_000))
    assert n._n == 1 and n._n_fallback == 1 and n._fail == 0


def test_scan_far_ahead_is_skipped():
    n = _node()
    n._on_scan(_scan(11, 500_000_000))
    assert n._n == 0 and n._fail == 1 and 'extrapolation' in n._last_err.lower()
