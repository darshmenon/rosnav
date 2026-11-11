"""collision_monitor relay: slowdown must scale every velocity axis (mecanum strafe used to be dropped)."""
import importlib.util
import os

from geometry_msgs.msg import Twist

_SCRIPT = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'collision_monitor.py')
_spec = importlib.util.spec_from_file_location('collision_monitor', _SCRIPT)
cm = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(cm)


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, m):
        self.sent.append(m)


def _monitor(state, vel):
    m = cm.CollisionMonitor.__new__(cm.CollisionMonitor)
    m._state, m._factor, m._sensor_dead = state, 0.4, False
    m._relay_vel, m._cmd_pub = vel, _Pub()
    return m


def _twist(x=0.0, y=0.0, z=0.0):
    t = Twist()
    t.linear.x, t.linear.y, t.angular.z = x, y, z
    return t


def test_slowdown_scales_strafe_too():
    m = _monitor(cm.SLOWDOWN, _twist(0.5, 0.3, 1.0))
    m._publish_relay()
    out = m._cmd_pub.sent[-1]
    assert abs(out.linear.x - 0.2) < 1e-9 and abs(out.linear.y - 0.12) < 1e-9 and abs(out.angular.z - 0.4) < 1e-9


def test_stop_blocks_forward_keeps_reverse_and_turn():
    m = _monitor(cm.STOP, _twist(0.5, 0.0, 1.0))
    m._publish_relay()
    out = m._cmd_pub.sent[-1]
    assert out.linear.x == 0.0 and out.angular.z == 1.0
    m = _monitor(cm.STOP, _twist(0.5, 0.5, 0.0))
    m._publish_relay()
    out = m._cmd_pub.sent[-1]
    assert out.linear.x == 0.0 and abs(out.linear.y - 0.2) < 1e-9     # strafe allowed, slowed
    m = _monitor(cm.STOP, _twist(-0.2, 0.0, 0.0))
    m._publish_relay()
    assert m._cmd_pub.sent[-1].linear.x == -0.2


def test_clear_passes_everything():
    v = _twist(0.5, 0.3, 1.0)
    m = _monitor(cm.CLEAR, v)
    m._publish_relay()
    assert m._cmd_pub.sent[-1] is v
