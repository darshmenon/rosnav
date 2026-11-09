"""The real CLI against a fake SLAM graph (map->odom with a correction, moving odom, a frozen /map)."""
import json
import os
import subprocess
import sys
import time

import pytest

try:
    import rclpy
except ImportError:                                   # unit-only CI job has no ROS
    rclpy = None
pytestmark = pytest.mark.skipif(rclpy is None, reason='needs rclpy')
import slam_doctor
_DOM = 200 + os.getpid() % 25   # per-process ROS domain: never collides with a sim on a shared machine
os.environ['ROS_LOCALHOST_ONLY'] = '1'

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(slam_doctor.__file__)))   # repo root, or the vendored tools dir
HERE = os.path.abspath(__file__)
PP = ROOT + os.pathsep + os.environ.get('PYTHONPATH', '')


def _run_graph(domain, seconds, jump_at=None, freeze_map=True, move=True):
    """Publish a fake SLAM for `seconds`; returns when done."""
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSProfile
    from tf2_msgs.msg import TFMessage
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    rclpy.init()
    n = Node('fake_slam')
    tfp = n.create_publisher(TFMessage, '/tf', QoSProfile(depth=100))
    odp = n.create_publisher(Odometry, '/odom', 10)
    mpp = n.create_publisher(OccupancyGrid, '/map', QoSProfile(depth=1, durability=1, reliability=1))
    t0 = time.monotonic()

    def pub_map(free):
        m = OccupancyGrid(); m.info.width = m.info.height = 20; m.info.resolution = 0.05
        m.data = [0] * free + [-1] * (400 - free)
        mpp.publish(m)
    pub_map(100)

    def tick():
        t = time.monotonic() - t0
        tf = TransformStamped(); tf.header.frame_id, tf.child_frame_id = 'map', 'odom'
        tf.transform.translation.x = 0.4 if (jump_at is not None and t > jump_at) else 0.0
        tf.transform.rotation.w = 1.0
        tfp.publish(TFMessage(transforms=[tf]))
        o = Odometry(); o.pose.pose.position.x = t * 1.0 if move else 0.0
        odp.publish(o)
        if not freeze_map:
            pub_map(100 + int(t * 10))
    n.create_timer(0.1, tick)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
    n.destroy_node(); rclpy.shutdown()


def start_graph(domain, seconds, **kw):
    code = ('import importlib.util as u; s = u.spec_from_file_location("ti", %r); m = u.module_from_spec(s); '
            's.loader.exec_module(m); m._run_graph(%d, %s, **%r)' % (HERE, domain, seconds, kw))
    return subprocess.Popen([sys.executable, '-c', code], cwd=ROOT, env=dict(os.environ, PYTHONPATH=PP))


def doctor(domain, secs, *extra):
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), PYTHONPATH=PP)
    r = subprocess.run([sys.executable, '-m', 'slam_doctor.doctor', '--json', '--once', str(secs), *extra],
                       cwd=ROOT, env=env, capture_output=True, text=True, timeout=60)
    lines = [json.loads(x) for x in r.stdout.splitlines() if x.startswith('{')]
    return r, lines[-1]


def test_correction_is_logged_and_events_file_written(tmp_path):
    th = start_graph(_DOM, 16, jump_at=6.0, freeze_map=False); time.sleep(1.5)
    ev = tmp_path / 'ev.jsonl'
    r, rep = doctor(_DOM, 10, '--events', str(ev))
    th.wait()
    assert rep['summary']['corrections'] >= 1 and rep['summary']['largest_correction_m'] >= 0.35
    assert any('"correction"' in l for l in ev.read_text().splitlines())
    assert r.returncode == 0, r.stderr


def test_frozen_map_while_moving_is_an_error():
    import slam_doctor.core as c
    th = start_graph(_DOM + 1, 13); time.sleep(1.5)
    env_stall = dict(os.environ, ROS_DOMAIN_ID=str(_DOM + 1), PYTHONPATH=PP)
    code = ("import slam_doctor.core as c; c.STALL_S=3.0; c.STALL_TRAVEL_M=1.0; "
            "import sys; from slam_doctor.doctor import main; sys.exit(main(['--json','--once','8']))")
    r = subprocess.run([sys.executable, '-c', code], cwd=ROOT, env=env_stall, capture_output=True, text=True, timeout=60)
    th.wait()
    rep = json.loads([x for x in r.stdout.splitlines() if x.startswith('{')][-1])
    assert 'MAP_STALL' in {f['code'] for f in rep['findings']} and r.returncode == 1, r.stdout


def test_version():
    r = subprocess.run([sys.executable, '-m', 'slam_doctor.doctor', '--version'], cwd=ROOT,
                       env=dict(os.environ, PYTHONPATH=PP), capture_output=True, text=True)
    assert r.stdout.strip() == 'slam_doctor 0.1.0'


def test_versions_match():
    import re
    from slam_doctor import __version__
    assert re.search(r'<version>(.*?)</version>', open(os.path.join(ROOT, 'package.xml')).read()).group(1) == __version__
    assert f"version='{__version__}'" in open(os.path.join(ROOT, 'setup.py')).read()
