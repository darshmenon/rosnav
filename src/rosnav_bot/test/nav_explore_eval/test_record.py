"""`record` against a fake /map that reveals the room left-to-right. Needs a sourced ROS 2 env."""
import json, os, subprocess, sys, threading, time
import numpy as np
import pytest
rclpy = pytest.importorskip('rclpy')
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

ROOT = os.path.join(os.path.dirname(__file__), '..')
sys.path.insert(0, os.path.dirname(__file__)); sys.path.insert(0, ROOT)
from test_metrics import room, write_map


class FakeSlam(Node):
    def __init__(self, g, speed):
        super().__init__('fake_slam')
        self.g, self.t0, self.speed = g, time.monotonic(), speed
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.create_timer(0.5, self.tick)

    def tick(self):
        t = time.monotonic() - self.t0
        w = self.g.shape[1]
        reveal = min(w, int(t * self.speed))
        d = self.g.data.copy(); d[:, reveal:] = -1
        m = OccupancyGrid()
        m.info.resolution, m.info.width, m.info.height = self.g.resolution, w, self.g.shape[0]
        m.info.origin.position.x, m.info.origin.position.y = self.g.ox + 0.5, self.g.oy + 0.5   # frame offset
        m.data = d.flatten().tolist()
        self.map_pub.publish(m)
        o = Odometry(); o.pose.pose.position.x = t * 0.5
        self.odom_pub.publish(o)


def test_record_reports_progress(tmp_path):
    g = room()
    truth = write_map(tmp_path, 'truth', g)
    os.environ['ROS_DOMAIN_ID'] = '171'
    rclpy.init(); node = FakeSlam(g, speed=6.0)            # 60 cols revealed in ~10s
    ex = SingleThreadedExecutor(); ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True); th.start()
    out = tmp_path / 'run.json'
    try:
        p = subprocess.run([sys.executable, '-m', 'nav_explore_eval.cli', 'record', '--truth', truth,
                            '--label', 'fake', '--out', str(out), '--align', '--interval', '1',
                            '--duration', '14', '--stop-at', '95'],
                           capture_output=True, text=True, cwd=ROOT, timeout=60)
    finally:
        ex.shutdown(); th.join(2); node.destroy_node(); rclpy.shutdown()
    assert p.returncode == 0, p.stdout + p.stderr
    r = json.loads(out.read_text())
    cov = [s['coverage_pct'] for s in r['series']]
    assert cov == sorted(cov) or max(cov) > 90                    # grows over time
    assert r['final']['coverage_pct'] > 85 and r['final']['wall_f1_pct'] > 85
    assert r['time_to_pct_s']['50'] is not None and r['time_to_pct_s']['25'] < r['time_to_pct_s']['50']
    assert r['path_length_m'] > 1.0 and r['label'] == 'fake'


def test_record_without_map_fails_cleanly(tmp_path):
    truth = write_map(tmp_path, 'truth', room())
    env = dict(os.environ, ROS_DOMAIN_ID='172')
    p = subprocess.run([sys.executable, '-m', 'nav_explore_eval.cli', 'record', '--truth', truth,
                        '--out', str(tmp_path / 'x.json'), '--duration', '3', '--interval', '1'],
                       capture_output=True, text=True, cwd=ROOT, env=env, timeout=30)
    assert p.returncode == 1 and 'no /map' in p.stderr
