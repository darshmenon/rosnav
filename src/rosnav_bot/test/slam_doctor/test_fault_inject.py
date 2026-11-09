"""nav2_fault_inject freezes the SLAM process; slam_doctor must raise TRACKING_LOST and recover (needs nav2_fault_inject)."""
import os
import subprocess
import sys
import time

import pytest

try:
    import nav2_fault_inject
    import rclpy  # noqa: F401
except ImportError:                                   # optional: only runs where nav2_fault_inject is installed
    nav2_fault_inject = None
import slam_doctor

pytestmark = pytest.mark.skipif(nav2_fault_inject is None, reason='needs nav2_fault_inject and rclpy')

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(slam_doctor.__file__)))
FI = os.path.dirname(os.path.dirname(os.path.abspath(nav2_fault_inject.__file__))) if nav2_fault_inject else ''
_DOM = 200 + os.getpid() % 25   # per-process ROS domain: never collides with a sim on a shared machine
os.environ['ROS_LOCALHOST_ONLY'] = '1'
PP = os.pathsep.join([ROOT, FI, os.environ.get('PYTHONPATH', '')])
GRAPH = '''
import importlib.util as u
s = u.spec_from_file_location("ti", %r); m = u.module_from_spec(s); s.loader.exec_module(m)
m._run_graph(%d, 40, freeze_map=False)
'''


def test_frozen_slam_is_flagged_then_recovers(tmp_path):
    dom = str(_DOM)
    env = dict(os.environ, ROS_DOMAIN_ID=dom, PYTHONPATH=PP)
    graph_py = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'test_integration.py')
    victim = subprocess.Popen([sys.executable, '-c', GRAPH % (graph_py, int(dom)), '--ros-args', '-r', '__node:=fake_slam'],
                              env=env, cwd=ROOT)
    try:
        time.sleep(2.0)
        scen = tmp_path / 's.yaml'
        scen.write_text('faults:\n - {at: 3, node: fake_slam, for: 8, expect: {detect: [TRACKING_LOST], within: 12, clear_within: 12}}\n')
        r = subprocess.run([sys.executable, '-m', 'nav2_fault_inject.cli', '--domain', dom, 'run', str(scen), '--check',
                            '--doctor-cmd', f'{sys.executable} -m slam_doctor.doctor --json --period 1'],
                           env=env, cwd=ROOT, capture_output=True, text=True, timeout=90)
    finally:
        victim.kill(); victim.wait()
    assert r.returncode == 0, r.stdout + r.stderr
    assert 'PASS' in r.stdout and 'detect TRACKING_LOST' in r.stdout
