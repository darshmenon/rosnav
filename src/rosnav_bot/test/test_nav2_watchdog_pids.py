"""nav2_bringup_watchdog._manager_pids must find only this domain's manager for this robot."""
import importlib.util
import os
import subprocess
import sys
import time

_DOM = str(200 + os.getpid() % 25)   # per-process ROS domain, never collides with a sim on a shared machine

_SCRIPT = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'nav2_bringup_watchdog.py')


def _load():
    pytest = __import__('pytest')
    pytest.importorskip('rclpy')
    pytest.importorskip('nav2_msgs')
    spec = importlib.util.spec_from_file_location('nav2_bringup_watchdog', _SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.Nav2BringupWatchdog._manager_pids


def _spawn(domain, ns):
    env = dict(os.environ, ROS_DOMAIN_ID=domain)
    return subprocess.Popen(
        [sys.executable, '-c', 'import time; time.sleep(30)', '--ros-args',
         '-r', '__node:=lifecycle_manager_navigation', '-r', f'__ns:=/{ns}'], env=env)


def test_manager_pids_matches_only_own_domain_and_robot(monkeypatch):
    find = _load()
    monkeypatch.setenv('ROS_DOMAIN_ID', _DOM)
    mine = _spawn(_DOM, 'robotT')
    other_domain = _spawn('99', 'robotT')
    other_robot = _spawn(_DOM, 'robotU')
    try:
        time.sleep(0.5)
        assert find(None, 'robotT') == [mine.pid]
        assert find(None, 'robotX') == []
    finally:
        for p in (mine, other_domain, other_robot):
            p.kill()
            p.wait()
