"""slam_stall_watchdog restart decision (_tick) and /proc child lookup, with time and TF stubbed."""
import importlib.util
import os
import subprocess
import sys
import time

_SCRIPT = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'slam_stall_watchdog.py')
_spec = importlib.util.spec_from_file_location('slam_stall_watchdog', _SCRIPT)
sw = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sw)


class _Log:
    def info(self, *a, **k): pass
    warn = error = info


class _Dur:
    def __init__(self, s): self.nanoseconds = int(s * 1e9)


class _Clock:
    def __init__(self, up): self._up = up
    def now(self):
        class _N:
            def __sub__(s2, other): return _Dur(self._up)
        return _N()


def _wd(a_mo, a_ob=0.1, up=200.0, scan_age=0.5, restarts=0, max_restarts=5, seen=True):
    w = sw.SlamStallWatchdog.__new__(sw.SlamStallWatchdog)
    w.get_logger = lambda: _Log()
    w.get_clock = lambda: _Clock(up)
    w._start = 0
    w._cool, w._last_restart = 40.0, time.monotonic() - 1000.0
    w._stale, w._grace, w._max, w._restarts = 8.0, 90.0, max_restarts, restarts
    w._seen, w._last_log, w._warned_no_scan = seen, time.monotonic(), 0.0
    w._last_scan_wall = time.monotonic() - scan_age
    w._map_f, w._odom_f, w._base_f = 'map', 'odom', 'base_link'
    w._age = lambda p, c: a_mo if (p, c) == ('map', 'odom') else a_ob
    w._maybe_restore = lambda now: None
    w._remember_pose = lambda: None
    w.killed = []
    w._kill_slam = lambda a, u: w.killed.append((a, u))
    return w


def test_stale_map_odom_with_live_scans_restarts_slam():
    w = _wd(a_mo=12.0)
    w._tick()
    assert w.killed


def test_fresh_map_odom_does_nothing():
    w = _wd(a_mo=0.5)
    w._tick()
    assert not w.killed


def test_silent_scan_does_not_restart():
    w = _wd(a_mo=12.0, scan_age=10.0)      # problem is upstream of slam_toolbox
    w._tick()
    assert not w.killed


def test_dead_odometry_does_not_restart():
    w = _wd(a_mo=12.0, a_ob=30.0)
    w._tick()
    assert not w.killed


def test_gives_up_after_max_restarts():
    w = _wd(a_mo=12.0, restarts=5, max_restarts=5)
    w._tick()
    assert not w.killed


def test_never_seen_map_odom_after_grace_counts_as_hung():
    w = _wd(a_mo=None, up=120.0, seen=False)
    w._tick()
    assert w.killed
    w = _wd(a_mo=None, up=30.0, seen=False)    # still inside the startup grace
    w._tick()
    assert not w.killed


def test_children_of_finds_a_spawned_child():
    p = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(5)'])
    try:
        time.sleep(0.3)
        pids = [pid for pid, _ in sw._children_of(os.getpid())]
        assert p.pid in pids
    finally:
        p.kill()
