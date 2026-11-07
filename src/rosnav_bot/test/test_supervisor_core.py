#!/usr/bin/env python3
"""scripts/supervisor_core.py: pure policy logic, no ROS needed."""
import importlib.util
import os
import sys

_spec = importlib.util.spec_from_file_location(
    'supervisor_core', os.path.join(os.path.dirname(__file__), '..', 'scripts', 'supervisor_core.py'))
sc = importlib.util.module_from_spec(_spec)
sys.modules['supervisor_core'] = sc
_spec.loader.exec_module(sc)

STALE = [('ERROR', 'map->odom stale (3.0s): SLAM not integrating scans')]
OK = [('OK', 'healthy')]


def test_blip_is_ignored():
    s = sc.Supervisor(persist_s=5)
    assert s.step(0, {'r1': STALE}) == []
    assert s.step(2, {'r1': OK}) == []
    assert s.step(10, {'r1': STALE}) == []        # streak restarted


def test_persistent_stale_tf_clears_costmaps_once_per_cooldown():
    s = sc.Supervisor(persist_s=5, cooldown_s=30)
    s.step(0, {'r1': STALE})
    assert s.step(6, {'r1': STALE}) == [('clear_costmaps', 'r1')]
    assert s.step(10, {'r1': STALE}) == []        # cooldown
    assert s.step(40, {'r1': STALE}) == [('clear_costmaps', 'r1')]


def test_max_actions_gives_up_and_says_so():
    s = sc.Supervisor(persist_s=0, cooldown_s=0, max_actions=2)
    n = sum(len(s.step(t, {'r1': STALE})) for t in range(1, 10))
    assert n == 2
    assert any('gave up' in e['event'] for e in s.timeline)


def test_dead_odometry_is_report_only():
    s = sc.Supervisor(persist_s=0)
    acts = s.step(1, {'r1': [('ERROR', 'no odometry: bridge/EKF down'),
                             ('ERROR', 'odom->base_link TF missing')]})
    assert acts and acts[0][0] == 'log'


def test_robots_are_independent_and_snapshot_has_timeline():
    s = sc.Supervisor(persist_s=0)
    acts = s.step(1, {'r1': STALE, 'r2': OK})
    assert acts == [('clear_costmaps', 'r1')]
    snap = s.snapshot()
    assert snap['robots']['r2']['level'] == 'OK' and snap['robots']['r1']['mitigations'] == 1
    assert snap['timeline']


def test_diagnostic_fields():
    lvl, msg, kv = sc.to_diagnostic_fields('r1', 'WARN', [('WARN', 'x')], {'odom_hz': 49.99, 'a': None})
    assert lvl == 1 and msg == 'x' and ('odom_hz', '49.99') in kv and ('a', '') in kv
