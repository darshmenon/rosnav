#!/usr/bin/env python3
"""scripts/rosnav_top.py render(): pure, no ROS needed (rich only)."""
import importlib.util
import os
import sys
import types

# rosnav_top imports rclpy lazily inside main(), so the module itself loads without ROS.
_spec = importlib.util.spec_from_file_location(
    'rosnav_top', os.path.join(os.path.dirname(__file__), '..', 'scripts', 'rosnav_top.py'))
top = importlib.util.module_from_spec(_spec)
sys.modules['rosnav_top'] = top
_spec.loader.exec_module(top)

from rich.console import Console

SNAP = {'mitigate': True,
        'robots': {'robot1': {'level': 'ERROR', 'findings': [['ERROR', 'map->odom stale (3.0s)']], 'mitigations': 2},
                   'robot2': {'level': 'OK', 'findings': [['OK', 'healthy']], 'mitigations': 0}},
        'metrics': {'robot1': {'odom_hz': 49.9, 'scan_hz': 10.0, 'tf_map_odom_age': 3.0, 'nav2_active': True},
                    'robot2': {'odom_hz': 50.0, 'scan_hz': 10.0, 'tf_map_odom_age': None, 'nav2_active': False}},
        'timeline': [{'t': 12.0, 'robot': 'robot1', 'event': 'MITIGATE clear_costmaps (#1)'}]}
PROCS = [{'name': 'controller_server', 'cpu': 12.0, 'rss_mb': 80.0},
         {'name': 'gz sim', 'cpu': 200.0, 'rss_mb': 900.0},
         {'name': 'slam_toolbox', 'cpu': 30.0, 'rss_mb': 150.0}]


def _text(r):
    c = Console(record=True, width=140, file=open(os.devnull, 'w'))
    c.print(r)
    return c.export_text()


def test_group_procs_first_match_and_skips_empty():
    g = top.group_procs(PROCS)
    assert g['Nav2']['cpu'] == 12.0 and g['SLAM']['n'] == 1 and g['Gazebo']['rss_mb'] == 900.0
    assert 'RViz' not in g


def test_render_shows_health_timeline_and_totals():
    out = _text(top.render(SNAP, PROCS))
    assert 'robot1' in out and 'ERROR' in out and 'MITIGATE clear_costmaps' in out
    assert 'DOWN' in out and 'total' in out and '1130' in out     # 80+900+150


def test_render_waiting_and_dry_run_and_silent():
    assert 'waiting' in _text(top.render(None, []))
    out = _text(top.render({**SNAP, 'mitigate': False}, PROCS, age_s=20))
    assert 'DRY RUN' in out and 'silent' in out
