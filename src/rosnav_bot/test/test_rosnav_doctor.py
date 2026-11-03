#!/usr/bin/env python3
"""scripts/rosnav_doctor.py diagnose(): pure logic, no ROS needed."""
import importlib.util
import os
import sys

_spec = importlib.util.spec_from_file_location(
    'rosnav_doctor', os.path.join(os.path.dirname(__file__), '..', 'scripts', 'rosnav_doctor.py'))
doc = importlib.util.module_from_spec(_spec)
sys.modules['rosnav_doctor'] = doc
_spec.loader.exec_module(doc)

OK = dict(tf_map_odom_age=0.1, tf_odom_base_age=0.05, nav2_active=True, odom_hz=50.0, scan_hz=10.0, slam=False)


def _msgs(**kw):
    return [m for _, m in doc.diagnose({**OK, **kw})]


def test_healthy_robot():
    assert doc.diagnose(OK) == [('OK', 'healthy')]


def test_stale_amcl_map_odom_points_to_static_localization():
    f = doc.diagnose({**OK, 'tf_map_odom_age': 5.0})
    assert doc.worst(f) == 'ERROR' and 'explore_localization:=static' in f[0][1]


def test_stale_slam_map_odom_blames_slam():
    f = doc.diagnose({**OK, 'tf_map_odom_age': 5.0, 'slam': True})
    assert 'SLAM' in f[0][1]


def test_missing_tfs_are_errors():
    assert any('odom->base_link TF missing' in m for m in _msgs(tf_odom_base_age=None))
    assert any('map->odom TF missing' in m for m in _msgs(tf_map_odom_age=None))


def test_inactive_nav2_names_the_watchdog():
    assert any('bringup aborted' in m for m in _msgs(nav2_active=False))


def test_low_rates_mean_overloaded_sim():
    assert any('overloaded' in m for m in _msgs(odom_hz=2.0))
    assert any('no odometry' in m for m in _msgs(odom_hz=0.0))
    assert any('no laser scan' in m for m in _msgs(scan_hz=0.0))


def test_static_map_warns_only_for_slam_robot():
    assert doc.diagnose({**OK, 'slam': True}, map_growing=False)[0][0] == 'WARN'
    assert doc.diagnose(OK, map_growing=False) == [('OK', 'healthy')]
