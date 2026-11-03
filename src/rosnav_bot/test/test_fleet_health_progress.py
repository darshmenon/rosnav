#!/usr/bin/env python3
"""fleet_health.progress_stalled: pure stall rule (rclpy import only, no graph)."""
import importlib.util
import os

_spec = importlib.util.spec_from_file_location(
    'fleet_health', os.path.join(os.path.dirname(__file__), '..', 'scripts', 'fleet_health.py'))
fh = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fh)


def test_stalled_when_frontiers_remain_and_nothing_changed():
    assert fh.progress_stalled(True, 61.0, 27, 60.0)


def test_not_stalled_before_the_window_ends():
    assert not fh.progress_stalled(True, 59.0, 27, 60.0)


def test_done_exploring_is_not_a_stall():
    assert not fh.progress_stalled(True, 600.0, 0, 60.0)


def test_no_stats_yet_is_not_a_stall():
    assert not fh.progress_stalled(False, 600.0, 27, 60.0)
