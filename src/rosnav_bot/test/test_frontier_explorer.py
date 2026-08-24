#!/usr/bin/env python3
"""Unit tests for the pure-logic pieces of scripts/frontier_explorer.py.

Loaded by `colcon test`. FrontierExplorer.__init__() blocks on
wait_for_server() and needs a running ROS graph, so these tests never
construct a real node — they build a bare instance via __new__() and set
only the attributes each method under test actually reads. That keeps the
grid math, polygon test, scoring and session-checkpoint logic reproducible
and checkable without Gazebo/Nav2 running.
"""
import importlib.util
import json
import math
import os

import numpy as np
import pytest

_SCRIPT = os.path.join(
    os.path.dirname(__file__), '..', 'scripts', 'frontier_explorer.py')
_spec = importlib.util.spec_from_file_location('frontier_explorer', _SCRIPT)
frontier_explorer = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(frontier_explorer)
FrontierExplorer = frontier_explorer.FrontierExplorer


class _FakeLogger:
    def info(self, *a, **k):
        pass

    def warn(self, *a, **k):
        pass


def make_bare(**attrs):
    """A FrontierExplorer instance with __init__ skipped (no rclpy/Nav2
    needed) and only the given attributes set, for testing one method."""
    obj = FrontierExplorer.__new__(FrontierExplorer)
    obj.get_logger = _FakeLogger
    for k, v in attrs.items():
        setattr(obj, k, v)
    return obj


# ----------------------------------------------------------------------
# Exploration boundary (polygon parse + point-in-polygon)
# ----------------------------------------------------------------------
def test_parse_boundary_valid_square():
    pts = FrontierExplorer._parse_boundary([0.0, 0.0, 4.0, 0.0, 4.0, 4.0, 0.0, 4.0])
    assert pts == [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]


def test_parse_boundary_odd_length_drops_trailing_value():
    pts = FrontierExplorer._parse_boundary([0.0, 0.0, 4.0, 0.0, 4.0, 4.0, 0.0, 4.0, 9.0])
    assert pts == [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]


def test_parse_boundary_too_few_points_disables():
    assert FrontierExplorer._parse_boundary([0.0, 0.0, 1.0, 1.0]) == []


def test_parse_boundary_empty_disables():
    assert FrontierExplorer._parse_boundary([]) == []


def test_in_boundary_unbounded_when_no_polygon():
    explorer = make_bare(_boundary=[])
    assert explorer._in_boundary(1000.0, -1000.0) is True


def test_in_boundary_square():
    square = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]
    explorer = make_bare(_boundary=square)
    assert explorer._in_boundary(2.0, 2.0) is True
    assert explorer._in_boundary(5.0, 2.0) is False
    assert explorer._in_boundary(-1.0, 2.0) is False


# ----------------------------------------------------------------------
# Frontier grid geometry (static methods, no ROS needed)
# ----------------------------------------------------------------------
def test_frontier_mask_detects_only_free_cells_touching_unknown():
    free = np.ones((3, 3), dtype=bool)
    unknown = np.zeros((3, 3), dtype=bool)
    unknown[2, 2] = True
    mask = FrontierExplorer._frontier_mask(free, unknown)
    expected = np.zeros((3, 3), dtype=bool)
    expected[1, 2] = True
    expected[2, 1] = True
    np.testing.assert_array_equal(mask, expected)


def test_line_free_open_path():
    free = np.ones((5, 5), dtype=bool)
    assert FrontierExplorer._line_free(0, 0, 4, 4, free) is True


def test_line_free_blocked_by_obstacle():
    free = np.ones((5, 5), dtype=bool)
    free[2, 2] = False
    assert FrontierExplorer._line_free(0, 0, 4, 4, free) is False


def test_line_free_out_of_bounds_endpoint():
    free = np.ones((3, 3), dtype=bool)
    assert FrontierExplorer._line_free(0, 0, 10, 10, free) is False


def test_cell_clearance_finds_nearest_hit():
    mask = np.zeros((7, 7), dtype=bool)
    mask[3, 5] = True  # 2 cells to the right of (3, 3)
    dist = FrontierExplorer._cell_clearance(3, 3, mask, radius_cells=4, resolution=0.5)
    assert dist == pytest.approx(1.0)  # 2 cells * 0.5m


def test_cell_clearance_no_hit_returns_radius_plus_one():
    mask = np.zeros((5, 5), dtype=bool)
    dist = FrontierExplorer._cell_clearance(2, 2, mask, radius_cells=2, resolution=0.1)
    assert dist == pytest.approx(0.3)  # (2 + 1) * 0.1


# ----------------------------------------------------------------------
# FOV-cast information gain (info_gain_mode='fov')
# ----------------------------------------------------------------------
def _fov_explorer(**overrides):
    attrs = dict(
        _info_gain_fov=math.radians(60), _info_gain_max_depth=0.5,
        _info_gain_angular_res=math.radians(5))
    attrs.update(overrides)
    return make_bare(**attrs)


def test_info_gain_fov_counts_cell_ahead_in_cone():
    unknown = np.zeros((21, 21), dtype=bool)
    occupied = np.zeros((21, 21), dtype=bool)
    unknown[10, 12] = True  # 2 cells "ahead" (heading=0 -> +x/+col)
    explorer = _fov_explorer()
    gain = explorer._info_gain_fov_cast((10, 10), 0.0, unknown, occupied, resolution=0.1)
    assert gain == pytest.approx(0.01)  # exactly 1 cell * 0.1^2


def test_info_gain_fov_excludes_cell_behind_heading():
    unknown = np.zeros((21, 21), dtype=bool)
    occupied = np.zeros((21, 21), dtype=bool)
    unknown[10, 7] = True  # 3 cells "behind" heading=0 (a 60deg-wide forward cone misses it)
    explorer = _fov_explorer()
    gain = explorer._info_gain_fov_cast((10, 10), 0.0, unknown, occupied, resolution=0.1)
    assert gain == pytest.approx(0.0)


def test_info_gain_fov_stops_at_occlusion():
    unknown = np.zeros((21, 21), dtype=bool)
    occupied = np.zeros((21, 21), dtype=bool)
    occupied[10, 11] = True  # wall 1 cell ahead
    unknown[10, 13] = True   # unknown cell 3 cells ahead, behind the wall
    explorer = _fov_explorer(_info_gain_max_depth=0.5)
    gain = explorer._info_gain_fov_cast((10, 10), 0.0, unknown, occupied, resolution=0.1)
    assert gain == pytest.approx(0.0)  # occluded — never seen


def test_info_gain_fov_respects_heading_direction():
    unknown = np.zeros((21, 21), dtype=bool)
    occupied = np.zeros((21, 21), dtype=bool)
    unknown[10, 8] = True  # 2 cells to the left
    explorer = _fov_explorer()
    facing_left = explorer._info_gain_fov_cast(
        (10, 10), math.pi, unknown, occupied, resolution=0.1)
    facing_right = explorer._info_gain_fov_cast(
        (10, 10), 0.0, unknown, occupied, resolution=0.1)
    assert facing_left == pytest.approx(0.01)
    assert facing_right == pytest.approx(0.0)


# ----------------------------------------------------------------------
# Scoring
# ----------------------------------------------------------------------
def test_suspicious_frontier_ratio_scales_with_size_over_clearance():
    explorer = make_bare(_suspect_buffer=1.0)
    small = explorer._suspicious_frontier_ratio(size_m=1.0, clearance=1.0)
    large = explorer._suspicious_frontier_ratio(size_m=10.0, clearance=0.0)
    assert large > small


def test_score_frontier_utility_prefers_closer_larger_frontier():
    explorer = make_bare(
        _scorer='utility', _suspect_ratio=3.0, _suspect_penalty=8.0,
        _suspect_buffer=1.0, _gain_scale=1.0, _potential_scale=3.0,
        _current_goal=None, _hyst_r=2.0, _hyst_gain=1.5,
    )
    near = {'size_m': 2.0, 'info_gain': 0.0, 'point': (1.0, 0.0), 'clearance': 1.0}
    far = {'size_m': 2.0, 'info_gain': 0.0, 'point': (10.0, 0.0), 'clearance': 1.0}
    assert explorer._score_frontier(near, 1.0) > explorer._score_frontier(far, 10.0)


def test_score_frontier_hysteresis_bonus_near_current_goal():
    explorer = make_bare(
        _scorer='utility', _suspect_ratio=3.0, _suspect_penalty=8.0,
        _suspect_buffer=1.0, _gain_scale=1.0, _potential_scale=3.0,
        _current_goal=(5.0, 0.0), _hyst_r=2.0, _hyst_gain=1.5,
    )
    close_to_current = {'size_m': 2.0, 'info_gain': 0.0, 'point': (5.5, 0.0), 'clearance': 1.0}
    far_from_current = {'size_m': 2.0, 'info_gain': 0.0, 'point': (5.5, 0.0), 'clearance': 1.0}
    with_bonus = explorer._score_frontier(close_to_current, 5.0)
    explorer._current_goal = None
    without_bonus = explorer._score_frontier(far_from_current, 5.0)
    assert with_bonus == pytest.approx(without_bonus + 1.5)


# ----------------------------------------------------------------------
# Session checkpoint (save / resume)
# ----------------------------------------------------------------------
def test_session_checkpoint_roundtrip(tmp_path):
    path = str(tmp_path / 'session.json')
    saver = make_bare(_session_path=path, _visited=[(1.0, 2.0), (3.0, 4.0)])
    saver._save_session_checkpoint()

    with open(path) as f:
        data = json.load(f)
    assert data == {'visited': [[1.0, 2.0], [3.0, 4.0]]}

    loader = make_bare(_session_path=path, _visited=[])
    loader._load_session()
    assert loader._visited == [(1.0, 2.0), (3.0, 4.0)]


def test_load_session_missing_file_leaves_visited_untouched():
    explorer = make_bare(_session_path='/nonexistent/path/session.json', _visited=[])
    explorer._load_session()
    assert explorer._visited == []


def test_save_session_checkpoint_noop_when_path_empty(tmp_path):
    explorer = make_bare(_session_path='', _visited=[(1.0, 1.0)])
    explorer._save_session_checkpoint()
    assert list(tmp_path.iterdir()) == []
