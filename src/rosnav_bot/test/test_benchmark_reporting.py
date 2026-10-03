#!/usr/bin/env python3
"""Fast tests for benchmark reporting and static exploration assets.

These tests intentionally avoid Gazebo/ROS graph startup. They protect the
offline tooling and world/config assets used to compare exploration backends.
"""

from __future__ import annotations

import csv
import importlib.util
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


_ROOT = Path(__file__).resolve().parents[3]
_SCRIPT = _ROOT / 'src' / 'rosnav_bot' / 'scripts' / 'gen_slam_explorer_compare.py'
_spec = importlib.util.spec_from_file_location('gen_slam_explorer_compare', _SCRIPT)
reporting = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(reporting)


def test_infer_parts_handles_explore_lite_labels():
    parts = reporting.infer_parts('explore_lite_house_v2fix')
    assert parts['explorer'] == 'explore_lite'
    assert parts['world'] == 'house'
    assert parts['slam_algo'] == '2d'


def test_read_json_extracts_slam_metrics(tmp_path):
    path = tmp_path / 'explore_lite_house_v2fix_slam.json'
    path.write_text(json.dumps({
        'label': 'explore_lite_house_v2fix',
        'mode': 'slam',
        'duration_sec': 120,
        'final_coverage_pct': 26.13,
        'time_to_converge_sec': 5.5,
        'coverage_timeline': [{'t': 0.0, 'coverage_pct': 20.0}],
    }))

    rows = reporting.read_json(path)

    assert len(rows) == 1
    assert rows[0]['explorer'] == 'explore_lite'
    assert rows[0]['world'] == 'house'
    assert rows[0]['coverage_pct'] == 26.13
    assert rows[0]['timeline'][0]['coverage_pct'] == 20.0


def test_read_csv_skips_world_only_and_malformed_rows(tmp_path):
    path = tmp_path / 'batch_results.csv'
    path.write_text(
        'world,start_epoch,launch_ok,final_free_cells,final_total_cells,final_pct,offgrid_errors,elapsed_sec\n'
        'coverage_100,1,1,100,100,100,0,20\n'
        '0,211,1,80,0\n'
    )

    assert reporting.read_csv(path) == []

    combined = tmp_path / 'batch_results_combined.csv'
    combined.write_text(
        'explorer,slam_algo,world,start_epoch,launch_ok,final_free_cells,final_total_cells,final_pct,offgrid_errors,elapsed_sec\n'
        'explore_lite,2d,coverage_100,1,1,100,100,100,0,20\n'
        'explore_lite,2d,aws_warehouse,1,0,,,,,\n'
    )

    rows = reporting.read_csv(combined)

    assert len(rows) == 1
    assert rows[0]['explorer'] == 'explore_lite'
    assert rows[0]['world'] == 'coverage_100'
    assert rows[0]['coverage_pct'] == 100.0
    assert rows[0]['elapsed_sec'] == 20.0


def test_write_summary_contains_expected_metrics(tmp_path):
    out = tmp_path / 'summary.csv'
    rows = [{
        'label': 'explore_lite_2d_coverage_100',
        'world': 'coverage_100',
        'explorer': 'explore_lite',
        'slam_algo': '2d',
        'mode': 'batch',
        'coverage_pct': 100.0,
        'elapsed_sec': 20.0,
        'source': 'synthetic.csv',
    }]

    reporting.write_summary(rows, out)

    with out.open(newline='') as f:
        parsed = list(csv.DictReader(f))
    assert parsed[0]['coverage_pct'] == '100'
    assert parsed[0]['elapsed_sec'] == '20'
    assert parsed[0]['source'] == 'synthetic.csv'


def test_coverage_100_world_is_small_asymmetric_and_parseable():
    path = _ROOT / 'src' / 'rosnav_bot' / 'worlds' / 'coverage_100.world'
    root = ET.parse(path).getroot()
    world = root.find('world')
    assert world is not None
    assert world.attrib['name'] == 'coverage_100'

    text = path.read_text()
    assert '<size>11.0 9.0 0.004</size>' in text
    assert "landmark_north_tab" in text
    assert "landmark_south_tab" in text
    assert "landmark_east_tab" in text
    assert "landmark_west_tab" in text


def test_multi_terrain_robot_diff_world_is_parseable_and_benchmark_ready():
    path = _ROOT / 'src' / 'rosnav_bot' / 'worlds' / 'multi_terrain_robot_diff.world'
    root = ET.parse(path).getroot()
    world = root.find('world')
    assert world is not None
    assert world.attrib['name'] == 'multi_terrain_robot_diff'

    text = path.read_text()
    for name in (
        'terrain_flat_concrete',
        'terrain_gravel_rough',
        'terrain_low_friction_tile',
        'terrain_wide_asphalt',
        'diff_chicane_gate_a',
        'mecanum_offset_gate_north',
        'ackermann_wide_bollard_a',
        'exit_marker_ackermann_lane',
    ):
        assert name in text


def test_fast_stable_slam_profile_disables_loop_closure():
    path = _ROOT / 'src' / 'rosnav_bot' / 'config' / 'mapper_params_online_async_fast_stable.yaml'
    data = yaml.safe_load(path.read_text())
    params = data['slam_toolbox']['ros__parameters']

    assert params['do_loop_closing'] is False
    assert params['max_laser_range'] == 6.0
    assert params['minimum_travel_distance'] == 0.15


def test_launch_declares_slam_debug_and_coverage_spawn():
    path = _ROOT / 'src' / 'rosnav_bot' / 'launch' / 'slam_nav.launch.py'
    text = path.read_text()

    assert "name='slam_debug'" in text
    assert "slam_debug_monitor.py" in text
    assert "'coverage_100': {'x': '-1.6', 'y': '-0.8'" in text
    assert "'multi_terrain_robot_diff': {'x': '-9.2', 'y': '0.0'" in text
