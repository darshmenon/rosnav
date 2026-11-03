"""Per-robot drive_type resolution in multi_robot.launch.py (no sim needed)."""
import importlib.util
import os

import pytest

_P = os.path.join(os.path.dirname(__file__), '..', 'launch', 'multi_robot.launch.py')
_s = importlib.util.spec_from_file_location('multi_robot_launch', _P)
ml = importlib.util.module_from_spec(_s)
_s.loader.exec_module(ml)


def test_normalise_carries_overrides():
    r = ml._normalise_robot({'x': 1, 'y': 2, 'drive_type': 'Mecanum'}, 0)
    assert r['drive_type'] == 'mecanum' and r['controller'] == ''


def test_profile_per_robot_beats_fleet():
    a = ml._normalise_robot({'x': 0, 'y': 0, 'drive_type': 'ackermann'}, 0)
    b = ml._normalise_robot({'x': 1, 'y': 0}, 1)
    pa = ml._robot_profile(a, '/pkg', 'diff', 'dwb', 'custom')
    pb = ml._robot_profile(b, '/pkg', 'diff', 'dwb', 'custom')
    assert pa['urdf'] == 'robot_ackermann.urdf.xacro'
    assert pb['urdf'] == 'robot.urdf.xacro'
    assert pa['template'] != pb['template']


def test_bad_drive_type_rejected():
    r = ml._normalise_robot({'x': 0, 'y': 0, 'drive_type': 'hover'}, 0)
    with pytest.raises(ValueError):
        ml._robot_profile(r, '/pkg', 'diff', 'dwb', 'custom')


def test_inject_sim_time_reaches_nested_costmap_params():
    params = {'bt_navigator': {'ros__parameters': {'a': 1}},
              'local_costmap': {'local_costmap': {'ros__parameters': {}}}}
    ml._inject_sim_time(params)
    assert params['bt_navigator']['ros__parameters']['use_sim_time'] is True
    assert params['local_costmap']['local_costmap']['ros__parameters']['use_sim_time'] is True
