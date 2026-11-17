import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'launch'))
import _common  # noqa: E402

CFG = os.path.join(HERE, '..', 'config')


def _load(name):
    return yaml.safe_load(open(os.path.join(CFG, name)))


def test_dwb_caps_raised_and_smoother_matches():
    d = _load('nav2_params.yaml')
    ch = _common.max_speed_patch(d, 0.5)
    fp = d['controller_server']['ros__parameters']['FollowPath']
    assert fp['max_vel_x'] == 0.5 and fp['max_speed_xy'] == 0.5
    assert d['velocity_smoother']['ros__parameters']['max_velocity'][0] == 0.5
    assert ch


def test_mppi_cap_raised():
    d = _load('nav2_params_mppi.yaml')
    _common.max_speed_patch(d, 0.5)
    assert d['controller_server']['ros__parameters']['FollowPath']['vx_max'] == 0.5


def test_noop_when_already_at_speed():
    d = _load('nav2_params.yaml')
    _common.max_speed_patch(d, 0.5)
    assert _common.max_speed_patch(d, 0.5) == []
