"""namespace_nav2_params must namespace every costmap observation source, not just `scan`."""
import importlib.util
import os
import sys

import pytest

_L = os.path.join(os.path.dirname(__file__), '..', 'launch')
sys.path.insert(0, _L)
try:
    spec = importlib.util.spec_from_file_location('_common', os.path.join(_L, '_common.py'))
    _common = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(_common)
except Exception as e:  # launch deps missing
    pytest.skip(f'cannot import _common: {e}', allow_module_level=True)


def _params():
    layer = {'observation_sources': 'scan points depth',
             'scan': {'topic': '/scan'}, 'points': {'topic': '/points'},
             'depth': {'topic': '/camera/depth/points'}}
    return {c: {c: {'ros__parameters': {'obstacle_layer': {k: (dict(v) if isinstance(v, dict) else v)
                                                            for k, v in layer.items()}}}}
            for c in ('global_costmap', 'local_costmap')}


def test_all_observation_sources_are_namespaced():
    p = _common.namespace_nav2_params(_params(), 'robot3')
    for c in ('global_costmap', 'local_costmap'):
        layer = p[c][c]['ros__parameters']['obstacle_layer']
        assert layer['scan']['topic'] == '/robot3/scan'
        assert layer['points']['topic'] == '/robot3/points'
        assert layer['depth']['topic'] == '/robot3/camera/depth/points'
