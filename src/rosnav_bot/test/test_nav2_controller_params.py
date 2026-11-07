"""Controller switch -> params file, and that each file declares the right Nav2 plugin."""
import os
import sys
import unittest

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'launch'))
import _common  # noqa: E402

CONFIG = os.path.join(HERE, '..', 'config')


def _plugin(fname):
    with open(os.path.join(CONFIG, fname)) as f:
        d = yaml.safe_load(f)
    return d['controller_server']['ros__parameters']['FollowPath']


class ControllerParams(unittest.TestCase):
    def test_selection(self):
        pick = lambda c: _common.nav2_params_filename(c, 'diff', 'humble')
        self.assertEqual(pick('rotation_shim'), 'nav2_params_rotation_shim.yaml')
        self.assertEqual(pick('graceful'), 'nav2_params_graceful.yaml')
        self.assertEqual(pick('rpp'), 'nav2_params_rpp.yaml')
        self.assertEqual(pick('dwb'), 'nav2_params.yaml')

    def test_plugins(self):
        self.assertEqual(_plugin('nav2_params_graceful.yaml')['plugin'],
                         'nav2_graceful_controller::GracefulController')
        shim = _plugin('nav2_params_rotation_shim.yaml')
        self.assertEqual(shim['plugin'], 'nav2_rotation_shim_controller::RotationShimController')
        self.assertIn('RegulatedPurePursuit', shim['primary_controller'])
        self.assertIn('lookahead_dist', shim)


if __name__ == '__main__':
    unittest.main()
