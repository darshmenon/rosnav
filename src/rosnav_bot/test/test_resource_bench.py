import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
import resource_bench as rb  # noqa: E402


class ResourceBench(unittest.TestCase):
    def test_classify(self):
        self.assertEqual(rb.classify('/opt/ros/humble/lib/nav2_controller/controller_server'), 'nav2')
        self.assertEqual(rb.classify('rviz2 -d x.rviz'), 'rviz')
        self.assertEqual(rb.classify('python3 foo.py'), 'other')

    def test_summarise(self):
        s = rb.summarise([{'nav2': (10.0, 100.0)}, {'nav2': (30.0, 120.0), 'rviz': (5.0, 50.0)}])
        self.assertEqual(s['nav2'], (20.0, 120.0))
        self.assertEqual(s['rviz'], (2.5, 50.0))

    def test_domain_filter_never_matches_other_instance(self):
        self.assertEqual(rb.domain_of({'ROS_DOMAIN_ID': '7'}), '7')
        self.assertNotEqual(rb.domain_of({}), '7')


if __name__ == '__main__':
    unittest.main()
