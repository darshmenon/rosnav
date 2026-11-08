import os
import sys

import pytest

_pkg = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'tools', 'nav_explore_eval'))
sys.path.insert(0, _pkg)
os.environ['PYTHONPATH'] = _pkg + os.pathsep + os.environ.get('PYTHONPATH', '')


def pytest_collection_modifyitems(items):
    # vendored copy: packaging/version checks only make sense in the standalone repo
    for it in items:
        if it.name.startswith('test_version'):
            it.add_marker(pytest.mark.skip(reason='vendored copy, no package.xml'))
