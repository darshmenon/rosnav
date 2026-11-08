"""Launch files only fail at launch time; catch NameError-class bugs statically."""
import glob
import os
import subprocess
import sys

import pytest

pytest.importorskip('pyflakes')
LAUNCH = os.path.join(os.path.dirname(__file__), '..', 'launch')


@pytest.mark.parametrize('path', sorted(glob.glob(os.path.join(LAUNCH, '*.py'))), ids=os.path.basename)
def test_no_undefined_names(path):
    out = subprocess.run([sys.executable, '-m', 'pyflakes', path], capture_output=True, text=True).stdout
    bad = [l for l in out.splitlines() if 'undefined name' in l]
    assert not bad, '\n'.join(bad)
