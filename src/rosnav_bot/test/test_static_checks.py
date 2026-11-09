"""Run scripts/check_static.py (compile, YAML, URDF, worlds) as part of the normal test suite."""
import os
import subprocess
import sys

SCRIPT = os.path.join(os.path.dirname(__file__), '..', 'scripts', 'check_static.py')


def test_static_checks_pass():
    # launch files need the built workspace on the ROS path, so CI runs that group separately
    out = subprocess.run([sys.executable, SCRIPT, '--only', 'python', 'yaml', 'urdf', 'worlds'],
                         capture_output=True, text=True, timeout=240)
    assert out.returncode == 0, out.stdout[-2000:]
