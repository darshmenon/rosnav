"""Explore auto-save must never default into the package maps/ dir (that overwrote a committed map)."""
import os
import sys

LAUNCH = os.path.join(os.path.dirname(__file__), '..', 'launch')
sys.path.insert(0, LAUNCH)
import _common  # noqa: E402


def test_default_prefix_is_outside_the_package(monkeypatch, tmp_path):
    monkeypatch.setenv('HOME', str(tmp_path))
    p = _common.default_map_save_prefix('maze')
    assert p == str(tmp_path / '.ros' / 'rosnav_maps' / 'map_maze')
    assert os.path.isdir(os.path.dirname(p))


def test_launch_files_use_the_shared_default():
    for name in ('multi_robot.launch.py', 'slam_nav.launch.py'):
        with open(os.path.join(LAUNCH, name)) as f:
            src = f.read()
        assert '_common.default_map_save_prefix(' in src, name
        assert "'maps', f'map_{world_stem}')" not in src, name
