"""map_saver writes unknown as grey 205 (occupancy 0.196); a free_thresh of 0.25 loads it as FREE."""
import glob
import os
import re

import pytest
import yaml

ROOT = os.path.join(os.path.dirname(__file__), '..')
UNKNOWN_P = (255 - 205) / 255.0
MAPS = sorted(glob.glob(os.path.join(ROOT, 'maps', '*.yaml')) + glob.glob(os.path.join(ROOT, 'test', 'data', '*.yaml')))


def _read(path):
    with open(path, errors='replace') as f:
        return f.read()


@pytest.mark.parametrize('path', MAPS, ids=os.path.basename)
def test_unknown_grey_does_not_load_as_free(path):
    meta = yaml.safe_load(_read(path))
    if not isinstance(meta, dict) or 'image' not in meta or meta.get('mode') in ('scale', 'raw'):
        pytest.skip('not a trinary map')
    assert meta['free_thresh'] <= UNKNOWN_P


def test_map_writers_use_a_safe_free_threshold():
    bad = []
    for path in glob.glob(os.path.join(ROOT, 'scripts', '*.py')) + glob.glob(os.path.join(ROOT, 'config', '*.yaml')):
        for value in re.findall(r"free_thresh(?:_default)?'?:\s*(0\.\d+)", _read(path)):
            if float(value) > UNKNOWN_P:
                bad.append((os.path.basename(path), value))
    assert not bad


def test_map_saver_calls_set_the_free_threshold():
    """map_saver_cli writes free_thresh 0.25 into the yaml unless told otherwise."""
    bad = []
    for path in glob.glob(os.path.join(ROOT, 'scripts', '*.py')) + glob.glob(os.path.join(ROOT, 'scripts', '*.sh')):
        for line in _read(path).splitlines():
            if re.search(r"map_saver_cli['\"],|nav2_map_server map_saver_cli ", line) and '--free' not in line:
                bad.append((os.path.basename(path), line.strip()))
    assert not bad
