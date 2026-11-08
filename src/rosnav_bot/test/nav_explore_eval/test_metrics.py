import json, os, subprocess, sys
import numpy as np
import pytest
ROOT = os.path.join(os.path.dirname(__file__), '..')
sys.path.insert(0, ROOT)
from nav_explore_eval.mapio import Grid, load_yaml_map
from nav_explore_eval.metrics import align, path_length, resample, score, time_to_percent


def room(n=60, res=0.1, ox=0.0, oy=0.0):
    d = np.zeros((n, n), dtype=np.int8)
    d[0, :] = d[-1, :] = d[:, 0] = d[:, -1] = 100
    d[20, 10:40] = 100; d[20:40, 40] = 100            # inner walls
    return Grid(d, res, ox, oy)


def unknown_like(g):
    return Grid(np.full_like(g.data, -1), g.resolution, g.ox, g.oy)


def test_perfect_score():
    g = room(); s = score(g, g)
    assert s['coverage_pct'] == 100 and s['wall_f1_pct'] == 100 and s['false_free_pct'] == 0


def test_empty_map_scores_zero():
    s = score(unknown_like(room()), room())
    assert s['coverage_pct'] == 0 and s['wall_recall_pct'] == 0


def test_half_explored_is_about_half():
    g = room(); h = Grid(g.data.copy(), g.resolution, g.ox, g.oy); h.data[:, 30:] = -1
    s = score(h, g)
    assert 40 < s['coverage_pct'] < 60


def test_false_free_detected():
    g = room(); h = Grid(g.data.copy(), g.resolution, g.ox, g.oy)
    h.data[20, 10:40] = 0; h.data[19, 10:40] = 0; h.data[21, 10:40] = 0     # inner wall erased
    assert score(h, g)['false_free_pct'] > 10


def test_spurious_walls_hurt_precision():
    g = room(); h = Grid(g.data.copy(), g.resolution, g.ox, g.oy); h.data[45:50, 5:50] = 100
    s = score(h, g)
    assert s['wall_precision_pct'] < 90 and s['wall_recall_pct'] > 95


def test_align_recovers_shift():
    t = room()
    live = Grid(t.data.copy(), t.resolution, t.ox + 0.7, t.oy - 0.4)      # same map, origin offset
    dx, dy = align(live, t)
    s = score(live, t, dx, dy)
    assert s['wall_f1_pct'] > 95 and s['coverage_pct'] > 95
    assert score(live, t)['wall_f1_pct'] < 60                             # un-aligned is clearly worse


def test_resample_different_resolution():
    t = room(); live = Grid(np.kron(t.data, np.ones((2, 2), dtype=np.int8)), 0.05, 0.0, 0.0)
    assert score(live, t)['wall_f1_pct'] > 99


def test_time_to_percent_interpolates():
    series = [(0, {'coverage_pct': 0}), (10, {'coverage_pct': 40}), (20, {'coverage_pct': 80})]
    tt = time_to_percent(series)
    assert tt['25'] == pytest.approx(6.25) and tt['50'] == pytest.approx(12.5) and tt['90'] is None


def test_path_length():
    assert path_length([(0, 0), (3, 4), (3, 8)]) == pytest.approx(9.0) and path_length([(1, 1)]) == 0


def write_map(tmp, name, g, flip=True):
    img = np.where(g.data == 100, 0, np.where(g.data == 0, 254, 205)).astype(np.uint8)[::-1]
    p = tmp / f'{name}.pgm'
    p.write_bytes(b'P5\n%d %d\n255\n' % (img.shape[1], img.shape[0]) + img.tobytes())
    y = tmp / f'{name}.yaml'
    y.write_text(f'image: {name}.pgm\nresolution: {g.resolution}\norigin: [{g.ox}, {g.oy}, 0.0]\n'
                 'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    return str(y)


def test_pgm_roundtrip(tmp_path):
    g = room(); back = load_yaml_map(write_map(tmp_path, 'm', g))
    assert (back.data == g.data).all() and back.resolution == g.resolution


def test_cli_score_and_compare(tmp_path):
    g = room(); a = write_map(tmp_path, 'a', g)
    shifted = Grid(g.data, g.resolution, 0.5, 0.3); b = write_map(tmp_path, 'b', shifted)
    out = subprocess.run([sys.executable, '-m', 'nav_explore_eval.cli', 'score', b, a, '--align', '--json'],
                         capture_output=True, text=True, cwd=ROOT)
    assert out.returncode == 0 and json.loads(out.stdout)['wall_f1_pct'] > 95
    rep = {'label': 'x', 'final': score(g, g), 'time_to_pct_s': time_to_percent([(0, {'coverage_pct': 0}), (5, {'coverage_pct': 100})]),
           'path_length_m': 12.0, 'series': [{'t': 0, 'coverage_pct': 0}, {'t': 5, 'coverage_pct': 100}]}
    p = tmp_path / 'x.json'; p.write_text(json.dumps(rep))
    out = subprocess.run([sys.executable, '-m', 'nav_explore_eval.cli', 'compare', str(p)],
                         capture_output=True, text=True, cwd=ROOT)
    assert 'x' in out.stdout and '100.0' in out.stdout and out.returncode == 0


def test_version():
    out = subprocess.run([sys.executable, '-m', 'nav_explore_eval.cli', '--version'], capture_output=True, text=True, cwd=ROOT)
    assert out.stdout.startswith('nav_explore_eval ')


def test_versions_match():
    import re
    from nav_explore_eval import __version__
    pkg = re.search(r'<version>(.*?)</version>', open(os.path.join(ROOT, 'package.xml')).read()).group(1)
    setup = re.search(r"version='(.*?)'", open(os.path.join(ROOT, 'setup.py')).read()).group(1)
    assert __version__ == pkg == setup
