"""Yaw alignment, multi-robot, includes/meshes, stats/report/leaderboard, rosbag scoring."""
import json, os, subprocess, sys
import numpy as np
import pytest
ROOT = os.path.join(os.path.dirname(__file__), '..')
sys.path.insert(0, os.path.dirname(__file__)); sys.path.insert(0, ROOT)
from test_metrics import room, write_map
from test_worldmap import BOX, room as wroom, world
from nav_explore_eval.mapio import Grid
from nav_explore_eval.metrics import align_yaw, multi_score, resample, score, time_to_percent
from nav_explore_eval.runreport import build_report, validate
from nav_explore_eval.stats import aggregate, html_report, leaderboard, stats_rows
from nav_explore_eval.worldmap import mesh_bounds, rasterize, shapes


def cli(*args, **kw):
    return subprocess.run([sys.executable, '-m', 'nav_explore_eval.cli', *map(str, args)],
                          capture_output=True, text=True, cwd=ROOT, **kw)


# ---- yaw alignment
def test_resample_yaw_roundtrip_identity():
    g = room()
    assert (resample(g, g, 0, 0, 0.0) == resample(g, g, 0, 0, 1e-12)).mean() > 0.95


def test_align_yaw_recovers_rotation_and_shift():
    truth = room(n=120, res=0.05)
    yaw = np.deg2rad(6.0)
    # build a live map = truth rotated by -yaw about the origin and shifted, so aligning needs +yaw
    live = Grid(truth.data.copy(), truth.resolution, truth.ox + 0.3, truth.oy - 0.2)
    rotated = resample(live, truth, 0, 0, -yaw)
    live2 = Grid(rotated, truth.resolution, truth.ox, truth.oy)
    dx, dy, got = align_yaw(live2, truth, max_shift_m=1.0, max_yaw_deg=10, step_deg=1.0)
    assert abs(np.rad2deg(got) - 6.0) <= 1.0
    s = score(live2, truth, dx, dy, 0.10, got)
    assert s['wall_f1_pct'] > 80 and s['yaw_deg'] == pytest.approx(np.rad2deg(got))


def test_rotate_beats_translation_only():
    truth = room(n=120, res=0.05)
    live2 = Grid(resample(truth, truth, 0, 0, np.deg2rad(-8)), truth.resolution, truth.ox, truth.oy)
    from nav_explore_eval.metrics import align
    f_t = score(live2, truth, *align(live2, truth, 1.0))['wall_f1_pct']
    dx, dy, yaw = align_yaw(live2, truth, 1.0, 12)
    assert score(live2, truth, dx, dy, 0.10, yaw)['wall_f1_pct'] > f_t + 5


# ---- multi-robot
def test_multi_score_merges_and_measures_overlap():
    g = room()
    a = Grid(g.data.copy(), g.resolution, g.ox, g.oy); a.data[:, 35:] = -1      # left 0..35
    b = Grid(g.data.copy(), g.resolution, g.ox, g.oy); b.data[:, :25] = -1      # right 25..60
    m, per = multi_score([a, b], g)
    assert m['coverage_pct'] > 95 and m['robots'] == 2
    assert 5 < m['overlap_pct'] < 40
    assert per[0]['unique_pct'] > 20 and per[1]['unique_pct'] > 20
    assert per[0]['coverage_pct'] < m['coverage_pct']


def test_multi_single_robot_has_no_overlap():
    g = room()
    m, per = multi_score([g], g)
    assert m['overlap_pct'] == 0 and per[0]['unique_pct'] == pytest.approx(per[0]['coverage_pct'])


# ---- include / nested / mesh
def test_nested_model_pose_is_composed():
    inner = BOX.format(n='in', x=0, y=0, z=1, yaw=0, sx=1, sy=1, sz=2)
    w = world("<model name='outer'><pose>5 3 0 0 0 0</pose>" + inner.replace('<model name=\'in\'><static>true</static>', '<model name=\'in\'>') + '</model>')
    sh = shapes(w)
    assert len(sh) == 1 and sh[0][1:3] == pytest.approx((5.0, 3.0))


def test_include_model_uri(tmp_path):
    mdir = tmp_path / 'models' / 'pillar'
    mdir.mkdir(parents=True)
    (mdir / 'model.sdf').write_text("<sdf version='1.6'>" + BOX.format(n='pillar', x=0, y=0, z=1, yaw=0, sx=1, sy=1, sz=2) + '</sdf>')
    w = world(extra="<include><uri>model://pillar</uri><pose>2 -1 0 0 0 0</pose></include>")
    sh = shapes(w, model_paths=[str(tmp_path / 'models')])
    assert len(sh) == 1 and sh[0][1:3] == pytest.approx((2.0, -1.0))


def test_missing_include_warns_not_crashes():
    msgs = []
    sh = shapes(world(*wroom(), extra="<include><uri>model://nope</uri></include>"), warn=msgs.append)
    assert len(sh) == 4 and any('nope' in m for m in msgs)


def test_obj_mesh_bounds_and_footprint(tmp_path):
    (tmp_path / 'b.obj').write_text('v 0 0 0\nv 2 0 0\nv 2 1 0\nv 0 1 3\n')
    lo, hi = mesh_bounds(str(tmp_path / 'b.obj'))
    assert list(hi - lo) == [2, 1, 3]
    w = world("<model name='m'><link name='l'><collision name='c'><geometry><mesh><uri>b.obj</uri><scale>2 2 1</scale></mesh>"
              "</geometry></collision></link></model>")
    sh = shapes(w, z=1.0, base_dir=str(tmp_path))
    assert sh[0][0] == 'box' and sh[0][1:3] == pytest.approx((2.0, 1.0)) and sh[0][4] == pytest.approx((4.0, 2.0))
    assert shapes(w, z=5.0, base_dir=str(tmp_path)) == []                       # above the mesh


def test_stl_ascii_and_dae_bounds(tmp_path):
    (tmp_path / 'a.stl').write_text('solid s\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 2 0\nendloop\nendfacet\nendsolid')
    lo, hi = mesh_bounds(str(tmp_path / 'a.stl'))
    assert list(hi - lo) == [1, 2, 0]
    (tmp_path / 'c.dae').write_text('<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema"><asset><unit meter="0.01"/><up_axis>Z_UP</up_axis></asset>'
                                    '<float_array id="mesh-positions-array" count="6">0 0 0 100 200 300</float_array></COLLADA>')
    lo, hi = mesh_bounds(str(tmp_path / 'c.dae'))
    assert list(hi - lo) == pytest.approx([1, 2, 3])


# ---- reports, stats, leaderboard
def fake_run(label, cov, t_scale=1.0, f1=90.0):
    series = [(i * 10 * t_scale, {'coverage_pct': c, 'wall_f1_pct': f1, 'false_free_pct': 0.5,
                                  'covered_area_m2': c}) for i, c in enumerate([0, 30, 60, cov])]
    return build_report(label, 'truth.yaml', series, [(0, 0), (3, 4)])


def test_build_report_and_validate():
    r = fake_run('a', 95)
    assert validate(r) == [] and r['efficiency']['efficiency_m2_per_m'] == pytest.approx(95 / 5)
    assert r['time_to_pct_s']['90'] is not None
    assert validate({'label': 'x'}) and validate([]) == ['not a JSON object']
    r['final']['coverage_pct'] = 140
    assert any('outside' in e for e in validate(r))


def test_aggregate_mean_std_and_unreached():
    agg = aggregate([fake_run('a', 95), fake_run('a', 85, 2.0), fake_run('b', 50)])
    assert agg['a']['n'] == 2 and agg['a']['coverage_pct'][0] == pytest.approx(90.0)
    assert agg['a']['coverage_pct'][1] > 0
    assert agg['b']['time_to'][90][0] is None
    rows = stats_rows(agg)
    assert rows[1][0] == 'a' and rows[2][0] == 'b' and '±' in rows[1][2]          # ranked, std shown for n>1


def test_html_report_is_self_contained():
    h = html_report([fake_run('a', 95), fake_run('b', 70)])
    assert h.startswith('<!doctype html>') and 'data:image/png;base64' in h and '<td>a</td>' in h
    assert 'http://' not in h and 'src="http' not in h


def test_leaderboard_ranks_and_flags_invalid(tmp_path):
    for n, c in (('good', 95), ('ok', 60)):
        json.dump(fake_run(n, c), open(tmp_path / f'{n}.json', 'w'))
    md, problems = leaderboard(str(tmp_path))
    assert problems == [] and md.index('good') < md.index('ok')
    json.dump({'label': 'bad'}, open(tmp_path / 'bad.json', 'w'))
    _, problems = leaderboard(str(tmp_path))
    assert any('bad.json' in p for p in problems)


def test_cli_stats_report_leaderboard_validate(tmp_path):
    for i, c in enumerate((95, 85)):
        json.dump(fake_run('x', c), open(tmp_path / f'x{i}.json', 'w'))
    assert 'x' in cli('stats', tmp_path).stdout
    assert cli('validate', tmp_path / 'x0.json').returncode == 0
    out = tmp_path / 'r.html'
    assert cli('report', tmp_path / 'x0.json', tmp_path / 'x1.json', '--out', out).returncode == 0 and out.exists()
    board = tmp_path / 'LB.md'
    assert cli('leaderboard', tmp_path, '--out', board).returncode == 0
    assert cli('leaderboard', tmp_path, '--out', board, '--check').returncode == 0
    board.write_text('stale')
    assert cli('leaderboard', tmp_path, '--out', board, '--check').returncode == 1


def test_cli_score_multi_and_rotate(tmp_path):
    g = room(n=80, res=0.1)
    truth = write_map(tmp_path, 'truth', g)
    mp = write_map(tmp_path, 'm1', g)
    p = cli('score-multi', mp, mp, '--truth', truth, '--json')
    assert p.returncode == 0 and json.loads(p.stdout)['merged']['robots'] == 2
    p = cli('score', mp, truth, '--rotate', '--json')
    assert p.returncode == 0 and abs(json.loads(p.stdout)['yaw_deg']) < 1.5


def test_cli_truth_from_world_with_model_path(tmp_path):
    mdir = tmp_path / 'm' / 'wall'
    mdir.mkdir(parents=True)
    (mdir / 'model.sdf').write_text("<sdf version='1.6'>" + BOX.format(n='w', x=0, y=0, z=1, yaw=0, sx=4.2, sy=0.2, sz=2) + '</sdf>')
    inc = "".join(f"<include><uri>model://wall</uri><pose>0 {y} 0 0 0 0</pose></include>" for y in (2, -2))
    side = BOX.format(n='e', x=2, y=0, z=1, yaw=0, sx=0.2, sy=4, sz=2) + BOX.format(n='w2', x=-2, y=0, z=1, yaw=0, sx=0.2, sy=4, sz=2)
    wf = tmp_path / 'w.world'
    wf.write_text(world(side, extra=inc))
    p = cli('truth-from-world', wf, '--out', tmp_path / 'truth', '--model-path', tmp_path / 'm')
    assert p.returncode == 0, p.stderr
    assert 'explorable area 14' in p.stdout or 'explorable area 15' in p.stdout


# ---- rosbag
def topic(rosbag2_py, name, typ):
    try:
        return rosbag2_py.TopicMetadata(name=name, type=typ, serialization_format='cdr')
    except TypeError:                                         # newer rosbag2 wants an id first
        return rosbag2_py.TopicMetadata(id=0, name=name, type=typ, serialization_format='cdr')


def test_bag_scoring(tmp_path):
    rosbag2_py = pytest.importorskip('rosbag2_py')
    pytest.importorskip('rclpy')
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.serialization import serialize_message
    g = room()
    truth = write_map(tmp_path, 'truth', g)
    bag = str(tmp_path / 'bag')
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    for name, typ in (('/map', 'nav_msgs/msg/OccupancyGrid'), ('/odom', 'nav_msgs/msg/Odometry')):
        w.create_topic(topic(rosbag2_py, name, typ))
    for k in range(1, 11):                                   # reveal 6 columns per 2 s, odom moving
        t = int(k * 2e9)
        d = g.data.copy(); d[:, k * 6:] = -1
        m = OccupancyGrid(); m.info.resolution, m.info.width, m.info.height = g.resolution, g.shape[1], g.shape[0]
        m.data = d.flatten().tolist()
        w.write('/map', serialize_message(m), t)
        o = Odometry(); o.pose.pose.position.x = float(k)
        w.write('/odom', serialize_message(o), t)
    del w
    out = tmp_path / 'run.json'
    p = cli('bag', bag, '--truth', truth, '--label', 'bagged', '--out', out, '--interval', '2')
    assert p.returncode == 0, p.stdout + p.stderr
    r = json.loads(out.read_text())
    cov = [s['coverage_pct'] for s in r['series']]
    assert validate(r) == [] and r['label'] == 'bagged' and r['source'] == 'rosbag'
    assert cov == sorted(cov) and r['final']['coverage_pct'] > 85
    assert r['path_length_m'] == pytest.approx(9.0, abs=0.1)
    assert r['time_to_pct_s']['25'] < r['time_to_pct_s']['75']


def test_bag_without_map_fails_cleanly(tmp_path):
    rosbag2_py = pytest.importorskip('rosbag2_py')
    truth = write_map(tmp_path, 'truth', room())
    bag = str(tmp_path / 'bag')
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    w.create_topic(topic(rosbag2_py, '/other', 'std_msgs/msg/String'))
    del w
    p = cli('bag', bag, '--truth', truth, '--out', tmp_path / 'x.json')
    assert p.returncode == 1 and 'no /map' in p.stderr


# ---- localisation error
def test_ate_zero_for_identical_and_frame_offset_ignored():
    from nav_explore_eval.metrics import localisation_error
    gt = [(t, t * 0.5, np.sin(t)) for t in np.arange(0, 20, 0.1)]
    assert localisation_error(gt, gt)['ate_rmse_m'] < 1e-9
    c, s = np.cos(0.4), np.sin(0.4)                      # est in a rotated, shifted frame: aligned ATE ~ 0
    est = [(t, c * x - s * y + 3, s * x + c * y - 2) for t, x, y in gt]
    assert localisation_error(est, gt)['ate_rmse_m'] < 1e-6
    assert localisation_error(est, gt, align=False)['ate_rmse_m'] > 1.0


def test_ate_detects_drift_and_handles_no_overlap():
    from nav_explore_eval.metrics import localisation_error
    gt = [(t, t, 0.0) for t in np.arange(0, 10, 0.1)]
    est = [(t, t, 0.02 * t) for t in np.arange(0, 10, 0.1)]            # slow lateral drift
    r = localisation_error(est, gt, align=False)
    assert 0.05 < r['ate_rmse_m'] < 0.2 and r['final_error_m'] == pytest.approx(0.198, abs=0.01)
    assert r['drift_pct'] == pytest.approx(2.0, abs=0.2)
    assert localisation_error([(100, 0, 0), (101, 1, 0)], gt) is None
    assert localisation_error([(0, 0, 0)], gt) is None


def test_stats_shows_ate_column():
    r = fake_run('a', 90); r['localisation'] = {'ate_rmse_m': 0.123}
    rows = stats_rows(aggregate([r]))
    assert rows[0][-1] == 'ate_m' and rows[1][-1] == '0.123'
    assert 'ate_m' not in stats_rows(aggregate([fake_run('b', 90)]))[0]


def test_bag_with_ground_truth_pose(tmp_path):
    rosbag2_py = pytest.importorskip('rosbag2_py')
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.serialization import serialize_message
    g = room()
    truth = write_map(tmp_path, 'truth', g)
    bag = str(tmp_path / 'bag')
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    for name, typ in (('/map', 'nav_msgs/msg/OccupancyGrid'), ('/odom', 'nav_msgs/msg/Odometry'),
                      ('/gt', 'nav_msgs/msg/Odometry')):
        w.create_topic(topic(rosbag2_py, name, typ))
    for k in range(1, 21):
        t = int(k * 1e9)
        m = OccupancyGrid(); m.info.resolution, m.info.width, m.info.height = g.resolution, g.shape[1], g.shape[0]
        m.data = g.data.flatten().tolist()
        w.write('/map', serialize_message(m), t)
        for name, y in (('/odom', 0.01 * k), ('/gt', 0.0)):          # odom drifts sideways, gt is straight
            o = Odometry(); o.pose.pose.position.x = float(k); o.pose.pose.position.y = y
            w.write(name, serialize_message(o), t)
    del w
    out = tmp_path / 'run.json'
    p = cli('bag', bag, '--truth', truth, '--out', out, '--gt-odom', '/gt')
    assert p.returncode == 0, p.stdout + p.stderr
    loc = json.loads(out.read_text())['localisation']
    assert 0 < loc['ate_rmse_m'] < 0.2 and loc['pairs'] >= 18 and 'localisation' in p.stdout
