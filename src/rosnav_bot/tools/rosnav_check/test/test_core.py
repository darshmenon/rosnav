import json

from rosnav_check.cli import main
from rosnav_check.core import Item, correlate, exit_code, load, merge, normalize, report


def doc(*pairs):
    return {'findings': [{'level': lv, 'code': c, 'msg': c.lower(), 'hint': 'h'} for lv, c in pairs]}


def test_normalize_and_sort():
    items = normalize(doc(('WARN', 'A'), ('ERROR', 'B'), ('OK', 'C')), 'slam_doctor')
    m = merge(items)
    assert [i.code for i in m] == ['B', 'A'] and m[0].tool == 'slam_doctor'


def test_ok_only_kept():
    assert [i.code for i in merge(normalize(doc(('OK', 'HEALTHY')), 'x'))] == ['HEALTHY']


def test_exit_codes():
    assert exit_code([]) == 0
    assert exit_code([Item('t', 'c', 'INFO')]) == 0
    assert exit_code([Item('t', 'c', 'WARN')]) == 1
    assert exit_code([Item('t', 'c', 'WARN'), Item('t', 'd', 'ERROR')]) == 2


def test_cloud_upstream_rule():
    items = normalize(doc(('ERROR', 'STAMP_AT_END')), 'lio_doctor') + normalize(doc(('WARN', 'DRIFT_HIGH')), 'slam_doctor')
    ids = [h['rule'] for h in correlate(items)]
    assert 'CLOUD_UPSTREAM' in ids and 'upstream' in correlate(items)[0]['msg']


def test_drift_high_same_code_other_tool_does_not_fire():
    # lio's own DRIFT_HIGH must not satisfy the slam_doctor side
    items = normalize(doc(('ERROR', 'STAMP_AT_END'), ('WARN', 'DRIFT_HIGH')), 'lio_doctor')
    ids = [h['rule'] for h in correlate(items)]
    assert 'CLOUD_UPSTREAM' not in ids and 'CLOUD_UPSTREAM_LIO' in ids


def test_tf_rules():
    n = normalize(doc(('ERROR', 'TF_FREEZE'), ('WARN', 'FILTER_NO_MASK')), 'nav2_doctor')
    s = normalize(doc(('WARN', 'NO_MAP_ODOM')), 'slam_doctor')
    ids = {h['rule'] for h in correlate(n + s)}
    assert {'SLAM_TF_ROOT', 'TF_FREEZE_FILTERS'} <= ids


def test_single_tool_no_rule():
    assert correlate(normalize(doc(('WARN', 'DRIFT_HIGH')), 'slam_doctor')) == []


def test_load_by_filename_and_prefix(tmp_path):
    f = tmp_path / 'slam_doctor_out.json'
    f.write_text(json.dumps(doc(('WARN', 'MAP_STALL'))))
    g = tmp_path / 'x.json'
    g.write_text(json.dumps(doc(('WARN', 'COSTMAP_STALE'))))
    assert load(str(f))[0].tool == 'slam_doctor'
    assert load(f'nav2_doctor={g}')[0].tool == 'nav2_doctor'


def test_cli_merge_exit_and_json(tmp_path, capsys):
    a = tmp_path / 'lio.json'
    a.write_text(json.dumps(doc(('ERROR', 'CLOUD_NO_TIME'))))
    b = tmp_path / 'slam.json'
    b.write_text(json.dumps(doc(('WARN', 'NO_MAP_ODOM'))))
    assert main(['merge', str(a), str(b), '--json']) == 2
    rep = json.loads(capsys.readouterr().out)
    assert rep['root_causes'][0]['rule'] == 'CLOUD_UPSTREAM' and rep['tools'] == ['lio_doctor', 'slam_doctor']


def test_cli_merge_bad_input(tmp_path, capsys):
    assert main(['merge', str(tmp_path / 'nope.json')]) == 2
    assert 'BAD_INPUT' in capsys.readouterr().out


def test_report_empty():
    assert report([])['exit_code'] == 0
