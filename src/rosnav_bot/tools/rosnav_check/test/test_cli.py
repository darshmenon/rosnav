import json

from rosnav_check.cli import main

SPEC = {'fields': [['x', 0, 7, 1], ['y', 4, 7, 1], ['z', 8, 7, 1], ['intensity', 12, 7, 1], ['ring', 16, 4, 1], ['time', 18, 7, 1]],
        'point_step': 20, 'width': 10, 'height': 1, 'row_step': 200, 'data_len': 220}


def test_preflight_cloud_spec(tmp_path, capsys):
    f = tmp_path / 's.json'
    f.write_text(json.dumps(SPEC))
    rc = main(['preflight', '--cloud-spec', str(f), '--json'])
    out = capsys.readouterr().out
    rep = json.loads(out)
    assert rc == 2 and any(x['code'] == 'CLOUD_FIELD_OVERRUN' for x in rep['findings'])


def test_preflight_nothing_to_run(capsys):
    assert main(['preflight']) == 1
    assert 'TOOL_UNAVAILABLE' in capsys.readouterr().out


def test_eval_missing(monkeypatch, capsys):
    monkeypatch.setattr('rosnav_check.cli.FUSION_EVAL', '/nonexistent/eval.sh')
    assert main(['eval', 'r.tum', 'e.tum']) == 2
