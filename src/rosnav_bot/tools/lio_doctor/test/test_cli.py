import json

from lio_doctor.doctor import check_cloud_main, main

SPEC = {'fields': [['x', 0, 7, 1], ['y', 4, 7, 1], ['z', 8, 7, 1], ['intensity', 12, 7, 1], ['ring', 16, 4, 1], ['time', 18, 7, 1]],
        'point_step': 20, 'width': 10, 'height': 1, 'row_step': 200, 'data_len': 220, 'imu_hz': 200}


def test_check_cloud_flags_bad_spec(tmp_path, capsys):
    f = tmp_path / 's.json'
    f.write_text(json.dumps(SPEC))
    assert check_cloud_main(str(f)) == 1
    out = capsys.readouterr().out
    assert 'CLOUD_FIELD_OVERRUN' in out and 'BLOCKED' in out


def test_check_cloud_good_json(tmp_path, capsys):
    spec = dict(SPEC, point_step=24, row_step=240, data_len=240)
    spec['fields'] = [['x', 0, 7, 1], ['y', 4, 7, 1], ['z', 8, 7, 1], ['intensity', 12, 7, 1], ['ring', 16, 4, 1], ['time', 20, 7, 1]]
    spec['stamp_position'] = 'start'
    f = tmp_path / 's.json'
    f.write_text(json.dumps(spec))
    assert main(['--check-cloud', str(f), '--json']) == 0
    assert json.loads(capsys.readouterr().out)['readiness']['LIO-SAM']['verdict'] == 'READY'


def test_check_cloud_imu_yaw_samples(tmp_path, capsys):
    spec = dict(SPEC, point_step=24, row_step=240, data_len=240, stamp_position='start')
    spec['fields'] = [['x', 0, 7, 1], ['y', 4, 7, 1], ['z', 8, 7, 1], ['intensity', 12, 7, 1], ['ring', 16, 4, 1], ['time', 20, 7, 1]]
    s = -0.7071067811865476   # sin/cos of -45 deg: a -90 deg yaw
    spec['imu_yaw'] = {'imu': [[i * 0.01, [0, 0, s, -s]] for i in range(20)], 'odom': [[i * 0.01, [0, 0, 0, 1]] for i in range(20)]}
    f = tmp_path / 's.json'
    f.write_text(json.dumps(spec))
    assert main(['--check-cloud', str(f), '--json']) == 1
    out = json.loads(capsys.readouterr().out)
    assert any(x['code'] == 'IMU_YAW_OFFSET' for x in out['findings'])
    assert out['readiness']['LIO-SAM']['verdict'] == 'DEGRADED'
