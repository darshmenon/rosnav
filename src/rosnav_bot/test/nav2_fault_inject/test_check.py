from nav2_fault_inject.check import evaluate, normalize, parse_report
import pytest

def R(t, ok, *codes): return {'time': t, 'ok': ok, 'codes': set(codes)}

F = {'node': 'bridge', 'expect': {'detect': ['SCAN_STALE'], 'within': 5, 'clear_within': 10, 'forbid': ['GOAL_FLAPPING']}}

def test_pass():
    reps = [R(100, True), R(103, False, 'SCAN_STALE'), R(111, True)]
    v = evaluate(F, 101, 108, reps)
    assert v['passed'], v

def test_not_detected():
    v = evaluate(F, 101, 108, [R(103, True), R(111, True)])
    assert not v['passed'] and not v['checks'][0]['ok']

def test_not_cleared():
    v = evaluate(F, 101, 108, [R(103, False, 'SCAN_STALE'), R(111, False, 'SCAN_STALE')])
    assert not v['passed'] and v['checks'][1]['name'] == 'clear' and not v['checks'][1]['ok']

def test_forbid():
    v = evaluate(F, 101, 108, [R(103, False, 'SCAN_STALE', 'GOAL_FLAPPING'), R(111, True)])
    assert not v['passed']

def test_never_injected():
    assert not evaluate(F, None, None, [])['passed']

def test_normalize_validates():
    with pytest.raises(ValueError):
        normalize({'detects': ['X']})
    assert normalize({'detect': 'A'})['detect'] == ['A']

def test_parse_report():
    assert parse_report('{"time": 5, "ok": false, "findings": [{"level":"ERROR","code":"X","msg":"m"}]}') == {'time': 5.0, 'ok': False, 'codes': {'X'}}
    assert parse_report('garbage') is None and parse_report('{"x":1}') is None


def test_doctor_cmd_from_env(monkeypatch):
    monkeypatch.setenv('NAV2_DOCTOR_CMD', 'ros2 run rosnav_bot nav2_doctor.py --json')
    from nav2_fault_inject.check import DoctorWatcher
    assert DoctorWatcher().cmd == ['ros2', 'run', 'rosnav_bot', 'nav2_doctor.py', '--json']


def test_explicit_cmd_beats_env(monkeypatch):
    monkeypatch.setenv('NAV2_DOCTOR_CMD', 'from-env')
    from nav2_fault_inject.check import DoctorWatcher
    assert DoctorWatcher('mine --json').cmd == ['mine', '--json']
