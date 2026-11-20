import math
import struct

from lio_doctor.core import (CloudInfo, LioDoctor, align_drift, check_cloud_fields, estimate_time_offset, field_range,
                             readiness, stamp_position)

F, U = 7, 4


def good_info(n=100):
    fields = [('x', 0, F, 1), ('y', 4, F, 1), ('z', 8, F, 1), ('intensity', 12, F, 1), ('ring', 16, U, 1), ('time', 20, F, 1)]
    return CloudInfo(fields, 24, n, 1, 24 * n, 24 * n)


def bad_info(n=100):
    """The LIO-SAM bug (concepts.md sec 44 bug 3): 22-byte records, declared point_step 20, unaligned time."""
    fields = [('x', 0, F, 1), ('y', 4, F, 1), ('z', 8, F, 1), ('intensity', 12, F, 1), ('ring', 16, U, 1), ('time', 18, F, 1)]
    return CloudInfo(fields, 20, n, 1, 20 * n, 22 * n)


def codes(fs):
    return {x.code for x in fs}


def test_good_cloud_clean():
    assert check_cloud_fields(good_info()) == []


def test_malformed_22_byte_record():
    c = codes(check_cloud_fields(bad_info()))
    assert {'CLOUD_FIELD_OVERRUN', 'CLOUD_UNALIGNED', 'CLOUD_DATA_SIZE'} <= c


def test_ring_wrong_type_and_missing_xyz():
    i = good_info()
    i.fields[4] = ('ring', 16, 6, 1)
    assert 'CLOUD_RING_TYPE' in codes(check_cloud_fields(i))
    assert 'CLOUD_NO_XYZ' in codes(check_cloud_fields(CloudInfo([('x', 0, F, 1)], 4)))


def test_overlap_and_step():
    i = CloudInfo([('x', 0, F, 1), ('y', 2, F, 1), ('z', 8, F, 1)], 14)
    c = codes(check_cloud_fields(i))
    assert 'CLOUD_FIELD_OVERLAP' in c and 'CLOUD_STEP_UNALIGNED' in c and 'CLOUD_UNALIGNED' in c


def test_field_range_uses_declared_layout():
    n = 10
    data = b''.join(struct.pack('<fffIHxxf', 0, 0, 0, 0, 1, -0.1 * k)[:24] for k in range(n))
    assert len(data) == 24 * n
    lo, hi = field_range(data, good_info(n), 'time')
    assert math.isclose(lo, -0.9, abs_tol=1e-5) and math.isclose(hi, 0.0, abs_tol=1e-6)


def test_stamp_position():
    assert stamp_position((0.0, 0.1), 0.1) == 'start'
    assert stamp_position((-0.1, 0.0), 0.1) == 'end'
    assert stamp_position(None, 0.1) == 'unknown'
    assert stamp_position((-0.05, 0.05), 0.1) == 'unknown'


def test_readiness_good_cloud():
    r = readiness(good_info(), 'start', 200.0, True)
    assert [r[k][0] for k in ('KISS-ICP', 'RKO-LIO', 'LIO-SAM')] == ['READY'] * 3


def test_readiness_xyz_only_cloud():
    i = CloudInfo([('x', 0, F, 1), ('y', 4, F, 1), ('z', 8, F, 1)], 12, 10, 1, 120, 120)   # the raw gz cloud
    r = readiness(i, 'unknown', 100.0, True)
    assert r['KISS-ICP'][0] == 'DEGRADED'
    assert r['RKO-LIO'][0] == 'DEGRADED' and 'deskew:=false' in r['RKO-LIO'][1][0]
    assert r['LIO-SAM'][0] == 'BLOCKED'


def test_readiness_stamp_end_and_no_imu():
    r = readiness(good_info(), 'end', None, None)
    assert r['LIO-SAM'][0] == 'BLOCKED' and r['RKO-LIO'][0] == 'BLOCKED' and r['KISS-ICP'][0] == 'READY'


def test_readiness_malformed_cloud_blocks_all():
    assert all(v[0] == 'BLOCKED' for v in readiness(bad_info(), 'start', 200.0, True).values())


def feed_odom(d, n=60, hz=10.0, speed=0.5, jump_at=None, start=100.0):
    for k in range(n):
        x = speed * k / hz + (3.0 if jump_at is not None and k >= jump_at else 0.0)
        d.odom(start + k / hz, x, 0.0, 0.0, start + k / hz)


def test_healthy_odom():
    d = LioDoctor(expected_hz=10.0)
    feed_odom(d)
    d.cloud(good_info(), 0.0, (0.0, 0.1))
    for k in range(60):
        d.cloud(good_info(), 100.0 + k * 0.1, (0.0, 0.1))
    for k in range(600):
        d.imu(100 + k * 0.01, 100 + k * 0.01, 0.0, True)
    d.tf(['odom', 'base_link'])
    d.frames = ('odom', 'base_link')
    assert codes(d.diagnose(106.0)) == {'HEALTHY'}


def test_pose_jump_and_rate_and_gap():
    d = LioDoctor(expected_hz=10.0)
    feed_odom(d, jump_at=30)
    c = codes(d.diagnose(105.9))
    assert 'POSE_JUMP' in c
    d2 = LioDoctor(expected_hz=10.0)
    feed_odom(d2, hz=4.0)
    assert 'ODOM_RATE_LOW' in codes(d2.diagnose(115.0))
    d3 = LioDoctor()
    for k, t in enumerate([0, .1, .2, .3, .4, 1.5, 1.6]):
        d3.odom(t, 0, 0, 0, t)
    assert 'STAMP_GAP' in codes(d3.diagnose(1.7))


def test_stamp_backwards_and_none():
    d = LioDoctor()
    assert 'ODOM_NONE' in codes(d.diagnose(1.0))
    d.odom(5.0, 0, 0, 0, 5.0)
    d.odom(4.0, 0, 0, 0, 5.1)
    assert 'STAMP_BACKWARDS' in codes(d.diagnose(5.2))


def test_cloud_findings_in_diagnose():
    d = LioDoctor()
    d.cloud(bad_info(), 0.0, (-0.1, 0.0))
    for k in range(5):
        d.cloud(bad_info(), 0.1 * k, (-0.1, 0.0))
    c = codes(d.diagnose(1.0))
    assert {'CLOUD_FIELD_OVERRUN', 'STAMP_AT_END', 'IMU_NONE'} <= c


def test_tf_missing():
    d = LioDoctor(frames=('odom', 'laser_frame'))
    d.tf(['odom'])
    assert 'TF_MISSING' in codes(d.diagnose(1.0))


def test_imu_rate_low():
    d = LioDoctor()
    for k in range(10):
        d.imu(k * 0.1, k * 0.1)
    assert 'IMU_RATE_LOW' in codes(d.diagnose(1.0))


def yaw_signal(t):
    return 0.6 * math.sin(2 * math.pi * 0.3 * t) + 0.3 * math.sin(2 * math.pi * 0.11 * t)


def test_time_offset_recovered():
    off = 0.08
    dt = 0.01
    imu_t = [k * dt for k in range(2000)]
    imu_w = [yaw_signal(t) for t in imu_t]
    odom_t = [k * 0.05 for k in range(400)]
    a = 0.0
    yaw = []
    for t in odom_t:                    # odom stamps lag the IMU: its rate at stamp t is the true rate at t - off
        yaw.append(a)
        a += yaw_signal(t + 0.025 - off) * 0.05     # rate belongs to the interval midpoint
    r = estimate_time_offset(imu_t, imu_w, odom_t, yaw)
    assert r is not None and abs(abs(r[0]) - off) < 0.03 and r[1] > 0.8


def test_time_offset_too_little_data():
    assert estimate_time_offset([0, 1], [0, 0], [0, 1], [0, 0]) is None


def test_align_drift_ignores_rigid_offset_and_sees_drift():
    ref = [(0.1 * k, 0.0) for k in range(60)]
    c, s = math.cos(1.0), math.sin(1.0)
    rot = [(c * x - s * y + 5, s * x + c * y - 2) for x, y in ref]
    assert align_drift(rot, ref)[1] < 1e-6
    drifting = [(x * 1.2, y) for x, y in ref]
    rmse, end, path = align_drift(drifting, ref)
    assert end > 0.3 and math.isclose(path, 5.9, abs_tol=1e-6)


def test_drift_high_in_diagnose():
    d = LioDoctor(drift_window_s=60.0)
    feed_odom(d, n=200, speed=1.0)
    for k in range(200):
        t = 100.0 + k / 10.0
        d.ref_odom(t, 0.8 * k / 10.0, 0.0)
    assert 'DRIFT_HIGH' in codes(d.diagnose(120.0))


# ---- IMU heading vs odometry ----
from lio_doctor.core import check_imu_yaw, imu_yaw_offset  # noqa: E402


def _q(yaw_deg):
    h = math.radians(yaw_deg) / 2
    return (0.0, 0.0, math.sin(h), math.cos(h))


def _series(offset_deg, n=50, noise=0.0, base=lambda i: 3.0 * i, dt=0.01, gap=0.0):
    odom = [(i * dt, _q(base(i))) for i in range(n)]
    imu = [(i * dt + gap, _q(base(i) + offset_deg + (noise if i % 2 else -noise))) for i in range(n)]
    return imu, odom


def test_yaw_offsets_by_size():
    for off, code in ((-90, 'IMU_YAW_OFFSET'), (0, 'IMU_YAW_OK'), (4, 'IMU_YAW_OK'), (6, 'IMU_YAW_OFFSET')):
        f = check_imu_yaw(*_series(off))
        assert codes(f) == {code}, off
    m, s, n = imu_yaw_offset(*_series(-90))
    assert abs(m + 90) < 1e-6 and s < 1e-3 and n == 50
    msg = check_imu_yaw(*_series(-90))[0]
    assert msg.level == 'ERROR' and '-90.0' in msg.msg


def test_yaw_offset_message_names_the_relay_fix():
    msg = check_imu_yaw(*_series(-90))[0].msg
    assert 'yaw_deg:=+90.0' in msg
    assert 'yaw_deg:=-90.0' in check_imu_yaw(*_series(90))[0].msg


def test_yaw_noisy():
    f = check_imu_yaw(*_series(0, n=200, noise=15))
    assert f[0].level == 'WARN' and f[0].code == 'IMU_YAW_NOISY'


def test_yaw_wraparound():
    imu, odom = _series(0, base=lambda i: 175 + 0.2 * i)   # crosses +-180
    assert codes(check_imu_yaw(imu, odom)) == {'IMU_YAW_OK'}
    imu, odom = _series(8, base=lambda i: 175 + 0.2 * i)
    m, s, _ = imu_yaw_offset(imu, odom)
    assert abs(m - 8) < 1e-6 and s < 1e-3
    imu, odom = _series(-175, base=lambda i: 170)           # +-180 straddle of the difference itself
    assert abs(abs(imu_yaw_offset(imu, odom)[0]) - 175) < 1e-6


def test_yaw_too_few_or_unmatched():
    assert codes(check_imu_yaw(*_series(-90, n=9))) == {'IMU_YAW_SKIPPED'}
    assert codes(check_imu_yaw([], _series(0)[1])) == {'IMU_YAW_SKIPPED'}
    assert codes(check_imu_yaw(*_series(-90, gap=2.0))) == {'IMU_YAW_SKIPPED'}   # beyond the 50 ms match gap


def test_yaw_readiness_effect():
    r = readiness(good_info(), 'start', 200, True, -90.0)
    assert r['LIO-SAM'][0] == 'DEGRADED' and 'IMU_YAW_OFFSET' in r['LIO-SAM'][1][0]
    assert r['KISS-ICP'][0] == 'READY'
    assert readiness(good_info(), 'start', 200, True, 3.0)['LIO-SAM'][0] == 'READY'
    assert readiness(good_info(), 'start', 200, True)['LIO-SAM'][0] == 'READY'


def test_live_doctor_yaw():
    d = LioDoctor()
    for i in range(50):
        t = i * 0.01
        d.odom(t, 0, 0, 0.0, t)
        d.imu(t, t, 0.0, True, _q(-90))
    assert any(x.code == 'IMU_YAW_SKIPPED' for x in d.diagnose(0.5))   # no reference odom: INFO skip
    for i in range(50):
        d.ref_odom(i * 0.01, 0, 0, _q(0))
    assert any(x.code == 'IMU_YAW_OFFSET' for x in d.diagnose(0.5))
    assert abs(d.imu_yaw_deg() + 90) < 1e-6


def test_stalled_cloud_and_imu_are_flagged():
    d = LioDoctor(expected_hz=10.0)
    for k in range(40):
        t = 100.0 + k * 0.1
        d.odom(t, 0, 0, 0, t)
        d.cloud(good_info(), t)
        d.imu(t, t)
    assert not {'CLOUD_STALE', 'IMU_STALE'} & codes(d.diagnose(103.95))
    c = codes(d.diagnose(107.0))           # everything stopped ~3 s ago
    assert {'CLOUD_STALE', 'IMU_STALE'} <= c
    d2 = LioDoctor()
    for k in range(40):
        t = 100.0 + k * 0.1
        d2.odom(t, 0, 0, 0, t)
        d2.cloud(good_info(), t)
        d2.imu(t, t)
    d2.cloud(good_info(), 107.0)           # cloud back, IMU still dead
    c2 = codes(d2.diagnose(107.1))
    assert 'IMU_STALE' in c2 and 'CLOUD_STALE' not in c2


def test_health_overlay_text_and_colour():
    from lio_doctor.core import Finding, health_overlay
    summ = {'score': 60, 'odom_hz': 9.8, 'cloud_hz': 10.0, 'imu_hz': None}
    t, col = health_overlay([Finding('ERROR', 'CLOUD_STALE', 'x'), Finding('WARN', 'IMU_RATE_LOW', 'y')], summ)
    assert t.splitlines()[0] == 'LIO health 60/100' and 'ERROR CLOUD_STALE' in t and 'imu' not in t and col[0] == 1.0 and col[1] < 0.5
    t, col = health_overlay([Finding('OK', 'HEALTHY', 'z')], dict(summ, score=100))
    assert 'HEALTHY' not in t and col[1] == 1.0
