import math

from slam_doctor.core import SlamDoctor


def codes(d, now):
    return {f.code for f in d.diagnose(now)}


def feed(d, t0, t1, step=0.1, x=0.0, y=0.0, yaw=0.0):
    t = t0
    while t < t1:
        d.map_odom(x, y, yaw, t)
        t += step


def test_healthy():
    d = SlamDoctor()
    feed(d, 0, 10); d.map_stats(1000, 50, 5000, 9)
    assert codes(d, 10) == {'HEALTHY'}


def test_no_map_odom_and_no_map():
    assert codes(SlamDoctor(), 1) == {'NO_MAP_ODOM', 'MAP_NONE'}


def test_tracking_lost_after_silence():
    d = SlamDoctor(); feed(d, 0, 5); d.map_stats(10, 0, 0, 4)
    assert 'TRACKING_LOST' not in codes(d, 6)
    assert 'TRACKING_LOST' in codes(d, 11)


def test_loop_closure_logged_with_size():
    d = SlamDoctor(); feed(d, 0, 2); feed(d, 2, 3, x=0.3, yaw=math.radians(5))
    ev = [e for e in d.events if e['kind'] == 'correction']
    assert len(ev) == 1 and abs(ev[0]['dist'] - 0.3) < 1e-6 and abs(ev[0]['deg'] - 5) < 0.01


def test_small_motion_is_not_a_correction():
    d = SlamDoctor()
    for i in range(50):
        d.map_odom(i * 0.001, 0, 0, i * 0.1)
    assert d.events == []


def test_big_jump_and_storm():
    d = SlamDoctor(); d.map_stats(1, 0, 0, 0)
    for i in range(5):
        feed(d, i * 5, i * 5 + 2, x=i * 0.6)
    c = codes(d, 24)
    assert 'JUMP_STORM' in c and 'BIG_JUMP' in c


def test_flap_counts_resumes():
    d = SlamDoctor(); d.map_stats(1, 0, 0, 0)
    t = 0.0
    for _ in range(4):
        feed(d, t, t + 1); t += 3.0          # 2 s silence each cycle
    d.map_odom(0, 0, 0, t)
    assert 'TRACKING_FLAP' in codes(d, t + 0.1)


def test_map_stall_needs_motion():
    d = SlamDoctor(); feed(d, 0, 40); d.map_stats(500, 0, 0, 0)
    d.map_stats(500, 0, 0, 1)
    assert 'MAP_STALL' not in codes(d, 40)   # not moving: standing still is fine
    for i in range(30):
        d.odom(i * 0.1, 0, 1 + i)            # now drive 2.9 m with a frozen map
    d.map_stats(500, 0, 0, 31)
    d.map_odom(0, 0, 0, 40)
    assert 'MAP_STALL' in codes(d, 41)


def test_map_growth_clears_stall():
    d = SlamDoctor(); feed(d, 0, 60); d.map_stats(500, 0, 0, 0)
    for i in range(30):
        d.odom(i * 0.1, 0, 1 + i)
        d.map_stats(500 + i, 0, 0, 1 + i)
    d.map_odom(0, 0, 0, 59.9)
    assert 'MAP_STALL' not in codes(d, 60)


def test_drift_ratio():
    d = SlamDoctor(); d.map_stats(1, 0, 0, 0)
    for i in range(70):
        d.odom(i * 0.1, 0, i)                # 6.9 m travelled
    for i in range(4):
        feed(d, i * 5, i * 5 + 1, x=i * 0.5)  # 1.5 m of corrections
    assert 'DRIFT_HIGH' in codes(d, 21)


def test_score_and_summary():
    d = SlamDoctor(); feed(d, 0, 3); d.map_stats(1, 0, 0, 0)
    assert d.score(3) == 100
    assert d.score(20) == 60                 # TRACKING_LOST error costs 40
    assert d.summary(3)['corrections'] == 0
