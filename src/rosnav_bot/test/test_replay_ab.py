import argparse
import os
import re
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))
import replay_ab as ra  # noqa: E402

GOALS = [(10.0, 0.0), (-10.0, 0.0)]


def head_on(seconds=15.0, lanes=(-0.6, 0.0, 0.6)):
    """Three pedestrians walk toward a robot that starts at the origin heading to (10, 0)."""
    t = np.arange(0, seconds, ra.DT)
    frames = [[dict(id=i + 1, x=10.0 - 1.1 * tt, y=y) for i, y in enumerate(lanes)] for tt in t]
    return ra.Recording.from_frames(t, frames, None, {'source': 'test'})


def test_metric_constants_match_social_metrics_node():
    src = open(os.path.join(HERE, '..', 'scripts', 'social_metrics.py')).read()
    assert 'robot_r=0.30' in src and 'ped_r=0.22' in src and 'personal_space=1.2' in src
    assert (ra.ROBOT_R, ra.PED_R, ra.PERSONAL_SPACE) == (0.30, 0.22, 1.2)


def test_metrics_stationary_robot_one_contact_episode():
    m = ra.Metrics()
    for tt in np.arange(0, 6.0, ra.DT):                       # ped walks straight through the robot at 1 m/s
        m.update(np.zeros(2), np.array([[-3.0 + tt, 0.0]]))
    r = m.result()
    assert r['collisions'] == 1                               # one contact EPISODE, not one per sample
    assert -0.56 < r['min_clearance_m'] < -0.45               # passes through the centre: -(0.30+0.22)
    assert 2.2 < r['intrusion_s'] < 2.7                       # ~2.4 s within 1.2 m
    assert r['path_m'] == 0.0


def test_resampling_irregular_stamps_and_gaps():
    t = [0.0, 0.37, 0.81, 1.22, 1.6]
    frames = [[dict(id=7, x=2.0 * tt, y=1.0)] for tt in t]
    frames[2] = []                                            # one dropped frame
    rec = ra.Recording.from_frames(t, frames)
    k = int(round(1.0 / ra.DT))
    assert abs(rec.pos[k, 0, 0] - 2.0) < 1e-6 and abs(rec.pos[k, 0, 1] - 1.0) < 1e-6   # linear motion recovered
    assert np.isfinite(rec.pos[:, 0]).all()


def test_scene_is_immutable_and_json_roundtrips(tmp_path):
    rec = head_on()
    with pytest.raises(ValueError):
        rec.pos[0, 0, 0] = 99.0                               # a planner cannot alter the shared scene
    p = tmp_path / 'r.json'
    rec.save(str(p))
    back = ra.Recording.load(str(p))
    assert back.pos.shape == rec.pos.shape and np.nanmax(np.abs(back.pos - rec.pos)) < 2e-3


def test_replay_identical_scene_and_deterministic():
    rec = head_on()
    fp = rec.fingerprint()
    a = ra.run_episode(rec, 'cv-risk', (0.0, 0.0), GOALS)
    b = ra.run_episode(rec, 'cv-risk', (0.0, 0.0), GOALS)
    ra.run_episode(rec, 'reactive', (0.0, 0.0), GOALS)
    assert a == b                                             # same recording + planner + start -> same numbers
    assert rec.fingerprint() == fp                            # no planner touched the stream


def test_risk_planner_not_worse_than_reactive_on_same_stream():
    rec = head_on()
    re_ = ra.run_episode(rec, 'reactive', (0.0, 0.0), GOALS)
    rk = ra.run_episode(rec, 'cv-risk', (0.0, 0.0), GOALS)
    assert rk['collisions'] <= re_['collisions']
    assert (rk['min_clearance_m'] or 0) >= (re_['min_clearance_m'] or 0) - 1e-9


def test_extract_from_a_real_bag_and_score(tmp_path):
    rosbag2_py = pytest.importorskip('rosbag2_py')
    import json
    from nav_msgs.msg import Odometry
    from rclpy.serialization import serialize_message
    from std_msgs.msg import String
    uri = str(tmp_path / 'bag')
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    w.create_topic(rosbag2_py.TopicMetadata(name='/social_peds/gt', type='std_msgs/msg/String',
                                            serialization_format='cdr'))
    w.create_topic(rosbag2_py.TopicMetadata(name='/odom', type='nav_msgs/msg/Odometry', serialization_format='cdr'))
    for k in range(100):
        tt = 100.0 + k * 0.1
        gt = String(data=json.dumps({'t': tt, 'peds': [dict(id=1, x=3.0 - 0.5 * k * 0.1, y=0.0, vx=-0.5, vy=0.0)]}))
        w.write('/social_peds/gt', serialize_message(gt), int(tt * 1e9))
        od = Odometry()
        od.pose.pose.position.x, od.pose.pose.orientation.w = 0.3 * k * 0.1, 1.0
        w.write('/odom', serialize_message(od), int(tt * 1e9))
    del w
    out = str(tmp_path / 'rec.json')
    ra.cmd_extract(argparse.Namespace(bag=uri, out=out, gt_topic='/social_peds/gt', odom_topic='/odom',
                                      odom_offset=[1.0, 0.0], map='', storage='sqlite3'))
    rec = ra.Recording.load(out)
    assert rec.steps >= 95 and rec.ids == [1]
    assert abs(rec.pos[0, 0, 0] - 3.0) < 1e-2 and abs(rec.robot[0, 0] - 1.0) < 1e-2     # odom offset applied
    r = ra.cmd_score(argparse.Namespace(recording=out))
    assert r['path_m'] > 2.0 and r['samples'] >= 95
