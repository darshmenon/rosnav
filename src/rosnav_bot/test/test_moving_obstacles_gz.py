import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import moving_obstacles_gz as m  # noqa: E402


def test_ease_yaw_turns_gradually_and_the_short_way():
    assert m.ease_yaw(None, 1.0, 0.1) == 1.0                       # first tick: no history, take the target
    y = m.ease_yaw(0.0, math.pi / 2, 0.1)                          # 90 deg corner at 1.6 rad/s, 0.1 s
    assert abs(y - 0.16) < 1e-9
    y = m.ease_yaw(3.0, -3.0, 0.1)                                 # crosses +-pi: short way is increasing yaw
    assert y > 3.0
    assert abs(m.ease_yaw(0.0, 0.05, 0.1) - 0.05) < 1e-9                     # small error is reached exactly


def test_set_poses_builds_one_request_for_all_models(monkeypatch):
    calls = []
    monkeypatch.setattr(m.subprocess, 'call', lambda cmd, **k: calls.append(cmd) or 0)
    assert m.set_poses('crowd', [('a', 1, 2, 0, 0.0), ('b', 3, 4, 0, 1.0)]) == 0
    assert len(calls) == 1 and calls[0].count('pose {') == 0 and calls[0][-1].count('pose {') == 2
    assert 'set_pose_vector' in calls[0][3]
