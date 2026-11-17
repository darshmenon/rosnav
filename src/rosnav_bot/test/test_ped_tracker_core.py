import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import ped_tracker_core as pt  # noqa: E402


def _scene(n=5, steps=200, seed=0, dropout=0.1, noise=0.05):
    rng = np.random.default_rng(seed)
    pos = rng.uniform(0, 10, (n, 2))
    vel = rng.uniform(-1, 1, (n, 2))
    for k in range(steps):
        pos = pos + vel * 0.1
        det, who = [], []
        for i in range(n):
            if rng.random() > dropout:
                det.append(pos[i] + rng.normal(0, noise, 2)); who.append(i)
        order = rng.permutation(len(det))              # detections arrive unordered
        yield [det[j] for j in order], [who[j] for j in order], vel


def test_ids_stable_and_velocity_estimated():
    pt._Track._next = 1
    tr = pt.PedTracker(dt=0.1)
    verr = []
    for k, (det, who, vel) in enumerate(_scene()):
        out = tr.update(det)
        if k == 100:
            ids = {t['id'] for t in out}
            assert len(ids) >= 4                        # most people tracked at mid-run
        if k > 150:
            for t in out:
                verr.append(min(np.hypot(t['vx'] - v[0], t['vy'] - v[1]) for v in vel))
    assert np.mean(verr) < 0.25                         # velocity within 0.25 m/s on average
    assert len(tr.tracks) <= 8                          # no track explosion


def test_track_survives_short_occlusion():
    pt._Track._next = 1
    tr = pt.PedTracker(dt=0.1, max_age=8, min_hits=2)
    for k in range(20):
        tr.update([(1.0 + 0.1 * k, 0.0)])
    tid = tr.tracks[0].id
    for k in range(20, 24):
        tr.update([])                                   # 4 missed frames
    out = tr.update([(1.0 + 0.1 * 24, 0.0)])
    assert out and out[0]['id'] == tid


def test_backprojection_centre_is_straight_ahead():
    f, l = pt.pixel_depth_to_xy(320, 240, 3.0, 500, 500, 320, 240)
    assert f == 3.0 and abs(l) < 1e-9
