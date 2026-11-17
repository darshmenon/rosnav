import json
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import ped_detector_core as dc  # noqa: E402
import ped_tracker_core as pt  # noqa: E402

W, H, FOV = 640, 480, math.pi / 2
FX = W / 2 / math.tan(FOV / 2)
K = (FX, FX, W / 2, H / 2)
CAM = dc.yaw_transform(0.1, 0.0, 0.0, z=0.25) @ dc.OPTICAL_TO_BODY      # camera on the robot, looking forward


def render(peds_xy, rng, dropout=0.0, noise=0.01):
    """Synthetic RGB-D view: for each person a box + a depth image with the torso at its front-surface depth."""
    depth = np.full((H, W), 9.0, np.float32)
    boxes = []
    inv = np.linalg.inv(CAM)
    for x, y in peds_xy:
        pc = (inv @ np.array([x, y, 0.9, 1.0]))[:3]                 # body centre (z=0.9 m) in the optical frame
        if pc[2] < 0.5:
            continue
        u, v = K[2] + K[0] * pc[0] / pc[2], K[3] + K[1] * pc[1] / pc[2]
        bw, bh = K[0] * 0.44 / pc[2], K[1] * 1.7 / pc[2]
        x1, x2, y1, y2 = u - bw / 2, u + bw / 2, v - bh / 2, v + bh / 2
        front = pc[2] - 0.2
        depth[int(max(0, y1)):int(min(H, y2)), int(max(0, x1)):int(min(W, x2))] = front
        if rng.random() > dropout and 0 <= u < W:
            boxes.append((x1, y1, x2, y2, 0.9))
    depth += rng.normal(0, noise, depth.shape).astype(np.float32)
    return boxes, depth


def test_backprojection_roundtrip():
    rng = np.random.default_rng(0)
    for xy in [(4.0, 0.0), (5.0, 1.5), (3.0, -1.0)]:
        boxes, depth = render([xy], rng)
        det = dc.detections_from_boxes(boxes, depth, K, CAM, body_offset=0.2)
        assert len(det) == 1
        assert math.hypot(det[0]['x'] - xy[0], det[0]['y'] - xy[1]) < 0.15


def test_low_confidence_and_missing_depth_dropped():
    rng = np.random.default_rng(1)
    boxes, depth = render([(4.0, 0.0)], rng)
    assert dc.detections_from_boxes([(*boxes[0][:4], 0.1)], depth, K, CAM) == []
    depth[:] = np.nan
    assert dc.detections_from_boxes(boxes, depth, K, CAM) == []


def test_camera_detections_round_trip_through_kalman_tracker():
    rng = np.random.default_rng(2)
    pt._Track._next = 1
    tr = pt.PedTracker(dt=0.1, meas_var=0.05)
    pos = np.array([[4.0, -1.2], [5.0, 0.2], [3.5, 1.3]])
    vel = np.array([[0.3, 0.08], [-0.2, 0.0], [0.4, -0.08]])        # three lanes, nobody crosses (ID swaps at crossings are the tracker's job, not the detector's)
    perr, verr, ids_seen = [], [], set()
    for k in range(60):
        pos = pos + vel * 0.1
        boxes, depth = render(pos, rng, dropout=0.1)
        dets = dc.detections_from_boxes(boxes, depth, K, CAM, body_offset=0.2)
        tracks = tr.update([(d['x'], d['y']) for d in dets])
        if k >= 10:
            ids_seen |= {t['id'] for t in tracks}
        if k > 40:
            for t in tracks:
                j = int(np.argmin(np.hypot(pos[:, 0] - t['x'], pos[:, 1] - t['y'])))
                perr.append(math.hypot(pos[j, 0] - t['x'], pos[j, 1] - t['y']))
                verr.append(math.hypot(vel[j, 0] - t['vx'], vel[j, 1] - t['vy']))
    assert len(ids_seen) == 3                                 # three people -> exactly three IDs, no switches/duplicates
    assert len(tr.tracks) <= 4                                # no track explosion from the 10% dropouts
    assert np.mean(perr) < 0.2 and np.mean(verr) < 0.3


def test_state_json_matches_obstacle_tracker_format():
    s = json.loads(dc.to_state_json([dict(id=3, x=1.234, y=-2.0, vx=0.5, vy=0.1, hits=7)]))
    o = s['moving_obstacles'][0]
    for k in ('id', 'x', 'y', 'vx', 'vy', 'speed', 'points', 'length', 'width', 'orientation'):
        assert k in o                                         # social_nav_node reads id/x/y/vx/vy
    assert o['x'] == 1.23 and abs(o['speed'] - 0.51) < 1e-9


def _scan(n=360, targets=(), wall_r=4.0):
    ang = -math.pi + 2 * math.pi * np.arange(n) / n
    r = np.full(n, wall_r)
    for cx, cy, rad in targets:
        for i, a in enumerate(ang):
            d = np.array([math.cos(a), math.sin(a)])
            b = d @ np.array([cx, cy])
            disc = b * b - (cx * cx + cy * cy - rad * rad)
            if disc >= 0 and b > 0:
                r[i] = min(r[i], b - math.sqrt(disc))
    return r, ang[0], ang[1] - ang[0]


def test_lidar_cluster_fallback_finds_people_not_walls():
    r, a0, da = _scan(targets=[(2.0, 0.5, 0.2), (-1.5, -2.0, 0.2)])
    T = dc.yaw_transform(1.0, 2.0, 0.0)
    det = dc.lidar_clusters(r, a0, da, T)
    got = sorted((round(d['x'] - 1.0, 1), round(d['y'] - 2.0, 1)) for d in det)
    assert len(det) == 2
    for (gx, gy), (tx, ty) in zip(got, sorted([(-1.5, -2.0), (2.0, 0.5)])):
        assert math.hypot(gx - tx, gy - ty) < 0.35            # centroid within a person-width of the truth
    # a static filter drops everything on known structure
    assert dc.lidar_clusters(r, a0, da, T, static_fn=lambda x, y: True) == []


YOLO = os.path.expanduser('~/rosnav/yolov8n.pt')


@pytest.mark.skipif(not os.path.exists(YOLO), reason='no local YOLO weights')
def test_yolo_finds_people_in_a_real_photo():
    ul = pytest.importorskip('ultralytics')
    img = os.path.join(os.path.dirname(ul.__file__), 'assets', 'bus.jpg')
    if not os.path.exists(img):
        pytest.skip('no sample image')
    res = ul.YOLO(YOLO)(img, classes=[0], verbose=False, device='cpu')[0]
    assert len(res.boxes) >= 3                                # the bus photo has four people
