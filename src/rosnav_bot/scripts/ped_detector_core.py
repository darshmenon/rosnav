#!/usr/bin/env python3
"""
ped_detector_core.py — detections -> map-frame pedestrian points (pure Python / numpy, no ROS, no model).

Two detection sources, both ending as a list of {x, y, conf, src} in the map frame:

  * camera : person boxes (from YOLO, or anything) + a depth image + pinhole intrinsics + the camera->map transform.
             depth is the MEDIAN of a small patch on the box's central torso band (robust to background pixels
             inside the box), then the 3D point is back-projected and transformed to the map frame.
  * lidar  : "leg / body cluster" fallback for robots with no camera (the default diff_drive has a 2D lidar only):
             split the scan at range jumps, keep clusters whose width looks like a person, centroid -> map frame.

to_state_json() writes the same JSON that obstacle_tracker.py publishes on /obstacle_tracker/state, so
social_nav_node.py consumes a camera-driven tracker without any change.

Frames: camera optical frame is X right, Y down, Z forward (ROS convention).
"""
import json
import math

import numpy as np


# ---------------------------------------------------------------- transforms
def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def make_transform(tx, ty, tz, qx, qy, qz, qw):
    """4x4 matrix of a ROS transform (child frame points -> parent frame)."""
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(qx, qy, qz, qw)
    T[:3, 3] = (tx, ty, tz)
    return T


def yaw_transform(x, y, yaw, z=0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    T = np.eye(4)
    T[:2, :2] = [[c, -s], [s, c]]
    T[:3, 3] = (x, y, z)
    return T


OPTICAL_TO_BODY = np.array([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]], float)
"""optical (X right, Y down, Z fwd) -> body (X fwd, Y left, Z up) rotation, for building camera->map when only
the camera_link pose is known."""


def intrinsics(info_k):
    """CameraInfo.k (row-major 3x3) -> fx, fy, cx, cy."""
    return float(info_k[0]), float(info_k[4]), float(info_k[2]), float(info_k[5])


def backproject(u, v, depth, fx, fy, cx, cy):
    """Pixel + depth along the optical axis -> 3D point in the optical frame."""
    return np.array([(u - cx) * depth / fx, (v - cy) * depth / fy, depth])


# ---------------------------------------------------------------- camera
def box_depth(depth_img, box, band=(0.35, 0.65), shrink=0.5, min_valid=8, max_depth=12.0):
    """Median valid depth (m) on the central `shrink` width of the box, rows `band` of its height (torso).
    Returns None if too few valid pixels. depth_img: float metres, NaN/0/inf = invalid."""
    x1, y1, x2, y2 = box
    h, w = depth_img.shape
    cx, bw = 0.5 * (x1 + x2), (x2 - x1) * shrink
    ua, ub = int(max(0, cx - bw / 2)), int(min(w, cx + bw / 2 + 1))
    va, vb = int(max(0, y1 + band[0] * (y2 - y1))), int(min(h, y1 + band[1] * (y2 - y1) + 1))
    if ub <= ua or vb <= va:
        return None
    patch = depth_img[va:vb, ua:ub].ravel()
    patch = patch[np.isfinite(patch) & (patch > 0.05) & (patch < max_depth)]
    if len(patch) < min_valid:
        return None
    return float(np.median(patch))


def detections_from_boxes(boxes, depth_img, K, T_cam_to_map, body_offset=0.0, min_conf=0.35):
    """boxes: iterable of (x1, y1, x2, y2, conf) in pixels. K = (fx, fy, cx, cy).
    T_cam_to_map: 4x4, OPTICAL frame -> map. body_offset: metres added along the ray (the depth hits the
    front surface of the torso; the body centre is ~0.15-0.25 m further). Returns [{x, y, conf, depth, src}]."""
    fx, fy, cx, cy = K
    out = []
    for x1, y1, x2, y2, conf in boxes:
        if conf < min_conf:
            continue
        d = box_depth(depth_img, (x1, y1, x2, y2))
        if d is None:
            continue
        u, v = 0.5 * (x1 + x2), y1 + 0.5 * (y2 - y1)
        p_cam = backproject(u, v, d + body_offset, fx, fy, cx, cy)
        p = T_cam_to_map @ np.append(p_cam, 1.0)
        out.append(dict(x=float(p[0]), y=float(p[1]), conf=float(conf), depth=d, src='camera'))
    return out


# ---------------------------------------------------------------- lidar fallback
def lidar_clusters(ranges, angle_min, angle_inc, T_laser_to_map, max_range=6.0, jump=0.25,
                   min_pts=3, width=(0.08, 0.9), static_fn=None):
    """Person-sized clusters in a 2D scan -> [{x, y, conf, width, src}] in the map frame.
    Rays are grouped while consecutive range jumps stay below `jump` metres. A cluster is kept when its
    chord width (first-to-last point) is within `width`. static_fn(x, y) -> True drops clusters sitting on
    known static structure (pass e.g. a map-distance test so wall corners are not 'people')."""
    r = np.asarray(ranges, float)
    ang = angle_min + angle_inc * np.arange(len(r))
    ok = np.isfinite(r) & (r > 0.1) & (r < max_range)
    pts = np.stack([r * np.cos(ang), r * np.sin(ang)], 1)
    groups, cur = [], []
    for i in range(len(r)):
        if not ok[i]:
            if cur:
                groups.append(cur); cur = []
            continue
        if cur and abs(r[i] - r[cur[-1]]) > jump:
            groups.append(cur); cur = []
        cur.append(i)
    if cur:
        groups.append(cur)
    out = []
    for g in groups:
        if len(g) < min_pts:
            continue
        P = pts[g]
        w = float(np.linalg.norm(P[-1] - P[0]))
        if not (width[0] <= w <= width[1]):
            continue
        c = P.mean(0)
        c = c + 0.12 * c / (np.linalg.norm(c) + 1e-9)          # lidar sees the near surface; centre is behind it
        p = T_laser_to_map @ np.array([c[0], c[1], 0.0, 1.0])
        if static_fn is not None and static_fn(p[0], p[1]):
            continue
        out.append(dict(x=float(p[0]), y=float(p[1]), conf=min(1.0, len(g) / 8.0), width=w, src='lidar'))
    return out


# ---------------------------------------------------------------- output format
def to_state_json(tracks):
    """PedTracker tracks -> the JSON obstacle_tracker.py publishes on /obstacle_tracker/state."""
    return json.dumps({'moving_obstacles': [
        {'id': int(t['id']), 'x': round(t['x'], 2), 'y': round(t['y'], 2), 'vx': round(t['vx'], 2),
         'vy': round(t['vy'], 2), 'speed': round(math.hypot(t['vx'], t['vy']), 2), 'points': int(t.get('hits', 0)),
         'length': 0.5, 'width': 0.5, 'orientation': 0.0} for t in tracks]})


def to_detections_json(dets, stamp, frame):
    return json.dumps({'t': stamp, 'frame': frame, 'detections': [
        {k: (round(v, 3) if isinstance(v, float) else v) for k, v in d.items()} for d in dets]})
