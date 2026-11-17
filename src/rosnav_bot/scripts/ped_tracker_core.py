#!/usr/bin/env python3
"""
ped_tracker_core.py — multi-pedestrian tracker (pure Python, no ROS).

Camera/lidar detections (any source: YOLO person boxes + depth, DR-SPAAM, ground
truth) -> persistent IDs + velocity estimates, in the format the predictor needs
(positions in a common frame). Constant-velocity Kalman filter per track,
global nearest-neighbour association with the Hungarian algorithm
(scipy.optimize.linear_sum_assignment) and Mahalanobis gating; tracks coast
through short occlusions and are retired after `max_age` missed frames.

  tracker = PedTracker(dt=0.1)
  tracks = tracker.update([(x, y), ...])      # -> [{'id','x','y','vx','vy','age','hits'}]

Helper: pixel_depth_to_xy() back-projects a detection (bbox bottom-centre pixel + depth)
through a pinhole camera into the robot/base frame so a YOLO person box becomes a point.
"""
import math

import numpy as np
from scipy.optimize import linear_sum_assignment


class _Track:
    _next = 1

    def __init__(self, z, dt, q, r):
        self.id = _Track._next
        _Track._next += 1
        self.x = np.array([z[0], z[1], 0.0, 0.0])
        self.P = np.diag([r, r, 1.0, 1.0])
        self.hits, self.age, self.miss = 1, 1, 0
        self.F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], float)
        g = np.array([[0.5 * dt * dt, 0], [0, 0.5 * dt * dt], [dt, 0], [0, dt]])
        self.Q = q * g @ g.T
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], float)
        self.R = np.eye(2) * r

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.age += 1

    def gate_dist(self, z):
        S = self.H @ self.P @ self.H.T + self.R
        y = np.asarray(z) - self.H @ self.x
        return float(y @ np.linalg.solve(S, y))

    def update(self, z):
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (np.asarray(z) - self.H @ self.x)
        self.P = (np.eye(4) - K @ self.H) @ self.P
        self.hits += 1
        self.miss = 0


class PedTracker:
    def __init__(self, dt=0.1, process_noise=2.0, meas_var=0.05, gate=9.21, max_age=8, min_hits=3):
        self.dt, self.q, self.r, self.gate = dt, process_noise, meas_var, gate  # 9.21 = chi2(2), 99%
        self.max_age, self.min_hits = max_age, min_hits
        self.tracks = []

    def update(self, detections):
        for t in self.tracks:
            t.predict()
        det = [tuple(d) for d in detections]
        matched_t, matched_d = set(), set()
        if self.tracks and det:
            cost = np.array([[t.gate_dist(d) for d in det] for t in self.tracks])
            cost_g = np.where(cost > self.gate, 1e6, cost)
            for ti, di in zip(*linear_sum_assignment(cost_g)):
                if cost[ti, di] <= self.gate:
                    self.tracks[ti].update(det[di])
                    matched_t.add(ti); matched_d.add(di)
        for ti, t in enumerate(self.tracks):
            if ti not in matched_t:
                t.miss += 1
        for di, d in enumerate(det):
            if di not in matched_d:
                self.tracks.append(_Track(d, self.dt, self.q, self.r))
        self.tracks = [t for t in self.tracks if t.miss <= self.max_age]
        return [dict(id=t.id, x=float(t.x[0]), y=float(t.x[1]), vx=float(t.x[2]), vy=float(t.x[3]),
                     age=t.age, hits=t.hits)
                for t in self.tracks if t.hits >= self.min_hits and t.miss == 0]


def pixel_depth_to_xy(u, v, depth, fx, fy, cx, cy):
    """Pinhole back-projection of pixel (u, v) at `depth` metres -> (forward, left) in the camera's
    horizontal plane: forward = depth, left = -(u - cx) * depth / fx. (v is unused for a ground plane.)"""
    return depth, -(u - cx) * depth / fx
