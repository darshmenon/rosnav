#!/usr/bin/env python3
"""
social_nav_node.py — learned pedestrian trajectory prediction for Nav2.

  /obstacle_tracker/state (JSON tracks) -> per-track history -> predictor
        |-> /social_nav/predicted_paths   MarkerArray (RViz: predicted paths + uncertainty discs)
        |-> /social_nav/keepout_mask      OccupancyGrid, cost where people WILL be
        |-> cmd_vel_in -> CBF safety layer -> cmd_vel_out   (optional, use_cbf:=true)

Predictor: the social-attention transformer from train_ped_predictor.py when
`weights` is given and >= 3.2 s of history exists for the scene; otherwise
constant velocity. The learned output NEVER drives the robot directly: it only
shapes the costmap, and the optional CBF layer clamps whatever cmd_vel Nav2
produces (fallback = plain Nav2 behaviour if prediction or TF is missing).

Uncertainty (unique touch): the mask disc around each predicted point grows with
lookahead (r = base + growth * t) so far-future cells are "softer but wider".

  ros2 run rosnav_bot social_nav_node.py --ros-args \
      -p weights:=/home/asimov/rosnav/outputs/ped_predictor_n24.pt -p use_cbf:=true \
      -r cmd_vel_in:=/cmd_vel_nav -r cmd_vel_out:=/cmd_vel
"""
import collections
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

OBS, PRED, DT = 8, 12, 0.4
_MASK_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)


def cv_paths(tracks, steps=PRED, dt=DT):
    """Constant-velocity fallback: {id: (steps,2)}."""
    t = np.arange(1, steps + 1)[:, None] * dt
    return {k['id']: np.array([k['x'], k['y']]) + t * np.array([k['vx'], k['vy']])
            for k in tracks}


def resample(hist, now):
    """hist: deque of (t,x,y), returns (OBS,2) at DT spacing ending at now, or None."""
    if len(hist) < 4 or now - hist[0][0] < (OBS - 1) * DT:
        return None
    a = np.array(hist)
    ts = now - (OBS - 1 - np.arange(OBS)) * DT
    return np.stack([np.interp(ts, a[:, 0], a[:, 1]), np.interp(ts, a[:, 0], a[:, 2])], 1)


class SocialNav(Node):
    def __init__(self):
        super().__init__('social_nav')
        for k, v in dict(weights='', robot_ns='', map_frame='map', base_frame='base_link',
                         mask_topic='/social_nav/keepout_mask', resolution=0.10,
                         base_radius=0.35, radius_growth=0.12, use_cbf=False,
                         cbf_safe_dist=0.9, publish_period=0.25, max_history_s=6.0).items():
            self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value
        self.frame, self.res = g('map_frame'), float(g('resolution'))
        self.r0, self.rg = float(g('base_radius')), float(g('radius_growth'))
        ns = g('robot_ns')
        pre = f'/{ns}' if ns else ''
        self.hist = collections.defaultdict(lambda: collections.deque(maxlen=200))
        self.max_hist = float(g('max_history_s'))
        self.tracks, self.paths, self.mode = [], {}, 'cv'
        self.net = self._load(g('weights'))

        self.create_subscription(String, f'{pre}/obstacle_tracker/state', self._on_state, 10)
        self.pub_mask = self.create_publisher(OccupancyGrid, g('mask_topic'), _MASK_QOS)
        self.pub_mark = self.create_publisher(MarkerArray, '/social_nav/predicted_paths', 10)
        self.create_timer(float(g('publish_period')), self._tick)

        self.use_cbf = bool(g('use_cbf'))
        if self.use_cbf:
            import tf2_ros
            from cbf_safety_filter import cbf_filter
            self._cbf = cbf_filter
            self.cbf_k = dict(safe_dist=float(g('cbf_safe_dist')))
            self.base_frame = g('base_frame')
            self.buf = tf2_ros.Buffer()
            self.tl = tf2_ros.TransformListener(self.buf, self)
            self.pub_cmd = self.create_publisher(Twist, 'cmd_vel_out', 10)
            self.create_subscription(Twist, 'cmd_vel_in', self._on_cmd, 10)
        self.get_logger().info(
            f'social_nav up: predictor={self.mode} cbf={self.use_cbf} '
            f'weights={g("weights") or "none"} frame={self.frame}')

    def _load(self, w):
        if not w:
            return None
        try:
            import torch
            import train_ped_predictor as tp
            self.torch, self.tp = torch, tp
            net = tp.SocialTransformer()
            net.load_state_dict(torch.load(w, map_location='cpu'))
            net.eval()
            self.mode = 'learned(CV fallback until history fills)'
            return net
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'learned predictor unavailable ({e}); using constant velocity')
            return None

    # -- inputs
    def _on_state(self, msg):
        try:
            self.tracks = json.loads(msg.data).get('moving_obstacles', [])
        except (json.JSONDecodeError, TypeError):
            return
        now = time.monotonic()
        for t in self.tracks:
            h = self.hist[t['id']]
            h.append((now, t['x'], t['y']))
            while h and now - h[0][0] > self.max_hist:
                h.popleft()
        self.paths = self._predict(now)

    def _predict(self, now):
        paths = cv_paths(self.tracks)
        if self.net is None or not self.tracks:
            return paths
        ids, obs = [], []
        for t in self.tracks[:self.tp.NMAX]:
            o = resample(self.hist[t['id']], now)
            if o is None:
                return paths                     # not enough history -> CV for all
            ids.append(t['id']); obs.append(o)
        x = np.zeros((1, self.tp.NMAX, OBS, 2), np.float32)
        m = np.zeros((1, self.tp.NMAX), bool)
        x[0, :len(obs)] = np.stack(obs); m[0, :len(obs)] = True
        t0 = time.perf_counter()
        with self.torch.no_grad():
            y = self.net(self.torch.tensor(x), self.torch.tensor(m))[0].numpy()
        self.last_ms = (time.perf_counter() - t0) * 1e3
        for i, k in enumerate(ids):
            paths[k] = y[i]
        return paths

    # -- outputs
    def _tick(self):
        stamp = self.get_clock().now().to_msg()
        ma = MarkerArray()
        clear = Marker(); clear.action = Marker.DELETEALL; ma.markers.append(clear)
        for k, p in self.paths.items():
            mk = Marker()
            mk.header.frame_id, mk.header.stamp = self.frame, stamp
            mk.ns, mk.id, mk.type = 'pred_path', int(k), Marker.LINE_STRIP
            mk.scale.x = 0.05
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.5, 0.0, 0.9
            mk.points = [Point(x=float(a), y=float(b), z=0.1) for a, b in p]
            ma.markers.append(mk)
        self.pub_mark.publish(ma)
        self._publish_mask(stamp)

    def _publish_mask(self, stamp):
        pts = []   # (x, y, radius)
        for t in self.tracks:
            pts.append((t['x'], t['y'], self.r0))
        for p in self.paths.values():
            for i, (a, b) in enumerate(p):
                pts.append((float(a), float(b), self.r0 + self.rg * (i + 1) * DT))
        grid = OccupancyGrid()
        grid.header.stamp, grid.header.frame_id = stamp, self.frame
        grid.info.resolution = self.res
        grid.info.origin.orientation.w = 1.0
        if not pts:
            grid.info.width = grid.info.height = 1
            grid.data = [0]
            self.pub_mask.publish(grid)
            return
        pad = max(r for *_, r in pts)
        x0 = min(p[0] for p in pts) - pad; x1 = max(p[0] for p in pts) + pad
        y0 = min(p[1] for p in pts) - pad; y1 = max(p[1] for p in pts) + pad
        w = max(1, int(math.ceil((x1 - x0) / self.res)))
        h = max(1, int(math.ceil((y1 - y0) / self.res)))
        yy, xx = np.mgrid[0:h, 0:w]
        wx, wy = x0 + (xx + 0.5) * self.res, y0 + (yy + 0.5) * self.res
        occ = np.zeros((h, w), bool)
        for cx, cy, r in pts:
            occ |= (wx - cx) ** 2 + (wy - cy) ** 2 <= r * r
        grid.info.width, grid.info.height = w, h
        grid.info.origin.position.x, grid.info.origin.position.y = x0, y0
        grid.data = (occ.astype(np.int8) * 100).flatten().tolist()
        self.pub_mask.publish(grid)

    # -- safety layer
    def _on_cmd(self, msg):
        try:
            tf = self.buf.lookup_transform(self.frame, self.base_frame, rclpy.time.Time()).transform
            q = tf.rotation
            me = (tf.translation.x, tf.translation.y,
                  math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        except Exception:  # noqa: BLE001
            self.pub_cmd.publish(msg)            # no TF: pass through (Nav2 monitor still runs)
            return
        nb = [(t['x'], t['y'], t['vx'], t['vy']) for t in self.tracks]
        nb += [(t['x'] + 0.8 * t['vx'], t['y'] + 0.8 * t['vy'], t['vx'], t['vy'])
               for t in self.tracks]             # also keep clear of where they will be
        v, w, bad = self._cbf((msg.linear.x, msg.angular.z), me, nb, **self.cbf_k)
        out = Twist(); out.linear.x, out.angular.z = (0.0 if bad else v), w
        self.pub_cmd.publish(out)


def main():
    rclpy.init()
    n = SocialNav()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
