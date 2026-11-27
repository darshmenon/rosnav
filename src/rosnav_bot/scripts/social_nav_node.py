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
import online_conformal as oc  # noqa: E402
import prediction_watchdog as pw  # noqa: E402

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
                         cbf_safe_dist=0.9, publish_period=0.25, max_history_s=6.0,
                         watchdog=True, latency_budget_ms=50.0, max_track_age_s=1.0,
                         online_conformal=True, conformal_json='',
                         mask_steps=4, mask_scale=0.25, mask_max_radius=0.8,
                         online_gate=True).items():
            self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value
        self.frame, self.res = g('map_frame'), float(g('resolution'))
        self.r0, self.rg = float(g('base_radius')), float(g('radius_growth'))
        # mask size limits: a mask covering the whole 4.8 s horizon with 2 m discs fills the corridor and freezes Nav2
        self.mask_steps, self.mask_scale, self.mask_rmax = int(g('mask_steps')), float(g('mask_scale')), float(g('mask_max_radius'))
        ns = g('robot_ns')
        pre = f'/{ns}' if ns else ''
        self.hist = collections.defaultdict(lambda: collections.deque(maxlen=200))
        self.max_hist = float(g('max_history_s'))
        self.tracks, self.paths, self.mode = [], {}, 'cv'
        self.net = self._load(g('weights'))
        # safety supervision: learned paths only shape the costmap while the watchdog says OK
        self.wd = pw.PredictionWatchdog(max_track_age_s=float(g('max_track_age_s')),
                                        latency_budget_ms=float(g('latency_budget_ms'))) if g('watchdog') else None
        self.last_track_t, self.last_ms, self.infer_error = None, None, False
        self.lat = collections.deque(maxlen=7)
        self.learned, self.cv, self.reasons = {}, {}, []
        # online conformal radii (start from the offline calibration when available)
        cj = g('conformal_json') or (g('weights') + '.conformal.json' if g('weights') else '')
        r0 = [0.5] * PRED
        if cj and os.path.exists(cj):
            try:
                r0 = json.load(open(cj))['radii']
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'conformal json unreadable ({e}); default radii')
        self.conf = oc.OnlineConformal(r0) if g('online_conformal') else None
        self.ledger = oc.PredictionLedger(self.conf, dt=DT) if self.conf else None
        self.pub_status = self.create_publisher(String, '/social_nav/status', 10)
        # online gate: blend learned toward constant velocity when CV has been closer to what really happened
        self.gate = oc.OnlineGate() if (g('online_gate') and self.net is not None) else None
        self.learned_raw = {}

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
        self.last_track_t = now
        for t in self.tracks:
            h = self.hist[t['id']]
            h.append((now, t['x'], t['y']))
            if self.ledger is not None:
                self.ledger.observe(t['id'], now, (t['x'], t['y']))
            while h and now - h[0][0] > self.max_hist:
                h.popleft()
        self.paths = self._predict(now)
        self.learned_raw, self.cv = self.paths, cv_paths(self.tracks)
        self.learned = self.learned_raw
        if self.gate is not None:
            for t in self.tracks:
                self.gate.observe(t['id'], now, (t['x'], t['y']))
            for k, p in self.learned_raw.items():
                if k in self.cv and not np.allclose(p, self.cv[k]):      # only genuinely learned paths
                    self.gate.add(k, now, p, self.cv[k])
            self.learned = self.gate.blend(self.learned_raw, self.cv)
        if self.ledger is not None:
            for k, p in self.paths.items():
                self.ledger.add(k, now, p)

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
        try:
            with self.torch.no_grad():
                y = self.net(self.torch.tensor(x), self.torch.tensor(m))[0].numpy()
            self.infer_error = False
        except Exception as e:  # noqa: BLE001
            self.infer_error = True
            self.get_logger().error(f'inference failed ({e}); watchdog will fall back')
            return paths
        self.last_ms = (time.perf_counter() - t0) * 1e3
        self.lat.append(self.last_ms)               # rolling median: ignore warm-up / one-off spikes
        for i, k in enumerate(ids):
            paths[k] = y[i]
        return paths

    # -- outputs
    def _supervise(self):
        """Run the watchdog; returns the paths the costmap may use (possibly none)."""
        if self.wd is None:
            return self.paths
        lat = float(np.median(self.lat)) if len(self.lat) >= 3 else None     # need >=3 samples (warm-up)
        mode, reasons, tr = self.wd.check(time.monotonic(), self.last_track_t, self.learned_raw or self.learned, self.cv,
                                          lat, self.infer_error)
        self.reasons, self.lat_med = reasons, lat
        if tr:
            self.get_logger().warn(f'watchdog {tr[0]} -> {tr[1]}: {",".join(reasons) or "recovered"}'
                                   f' (median latency {lat and round(lat, 1)} ms)')
        return pw.sanitize_paths(self.learned, self.cv, mode)

    def _tick(self):
        stamp = self.get_clock().now().to_msg()
        self.paths = self._supervise()
        mode = self.wd.mode if self.wd else 'OFF'
        st = dict(gate_g=None if self.gate is None else round(self.gate.g, 2), mode=mode, reasons=self.reasons, latency_ms=self.last_ms, tracks=len(self.tracks),
                  conformal_r=None if self.conf is None else [round(float(r), 2) for r in self.conf.radii[[0, 5, 11]]],
                  coverage=None if self.conf is None else [round(float(c), 2) for c in self.conf.cov[[0, 5, 11]]])
        self.pub_status.publish(String(data=json.dumps(st)))
        if mode == pw.FALLBACK:
            self.tracks = []                       # plain Nav2: publish an empty mask
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
            for i, (a, b) in enumerate(p[:self.mask_steps]):
                grow = (self.mask_scale * float(self.conf.r[min(i, len(self.conf.r) - 1)]) if self.conf is not None
                        else self.rg * (i + 1) * DT)
                pts.append((float(a), float(b), min(self.mask_rmax, self.r0 + grow)))
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
        if self.wd is not None and self.wd.mode == pw.FALLBACK:
            self.pub_cmd.publish(msg)                # tracks stale: don't act on them, plain Nav2 command
            return
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
        try:
            rclpy.spin(n)
        except Exception:  # noqa: BLE001 - a signal can invalidate the context mid-spin
            if rclpy.ok():
                raise
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
