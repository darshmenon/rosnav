#!/usr/bin/env python3
"""
visual_amcl_fusion.py — use a visual (RTAB-Map localization) pose to rescue
AMCL while navigating on a pre-built map (slam:=false visual_reloc:=true).

AMCL stays the sole map->odom publisher. RTAB-Map runs in localization mode
with publish_tf=false and only emits a pose estimate. This node compares the
two (both in the map frame):

  * logs/publishes the AMCL<->visual disagreement (std_msgs/Float32
    `visual_reloc/disagreement`, metres);
  * if AMCL looks lost (covariance trace above `cov_trace_thresh`) OR the
    disagreement stays above `disagree_thresh` for `disagree_hold_sec`, and
    the visual pose is fresh and confident, re-seeds AMCL by publishing the
    visual pose on /initialpose (rate-limited by `cooldown_sec`).

Visual pose is only trusted if its covariance is small (`visual_cov_max`),
so a bad visual match can't drag a healthy AMCL away.

Hardening against perceptual aliasing (all on by default, each can be switched off):
  * consistency gate  — the last `consistency_n` visual poses must agree within
    `consistency_xy` m / `consistency_yaw_deg` before a re-seed is considered;
  * scan verification — the candidate pose is scored with a laser likelihood field
    against /map; it must score >= `verify_min_score` and beat AMCL's own pose by
    `verify_margin`, so a wrong-but-confident visual match can't override a good AMCL;
  * inflated re-seed covariance — AMCL re-spreads particles around the visual pose
    (`reseed_xy_std`, `reseed_yaw_std`) instead of collapsing onto one hypothesis;
  * global-localization fallback — if AMCL stays lost for `global_loc_after_sec` with
    no usable visual pose, call /reinitialize_global_localization.
"""
import collections
import math

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from scipy.ndimage import distance_transform_edt
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_srvs.srv import Empty, Trigger


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class VisualAmclFusion(Node):
    def __init__(self):
        super().__init__('visual_amcl_fusion')
        self.declare_parameter('visual_pose_topic', '/localization_pose')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('cov_trace_thresh', 0.5)
        self.declare_parameter('disagree_thresh', 1.0)
        self.declare_parameter('disagree_hold_sec', 5.0)
        self.declare_parameter('visual_cov_max', 0.05)
        self.declare_parameter('visual_max_age_sec', 1.0)
        self.declare_parameter('cooldown_sec', 15.0)
        self.declare_parameter('consistency_n', 2)
        self.declare_parameter('consistency_xy', 0.3)
        self.declare_parameter('consistency_yaw_deg', 15.0)
        self.declare_parameter('verify_scan', True)
        self.declare_parameter('verify_min_score', 0.45)
        self.declare_parameter('verify_margin', 0.05)
        self.declare_parameter('verify_sigma', 0.15)
        self.declare_parameter('verify_max_beams', 90)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('reseed_xy_std', 0.35)
        self.declare_parameter('reseed_yaw_std', 0.25)
        self.declare_parameter('global_loc_after_sec', 40.0)

        self.visual = None
        self.visual_t = None
        self.amcl = None
        self.disagree_since = None
        self.last_reseed = None
        self.recent = collections.deque(maxlen=max(1, int(self.get_parameter('consistency_n').value)))
        self.scan = None
        self.laser_tf = None  # (x, y, yaw) of laser in base frame
        self.dist = None      # distance-to-nearest-obstacle grid [m]
        self.map_info = None
        self.lost_since = None
        self.last_global = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value, self._on_map,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.global_cli = self.create_client(Empty, '/reinitialize_global_localization')
        self.score_pub = self.create_publisher(Float32, 'visual_reloc/visual_score', 1)

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('visual_pose_topic').value, self._on_visual, qos)
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('amcl_pose_topic').value, self._on_amcl, qos)
        self.init_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 1)
        self.err_pub = self.create_publisher(Float32, 'visual_reloc/disagreement', 1)
        self.create_service(Trigger, 'visual_reloc/reseed', self._on_reseed_srv)
        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f'visual_amcl_fusion up — visual={self.get_parameter("visual_pose_topic").value} '
            f'amcl={self.get_parameter("amcl_pose_topic").value}')

    def _on_visual(self, msg):
        self.visual = msg
        self.visual_t = self.get_clock().now()
        p = msg.pose.pose
        self.recent.append((p.position.x, p.position.y, yaw_of(p.orientation)))

    def _on_scan(self, msg):
        self.scan = msg
        if self.laser_tf is None:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.get_parameter('base_frame').value, msg.header.frame_id, rclpy.time.Time())
                self.laser_tf = (t.transform.translation.x, t.transform.translation.y,
                                 yaw_of(t.transform.rotation))
            except Exception:
                pass

    def _on_map(self, msg):
        w, h = msg.info.width, msg.info.height
        grid = np.asarray(msg.data, dtype=np.int8).reshape(h, w)
        free = grid < 65  # unknown (-1) counts as traversable: only known walls attract
        self.dist = distance_transform_edt(free) * msg.info.resolution
        self.map_info = msg.info
        self.get_logger().info(f'scan verification map ready ({w}x{h} @ {msg.info.resolution:.3f} m)')

    def _consistent(self):
        n = self.recent.maxlen
        if len(self.recent) < n:
            return False
        ref = self.recent[-1]
        xy = self.get_parameter('consistency_xy').value
        yw = math.radians(self.get_parameter('consistency_yaw_deg').value)
        return all(math.hypot(p[0] - ref[0], p[1] - ref[1]) <= xy and abs(wrap(p[2] - ref[2])) <= yw
                   for p in self.recent)

    def _score(self, x, y, yaw):
        """Mean laser likelihood-field score in [0,1] of the latest scan at map pose (x,y,yaw)."""
        if self.scan is None or self.dist is None or self.laser_tf is None:
            return None
        lx, ly, lyaw = self.laser_tf
        c, s = math.cos(yaw), math.sin(yaw)
        ox, oy, oyaw = x + c * lx - s * ly, y + s * lx + c * ly, yaw + lyaw
        sc = self.scan
        r = np.asarray(sc.ranges, dtype=np.float64)
        ang = sc.angle_min + np.arange(len(r)) * sc.angle_increment
        ok = np.isfinite(r) & (r > max(sc.range_min, 0.05)) & (r < min(sc.range_max, 30.0) - 0.05)
        idx = np.flatnonzero(ok)
        if idx.size < 10:
            return None
        step = max(1, idx.size // int(self.get_parameter('verify_max_beams').value))
        idx = idx[::step]
        ex = ox + r[idx] * np.cos(oyaw + ang[idx])
        ey = oy + r[idx] * np.sin(oyaw + ang[idx])
        info = self.map_info
        gx = np.floor((ex - info.origin.position.x) / info.resolution).astype(int)
        gy = np.floor((ey - info.origin.position.y) / info.resolution).astype(int)
        inside = (gx >= 0) & (gx < info.width) & (gy >= 0) & (gy < info.height)
        d = np.full(idx.size, 5.0)
        d[inside] = self.dist[gy[inside], gx[inside]]
        sig = self.get_parameter('verify_sigma').value
        return float(np.mean(np.exp(-0.5 * (d / sig) ** 2)))

    def _verified(self):
        """True if the visual pose explains the scan well and better than AMCL's pose."""
        if not self.get_parameter('verify_scan').value:
            return True
        vp = self.visual.pose.pose
        sv = self._score(vp.position.x, vp.position.y, yaw_of(vp.orientation))
        if sv is None:
            self.get_logger().info('scan verification unavailable (no scan/map/laser tf yet) — holding',
                                   throttle_duration_sec=5.0)
            return False
        self.score_pub.publish(Float32(data=sv))
        ap = self.amcl.pose.pose
        sa = self._score(ap.position.x, ap.position.y, yaw_of(ap.orientation)) or 0.0
        good = sv >= self.get_parameter('verify_min_score').value and \
            sv >= sa + self.get_parameter('verify_margin').value
        self.get_logger().info(
            f'scan verify: visual={sv:.2f} amcl={sa:.2f} -> {"ACCEPT" if good else "REJECT"}',
            throttle_duration_sec=2.0)
        return good

    def _maybe_global_loc(self, now, amcl_lost):
        if not amcl_lost:
            self.lost_since = None
            return
        if self.lost_since is None:
            self.lost_since = now
        lost_for = (now - self.lost_since).nanoseconds * 1e-9
        if lost_for < self.get_parameter('global_loc_after_sec').value:
            return
        if self.last_global is not None and \
                (now - self.last_global).nanoseconds * 1e-9 < self.get_parameter('global_loc_after_sec').value:
            return
        if self.global_cli.service_is_ready():
            self.global_cli.call_async(Empty.Request())
            self.last_global = now
            self.get_logger().warn(f'AMCL lost {lost_for:.0f}s with no verified visual pose — '
                                   f'global localization')

    def _on_amcl(self, msg):
        self.amcl = msg

    def _reseed(self, now, reason, dist, amcl_cov):
        vp = self.visual.pose.pose.position
        out = PoseWithCovarianceStamped()
        out.header.frame_id = 'map'
        out.header.stamp = now.to_msg()
        out.pose = self.visual.pose
        cov = list(out.pose.covariance)
        xy = self.get_parameter('reseed_xy_std').value ** 2
        yw = self.get_parameter('reseed_yaw_std').value ** 2
        cov[0], cov[7], cov[35] = max(cov[0], xy), max(cov[7], xy), max(cov[35], yw)
        out.pose.covariance = cov
        self.init_pub.publish(out)
        self.last_reseed = now
        self.disagree_since = None
        self.get_logger().warn(
            f'RE-SEEDING AMCL from visual pose ({vp.x:.2f},{vp.y:.2f}) — '
            f'reason={reason} dist={dist:.2f}m amcl_cov={amcl_cov:.3f}')

    def _on_reseed_srv(self, _req, resp):
        # Manual / BT-triggered re-seed (ignores cooldown, still needs a fresh visual pose).
        now = self.get_clock().now()
        if self.visual is None or \
                (now - self.visual_t).nanoseconds * 1e-9 > self.get_parameter('visual_max_age_sec').value:
            resp.success, resp.message = False, 'no fresh visual pose'
            return resp
        ap = self.amcl.pose.pose.position if self.amcl else None
        vp = self.visual.pose.pose.position
        dist = math.hypot(vp.x - ap.x, vp.y - ap.y) if ap else float('nan')
        self._reseed(now, 'service_call', dist, self._cov_trace_xy(self.amcl) if self.amcl else float('nan'))
        resp.success, resp.message = True, f'reseeded at ({vp.x:.2f},{vp.y:.2f})'
        return resp

    @staticmethod
    def _cov_trace_xy(msg):
        c = msg.pose.covariance
        return c[0] + c[7]

    def _tick(self):
        if self.visual is None or self.amcl is None:
            self.get_logger().info(
                f'waiting for poses (visual={self.visual is not None}, '
                f'amcl={self.amcl is not None})', throttle_duration_sec=5.0)
            return
        now = self.get_clock().now()
        age = (now - self.visual_t).nanoseconds * 1e-9
        if age > self.get_parameter('visual_max_age_sec').value:
            self.get_logger().info(
                f'visual pose stale ({age:.1f}s) — no fusion', throttle_duration_sec=5.0)
            self.disagree_since = None
            self._maybe_global_loc(now, self._cov_trace_xy(self.amcl) > self.get_parameter('cov_trace_thresh').value)
            return

        vp, ap = self.visual.pose.pose.position, self.amcl.pose.pose.position
        dist = math.hypot(vp.x - ap.x, vp.y - ap.y)
        self.err_pub.publish(Float32(data=float(dist)))
        amcl_cov = self._cov_trace_xy(self.amcl)
        vis_cov = self._cov_trace_xy(self.visual)
        self.get_logger().info(
            f'disagreement={dist:.2f}m amcl_cov={amcl_cov:.3f} visual_cov={vis_cov:.3f}',
            throttle_duration_sec=5.0)

        if vis_cov > self.get_parameter('visual_cov_max').value:
            self.disagree_since = None
            return

        amcl_lost = amcl_cov > self.get_parameter('cov_trace_thresh').value
        if dist > self.get_parameter('disagree_thresh').value:
            if self.disagree_since is None:
                self.disagree_since = now
            held = (now - self.disagree_since).nanoseconds * 1e-9
            persistent = held >= self.get_parameter('disagree_hold_sec').value
        else:
            self.disagree_since = None
            persistent = False

        self._maybe_global_loc(now, amcl_lost)
        if not (amcl_lost or persistent):
            return
        if self.last_reseed is not None and \
                (now - self.last_reseed).nanoseconds * 1e-9 < self.get_parameter('cooldown_sec').value:
            return

        if not self._consistent():
            self.get_logger().info('visual pose not yet consistent — holding', throttle_duration_sec=5.0)
            return
        if not self._verified():
            return
        self._reseed(now, 'amcl_lost' if amcl_lost else 'persistent_disagreement', dist, amcl_cov)

def main():
    rclpy.init()
    node = VisualAmclFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
