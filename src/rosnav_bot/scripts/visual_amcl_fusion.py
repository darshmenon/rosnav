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
"""
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float32
from std_srvs.srv import Trigger


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

        self.visual = None
        self.visual_t = None
        self.amcl = None
        self.disagree_since = None
        self.last_reseed = None

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

    def _on_amcl(self, msg):
        self.amcl = msg

    def _reseed(self, now, reason, dist, amcl_cov):
        vp = self.visual.pose.pose.position
        out = PoseWithCovarianceStamped()
        out.header.frame_id = 'map'
        out.header.stamp = now.to_msg()
        out.pose = self.visual.pose
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

        if not (amcl_lost or persistent):
            return
        if self.last_reseed is not None and \
                (now - self.last_reseed).nanoseconds * 1e-9 < self.get_parameter('cooldown_sec').value:
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
