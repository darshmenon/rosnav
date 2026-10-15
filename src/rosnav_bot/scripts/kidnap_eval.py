#!/usr/bin/env python3
"""
kidnap_eval.py — localization-recovery benchmark on a pre-built map (slam:=false).

1. Wait `settle_sec` for AMCL to converge, align ground truth onto the map frame
   (same first-pair rigid alignment as slam_accuracy_monitor.py).
2. "Kidnap": publish a wrong /initialpose (true pose + offset_x/offset_y/offset_yaw_deg).
3. Track map->base_link error vs ground truth. Recovered = error < `ok_err` m for
   `ok_hold_sec` consecutive seconds. Gives up after `timeout_sec`.

Writes <out_dir>/<label>.json: pre-kidnap error, kidnap-time error, recovery time
(null if never), final error, mean gs_reloc/color_error before/after (if published).
"""
import json
import math
import time
from pathlib import Path

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class KidnapEval(Node):
    def __init__(self):
        super().__init__('kidnap_eval')
        for k, v in dict(label='run', out_dir='/tmp', gt_topic='/ground_truth', settle_sec=25.0,
                         offset_x=2.0, offset_y=1.0, offset_yaw_deg=90.0, ok_err=0.5, ok_hold_sec=3.0,
                         timeout_sec=90.0).items():
            self.declare_parameter(k, v)
        p = lambda k: self.get_parameter(k).value
        self.p = p
        self.buf = tf2_ros.Buffer(node=self)
        self.lis = tf2_ros.TransformListener(self.buf, self)
        self.gt = None
        self.align = None
        self.create_subscription(Odometry, p('gt_topic'), lambda m: setattr(self, 'gt', m), qos_profile_sensor_data)
        self.create_subscription(Float32, 'gs_reloc/color_error', self._on_gs, 10)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.t0 = time.time()
        self.kidnap_t = None
        self.ok_since = None
        self.res = dict(label=p('label'), pre_err=None, kidnap_err=None, recovery_sec=None, final_err=None)
        self.gs_pre, self.gs_post = [], []
        self.create_timer(0.5, self.tick)

    def _on_gs(self, m):
        (self.gs_post if self.kidnap_t else self.gs_pre).append(m.data)

    def err(self):
        try:
            t = self.buf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        if self.gt is None:
            return None
        g = self.gt.pose.pose
        gx, gy, gyaw = g.position.x, g.position.y, yaw_of(g.orientation)
        mx, my, myaw = t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)
        if self.align is None:
            d = myaw - gyaw
            self.align = (d, mx - (gx * math.cos(d) - gy * math.sin(d)), my - (gx * math.sin(d) + gy * math.cos(d)))
        d, dx, dy = self.align
        ax = gx * math.cos(d) - gy * math.sin(d) + dx
        ay = gx * math.sin(d) + gy * math.cos(d) + dy
        return math.hypot(mx - ax, my - ay), (ax, ay, gyaw + d)

    def kidnap(self, truth):
        ax, ay, ayaw = truth
        yaw = ayaw + math.radians(self.p('offset_yaw_deg'))
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.pose.pose.position.x = ax + self.p('offset_x')
        m.pose.pose.position.y = ay + self.p('offset_y')
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.05
        m.pose.covariance[35] = 0.03
        self.pub.publish(m)

    def tick(self):
        e = self.err()
        if e is None:
            return
        err, truth = e
        now = time.time()
        if self.kidnap_t is None:
            if now - self.t0 < self.p('settle_sec'):
                return
            self.res['pre_err'] = round(err, 3)
            self.kidnap(truth)
            self.kidnap_t = now
            self.get_logger().info(f'KIDNAP at pre_err={err:.3f}m')
            return
        dt = now - self.kidnap_t
        if self.res['kidnap_err'] is None and dt > 1.5:
            self.res['kidnap_err'] = round(err, 3)
        if dt < 1.5:
            return
        if err < self.p('ok_err'):
            self.ok_since = self.ok_since or now
            if now - self.ok_since >= self.p('ok_hold_sec'):
                self.res['recovery_sec'] = round(self.ok_since - self.kidnap_t, 1)
                self.finish(err)
        else:
            self.ok_since = None
        if dt > self.p('timeout_sec'):
            self.finish(err)

    def finish(self, err):
        mean = lambda a: round(sum(a) / len(a), 4) if a else None
        self.res.update(final_err=round(err, 3), gs_color_err_pre=mean(self.gs_pre), gs_color_err_post=mean(self.gs_post))
        Path(self.p('out_dir'), f"{self.p('label')}.json").write_text(json.dumps(self.res))
        self.get_logger().info(f'RESULT {json.dumps(self.res)}')
        raise SystemExit(0)


def main():
    rclpy.init()
    try:
        rclpy.spin(KidnapEval())
    except SystemExit:
        pass


if __name__ == '__main__':
    main()
