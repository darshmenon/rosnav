#!/usr/bin/env python3
"""Compare pose-estimate accuracy against Gazebo ground truth over one run.

Estimators: wheel odometry (/odom), EKF (/odometry/filtered), SLAM (map->base_link via TF),
GPS-fused EKF (/odometry/global, only if the GPS stack is running),
KISS-ICP lidar odometry (/kiss/odometry, only if kiss_icp is running).
Ground truth: the model's world-frame pose from gz (/model/<model>/odometry_with_covariance, which
equals the true pose), bridged as nav_msgs/Odometry on /gt_odom (start it with: ros_gz_bridge
parameter_bridge /model/<model>/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance
--ros-args -r /model/<model>/odometry_with_covariance:=/gt_odom).
Each estimator is rigidly aligned to ground truth by a least-squares SE(2) fit over the first `align_n` paired samples (translation + yaw),
then position error is sampled at `rate` Hz for `duration` s (sim time).

  ros2 run rosnav_bot localization_accuracy_bench.py --ros-args -p use_sim_time:=true \
      -p duration:=120.0 -p model:=diff_drive -p out:=/tmp/loc_acc.json
"""
import json
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Bench(Node):
    def __init__(self):
        super().__init__('localization_accuracy_bench')
        self.declare_parameter('duration', 120.0)
        self.declare_parameter('rate', 2.0)
        self.declare_parameter('model', 'diff_drive')
        self.declare_parameter('out', '')
        self.duration = float(self.get_parameter('duration').value)
        self.model = self.get_parameter('model').value
        self.out = self.get_parameter('out').value
        self.gt = None
        self.est = {'wheel_odom': None, 'ekf': None, 'slam(map->base)': None, 'gps_ekf': None, 'kiss_icp': None}
        self.align = {}
        self.buf = {}
        self.declare_parameter('align_n', 20)  # first N paired samples (rate 2 Hz -> 10 s) fit the SE(2) alignment
        self.align_n = int(self.get_parameter('align_n').value)
        self.err = {k: [] for k in self.est}
        self.yerr = {k: [] for k in self.est}
        self.path_len = 0.0
        self._last_gt = None
        self.t0 = None
        self.create_subscription(Odometry, '/gt_odom', self.on_gt, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', lambda m: self.on_odo('wheel_odom', m), 10)
        self.create_subscription(Odometry, '/odometry/filtered', lambda m: self.on_odo('ekf', m), 10)
        self.create_subscription(Odometry, '/odometry/global', lambda m: self.on_odo('gps_ekf', m), 10)
        self.create_subscription(Odometry, '/kiss/odometry', lambda m: self.on_odo('kiss_icp', m), 10)  # only if KISS-ICP runs
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self.tick)

    def on_gt(self, msg):
        p = msg.pose.pose
        self.gt = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_odo(self, name, m):
        p = m.pose.pose
        self.est[name] = (p.position.x, p.position.y, yaw_of(p.orientation))

    def slam_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            p = t.transform.translation
            return (p.x, p.y, yaw_of(t.transform.rotation))
        except Exception:  # noqa: BLE001
            return None

    @staticmethod
    def fit_se2(pairs):
        n = len(pairs)
        ex = sum(e[0] for e, _ in pairs) / n; ey = sum(e[1] for e, _ in pairs) / n
        gx = sum(g[0] for _, g in pairs) / n; gy = sum(g[1] for _, g in pairs) / n
        sxx = sum((e[0] - ex) * (g[0] - gx) + (e[1] - ey) * (g[1] - gy) for e, g in pairs)
        sxy = sum((e[0] - ex) * (g[1] - gy) - (e[1] - ey) * (g[0] - gx) for e, g in pairs)
        dyaw = math.atan2(sxy, sxx)
        return (dyaw, ex, ey, gx, gy)

    def score(self, k, e, gt):
        dyaw, ex0, ey0, gx0, gy0 = self.align[k]
        c, s = math.cos(dyaw), math.sin(dyaw)
        dx, dy = e[0] - ex0, e[1] - ey0
        px, py = gx0 + c * dx - s * dy, gy0 + s * dx + c * dy
        self.err[k].append(math.hypot(px - gt[0], py - gt[1]))
        d = e[2] + dyaw - gt[2]
        self.yerr[k].append(math.atan2(math.sin(d), math.cos(d)))

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.gt is None:
            return
        if self.t0 is None:
            self.t0 = now
        self.est['slam(map->base)'] = self.slam_pose()
        if self._last_gt is not None:
            self.path_len += math.hypot(self.gt[0] - self._last_gt[0], self.gt[1] - self._last_gt[1])
        self._last_gt = self.gt
        for k, e in self.est.items():
            if e is None:
                continue
            if k not in self.align:  # collect first `align_n` pairs, then fit one rigid SE(2) (Kabsch)
                buf = self.buf.setdefault(k, [])
                buf.append((e, self.gt))
                if len(buf) < self.align_n:
                    continue
                self.align[k] = self.fit_se2(buf)
                for be, bg in buf:  # back-fill the window so every sample is scored
                    self.score(k, be, bg)
                continue
            self.score(k, e, self.gt)
        if now - self.t0 >= self.duration:
            self.report()
            raise SystemExit

    def report(self):
        res = {'duration_s': self.duration, 'ground_truth_path_m': round(self.path_len, 2), 'estimators': {}}
        for k, v in self.err.items():
            if v:
                rmse = math.sqrt(sum(x * x for x in v) / len(v))
                res['estimators'][k] = {'samples': len(v), 'rmse_m': round(rmse, 3),
                                        'max_m': round(max(v), 3), 'final_m': round(v[-1], 3)}
                y = self.yerr[k]
                res['estimators'][k]['yaw_rmse_deg'] = round(math.degrees(math.sqrt(sum(x * x for x in y) / len(y))), 2)
                res['estimators'][k]['yaw_final_deg'] = round(math.degrees(y[-1]), 2)
                if self.path_len > 0:
                    res['estimators'][k]['final_pct_of_path'] = round(100.0 * v[-1] / self.path_len, 2)
        print(json.dumps(res, indent=2), flush=True)
        if self.out:
            with open(self.out, 'w') as f:
                json.dump(res, f, indent=2)


def main():
    rclpy.init()
    n = Bench()
    try:
        rclpy.spin(n)
    except (SystemExit, KeyboardInterrupt):
        pass
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
