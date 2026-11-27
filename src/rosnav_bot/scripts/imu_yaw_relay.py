#!/usr/bin/env python3
"""imu_yaw_relay - republish /imu as /imu_rot with the orientation quaternion rotated about z by a constant yaw.

The yaw is measured, not guessed: with auto_cal (default) the node compares IMU yaw with `ref_odom` yaw for
`cal_samples` messages (circular mean, accepted only if the scatter is under `cal_max_std_deg`) and rotates by
the difference; until then, or if it never settles, it uses yaw_deg (default 0 = pass-through).

Why: the gz IMU orientation can be referenced to a world frame that differs from the robot's odom heading by a constant
yaw (-89.96 deg in the cafe weave bag; 0 deg in a later live cafe run, so a hard-coded value is wrong one of the two ways;
roll/pitch match). LIO-SAM takes its initial guess and deskew attitude from the
IMU orientation, so the offset made imuPreintegration blow up ("Large velocity, reset IMU-preintegration!").
Acceleration and angular velocity are body-frame and are passed through untouched.
"""
import collections
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def rotate_yaw(q, yaw_deg):
    """Return (x, y, z, w) of q_z(yaw) * q for q given as (x, y, z, w)."""
    h = math.radians(yaw_deg) / 2.0
    cw, sz = math.cos(h), math.sin(h)
    x, y, z, w = q
    return (cw * x - sz * y, cw * y + sz * x, cw * z + sz * w, cw * w - sz * z)


def quat_yaw(q):
    """Yaw (rad) of q given as (x, y, z, w)."""
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def circ_mean_std_deg(angles_rad):
    """(circular mean deg, circular std deg) of a list of angles in radians."""
    n = len(angles_rad)
    c = sum(math.cos(a) for a in angles_rad) / n
    s = sum(math.sin(a) for a in angles_rad) / n
    r = math.hypot(c, s)
    std = math.sqrt(max(0.0, -2.0 * math.log(min(r, 1.0)))) if r > 1e-12 else math.pi
    return math.degrees(math.atan2(s, c)), math.degrees(std)


def nearest_yaw(buf, t, max_gap):
    """Yaw of the (stamp, yaw) sample in `buf` closest in time to t, or None if none is within max_gap seconds."""
    best = min(buf, key=lambda sy: abs(sy[0] - t), default=None)
    return best[1] if best is not None and abs(best[0] - t) <= max_gap else None


class ImuYawRelay(Node):
    def __init__(self):
        super().__init__('imu_yaw_relay')
        self.declare_parameter('yaw_deg', 0.0)
        self.declare_parameter('auto_cal', True)
        self.declare_parameter('ref_odom', '/odom')
        self.declare_parameter('cal_samples', 100)
        self.declare_parameter('cal_max_std_deg', 5.0)
        self.declare_parameter('in_topic', '/imu')
        self.declare_parameter('out_topic', '/imu_rot')
        g = lambda k: self.get_parameter(k).value  # noqa: E731
        self.yaw = float(g('yaw_deg'))
        self.pub = self.create_publisher(Imu, g('out_topic'), qos_profile_sensor_data)
        self.create_subscription(Imu, g('in_topic'), self.cb, qos_profile_sensor_data)
        self.auto = bool(g('auto_cal'))
        self.n_cal, self.max_std = int(g('cal_samples')), float(g('cal_max_std_deg'))
        self.ref_buf, self.diffs, self.calibrated = collections.deque(maxlen=400), [], False
        if self.auto:
            self.create_subscription(Odometry, g('ref_odom'), self.on_ref, 10)
        self.get_logger().info(f'{g("in_topic")} -> {g("out_topic")}, orientation yaw += {self.yaw} deg (acc/gyro untouched); '
                               + (f'auto_cal vs {g("ref_odom")} over {self.n_cal} samples (max std {self.max_std} deg)' if self.auto
                                  else 'auto_cal off'))

    def on_ref(self, m):
        q = m.pose.pose.orientation
        self.ref_buf.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, quat_yaw((q.x, q.y, q.z, q.w))))

    def calibrate(self, imu_q, stamp):
        """Pair this IMU sample with the odom sample nearest in TIME (the latest-received one lags by seconds under CPU load)."""
        if self.calibrated:
            return
        ref = nearest_yaw(self.ref_buf, stamp, 0.05)
        if ref is None:
            return
        d = ref - quat_yaw(imu_q)
        self.diffs.append(math.atan2(math.sin(d), math.cos(d)))
        if len(self.diffs) < self.n_cal:
            return
        mean, std = circ_mean_std_deg(self.diffs)
        if std <= self.max_std:
            self.yaw, self.calibrated = mean, True
            self.get_logger().info(f'auto_cal done: IMU yaw is {(-mean) + 0.0:+.1f} deg off odom yaw (std {std:.2f} deg, {len(self.diffs)} samples) '
                                   f'-> rotating orientation by {mean:+.1f} deg')
        else:
            self.get_logger().warn(f'auto_cal: scatter {std:.1f} deg > {self.max_std} deg (robot turning?), still using yaw_deg={self.yaw}; retrying')
        self.diffs = []

    def cb(self, m):
        o = m.orientation
        if self.auto:
            self.calibrate((o.x, o.y, o.z, o.w), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        o.x, o.y, o.z, o.w = rotate_yaw((o.x, o.y, o.z, o.w), self.yaw)
        self.pub.publish(m)


def main():
    rclpy.init()
    try:
        rclpy.spin(ImuYawRelay())
    except KeyboardInterrupt:
        pass
    except Exception:  # noqa: BLE001 - a signal can invalidate the context mid-spin
        if rclpy.ok():
            raise


if __name__ == '__main__':
    main()
