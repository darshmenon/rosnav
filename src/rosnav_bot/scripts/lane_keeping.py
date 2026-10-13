#!/usr/bin/env python3
"""
lane_keeping.py — keep the robot centred in a lane / corridor from the lidar (Stanley-style steering).

Fits a line to the scan points on each side (|y| < side_max), gives the lateral offset e from the lane centre and the
heading error psi, and sets angular.z = k_psi*psi + atan(k_e*e / (v + v0)) (clamped). Forward speed passes through.
`cmd_vel_in` -> `cmd_vel_out`. Parameters: k_psi 1.5, k_e 1.2, v0 0.2, max_w 1.0, side_max 2.0, x_max 3.0, min_pts 8.
Not wired into launch files. Offline test: python3 lane_keeping.py --selftest
"""
import math
import sys

import numpy as np


def lane_error(xy, side_max=2.0, x_max=3.0, min_pts=8):
    """xy (N,2) scan points in the robot frame (x forward, y left). Returns (e, psi) or None.
    e > 0: robot is left of the lane centre; psi > 0: robot is rotated left of the lane direction."""
    xy = xy[(np.abs(xy[:, 0]) < x_max) & (np.abs(xy[:, 1]) < side_max)]
    left, right = xy[xy[:, 1] > 0.2], xy[xy[:, 1] < -0.2]
    if len(left) < min_pts or len(right) < min_pts:
        return None
    fits = []
    for pts in (left, right):                       # y = a*x + b
        a, b = np.polyfit(pts[:, 0], pts[:, 1], 1)
        fits.append((a, b))
    a = 0.5 * (fits[0][0] + fits[1][0])             # wall slope in the robot frame = -psi
    centre = 0.5 * (fits[0][1] + fits[1][1])        # lane centre's y at x=0, in the robot frame
    return -centre, -math.atan(a)


def steer(e, psi, v, k_psi=1.5, k_e=1.2, v0=0.2, max_w=1.0):
    w = k_psi * psi + math.atan2(k_e * e, abs(v) + v0)
    return max(-max_w, min(max_w, -w))


def selftest():
    W, v, dt = 2.0, 0.4, 0.05
    x, y, yaw = 0.0, 0.5, math.radians(20)          # 0.5 m left of centre, 20 deg off
    for i in range(600):
        xs = np.linspace(-1.5, 3.0, 40)
        pts = []
        for sgn in (1, -1):                           # walls at y = +-W/2 in the world frame, seen from the robot
            wx = x + xs; wy = np.full_like(xs, sgn * W / 2)
            dx, dy = wx - x, wy - y
            pts.append(np.column_stack([math.cos(-yaw) * dx - math.sin(-yaw) * dy, math.sin(-yaw) * dx + math.cos(-yaw) * dy]))
        r = lane_error(np.vstack(pts))
        w = steer(r[0], r[1], v) if r else 0.0
        x += v * math.cos(yaw) * dt; y += v * math.sin(yaw) * dt; yaw += w * dt
        assert abs(y) < W / 2 - 0.2, f'hit the wall at step {i}'
    print(f'start (y=0.5 m, 20 deg) -> after 30 s: y={y:.3f} m, yaw={math.degrees(yaw):.1f} deg')
    assert abs(y) < 0.05 and abs(math.degrees(yaw)) < 3
    print('selftest OK')


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    class N(Node):
        def __init__(self):
            super().__init__('lane_keeping')
            d = dict(k_psi=1.5, k_e=1.2, v0=0.2, max_w=1.0, side_max=2.0, x_max=3.0, min_pts=8)
            for k, v in d.items():
                self.declare_parameter(k, v)
            g = lambda k: self.get_parameter(k).value
            self.s = dict(k_psi=g('k_psi'), k_e=g('k_e'), v0=g('v0'), max_w=g('max_w'))
            self.l = dict(side_max=g('side_max'), x_max=g('x_max'), min_pts=int(g('min_pts')))
            self.err = None
            self.pub = self.create_publisher(Twist, 'cmd_vel_out', 10)
            self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
            self.create_subscription(Twist, 'cmd_vel_in', self.on_cmd, 10)

        def on_scan(self, m):
            r = np.asarray(m.ranges, dtype=float)
            a = m.angle_min + np.arange(len(r)) * m.angle_increment
            ok = np.isfinite(r) & (r > m.range_min) & (r < m.range_max)
            self.err = lane_error(np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])]), **self.l)

        def on_cmd(self, m):
            out = Twist(); out.linear.x = m.linear.x; out.angular.z = m.angular.z
            if self.err is not None and abs(m.linear.x) > 0.02:
                out.angular.z = steer(self.err[0], self.err[1], m.linear.x, **self.s)
            self.pub.publish(out)

    rclpy.init()
    n = N()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    selftest() if '--selftest' in sys.argv else main()
