#!/usr/bin/env python3
"""
adas_acc.py — adaptive cruise control + automatic emergency braking for a car-like robot (drive_type:=ackermann).

Reads the front sector of /scan, finds the nearest object ahead (the lead vehicle) and its closing speed, and
limits the forward speed of `cmd_vel_in` -> `cmd_vel_out`:
  ACC  follow at a time gap:   v_cmd = clamp(v_set, ..., v_lead + k_gap * (gap - (d0 + t_gap * v)))
  AEB  if time-to-collision < ttc_brake, or gap < d_stop: v = 0 (hard stop)
Steering (angular.z) passes through untouched. Parameters: v_set (0.6), t_gap (1.5), d0 (0.8), k_gap (0.6),
ttc_brake (1.2), d_stop (0.5), front_deg (15), max_decel (1.5 m/s^2).
  ros2 run rosnav_bot adas_acc.py --ros-args -r cmd_vel_in:=/cmd_vel_nav -r cmd_vel_out:=/cmd_vel
Not wired into launch files; unit-tested offline (python3 adas_acc.py --selftest).
"""
import math
import sys


def acc_aeb(v_req, v_now, gap, closing, v_set=0.6, t_gap=1.5, d0=0.8, k_gap=0.6, ttc_brake=1.2, d_stop=0.5):
    """gap [m] to the lead (inf if none); closing [m/s] (+ = getting closer). Returns (v_cmd, mode)."""
    v_req = min(v_req, v_set)
    if gap is None or math.isinf(gap):
        return v_req, 'free'
    if gap < d_stop or (closing > 0.05 and gap / closing < ttc_brake):
        return 0.0, 'AEB'
    v_lead = v_now - closing
    v_follow = max(0.0, v_lead + k_gap * (gap - (d0 + t_gap * v_now)))
    return (min(v_req, v_follow), 'ACC') if v_follow < v_req else (v_req, 'free')


def selftest():
    # lead 0.6 m/s ahead, we start 3 m behind at 0.6: should settle near d0 + t_gap*v = 1.7 m and never collide
    x, v, lead, dt, dmin = 0.0, 0.6, 3.0, 0.05, 9
    for i in range(1200):
        lead += 0.4 * dt                                     # lead slower than set speed
        gap = lead - x
        cmd, _ = acc_aeb(0.6, v, gap, v - 0.4)
        a = max(-1.5, min(0.8, (cmd - v) / dt))
        v = max(0.0, v + a * dt); x += v * dt; dmin = min(dmin, gap)
    print(f'follow: final gap {lead - x:.2f} m (target {0.8 + 1.5 * 0.4:.2f}), min gap {dmin:.2f} m')
    assert dmin > 0.5 and abs((lead - x) - 1.4) < 0.4
    # stationary wall 1 m ahead at speed: AEB
    print('AEB   :', acc_aeb(0.6, 0.6, 0.55, 0.6))
    assert acc_aeb(0.6, 0.6, 0.55, 0.6)[1] == 'AEB'
    print('free  :', acc_aeb(0.6, 0.0, float('inf'), 0.0))
    print('selftest OK')


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    class N(Node):
        def __init__(self):
            super().__init__('adas_acc')
            d = dict(v_set=0.6, t_gap=1.5, d0=0.8, k_gap=0.6, ttc_brake=1.2, d_stop=0.5, front_deg=15.0)
            for k, v in d.items():
                self.declare_parameter(k, v)
            self.p = {k: self.get_parameter(k).value for k in d}
            self.front = math.radians(self.p.pop('front_deg'))
            self.gap, self.prev, self.closing, self.v = float('inf'), None, 0.0, 0.0
            self.mode, self.last = '', 0.0
            self.pub = self.create_publisher(Twist, 'cmd_vel_out', 10)
            self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
            self.create_subscription(Twist, 'cmd_vel_in', self.on_cmd, 10)

        def on_scan(self, m):
            best = float('inf')
            for i, r in enumerate(m.ranges):
                a = m.angle_min + i * m.angle_increment
                if abs(math.atan2(math.sin(a), math.cos(a))) <= self.front and m.range_min < r < m.range_max:
                    best = min(best, r)
            t = self.get_clock().now().nanoseconds / 1e9
            if self.prev and t - self.prev[1] > 0.02 and not math.isinf(best) and not math.isinf(self.prev[0]):
                self.closing = 0.5 * self.closing + 0.5 * (self.prev[0] - best) / (t - self.prev[1])
            elif math.isinf(best):
                self.closing = 0.0
            self.prev, self.gap = (best, t), best

        def on_cmd(self, m):
            v, mode = acc_aeb(m.linear.x, self.v, self.gap, self.closing, **self.p)
            self.v = v
            out = Twist(); out.linear.x, out.angular.z = v, m.angular.z
            if mode != self.mode and self.get_clock().now().nanoseconds / 1e9 - self.last > 1.0:
                self.mode, self.last = mode, self.get_clock().now().nanoseconds / 1e9
                self.get_logger().info(f'{mode}: gap={self.gap:.2f} m closing={self.closing:.2f} m/s -> v={v:.2f}')
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
