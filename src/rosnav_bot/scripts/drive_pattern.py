#!/usr/bin/env python3
"""drive_pattern.py - publish /cmd_vel on SIM time so a pattern keeps its shape when the sim runs below real time.

  ros2 run rosnav_bot drive_pattern.py --ros-args -p use_sim_time:=true -p pattern:=fig8 -p duration:=170.0

pattern: fig8  v 0.4, w +-0.4 flipping every half circle (pi/0.4 s): a figure-8 about 2 m x 4 m, stays inside a room
         weave v 0.5, w +-0.8 flipping every 4 s
         circle v 0.2, w 0.1
avoid:   (default true) turn in place when /scan shows something < 0.7 m straight ahead, resume when clear > 1.0 m.
         Without it the cafe figure-8 drives into table2 and sticks there (wheel odom keeps counting, the robot does not).
`timeout N ros2 topic pub` counts wall seconds: at real-time factor 0.2 it flips the turn direction every 0.2*N sim
seconds, so the robot jitters in place (this made the first weave/fig8 bench runs cover only ~13 m in 150 s).
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def command(pattern, t):
    """(v, w) for pattern at sim time t seconds since start."""
    if pattern == 'circle':
        return 0.2, 0.1
    if pattern == 'weave':
        return 0.5, 0.8 if int(t // 4.0) % 2 == 0 else -0.8
    if pattern == 'fig8':
        return 0.4, 0.4 if int(t // (math.pi / 0.4)) % 2 == 0 else -0.4
    raise ValueError(f'unknown pattern {pattern!r}')


def avoid(v, w, front_min, blocked, stop_m=0.7, clear_m=1.0, turn=0.6):
    """Obstacle guard around a pattern command. Returns (v, w, blocked). Hysteresis: once something is closer than stop_m
    straight ahead the robot turns in place until the way is clear beyond clear_m, then the pattern resumes."""
    if front_min is None:
        return v, w, blocked
    if blocked:
        blocked = front_min <= clear_m
    elif front_min < stop_m:
        blocked = True
    return (0.0, turn, True) if blocked else (v, w, False)


class Driver(Node):
    def __init__(self):
        super().__init__('drive_pattern')
        self.pattern = self.declare_parameter('pattern', 'fig8').value
        self.duration = float(self.declare_parameter('duration', 170.0).value)
        command(self.pattern, 0.0)                                   # fail early on a bad pattern
        self.avoid_on = bool(self.declare_parameter('avoid', True).value)
        self.front_min, self.blocked, self.turns = None, False, 0
        if self.avoid_on:
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import LaserScan
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.t0 = None
        self.create_timer(0.1, self.tick)                            # node clock = sim time when use_sim_time is set
        self.get_logger().info(f'drive_pattern {self.pattern} for {self.duration:.0f} sim s')

    def on_scan(self, m):
        """Closest valid return within +-40 deg of straight ahead (angle 0 = forward)."""
        lim = math.radians(40.0)
        rs = [r for k, r in enumerate(m.ranges)
              if abs(m.angle_min + k * m.angle_increment) <= lim and math.isfinite(r) and r >= m.range_min]
        self.front_min = min(rs) if rs else None

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        m = Twist()
        if t < self.duration:
            v, w = command(self.pattern, t)
            if self.avoid_on:
                was = self.blocked
                v, w, self.blocked = avoid(v, w, self.front_min, self.blocked)
                if self.blocked and not was:
                    self.turns += 1
                    self.get_logger().info(f'obstacle {self.front_min:.2f} m ahead at t={t:.0f}s: turning away (#{self.turns})')
            m.linear.x, m.angular.z = v, w
        self.pub.publish(m)
        if t >= self.duration + 1.0:
            raise SystemExit


def main():
    rclpy.init()
    try:
        try:
            rclpy.spin(Driver())
        except Exception:  # noqa: BLE001 - a signal can invalidate the context mid-spin
            if rclpy.ok():
                raise
    except (SystemExit, KeyboardInterrupt):
        pass


if __name__ == '__main__':
    main()
