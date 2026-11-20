#!/usr/bin/env python3
"""drive_pattern.py - publish /cmd_vel on SIM time so a pattern keeps its shape when the sim runs below real time.

  ros2 run rosnav_bot drive_pattern.py --ros-args -p use_sim_time:=true -p pattern:=fig8 -p duration:=170.0

pattern: fig8  v 0.4, w +-0.4 flipping every half circle (pi/0.4 s): a figure-8 about 2 m x 4 m, stays inside a room
         weave v 0.5, w +-0.8 flipping every 4 s
         circle v 0.2, w 0.1
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


class Driver(Node):
    def __init__(self):
        super().__init__('drive_pattern')
        self.pattern = self.declare_parameter('pattern', 'fig8').value
        self.duration = float(self.declare_parameter('duration', 170.0).value)
        command(self.pattern, 0.0)                                   # fail early on a bad pattern
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.t0 = None
        self.create_timer(0.1, self.tick)                            # node clock = sim time when use_sim_time is set
        self.get_logger().info(f'drive_pattern {self.pattern} for {self.duration:.0f} sim s')

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        m = Twist()
        if t < self.duration:
            m.linear.x, m.angular.z = command(self.pattern, t)
        self.pub.publish(m)
        if t >= self.duration + 1.0:
            raise SystemExit


def main():
    rclpy.init()
    try:
        rclpy.spin(Driver())
    except (SystemExit, KeyboardInterrupt):
        pass


if __name__ == '__main__':
    main()
