#!/usr/bin/env python3
"""Drive a fixed square loop open-loop on cmd_vel_nav (for repeatable localization benchmarks).

  ros2 run rosnav_bot scripted_drive.py --ros-args -p use_sim_time:=true -p side:=4.0 -p laps:=1

Speeds sit above the outdoor drive deadband (~0.08 m/s, ~0.3 rad/s). Exits when done and sends zero.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class ScriptedDrive(Node):
    def __init__(self):
        super().__init__('scripted_drive')
        self.declare_parameter('side', 4.0)
        self.declare_parameter('laps', 1)
        self.declare_parameter('v', 0.2)
        self.declare_parameter('w', 0.5)
        self.declare_parameter('topic', 'cmd_vel_nav')
        side, laps = float(self.get_parameter('side').value), int(self.get_parameter('laps').value)
        v, w = float(self.get_parameter('v').value), float(self.get_parameter('w').value)
        self.plan = []  # (duration_s, linear, angular)
        for _ in range(4 * laps):
            self.plan += [(side / v, v, 0.0), (0.5, 0.0, 0.0), ((math.pi / 2) / w, 0.0, w), (0.5, 0.0, 0.0)]
        self.pub = self.create_publisher(Twist, self.get_parameter('topic').value, 10)
        self.i, self.t_start = 0, None
        self.create_timer(0.05, self.tick)
        self.get_logger().info(f'square {side} m x {laps} lap(s): {len(self.plan)} segments, '
                               f'{sum(p[0] for p in self.plan):.0f}s sim time')

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t_start is None:
            self.t_start = now
        if self.i >= len(self.plan):
            self.pub.publish(Twist())
            self.get_logger().info('scripted drive done')
            raise SystemExit
        dur, lin, ang = self.plan[self.i]
        if now - self.t_start >= dur:
            self.i += 1
            self.t_start = now
            return
        m = Twist()
        m.linear.x, m.angular.z = lin, ang
        self.pub.publish(m)


def main():
    rclpy.init()
    n = ScriptedDrive()
    try:
        rclpy.spin(n)
    except (SystemExit, KeyboardInterrupt):
        pass
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
