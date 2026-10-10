#!/usr/bin/env python3
"""Gentle circling until a NEW /map is published, then stop and exit.

RTAB-Map (vslam/multisensor/3d) publishes /map once at t~2s then only when the map changes,
so a parked robot never re-publishes it; the frontier explorer (started later) waits for a /map
it never receives -> deadlock with the
robot parked at spawn. This breaks it with a small (~0.4 m radius) circle.
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class Bootstrap(Node):
    def __init__(self):
        super().__init__('slam_bootstrap_motion')
        self.declare_parameter('linear', 0.15)
        self.declare_parameter('angular', 0.4)
        self.declare_parameter('timeout_s', 45.0)
        self.have_map = False
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)  # fresh publishes only: the explorer misses the t=2s latched map
        self.create_subscription(OccupancyGrid, '/map', self._on_map, qos)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _on_map(self, _):
        if not self.have_map:
            self.get_logger().info('fresh /map received -> stopping bootstrap motion')
        self.have_map = True

    def run(self):
        lin = self.get_parameter('linear').value
        ang = self.get_parameter('angular').value
        timeout = self.get_parameter('timeout_s').value
        self.get_logger().info(
            f'no /map yet: circling (v={lin} m/s, w={ang} rad/s) up to {timeout:.0f}s until RTAB-Map publishes')
        t0 = time.monotonic()
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = lin, ang
        while rclpy.ok() and not self.have_map and time.monotonic() - t0 < timeout:
            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.1)
        # drive plugin keeps the last command forever -> always send an explicit stop
        for _ in range(5):
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)
        if not self.have_map:
            self.get_logger().warn(f'still no /map after {timeout:.0f}s of bootstrap motion')


def main():
    rclpy.init()
    node = Bootstrap()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
