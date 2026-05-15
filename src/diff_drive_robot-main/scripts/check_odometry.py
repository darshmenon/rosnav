#!/usr/bin/env python3
"""
check_odometry.py — Live odometry monitor.

Prints position, yaw, and velocity at 2 Hz.

Usage:
  ros2 run diff_drive_robot check_odometry.py
  ros2 run diff_drive_robot check_odometry.py --ros-args -p odom_topic:=/robot1/odom
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')

        self.declare_parameter('odom_topic', '/odom')
        topic = self.get_parameter('odom_topic').value

        self.create_subscription(Odometry, topic, self._cb, 10)
        self.get_logger().info(f'Monitoring {topic}')

    def _cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular

        yaw = math.degrees(math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

        speed = math.hypot(v.x, v.y)

        self.get_logger().info(
            f'pos=({p.x:.3f}, {p.y:.3f})  yaw={yaw:.1f}°  '
            f'speed={speed:.3f} m/s  omega={w.z:.3f} rad/s',
            throttle_duration_sec=0.5)


def main(args=None):
    rclpy.init(args=args)
    node = OdomSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
