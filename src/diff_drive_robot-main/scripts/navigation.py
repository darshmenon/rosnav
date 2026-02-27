#!/usr/bin/env python3
"""
Custom obstacle-avoidance navigator (no Nav2 required).

All tuning values are ROS 2 parameters — override at launch:
  ros2 run diff_drive_robot navigation.py --ros-args \
      -p goal_x:=3.0 -p goal_y:=2.0 -p base_speed:=0.8
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np


class ReliableObstacleNavigator(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_navigator')

        # ------------------------------------------------------------------
        # Declare all parameters (override via --ros-args -p name:=value)
        # ------------------------------------------------------------------
        self.declare_parameter('goal_x',             5.0)
        self.declare_parameter('goal_y',             4.0)
        self.declare_parameter('obstacle_threshold', 1.0)
        self.declare_parameter('clearance_required', 2.0)
        self.declare_parameter('move_distance',      2.5)
        self.declare_parameter('scan_angle_deg',     60.0)
        self.declare_parameter('front_angle_deg',    30.0)
        self.declare_parameter('base_speed',         1.5)
        self.declare_parameter('turn_speed',         3.5)
        self.declare_parameter('goal_tolerance',     0.3)
        self.declare_parameter('timer_period',       0.05)
        self.declare_parameter('cmd_vel_topic',  '/cmd_vel')
        self.declare_parameter('scan_topic',     '/scan')
        self.declare_parameter('odom_topic',     '/odom')

        # ------------------------------------------------------------------
        # Load parameters
        # ------------------------------------------------------------------
        self.goal = [
            self.get_parameter('goal_x').value,
            self.get_parameter('goal_y').value,
        ]
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.clearance_required = self.get_parameter('clearance_required').value
        self.move_distance      = self.get_parameter('move_distance').value
        self.scan_angle         = math.radians(self.get_parameter('scan_angle_deg').value)
        self.front_angle_range  = math.radians(self.get_parameter('front_angle_deg').value)
        self.base_speed         = self.get_parameter('base_speed').value
        self.turn_speed         = self.get_parameter('turn_speed').value
        self.goal_tolerance     = self.get_parameter('goal_tolerance').value

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        scan_topic    = self.get_parameter('scan_topic').value
        odom_topic    = self.get_parameter('odom_topic').value

        # ------------------------------------------------------------------
        # Publishers / Subscribers
        # ------------------------------------------------------------------
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self.state      = 'GOAL_SEEK'
        self.robot_pos  = [0.0, 0.0, 0.0]   # x, y, yaw
        self.start_pos  = [0.0, 0.0]
        self.target_yaw = 0.0
        self.laser_ranges: list = []
        self.laser_angles: list = []

        timer_period = self.get_parameter('timer_period').value
        self.create_timer(timer_period, self.navigate)

        self.get_logger().info(
            f'Navigator ready. Goal: ({self.goal[0]}, {self.goal[1]})')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        self.robot_pos[0] = msg.pose.pose.position.x
        self.robot_pos[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_pos[2] = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y ** 2 + q.z ** 2))

    def scan_callback(self, msg):
        self.laser_ranges = msg.ranges
        if not self.laser_angles:
            self.laser_angles = [
                msg.angle_min + i * msg.angle_increment
                for i in range(len(msg.ranges))]

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def get_front_obstacle_distance(self):
        if not self.laser_ranges or not self.laser_angles:
            return float('inf')
        mask = np.abs(self.laser_angles) <= self.front_angle_range
        front = np.array(self.laser_ranges)[mask]
        return float(np.min(front)) if front.size > 0 else float('inf')

    def find_clear_direction(self):
        if not self.laser_ranges or not self.laser_angles:
            return False, self.robot_pos[2]
        ranges = np.array(self.laser_ranges)
        angles = np.array(self.laser_angles)
        goal_angle = (math.atan2(
            self.goal[1] - self.robot_pos[1],
            self.goal[0] - self.robot_pos[0]) - self.robot_pos[2])
        best_angle, max_clearance = None, 0.0
        for angle in np.linspace(-np.pi / 2, np.pi / 2, num=5):
            mask = (angles > angle - self.scan_angle / 2) & \
                   (angles < angle + self.scan_angle / 2)
            sector = ranges[mask]
            if sector.size == 0:
                continue
            clearance = float(np.min(sector))
            if clearance > self.clearance_required and clearance > max_clearance:
                max_clearance = clearance
                best_angle = angle
        if best_angle is not None:
            return True, best_angle + self.robot_pos[2]
        return False, goal_angle

    def distance_moved(self):
        return math.hypot(
            self.robot_pos[0] - self.start_pos[0],
            self.robot_pos[1] - self.start_pos[1])

    # ------------------------------------------------------------------
    # Navigation FSM
    # ------------------------------------------------------------------
    def navigate(self):
        twist = Twist()
        goal_dist = math.hypot(
            self.goal[0] - self.robot_pos[0],
            self.goal[1] - self.robot_pos[1])

        if goal_dist < self.goal_tolerance:
            self.cmd_vel_pub.publish(Twist())
            self.get_logger().info('Goal reached!')
            return

        if self.state == 'GOAL_SEEK':
            if self.get_front_obstacle_distance() < self.obstacle_threshold:
                self.state = 'FIND_CLEAR'
            else:
                target_yaw = math.atan2(
                    self.goal[1] - self.robot_pos[1],
                    self.goal[0] - self.robot_pos[0])
                yaw_error = math.atan2(
                    math.sin(target_yaw - self.robot_pos[2]),
                    math.cos(target_yaw - self.robot_pos[2]))
                twist.linear.x  = self.base_speed * (1 - abs(yaw_error / math.pi))
                twist.angular.z = 2.0 * yaw_error

        elif self.state == 'FIND_CLEAR':
            _, self.target_yaw = self.find_clear_direction()
            yaw_error = math.atan2(
                math.sin(self.target_yaw - self.robot_pos[2]),
                math.cos(self.target_yaw - self.robot_pos[2]))
            if abs(yaw_error) < math.radians(5):
                self.state     = 'MOVE_CLEAR'
                self.start_pos = self.robot_pos[:2].copy()
            else:
                twist.angular.z = self.turn_speed * float(np.clip(yaw_error, -1, 1))

        elif self.state == 'MOVE_CLEAR':
            if self.distance_moved() >= self.move_distance:
                self.state = 'REALIGN'
            elif self.get_front_obstacle_distance() < self.obstacle_threshold:
                self.state = 'FIND_CLEAR'
            else:
                twist.linear.x = self.base_speed
                yaw_error = math.atan2(
                    math.sin(self.target_yaw - self.robot_pos[2]),
                    math.cos(self.target_yaw - self.robot_pos[2]))
                twist.angular.z = 1.0 * yaw_error

        elif self.state == 'REALIGN':
            target_yaw = math.atan2(
                self.goal[1] - self.robot_pos[1],
                self.goal[0] - self.robot_pos[0])
            yaw_error = math.atan2(
                math.sin(target_yaw - self.robot_pos[2]),
                math.cos(target_yaw - self.robot_pos[2]))
            if abs(yaw_error) < math.radians(5):
                self.state = 'GOAL_SEEK'
            else:
                twist.angular.z = self.turn_speed * float(np.clip(yaw_error, -1, 1))

        twist.linear.x  = float(np.clip(twist.linear.x,  -self.base_speed, self.base_speed))
        twist.angular.z = float(np.clip(twist.angular.z, -2.0, 2.0))

        if any(math.isnan(v) for v in [twist.linear.x, twist.angular.z]):
            twist = Twist()

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ReliableObstacleNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
