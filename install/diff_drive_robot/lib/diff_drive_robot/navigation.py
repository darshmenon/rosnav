#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np

class ReliableObstacleNavigator(Node):
    def __init__(self):
        super().__init__('reliable_obstacle_navigator')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal = [5.0, 8.0]
        self.obstacle_threshold = 1.5
        self.clearance_required = 2.5
        self.move_distance = 2.0
        self.scan_angle = math.radians(60)
        self.base_speed = 0.3
        self.turn_speed = 2
        self.goal_tolerance = 0.3
        self.states = ["GOAL_SEEK", "FIND_CLEAR", "MOVE_CLEAR", "REALIGN"]
        self.state = "GOAL_SEEK"
        self.robot_pos = [0.0, 0.0, 0.0]
        self.start_pos = [0.0, 0.0]
        self.target_yaw = 0.0
        self.laser_ranges = []
        self.laser_angles = []
        self.front_angle_range = math.radians(30)
        self.create_timer(0.1, self.navigate)

    def odom_callback(self, msg):
        self.robot_pos[0] = msg.pose.pose.position.x
        self.robot_pos[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_pos[2] = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))

    def scan_callback(self, msg):
        self.laser_ranges = msg.ranges
        if not self.laser_angles:
            self.laser_angles = [msg.angle_min + i*msg.angle_increment for i in range(len(msg.ranges))]

    def get_front_obstacle_distance(self):
        if not self.laser_ranges or not self.laser_angles:
            return float('inf')
        front_mask = (np.abs(self.laser_angles) <= self.front_angle_range)
        front_ranges = np.array(self.laser_ranges)[front_mask]
        return np.min(front_ranges) if front_ranges.size > 0 else float('inf')

    def find_clear_direction(self):
        if not self.laser_ranges or not self.laser_angles:
            return 0, self.robot_pos[2]
        ranges = np.array(self.laser_ranges)
        angles = np.array(self.laser_angles)
        goal_angle = math.atan2(self.goal[1]-self.robot_pos[1], self.goal[0]-self.robot_pos[0]) - self.robot_pos[2]
        sector_angles = np.linspace(-np.pi/2, np.pi/2, num=5)
        best_angle = None
        max_clearance = 0.0
        for angle in sector_angles:
            sector_mask = (angles > angle - self.scan_angle/2) & (angles < angle + self.scan_angle/2)
            sector_ranges = ranges[sector_mask]
            if sector_ranges.size == 0:
                continue
            sector_clearance = np.min(sector_ranges)
            if sector_clearance > self.clearance_required and sector_clearance > max_clearance:
                max_clearance = sector_clearance
                best_angle = angle
        if best_angle is not None:
            return True, best_angle + self.robot_pos[2]
        return False, goal_angle

    def distance_moved(self):
        return math.hypot(self.robot_pos[0]-self.start_pos[0], self.robot_pos[1]-self.start_pos[1])

    def navigate(self):
        twist = Twist()
        goal_dist = math.hypot(self.goal[0]-self.robot_pos[0], self.goal[1]-self.robot_pos[1])
        obstacle_dist = self.get_front_obstacle_distance()
        self.get_logger().info(f"Distance from obstacle: {obstacle_dist:.2f} m, Distance from goal: {goal_dist:.2f} m")
        if goal_dist < self.goal_tolerance:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info("Goal reached!")
            return
        if self.state == "GOAL_SEEK":
            front_dist = self.get_front_obstacle_distance()
            if front_dist < self.obstacle_threshold:
                self.state = "FIND_CLEAR"
                self.get_logger().info(f"Obstacle detected at {front_dist:.2f}m! Seeking clear path...")
                twist.linear.x = 0.0
            else:
                target_yaw = math.atan2(self.goal[1]-self.robot_pos[1], self.goal[0]-self.robot_pos[0])
                yaw_error = target_yaw - self.robot_pos[2]
                yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
                twist.linear.x = self.base_speed * (1 - abs(yaw_error/math.pi))
                twist.angular.z = 1.0 * yaw_error
        elif self.state == "FIND_CLEAR":
            found_clear, self.target_yaw = self.find_clear_direction()
            yaw_error = self.target_yaw - self.robot_pos[2]
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
            if abs(yaw_error) < math.radians(5):
                self.state = "MOVE_CLEAR"
                self.start_pos = self.robot_pos[:2]
                self.get_logger().info("Clear path found. Moving forward...")
            else:
                twist.angular.z = self.turn_speed * np.clip(yaw_error, -1, 1)
                twist.linear.x = 0.0
        elif self.state == "MOVE_CLEAR":
            if self.distance_moved() >= self.move_distance:
                self.state = "REALIGN"
                self.get_logger().info("Clearance move complete. Realigning...")
            else:
                front_dist = self.get_front_obstacle_distance()
                if front_dist < self.obstacle_threshold:
                    self.state = "FIND_CLEAR"
                    self.get_logger().info("New obstacle detected during movement!")
                else:
                    twist.linear.x = self.base_speed
                    yaw_error = self.target_yaw - self.robot_pos[2]
                    yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
                    twist.angular.z = 0.5 * yaw_error
        elif self.state == "REALIGN":
            target_yaw = math.atan2(self.goal[1]-self.robot_pos[1], self.goal[0]-self.robot_pos[0])
            yaw_error = target_yaw - self.robot_pos[2]
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
            if abs(yaw_error) < math.radians(5):
                self.state = "GOAL_SEEK"
                self.get_logger().info("Realignment complete. Resuming goal pursuit.")
            else:
                twist.angular.z = self.turn_speed * np.clip(yaw_error, -1, 1)
        twist.linear.x = np.clip(twist.linear.x, -self.base_speed, self.base_speed)
        twist.angular.z = np.clip(twist.angular.z, -1.5, 1.5)
        if any(math.isnan(v) for v in [twist.linear.x, twist.angular.z]):
            twist = Twist()
            self.get_logger().warn("Invalid command detected! Stopping.")
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    navigator = ReliableObstacleNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
