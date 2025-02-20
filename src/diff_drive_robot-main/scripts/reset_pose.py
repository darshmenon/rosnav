#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from ros_gz_interfaces.srv import SetEntityPose
from geometry_msgs.msg import Pose

class ResetPoseNode(Node):
    def __init__(self):
        super().__init__('reset_pose_node')
        
        # Service client for Gazebo
        self.cli = self.create_client(SetEntityPose, '/world/obstacles/set_pose')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Gazebo service not available, waiting...')
            
        # RViz initial pose publisher
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )
        
        # Reset both systems
        self.reset_gazebo_pose()
        self.reset_rviz_pose()

    def reset_gazebo_pose(self):
        """Reset actual simulation pose using Gazebo service"""
        req = SetEntityPose.Request()
        req.entity.name = 'diff_drive_robot'  # Your model name
        req.entity.pose.position.x = 0.0
        req.entity.pose.position.y = 0.0
        req.entity.pose.position.z = 0.2
        req.entity.pose.orientation.w = 1.0
        
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info('Gazebo pose reset successful!')
        else:
            self.get_logger().error('Gazebo reset failed!')

    def reset_rviz_pose(self):
        """Reset RViz localization pose"""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.orientation.w = 1.0
        self.initialpose_pub.publish(msg)
        self.get_logger().info('RViz pose reset published')

def main(args=None):
    rclpy.init(args=args)
    node = ResetPoseNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()