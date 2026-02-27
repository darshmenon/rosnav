#!/usr/bin/env python3
"""
Reset the robot pose in both Gazebo and RViz/AMCL.

Override any value at runtime:
  ros2 run diff_drive_robot reset_pose.py --ros-args \
      -p world_name:=my_world -p robot_name:=diff_bot \
      -p reset_x:=1.0 -p reset_y:=0.5
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from ros_gz_interfaces.srv import SetEntityPose


class ResetPoseNode(Node):
    def __init__(self):
        super().__init__('reset_pose_node')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter('world_name',      'obstacles')
        self.declare_parameter('robot_name',      'diff_bot')
        self.declare_parameter('reset_x',          0.0)
        self.declare_parameter('reset_y',          0.0)
        self.declare_parameter('reset_z',          0.2)
        self.declare_parameter('reset_yaw',        0.0)
        self.declare_parameter('odom_frame',      'odom')
        self.declare_parameter('initialpose_topic', '/initialpose')

        world_name  = self.get_parameter('world_name').value
        robot_name  = self.get_parameter('robot_name').value
        self._x     = self.get_parameter('reset_x').value
        self._y     = self.get_parameter('reset_y').value
        self._z     = self.get_parameter('reset_z').value
        self._frame = self.get_parameter('odom_frame').value
        init_topic  = self.get_parameter('initialpose_topic').value

        import math
        yaw = self.get_parameter('reset_yaw').value
        self._qz = math.sin(yaw / 2.0)
        self._qw = math.cos(yaw / 2.0)

        gz_service = f'/world/{world_name}/set_pose'

        # ------------------------------------------------------------------
        # Interfaces
        # ------------------------------------------------------------------
        self.cli = self.create_client(SetEntityPose, gz_service)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, init_topic, 10)

        self.get_logger().info(
            f'Waiting for Gazebo service: {gz_service}')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('  ...not available, waiting')

        self._robot_name = robot_name
        self.reset_gazebo_pose()
        self.reset_rviz_pose()

    def reset_gazebo_pose(self):
        req = SetEntityPose.Request()
        req.entity.name = self._robot_name
        req.entity.pose.position.x = self._x
        req.entity.pose.position.y = self._y
        req.entity.pose.position.z = self._z
        req.entity.pose.orientation.z = self._qz
        req.entity.pose.orientation.w = self._qw

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Gazebo pose reset successful.')
        else:
            self.get_logger().error('Gazebo reset failed.')

    def reset_rviz_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.orientation.z = self._qz
        msg.pose.pose.orientation.w = self._qw
        self.initialpose_pub.publish(msg)
        self.get_logger().info('RViz initial pose published.')


def main(args=None):
    rclpy.init(args=args)
    node = ResetPoseNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
