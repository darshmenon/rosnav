#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomTfBroadcaster(Node):
    def __init__(self) -> None:
        super().__init__('odom_tf_broadcaster')
        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, 'odom', self._on_odom, 50)

    def _on_odom(self, msg: Odometry) -> None:
        if not msg.header.frame_id or not msg.child_frame_id:
            return

        tf_msg = TransformStamped()
        tf_msg.header = msg.header
        tf_msg.child_frame_id = msg.child_frame_id
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation
        self._broadcaster.sendTransform(tf_msg)


def main() -> None:
    rclpy.init()
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
