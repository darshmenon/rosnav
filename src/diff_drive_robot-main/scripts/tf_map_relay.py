#!/usr/bin/env python3
"""
tf_map_relay.py — Forward map-frame transforms from /tf (global) into the
namespaced tf topic so Nav2 nodes can resolve map → base_link.

Problem
-------
SLAM toolbox publishes  map → robot1/odom  on the global /tf topic.
Nav2's navigation_launch.py remaps /tf → tf (which resolves to /{ns}/tf),
so Nav2 nodes never see the map→odom transform.

Solution
--------
This node runs inside each robot's namespace.  It subscribes to /tf (global,
absolute) and republishes any transforms whose parent_frame_id is "map"
onto the local "tf" topic (which resolves to /{ns}/tf).

The result:  Nav2's global_costmap can find  map → robotX/base_link.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TfMapRelay(Node):
    def __init__(self) -> None:
        super().__init__('tf_map_relay')
        self._ns = self.get_namespace().strip('/')

        qos = QoSProfile(depth=100,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)

        # Publish on the relative topic "tf" → resolves to /{ns}/tf
        self._pub = self.create_publisher(TFMessage, 'tf', qos)

        # Subscribe on the absolute global /tf
        self.create_subscription(TFMessage, '/tf', self._on_msg, qos)

    def _on_msg(self, msg: TFMessage) -> None:
        # Relay only this robot's map->odom transform into its local /{ns}/tf.
        # Republishing every robot's map transform into every namespace creates
        # disconnected mixed TF trees like map->robot1/odom plus robot2/odom->base_link.
        relevant = [
            t for t in msg.transforms
            if t.header.frame_id == 'map'
            and self._ns
            and t.child_frame_id.startswith(self._ns + '/')
        ]
        if relevant:
            out = TFMessage()
            out.transforms = relevant
            self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = TfMapRelay()
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
