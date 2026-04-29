#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TfStaticRelay(Node):
    def __init__(self) -> None:
        super().__init__('tf_static_relay')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pub = self.create_publisher(TFMessage, '/tf_static', qos)
        self.create_subscription(TFMessage, 'tf_static', self._on_msg, qos)

    def _on_msg(self, msg: TFMessage) -> None:
        if msg.transforms:
            self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = TfStaticRelay()
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
