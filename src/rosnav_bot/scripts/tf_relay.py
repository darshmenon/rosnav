#!/usr/bin/env python3
"""tf_relay.py — forward /tf and /tf_static to /tf_nav and /tf_static_nav.

Experiment for the Nav2 controller_server map->odom freeze: Nav2 is remapped onto
these topics (slam_nav.launch.py tf_relay:=true) so its TF listener hears one local
publisher instead of every /tf writer. Messages are forwarded untouched.
"""

import re
from collections import Counter

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TfRelay(Node):
    def __init__(self):
        super().__init__('tf_relay')
        # child_frame_id regex to NOT forward (Nav2 has no use for wheel/joint frames);
        # '' forwards everything. Per-frame counts are logged so the pattern can be tuned.
        self.declare_parameter('drop_regex', '')
        rx = str(self.get_parameter('drop_regex').value)
        self._drop = re.compile(rx) if rx else None
        self._seen = Counter()
        self._dropped = 0
        dyn = QoSProfile(depth=200, reliability=QoSReliabilityPolicy.RELIABLE)
        static = QoSProfile(depth=100, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        pub = self.create_publisher(TFMessage, '/tf_nav', dyn)
        spub = self.create_publisher(TFMessage, '/tf_static_nav', static)
        self._n = 0
        self.create_subscription(TFMessage, '/tf', lambda m: self._fwd(pub, m), dyn)
        self.create_subscription(TFMessage, '/tf_static', spub.publish, static)
        self.create_timer(10.0, self._report)
        self.get_logger().info('tf_relay: /tf -> /tf_nav, /tf_static -> /tf_static_nav')

    def _fwd(self, pub, msg):
        for t in msg.transforms:
            self._seen[f'{t.header.frame_id}->{t.child_frame_id}'] += 1
        if self._drop:
            keep = [t for t in msg.transforms if not self._drop.search(t.child_frame_id)]
            self._dropped += len(msg.transforms) - len(keep)
            if not keep:
                return
            msg = TFMessage(transforms=keep)
        self._n += 1
        pub.publish(msg)

    def _report(self):
        top = ', '.join(f'{k}:{v}' for k, v in self._seen.most_common(12))
        self.get_logger().info(f'forwarded {self._n} /tf msgs, dropped {self._dropped} transforms; '
                               f'transforms seen: {top}')
        self._seen.clear()


def main():
    rclpy.init()
    rclpy.spin(TfRelay())


if __name__ == '__main__':
    main()
