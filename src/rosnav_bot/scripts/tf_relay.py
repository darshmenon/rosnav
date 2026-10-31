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
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


class TfRelay(Node):
    def __init__(self):
        super().__init__('tf_relay')
        # child_frame_id regex to NOT forward (Nav2 has no use for wheel/joint frames);
        # '' forwards everything. Per-frame counts are logged so the pattern can be tuned.
        self.declare_parameter('drop_regex', '')
        # restamp: swallow the map->odom transform from /tf and re-publish the latest one at
        # `restamp_hz` with the relay's own clock as stamp (monotonic, evenly spaced). Tests whether
        # the controller's frozen map->odom is a stamp problem in its tf2 buffer.
        self.declare_parameter('restamp', False)
        self.declare_parameter('restamp_hz', 20.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        rx = str(self.get_parameter('drop_regex').value)
        self._drop = re.compile(rx) if rx else None
        self._seen = Counter()
        self._restamp = bool(self.get_parameter('restamp').value)
        self._map = str(self.get_parameter('map_frame').value)
        self._odom = str(self.get_parameter('odom_frame').value)
        self._latest_mo = None
        self._mo_in = 0
        self._dropped = 0
        dyn = QoSProfile(depth=200, reliability=QoSReliabilityPolicy.RELIABLE)
        static = QoSProfile(depth=100, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        pub = self.create_publisher(TFMessage, '/tf_nav', dyn)
        spub = self.create_publisher(TFMessage, '/tf_static_nav', static)
        self._n = 0
        self.create_subscription(TFMessage, '/tf', lambda m: self._fwd(pub, m), dyn)
        self.create_subscription(TFMessage, '/tf_static', spub.publish, static)
        self._pub = pub
        if self._restamp:
            self.create_timer(1.0 / float(self.get_parameter('restamp_hz').value), self._emit_mo)
        self.create_timer(10.0, self._report)
        self.get_logger().info('tf_relay: /tf -> /tf_nav, /tf_static -> /tf_static_nav')

    def _fwd(self, pub, msg):
        for t in msg.transforms:
            self._seen[f'{t.header.frame_id}->{t.child_frame_id}'] += 1
        keep = list(msg.transforms)
        if self._restamp:
            for t in keep:
                if t.header.frame_id == self._map and t.child_frame_id == self._odom:
                    self._latest_mo = t
                    self._mo_in += 1
            keep = [t for t in keep
                    if not (t.header.frame_id == self._map and t.child_frame_id == self._odom)]
        if self._drop:
            keep = [t for t in keep if not self._drop.search(t.child_frame_id)]
        self._dropped += len(msg.transforms) - len(keep)
        if not keep:
            return
        msg = TFMessage(transforms=keep)
        self._n += 1
        pub.publish(msg)

    def _emit_mo(self):
        if self._latest_mo is None:
            return
        out = TransformStamped()
        out.header.frame_id = self._latest_mo.header.frame_id
        out.child_frame_id = self._latest_mo.child_frame_id
        out.header.stamp = self.get_clock().now().to_msg()
        out.transform = self._latest_mo.transform
        self._pub.publish(TFMessage(transforms=[out]))

    def _report(self):
        top = ', '.join(f'{k}:{v}' for k, v in self._seen.most_common(12))
        self.get_logger().info(f'forwarded {self._n} /tf msgs, dropped {self._dropped} transforms, map->odom in {self._mo_in}; '
                               f'transforms seen: {top}')
        self._seen.clear()


def main():
    rclpy.init()
    rclpy.spin(TfRelay())


if __name__ == '__main__':
    main()
