#!/usr/bin/env python3
"""
tf_static_relay.py — Bidirectional static TF relay for multi-robot namespaces.

Problem
-------
In a namespaced multi-robot setup Nav2's navigation_launch.py remaps
/tf_static → tf_static (= /{ns}/tf_static).  But robot_state_publisher's
StaticTransformBroadcaster always publishes to the absolute /tf_static.
Result: Nav2 nodes never see the base_link → laser_frame static transform.

Conversely, root-level tools (slam_toolbox) subscribe to /tf_static and
can't see per-robot static transforms that might only appear on /{ns}/tf_static.

Solution
--------
This node runs inside each robot's namespace and relays in BOTH directions:

  Forward:  /{ns}/tf_static  →  /tf_static   (for SLAM, RViz, etc.)
  Reverse:  /tf_static       →  /{ns}/tf_static  (for Nav2 AMCL, costmaps)

Deduplication prevents infinite loops: every relayed transform is tracked
by (parent_frame, child_frame) and only forwarded once per direction.
"""

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

        # Get our namespace to identify "our" transforms
        self._ns = self.get_namespace().strip('/')

        # Track which (parent, child) pairs we've already relayed in each
        # direction so we don't create infinite loops.
        self._forwarded_to_global: set[tuple[str, str]] = set()
        self._forwarded_to_ns: set[tuple[str, str]] = set()

        # Forward: /{ns}/tf_static → /tf_static
        self._global_pub = self.create_publisher(TFMessage, '/tf_static', qos)
        self.create_subscription(TFMessage, 'tf_static', self._on_ns_msg, qos)

        # Reverse: /tf_static → /{ns}/tf_static  (= 'tf_static' in namespace)
        self._ns_pub = self.create_publisher(TFMessage, 'tf_static', qos)
        self.create_subscription(
            TFMessage, '/tf_static', self._on_global_msg, qos)

    # ── Forward: namespaced → global ─────────────────────────────────────
    def _on_ns_msg(self, msg: TFMessage) -> None:
        novel = []
        for t in msg.transforms:
            key = (t.header.frame_id, t.child_frame_id)
            if key not in self._forwarded_to_global:
                self._forwarded_to_global.add(key)
                novel.append(t)
        if novel:
            out = TFMessage()
            out.transforms = novel
            self._global_pub.publish(out)

    # ── Reverse: global → namespaced ─────────────────────────────────────
    def _on_global_msg(self, msg: TFMessage) -> None:
        novel = []
        for t in msg.transforms:
            key = (t.header.frame_id, t.child_frame_id)
            # Only relay transforms belonging to our robot (frame starts
            # with our namespace prefix) to avoid flooding with other
            # robots' transforms.
            if not self._ns:
                continue  # safety: skip if we have no namespace
            belongs = (
                t.header.frame_id.startswith(self._ns + '/')
                or t.child_frame_id.startswith(self._ns + '/')
            )
            if belongs and key not in self._forwarded_to_ns:
                self._forwarded_to_ns.add(key)
                novel.append(t)
        if novel:
            out = TFMessage()
            out.transforms = novel
            self._ns_pub.publish(out)


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
