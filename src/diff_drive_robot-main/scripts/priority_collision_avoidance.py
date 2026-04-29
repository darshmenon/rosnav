#!/usr/bin/env python3
"""
priority_collision_avoidance.py — Priority-based robot yield for multi-robot Nav2.

When two robots are predicted to enter a conflict zone, the lower-priority robot
yields by publishing zero-velocity at high frequency until the higher-priority
robot clears the area.  Once clear, publishing stops and Nav2 resumes naturally.

Priority is determined by namespace order: robot1 > robot2 > robot3, etc.
(The robot listed first in `robot_namespaces` has the highest priority.)

Parameters
──────────
  robot_namespaces  (str)   comma-separated, e.g. "robot1,robot2"
  danger_radius     (float) metres — separation that triggers yield  (default 0.7)
  safe_radius       (float) metres — separation that clears yield    (default 1.1)
  lookahead         (float) seconds — project velocity forward       (default 0.6)
  check_freq        (float) Hz — conflict-check rate                 (default 10.0)
  yield_freq        (float) Hz — zero-vel publish rate while yielding (default 20.0)

Usage
─────
  ros2 run diff_drive_robot priority_collision_avoidance.py \\
      --ros-args -p robot_namespaces:=robot1,robot2

  # Or started automatically by multi_robot.launch.py when fleet_mgmt:=true
"""

import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class PriorityCollisionAvoidance(Node):
    def __init__(self):
        super().__init__('priority_collision_avoidance')

        self.declare_parameter('robot_namespaces', 'robot1,robot2')
        self.declare_parameter('danger_radius',    0.7)
        self.declare_parameter('safe_radius',      1.1)
        self.declare_parameter('lookahead',        0.6)
        self.declare_parameter('check_freq',       10.0)
        self.declare_parameter('yield_freq',       20.0)

        ns_param = self.get_parameter('robot_namespaces').value
        self._robots = [r.strip() for r in ns_param.split(',') if r.strip()]
        self._priority = {ns: i for i, ns in enumerate(self._robots)}  # lower index = higher priority

        self._danger_r  = self.get_parameter('danger_radius').value
        self._safe_r    = self.get_parameter('safe_radius').value
        self._lookahead = self.get_parameter('lookahead').value

        self._lock   = threading.Lock()
        self._poses  = {}   # ns → (x, y)
        self._vels   = {}   # ns → (vx, vy)  (from odom twist)
        self._yields = set()  # robots currently held at zero velocity

        # Per-robot subscriptions and publishers
        self._vel_pubs = {}
        for ns in self._robots:
            self.create_subscription(
                Odometry, f'/{ns}/odom',
                lambda msg, n=ns: self._odom_cb(msg, n), 10)
            self._vel_pubs[ns] = self.create_publisher(
                Twist, f'/{ns}/cmd_vel', 10)

        check_dt  = 1.0 / self.get_parameter('check_freq').value
        yield_dt  = 1.0 / self.get_parameter('yield_freq').value

        self.create_timer(check_dt,  self._check_conflicts)
        self.create_timer(yield_dt,  self._publish_yields)

        self.get_logger().info(
            f'PriorityCollisionAvoidance ready  robots={self._robots}  '
            f'danger={self._danger_r}m  safe={self._safe_r}m')

    # ── Odom callback ─────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry, ns: str):
        x  = msg.pose.pose.position.x
        y  = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        with self._lock:
            self._poses[ns] = (x, y)
            self._vels[ns]  = (vx, vy)

    # ── Conflict check ────────────────────────────────────────────────────────

    def _predicted_pos(self, ns):
        x, y    = self._poses.get(ns, (0.0, 0.0))
        vx, vy  = self._vels.get(ns,  (0.0, 0.0))
        dt      = self._lookahead
        return x + vx * dt, y + vy * dt

    def _check_conflicts(self):
        with self._lock:
            if len(self._poses) < 2:
                return

            new_yields = set()

            for i, ns_a in enumerate(self._robots):
                for ns_b in self._robots[i + 1:]:
                    if ns_a not in self._poses or ns_b not in self._poses:
                        continue

                    # Current and predicted separation
                    ax, ay = self._poses[ns_a]
                    bx, by = self._poses[ns_b]
                    current_d = math.hypot(ax - bx, ay - by)

                    pax, pay = self._predicted_pos(ns_a)
                    pbx, pby = self._predicted_pos(ns_b)
                    predicted_d = math.hypot(pax - pbx, pay - pby)

                    # Determine which robot yields (lower priority = higher index)
                    low_prio = ns_b  # ns_a has lower index → higher priority

                    # Start yielding if predicted conflict
                    already_yielding = low_prio in self._yields
                    if predicted_d < self._danger_r:
                        new_yields.add(low_prio)
                        if not already_yielding:
                            self.get_logger().info(
                                f'[yield] {low_prio} yields to {ns_a}  '
                                f'sep={current_d:.2f}m → predicted {predicted_d:.2f}m')

                    # Clear yield once safely separated (hysteresis)
                    elif already_yielding and current_d > self._safe_r:
                        self.get_logger().info(
                            f'[clear] {low_prio} resumes  sep={current_d:.2f}m')

            self._yields = new_yields

    # ── Zero-vel publisher ────────────────────────────────────────────────────

    def _publish_yields(self):
        with self._lock:
            yielding = list(self._yields)
        stop = Twist()  # all zeros
        for ns in yielding:
            if ns in self._vel_pubs:
                self._vel_pubs[ns].publish(stop)


def main():
    rclpy.init()
    node = PriorityCollisionAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
