#!/usr/bin/env python3
"""
collision_monitor.py — Software safety layer (independent of Nav2 planner).

Monitors LaserScan data and enforces stop/slowdown zones.

Modes
─────
  watchdog (default, relay_mode:=false)
    Subscribes to scan only.  When an obstacle enters the stop zone it
    publishes Twist(0,0) at publish_hz — fast enough to override Nav2's
    10 Hz controller output and keep the robot stationary.

  relay (relay_mode:=true)
    Subscribes to cmd_vel_nav (pre-safety velocity from the controller).
    Passes through, scales, or zeroes the velocity based on zone.
    Publishes to cmd_vel.  Requires controller_server to be remapped so
    it publishes to cmd_vel_nav instead of cmd_vel.

State is published as JSON to /<ns>/collision_monitor/state.

Parameters
──────────
  robot_ns           namespace prefix (default: '')
  stop_distance      m — publish zero vel  (default: 0.30)
  slowdown_distance  m — scale vel down    (default: 0.70)
  slowdown_factor    scale [0,1]           (default: 0.40)
  front_angle_deg    total forward FOV     (default: 60)
  watch_all_around   ignore FOV, use 360°  (default: false)
  relay_mode         enable relay pipeline (default: false)
  publish_hz         watchdog timer rate   (default: 20)

Usage
─────
  ros2 run diff_drive_robot collision_monitor.py
  ros2 run diff_drive_robot collision_monitor.py --ros-args \\
      -p robot_ns:=robot1 -p stop_distance:=0.35 -p watch_all_around:=true
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

CLEAR    = 'CLEAR'
SLOWDOWN = 'SLOWDOWN'
STOP     = 'STOP'

# How long without a scan before we declare the sensor dead and latch STOP.
_SCAN_TIMEOUT = 2.0


class CollisionMonitor(Node):
    def __init__(self):
        super().__init__('collision_monitor')

        self.declare_parameter('robot_ns',          '')
        self.declare_parameter('stop_distance',     0.30)
        self.declare_parameter('slowdown_distance', 0.70)
        self.declare_parameter('slowdown_factor',   0.40)
        self.declare_parameter('front_angle_deg',   60.0)
        self.declare_parameter('watch_all_around',  False)
        self.declare_parameter('relay_mode',        False)
        self.declare_parameter('publish_hz',        20.0)

        ns             = self.get_parameter('robot_ns').value
        self._stop     = self.get_parameter('stop_distance').value
        self._slow     = self.get_parameter('slowdown_distance').value
        self._factor   = self.get_parameter('slowdown_factor').value
        self._half_fov = math.radians(self.get_parameter('front_angle_deg').value / 2.0)
        self._all      = self.get_parameter('watch_all_around').value
        self._relay    = self.get_parameter('relay_mode').value
        hz             = self.get_parameter('publish_hz').value

        self._state        = CLEAR
        self._min_range    = float('inf')
        self._relay_vel    = Twist()
        self._last_scan_t  = 0.0          # wall-clock time of last scan
        self._sensor_dead  = False

        pre = f'/{ns}' if ns else ''

        self._cmd_pub   = self.create_publisher(Twist,  f'{pre}/cmd_vel',                   10)
        self._state_pub = self.create_publisher(String, f'{pre}/collision_monitor/state',    10)

        self.create_subscription(LaserScan, f'{pre}/scan', self._scan_cb, 10)

        if self._relay:
            self.create_subscription(Twist, f'{pre}/cmd_vel_nav', self._relay_cb, 10)

        self.create_timer(1.0 / hz, self._tick)

        fov_str = '360°' if self._all else f'±{math.degrees(self._half_fov):.0f}°'
        mode    = 'relay' if self._relay else 'watchdog'
        self.get_logger().info(
            f'CollisionMonitor [{mode}]  ns={ns or "/"}  '
            f'stop={self._stop}m  slow={self._slow}m  fov={fov_str}')

    # ── Scan callback ─────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        self._last_scan_t = time.monotonic()
        self._sensor_dead = False

        if self._all:
            valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        else:
            valid = []
            for i, r in enumerate(msg.ranges):
                angle = msg.angle_min + i * msg.angle_increment
                if abs(angle) <= self._half_fov and msg.range_min < r < msg.range_max:
                    valid.append(r)

        self._min_range = min(valid) if valid else float('inf')

        if self._min_range < self._stop:
            new = STOP
        elif self._min_range < self._slow:
            new = SLOWDOWN
        else:
            new = CLEAR

        if new != self._state:
            self._state = new
            self.get_logger().info(
                f'State → {self._state}  (min_range={self._min_range:.2f} m)')

    def _relay_cb(self, msg: Twist):
        self._relay_vel = msg

    # ── Timer tick ────────────────────────────────────────────────────────────

    def _tick(self):
        # Sensor watchdog — if scan goes silent, treat as STOP
        if self._last_scan_t > 0.0:
            stale = time.monotonic() - self._last_scan_t > _SCAN_TIMEOUT
            if stale and not self._sensor_dead:
                self._sensor_dead = True
                self._state = STOP
                self.get_logger().warn(
                    f'Scan topic silent for >{_SCAN_TIMEOUT}s — latching STOP')

        if self._relay:
            self._publish_relay()
        else:
            self._publish_watchdog()

        payload = {
            'state':       self._state,
            'sensor_dead': self._sensor_dead,
            'min_range':   round(self._min_range, 3) if self._min_range != float('inf') else -1,
            'stop_dist':   self._stop,
            'slow_dist':   self._slow,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._state_pub.publish(msg)

    def _publish_watchdog(self):
        """
        Watchdog: publish a scaled/zero cmd_vel based on state.
        In STOP/SLOWDOWN we publish; in CLEAR we stay silent so Nav2 drives normally
        and there is only ever one active publisher on cmd_vel at a time.
        NOTE: in SLOWDOWN we scale down by publishing 0 vel — this IS a hard stop,
        not a proportional slow.  True proportional slowdown requires relay_mode:=true.
        """
        if self._state == STOP:
            self._cmd_pub.publish(Twist())
        elif self._state == SLOWDOWN:
            # Watchdog can't know Nav2's current cmd_vel, so we stop here too.
            # Use relay_mode:=true for proportional slowdown.
            self._cmd_pub.publish(Twist())

    def _publish_relay(self):
        if self._state in (STOP, SLOWDOWN) and self._sensor_dead:
            # Sensor dead — always hard stop
            self._cmd_pub.publish(Twist())
        elif self._state == STOP:
            self._cmd_pub.publish(Twist())
        elif self._state == SLOWDOWN:
            scaled = Twist()
            scaled.linear.x  = self._relay_vel.linear.x  * self._factor
            scaled.angular.z = self._relay_vel.angular.z * self._factor
            self._cmd_pub.publish(scaled)
        else:
            self._cmd_pub.publish(self._relay_vel)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
