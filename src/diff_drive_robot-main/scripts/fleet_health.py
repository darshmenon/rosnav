#!/usr/bin/env python3
"""
fleet_health.py — Per-robot system health monitor.

Watches each robot's data streams and Nav2 node presence, then publishes
a health report to /fleet/health (std_msgs/String JSON) at 1 Hz.

Checks per robot
────────────────
  odom_hz       Odometry publish rate (expected ≥ 5 Hz)
  scan_hz       LaserScan publish rate (expected ≥ 5 Hz)
  nav2_alive    bt_navigator node visible in ROS graph
  mission       Last known mission state (from mission_server)
  collision     Last known collision monitor state
  overall       OK | WARN | ERROR

Overall rules
─────────────
  ERROR   odom or scan dropped to 0 Hz
  WARN    odom or scan below threshold, OR nav2_alive = false
  OK      all checks green

Usage
─────
  # Run as daemon (publishes /fleet/health):
  ros2 run diff_drive_robot fleet_health.py

  # One-shot status print:
  ros2 run diff_drive_robot fleet_health.py status

  # Or via fleet_manager:
  ros2 run diff_drive_robot fleet_manager.py health
"""

import json
import sys
import threading
import time
from collections import defaultdict, deque

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

_ODOM_WARN_HZ  = 5.0
_SCAN_WARN_HZ  = 5.0
_WINDOW        = 2.0    # seconds to measure Hz over


class RateTracker:
    """Tracks arrival rate of messages over a sliding time window."""

    def __init__(self, window: float = _WINDOW):
        self._window = window
        self._times: deque = deque()

    def tick(self):
        now = time.monotonic()
        self._times.append(now)
        while self._times and now - self._times[0] > self._window:
            self._times.popleft()

    @property
    def hz(self) -> float:
        now = time.monotonic()
        while self._times and now - self._times[0] > self._window:
            self._times.popleft()
        return len(self._times) / self._window


class FleetHealthMonitor(Node):
    def __init__(self):
        super().__init__('fleet_health_monitor')

        self._lock           = threading.Lock()
        self._odom_rates:    dict[str, RateTracker] = {}
        self._scan_rates:    dict[str, RateTracker] = {}
        self._mission_states: dict[str, str]        = {}
        self._collision_states: dict[str, dict]     = {}
        self._known_robots:  set[str]               = set()
        self._odom_subs:     dict = {}
        self._scan_subs:     dict = {}
        self._col_subs:      dict = {}

        self.create_subscription(String, '/mission/state',  self._mission_cb,  10)
        self._pub = self.create_publisher(String, '/fleet/health', 10)

        self.create_timer(1.0, self._tick)

        self.get_logger().info('FleetHealthMonitor running — publishing to /fleet/health')

    # ── Discovery + per-robot subscriptions ──────────────────────────────────

    def _discover(self):
        topics = self.get_topic_names_and_types()
        robots = {
            t.split('/')[1]
            for t, _ in topics
            if t.count('/') >= 2 and t.endswith('/cmd_vel')
        }
        for ns in robots - self._known_robots:
            self._known_robots.add(ns)
            self._odom_rates[ns] = RateTracker()
            self._scan_rates[ns] = RateTracker()

            self._odom_subs[ns] = self.create_subscription(
                Odometry, f'/{ns}/odom',
                lambda msg, n=ns: self._odom_cb(n), 10)
            self._scan_subs[ns] = self.create_subscription(
                LaserScan, f'/{ns}/scan',
                lambda msg, n=ns: self._scan_cb(n), 10)
            self._col_subs[ns] = self.create_subscription(
                String, f'/{ns}/collision_monitor/state',
                lambda msg, n=ns: self._col_cb(msg, n), 10)

            self.get_logger().info(f'Health: tracking {ns}')

    def _odom_cb(self, ns: str):
        with self._lock:
            self._odom_rates[ns].tick()

    def _scan_cb(self, ns: str):
        with self._lock:
            self._scan_rates[ns].tick()

    def _col_cb(self, msg: String, ns: str):
        try:
            with self._lock:
                self._collision_states[ns] = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _mission_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            robot = data.get('robot', '')
            if robot:
                with self._lock:
                    self._mission_states[robot] = data.get('state', 'UNKNOWN')
        except json.JSONDecodeError:
            pass

    # ── Health tick ───────────────────────────────────────────────────────────

    def _tick(self):
        self._discover()

        nodes = set(self.get_node_names())

        with self._lock:
            robots = sorted(self._known_robots)
            report = {}

            for ns in robots:
                odom_hz = self._odom_rates[ns].hz
                scan_hz = self._scan_rates[ns].hz
                nav2_ok = any('bt_navigator' in n for n in nodes)
                mission = self._mission_states.get(ns, 'UNKNOWN')
                col     = self._collision_states.get(ns, {})
                col_state = col.get('state', 'UNKNOWN')

                if odom_hz < 0.1 or scan_hz < 0.1:
                    overall = 'ERROR'
                elif odom_hz < _ODOM_WARN_HZ or scan_hz < _SCAN_WARN_HZ or not nav2_ok:
                    overall = 'WARN'
                elif col_state == 'STOP':
                    overall = 'WARN'
                else:
                    overall = 'OK'

                report[ns] = {
                    'overall':   overall,
                    'odom_hz':   round(odom_hz, 1),
                    'scan_hz':   round(scan_hz, 1),
                    'nav2':      nav2_ok,
                    'mission':   mission,
                    'collision': col_state,
                }

        msg      = String()
        msg.data = json.dumps(report)
        self._pub.publish(msg)


# ── CLI ────────────────────────────────────────────────────────────────────────

def _status(node: Node):
    received = [None]

    def _cb(msg):
        received[0] = json.loads(msg.data)

    node.create_subscription(String, '/fleet/health', _cb, 10)
    spin = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin.start()
    deadline = time.time() + 3.0
    while received[0] is None and time.time() < deadline:
        time.sleep(0.05)

    if received[0]:
        data = received[0]
        print('\n── Fleet Health ──────────────────────────────────────')
        for ns, s in data.items():
            icon = {'OK': '✓', 'WARN': '⚠', 'ERROR': '✗'}.get(s['overall'], '?')
            print(f'  {icon} {ns:<10}  '
                  f'odom={s["odom_hz"]:4.1f}Hz  '
                  f'scan={s["scan_hz"]:4.1f}Hz  '
                  f'nav2={"✓" if s["nav2"] else "✗"}  '
                  f'mission={s["mission"]:<12}  '
                  f'collision={s["collision"]}')
        if not data:
            print('  No robots detected.')
        print('──────────────────────────────────────────────────────\n')
    else:
        print('No fleet_health monitor found (is it running?)')


def _usage():
    print(__doc__)
    sys.exit(0)


def main():
    argv = sys.argv[1:]

    if not argv:
        rclpy.init()
        node = FleetHealthMonitor()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return

    rclpy.init()
    node = Node('fleet_health_cli')

    if argv[0] == 'status':
        _status(node)
    else:
        _usage()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
