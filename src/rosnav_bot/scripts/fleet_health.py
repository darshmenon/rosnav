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

Optional ros2_pulse
───────────────────
  If ros2_pulse (apt: ros-humble-ros2-pulse) is running (LD_PRELOAD probe, jsonl logs, e.g.
  `multi_robot.launch.py pulse:=true`), odom_hz / scan_hz come from its logs and this node does
  not subscribe to every robot's /odom and /scan itself. Without fresh pulse logs, or for a
  robot pulse has not seen, it keeps the subscriptions below, so nothing needs installing.
  Pulse rates are 5 s window averages: slower to react than the 2 s subscription window.
  Parameter use_pulse: auto (default) | false (never read pulse).  Report has a per-robot "source".

Overall rules
─────────────
  ERROR   odom or scan dropped to 0 Hz
  WARN    odom or scan below threshold, OR nav2_alive = false, OR no exploration progress
          for stall_s (60 s) while frontiers remain (progress = stalled)
  OK      all checks green

Usage
─────
  # Run as daemon (publishes /fleet/health):
  ros2 run rosnav_bot fleet_health.py

  # One-shot status print:
  ros2 run rosnav_bot fleet_health.py status

  # Or via fleet_manager:
  ros2 run rosnav_bot fleet_manager.py health
"""

import json
import sys
import threading
import time
from collections import defaultdict, deque

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

try:                                    # optional: only used when ros2_pulse logs exist
    from pulse_rates import PulseRates
except ImportError:                     # pragma: no cover - module missing from an install
    PulseRates = None

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

        self.declare_parameter('use_pulse', 'auto')        # auto | false
        self.declare_parameter('pulse_glob', '')           # '' = $TMPDIR/topic_freq.*.log
        mode = str(self.get_parameter('use_pulse').value).strip().lower()
        self._pulse = None
        if mode != 'false' and PulseRates is not None:
            self._pulse = PulseRates(str(self.get_parameter('pulse_glob').value))
        self._sources:       dict[str, str] = {}
        self._stall_warned = False

        self.create_subscription(String, '/mission/state',  self._mission_cb,  10)
        # Exploration progress: odom/scan alive does not mean the fleet is getting anywhere.
        self.declare_parameter('stall_s', 60.0)
        self._stall_s = float(self.get_parameter('stall_s').value)
        self._prog_sig = None
        self._prog_changed = time.monotonic()
        self._frontiers = 0
        self.create_subscription(String, '/exploration/stats', self._stats_cb, 10)
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

            self._col_subs[ns] = self.create_subscription(
                String, f'/{ns}/collision_monitor/state',
                lambda msg, n=ns: self._col_cb(msg, n), 10)

            self.get_logger().info(f'Health: tracking {ns}')

    def _ensure_stream_subs(self, ns: str):
        """Own /odom and /scan subscriptions: the fallback when pulse has no data for this robot."""
        if ns in self._odom_subs:
            return
        self._odom_subs[ns] = self.create_subscription(
            Odometry, f'/{ns}/odom', lambda msg, n=ns: self._odom_cb(n), 10)
        self._scan_subs[ns] = self.create_subscription(
            LaserScan, f'/{ns}/scan', lambda msg, n=ns: self._scan_cb(n), 10)
        self.get_logger().info(f'Health: {ns} odom/scan from own subscriptions')

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

    def _stats_cb(self, msg: String):
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        sig = (d.get('succeeded_goals'), d.get('visited_count'), d.get('explored_area_m2'))
        with self._lock:
            if sig != self._prog_sig:
                self._prog_sig = sig
                self._prog_changed = time.monotonic()
            self._frontiers = int(d.get('frontier_count') or 0)

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

        node_ns_pairs = self.get_node_names_and_namespaces()
        pulse = self._pulse.read() if self._pulse is not None else {}

        # Per robot: pulse when it reports both streams, else our own subscriptions.
        for ns in sorted(self._known_robots):
            src = 'pulse' if (f'/{ns}/odom' in pulse and f'/{ns}/scan' in pulse) else 'subscription'
            if src == 'subscription':
                self._ensure_stream_subs(ns)
            if self._sources.get(ns) != src:
                self._sources[ns] = src
                self.get_logger().info(f'Health: {ns} rates from {src}')

        with self._lock:
            robots = sorted(self._known_robots)
            report = {}
            stalled_s = time.monotonic() - self._prog_changed
            stalled = (self._prog_sig is not None and self._frontiers > 0
                       and stalled_s >= self._stall_s)
            if stalled and not self._stall_warned:
                self.get_logger().warn(
                    f'Health: no exploration progress for {stalled_s:.0f}s with '
                    f'{self._frontiers} frontier(s) left - fleet is stalled')
            self._stall_warned = stalled

            for ns in robots:
                if self._sources.get(ns) == 'pulse':
                    odom_hz = pulse[f'/{ns}/odom']
                    scan_hz = pulse[f'/{ns}/scan']
                else:
                    odom_hz = self._odom_rates[ns].hz
                    scan_hz = self._scan_rates[ns].hz
                nav2_ok = any(
                    name == 'bt_navigator' and namespace.rstrip('/') == f'/{ns}'
                    for name, namespace in node_ns_pairs
                )
                mission = self._mission_states.get(ns, 'UNKNOWN')
                col     = self._collision_states.get(ns, {})
                col_state = col.get('state', 'UNKNOWN')

                if odom_hz < 0.1 or scan_hz < 0.1:
                    overall = 'ERROR'
                elif odom_hz < _ODOM_WARN_HZ or scan_hz < _SCAN_WARN_HZ or not nav2_ok:
                    overall = 'WARN'
                elif col_state == 'STOP':
                    overall = 'WARN'
                elif stalled:
                    overall = 'WARN'
                else:
                    overall = 'OK'

                report[ns] = {
                    'overall':   overall,
                    'odom_hz':   round(odom_hz, 1),
                    'scan_hz':   round(scan_hz, 1),
                    'source':    self._sources.get(ns, 'subscription'),
                    'nav2':      nav2_ok,
                    'mission':   mission,
                    'collision': col_state,
                    'progress':  'stalled' if stalled else 'ok',
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
    argv = remove_ros_args(args=sys.argv)[1:]

    if not argv:
        rclpy.init()
        node = FleetHealthMonitor()
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
        return

    rclpy.init()
    node = Node('fleet_health_cli')

    if argv[0] == 'status':
        _status(node)
    else:
        _usage()

    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
