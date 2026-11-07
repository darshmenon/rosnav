#!/usr/bin/env python3
"""
rosnav_supervisor.py - continuous diagnostics + safe self-healing for a rosnav fleet.

Inspired by EasyNav's recovery design: components expose health as standard diagnostics and a
separate node mitigates. This node is both halves, kept apart in code:
  * health  : the same checks as rosnav_doctor.diagnose() (TF age, odom/scan rate, Nav2 lifecycle),
              sampled continuously instead of once.
  * policy  : supervisor_core.Supervisor decides (persistence, cooldown, cap). Pure + unit-tested.
  * mitigate: only mild actions - clear the robot's costmaps. No lifecycle restarts (those
              deadlocked in a real TF freeze, see tf_freeze_watchdog.py).

Publishes
  /diagnostics                  diagnostic_msgs/DiagnosticArray, one status per robot
  /rosnav/supervisor/status     std_msgs/String JSON {robots, timeline, mitigate} for rosnav_top.py
Parameters
  robots ([]), window_sec (5.0), period_sec (1.0), persist_sec (5.0), cooldown_sec (30.0),
  max_actions (8), mitigate (true)  - false = observe only (dry run, timeline still shows intent)
"""
import json
import os
import sys
import time
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
import tf2_ros
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

import rosnav_doctor
import supervisor_core


class RosnavSupervisor(Node):
    def __init__(self):
        super().__init__('rosnav_supervisor')
        self.declare_parameter('robots', [''])
        self.declare_parameter('window_sec', 5.0)
        self.declare_parameter('period_sec', 1.0)
        self.declare_parameter('persist_sec', 5.0)
        self.declare_parameter('cooldown_sec', 30.0)
        self.declare_parameter('max_actions', 8)
        self.declare_parameter('mitigate', True)
        g = lambda k: self.get_parameter(k).value
        self._win = float(g('window_sec'))
        self._mitigate = bool(g('mitigate'))
        self._robots = [r for r in g('robots') if r]
        self._core = supervisor_core.Supervisor(float(g('persist_sec')), float(g('cooldown_sec')),
                                                int(g('max_actions')))
        self._buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._buf, self)
        self._arr = {}       # robot -> {'odom': deque, 'scan': deque}
        self._active = {}    # robot -> bool|None (Nav2 lifecycle)
        self._clients = {}
        self._pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self._pub_status = self.create_publisher(String, '/rosnav/supervisor/status', 10)
        self.create_timer(3.0, self._discover)
        self.create_timer(5.0, self._poll_lifecycle)
        self.create_timer(float(g('period_sec')), self._tick)
        self.get_logger().info(
            f'supervisor up: robots={self._robots or "auto-detect"} persist={g("persist_sec")}s '
            f'cooldown={g("cooldown_sec")}s max_actions={g("max_actions")} mitigate={self._mitigate}')

    # ----- discovery / subscriptions
    def _discover(self):
        if not self._robots:
            found = sorted({t.split('/')[1] for t, _ in self.get_topic_names_and_types()
                            if t.count('/') >= 2 and t.endswith('/cmd_vel')})
            if not found:
                return
            self._robots = found
            self.get_logger().info(f'auto-detected robots: {found}')
        for r in self._robots:
            if r in self._arr:
                continue
            self._arr[r] = {'odom': deque(maxlen=400), 'scan': deque(maxlen=400)}
            for key, typ in (('odom', Odometry), ('scan', LaserScan)):
                self.create_subscription(typ, f'/{r}/{key}',
                                         lambda m, r=r, key=key: self._arr[r][key].append(time.monotonic()),
                                         qos_profile_sensor_data)
            self._clients[r] = {
                'life': self.create_client(Trigger, f'/{r}/lifecycle_manager_navigation/is_active'),
                'g': self.create_client(ClearEntireCostmap, f'/{r}/global_costmap/clear_entirely_global_costmap'),
                'l': self.create_client(ClearEntireCostmap, f'/{r}/local_costmap/clear_entirely_local_costmap'),
            }
            self.get_logger().info(f'[{r}] monitoring odom/scan/TF/Nav2 lifecycle')

    def _poll_lifecycle(self):
        for r, c in self._clients.items():
            if not c['life'].service_is_ready():
                self._active[r] = None
                continue
            c['life'].call_async(Trigger.Request()).add_done_callback(
                lambda f, r=r: self._active.__setitem__(r, bool(f.result().success) if f.result() else False))

    # ----- measurement
    def _rate(self, dq):
        now = time.monotonic()
        recent = [t for t in dq if now - t <= self._win]
        if len(recent) < 3 or recent[-1] <= recent[0]:
            return 0.0
        return (len(recent) - 1) / (recent[-1] - recent[0])

    def _age(self, target, source):
        try:
            tr = self._buf.lookup_transform(target, source, rclpy.time.Time())
        except Exception:
            return None
        return max(0.0, (self.get_clock().now() - rclpy.time.Time.from_msg(tr.header.stamp)).nanoseconds / 1e9)

    def _tick(self):
        if not self._arr:
            return
        now = time.monotonic()
        report, metrics = {}, {}
        slam = any('slam_toolbox' in n for n, _ in self.get_node_names_and_namespaces())
        for i, r in enumerate(self._robots):
            m = {'tf_map_odom_age': self._age('map', f'{r}/odom'),
                 'tf_odom_base_age': self._age(f'{r}/odom', f'{r}/base_link'),
                 'nav2_active': self._active.get(r),
                 'odom_hz': self._rate(self._arr[r]['odom']), 'scan_hz': self._rate(self._arr[r]['scan']),
                 'slam': slam and i == 0}
            metrics[r] = m
            report[r] = rosnav_doctor.diagnose(m)
        for act in self._core.step(now, report):
            self._apply(act)
        self._publish(metrics, report)

    # ----- mitigation
    def _apply(self, act):
        kind, robot = act[0], act[1]
        if kind == 'log':
            self.get_logger().warn(f'[{robot}] needs operator: {act[2][:200]}')
        elif kind == 'clear_costmaps':
            if not self._mitigate:
                self.get_logger().warn(f'[{robot}] DRY RUN: would clear costmaps (mitigate:=false)')
                return
            self.get_logger().warn(f'[{robot}] MITIGATING: clearing global+local costmaps (map->odom stale)')
            for k in ('g', 'l'):
                c = self._clients[robot][k]
                if c.service_is_ready():
                    c.call_async(ClearEntireCostmap.Request())
                else:
                    self.get_logger().error(f'[{robot}] clear service for {k} costmap not available')

    def _publish(self, metrics, report):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        for r in self._robots:
            st = self._core.robots.get(r)
            lvl, msg, kv = supervisor_core.to_diagnostic_fields(r, st.level if st else 'OK', report[r], metrics[r])
            ds = DiagnosticStatus(level=bytes([lvl]), name=f'rosnav/{r}', message=msg, hardware_id=r)
            ds.values = [KeyValue(key=k, value=v) for k, v in kv]
            arr.status.append(ds)
        self._pub_diag.publish(arr)
        snap = self._core.snapshot()
        snap['mitigate'] = self._mitigate
        snap['metrics'] = {r: {k: (round(v, 2) if isinstance(v, float) else v) for k, v in m.items()}
                           for r, m in metrics.items()}
        self._pub_status.publish(String(data=json.dumps(snap)))


def main():
    rclpy.init()
    node = RosnavSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
