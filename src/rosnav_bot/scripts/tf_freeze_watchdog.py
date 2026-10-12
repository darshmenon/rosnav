#!/usr/bin/env python3
"""
tf_freeze_watchdog.py — recover from the Nav2 controller's frozen map->odom view.

Symptom (hospital/cafe/maze runs, 2026-10-11): controller_server logs
"Transform data too old when converting from map to odom" over and over while
its map->odom transform sits at a stale time and every other node still sees it
fresh. The robot stops driving, the explorer blacklists the frontiers it can't
reach, and the map stalls part-way. The root cause is not found; strip_inert_filters
(_common.py) removed one trigger but not all of them.

This node counts those log lines on /rosout. If `threshold` of them arrive within
`window_sec`, it asks the Nav2 lifecycle manager to RESET and then STARTUP the
navigation stack, which recreates controller_server and its costmaps with fresh TF
listeners. A cooldown and a restart cap keep it from looping.

NOTE (2026-10-12): EXPERIMENTAL — do not enable. (1) The "Transform data too old" lines come from the
non-node logger "tf_help" and never reach /rosout, so this node does not see them. (2) A manual
RESET+STARTUP of the navigation lifecycle manager deadlocked in planner_server/global_costmap cleanup
during a real freeze and left Nav2 down. Launch default is tf_watchdog:=false.

Parameters:
  window_sec        (float, 5.0)   sliding window for counting errors
  threshold         (int,   20)    errors inside the window that trigger a restart
  cooldown_sec      (float, 60.0)  minimum time between restarts
  max_restarts      (int,   6)     give up after this many (0 = unlimited)
  manager           (str, 'lifecycle_manager_navigation')
  match             (str, 'Transform data too old')
"""

import time
from collections import deque

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from nav2_msgs.srv import ManageLifecycleNodes


class TfFreezeWatchdog(Node):
    def __init__(self):
        super().__init__('tf_freeze_watchdog')
        self.declare_parameter('window_sec', 5.0)
        self.declare_parameter('threshold', 20)
        self.declare_parameter('cooldown_sec', 60.0)
        self.declare_parameter('max_restarts', 6)
        self.declare_parameter('manager', 'lifecycle_manager_navigation')
        self.declare_parameter('match', 'Transform data too old')

        self._window = float(self.get_parameter('window_sec').value)
        self._threshold = int(self.get_parameter('threshold').value)
        self._cooldown = float(self.get_parameter('cooldown_sec').value)
        self._max = int(self.get_parameter('max_restarts').value)
        self._match = str(self.get_parameter('match').value)
        mgr = str(self.get_parameter('manager').value)

        self._hits = deque()
        self._restarts = 0
        self._last_restart = -1e9
        self._busy = False
        self._total_seen = 0

        self._cli = self.create_client(ManageLifecycleNodes, f'/{mgr}/manage_nodes')
        self.create_subscription(Log, '/rosout', self._on_log, 200)
        self.create_timer(30.0, self._heartbeat)
        self.get_logger().info(
            f'watching /rosout for "{self._match}": >= {self._threshold} in {self._window:.0f}s '
            f'-> RESET+STARTUP via /{mgr}/manage_nodes (cooldown {self._cooldown:.0f}s, '
            f'max {self._max or "unlimited"} restarts)')

    def _heartbeat(self):
        self.get_logger().info(
            f'alive: {self._total_seen} matching errors seen, {self._restarts} restart(s) so far')

    def _on_log(self, msg: Log):
        if self._match not in msg.msg:
            return
        now = time.monotonic()
        self._total_seen += 1
        self._hits.append(now)
        while self._hits and now - self._hits[0] > self._window:
            self._hits.popleft()
        if (len(self._hits) >= self._threshold and not self._busy
                and now - self._last_restart >= self._cooldown):
            if self._max and self._restarts >= self._max:
                if self._restarts == self._max:
                    self.get_logger().error(
                        f'TF freeze again but already restarted {self._max} times — giving up.')
                    self._restarts += 1
                return
            self._restart(len(self._hits))

    def _restart(self, n_hits: int):
        self._busy = True
        self._restarts += 1
        self._last_restart = time.monotonic()
        self.get_logger().warn(
            f'TF FREEZE detected ({n_hits} "{self._match}" in {self._window:.0f}s, '
            f'total {self._total_seen}) — restarting Nav2 (restart #{self._restarts})')
        if not self._cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('lifecycle manager service not available — cannot restart')
            self._busy = False
            return
        self._hits.clear()
        self._call(ManageLifecycleNodes.Request.RESET, self._after_reset)

    def _call(self, command, cb):
        req = ManageLifecycleNodes.Request()
        req.command = command
        self._cli.call_async(req).add_done_callback(cb)

    def _after_reset(self, fut):
        ok = fut.result().success if fut.result() else False
        self.get_logger().info(f'RESET -> {"ok" if ok else "FAILED"}; sending STARTUP')
        self._call(ManageLifecycleNodes.Request.STARTUP, self._after_startup)

    def _after_startup(self, fut):
        ok = fut.result().success if fut.result() else False
        self._last_restart = time.monotonic()
        self._hits.clear()
        self._busy = False
        self.get_logger().info(
            f'STARTUP -> {"ok — Nav2 is back with fresh TF listeners" if ok else "FAILED"}')


def main():
    rclpy.init()
    node = TfFreezeWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
