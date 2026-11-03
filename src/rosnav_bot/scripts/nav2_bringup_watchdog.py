#!/usr/bin/env python3
"""
nav2_bringup_watchdog.py - restart a robot's Nav2 stack when its lifecycle bringup aborted.

Under load (many robots, low Gazebo real-time factor) a lifecycle_manager can start before a
node's lifecycle services are discovered and abort with "Failed to change state ... async_send_request
failed". Nav2 has no retry, so that robot's bt_navigator stays inactive and rejects every goal.
This node polls /<robot>/lifecycle_manager_navigation/is_active and, for a robot that is still
inactive after `grace_s`, calls manage_nodes RESET then STARTUP (at most `max_retries` times).

Parameters: robots (list), grace_s (default 45), period_s (default 10), max_retries (default 4).
"""
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ManageLifecycleNodes
from std_srvs.srv import Trigger

_STARTUP, _RESET = ManageLifecycleNodes.Request.STARTUP, ManageLifecycleNodes.Request.RESET


class Nav2BringupWatchdog(Node):
    def __init__(self):
        super().__init__('nav2_bringup_watchdog')
        self.declare_parameter('robots', ['robot1'])
        self.declare_parameter('grace_s', 45.0)
        self.declare_parameter('period_s', 10.0)
        self.declare_parameter('max_retries', 4)
        self._robots = list(self.get_parameter('robots').value)
        self._grace = float(self.get_parameter('grace_s').value)
        self._max = int(self.get_parameter('max_retries').value)
        self._t0 = self.get_clock().now()
        self._active = {r: False for r in self._robots}
        self._retries = {r: 0 for r in self._robots}
        self._busy = {r: False for r in self._robots}
        self._is_active = {r: self.create_client(Trigger, f'/{r}/lifecycle_manager_navigation/is_active')
                           for r in self._robots}
        self._manage = {r: self.create_client(ManageLifecycleNodes, f'/{r}/lifecycle_manager_navigation/manage_nodes')
                        for r in self._robots}
        self.create_timer(float(self.get_parameter('period_s').value), self._tick)
        self.get_logger().info(f'Nav2 bringup watchdog for {self._robots} (grace {self._grace:.0f}s, '
                               f'max {self._max} retries)')

    def _tick(self):
        up = (self.get_clock().now() - self._t0).nanoseconds / 1e9
        for r in self._robots:
            if self._busy[r] or self._active[r]:
                continue
            if not self._is_active[r].service_is_ready():
                continue                      # manager not up yet: Nav2 still starting
            self._busy[r] = True
            f = self._is_active[r].call_async(Trigger.Request())
            f.add_done_callback(lambda fut, ns=r, t=up: self._on_status(ns, fut, t))

    def _on_status(self, r, fut, up):
        try:
            ok = bool(fut.result().success)
        except Exception:
            ok = False
        if ok:
            if not self._active[r]:
                self.get_logger().info(f'[{r}] Nav2 active')
            self._active[r] = True
            self._busy[r] = False
            return
        if up < self._grace or self._retries[r] >= self._max:
            self._busy[r] = False
            return
        self._retries[r] += 1
        self.get_logger().warn(f'[{r}] Nav2 not active after {up:.0f}s - reset + startup '
                               f'(retry {self._retries[r]}/{self._max})')
        self._call(r, _RESET, lambda: self._call(r, _STARTUP, lambda: self._done(r)))

    def _call(self, r, command, then):
        if not self._manage[r].service_is_ready():
            self._busy[r] = False
            return
        req = ManageLifecycleNodes.Request()
        req.command = command
        f = self._manage[r].call_async(req)

        def _cb(fut):
            try:
                ok = bool(fut.result().success)
            except Exception:
                ok = False
            if not ok and command == _STARTUP:
                self.get_logger().warn(f'[{r}] startup call failed')
            then()
        f.add_done_callback(_cb)

    def _done(self, r):
        self._busy[r] = False


def main():
    rclpy.init()
    node = Nav2BringupWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
