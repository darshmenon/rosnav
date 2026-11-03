#!/usr/bin/env python3
"""
nav2_bringup_watchdog.py - restart a robot's Nav2 stack when its lifecycle bringup aborted.

Under load (many robots, low Gazebo real-time factor) a lifecycle_manager can start before a
node's lifecycle services are discovered and abort with "Failed to change state ... async_send_request
failed". Nav2 has no retry, so that robot's bt_navigator stays inactive and rejects every goal.
This node polls /<robot>/lifecycle_manager_navigation/is_active and, for a robot that is still
inactive after `grace_s`, calls manage_nodes RESET then STARTUP (at most `max_retries` times).

A call that gets no reply (lost service response under load) is abandoned after `call_timeout_s`
so the robot is retried instead of being blocked forever, and a manager whose services never
appear is reported as a warning every `period_s * 3` instead of being skipped silently.

Parameters: robots (list), grace_s (default 45), period_s (default 10), max_retries (default 4),
call_timeout_s (default 20), restart_after (default 2: consecutive unanswered calls before the
manager process is restarted).

A manager that hung waiting for a lost lifecycle reply never answers again, so after
`restart_after` unanswered calls the watchdog SIGTERMs that one process; the launch respawns it
(respawn=True in nav2_navigation_global_tf.launch.py) and the normal RESET + STARTUP path runs.
The process is found by its exact `__ns:=/<robot>` + node-name arguments and must carry this
watchdog's own ROS_DOMAIN_ID, so another session's robots are never touched.
"""
import os
import signal
import time

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
        self.declare_parameter('call_timeout_s', 20.0)
        self._call_timeout = float(self.get_parameter('call_timeout_s').value)
        self.declare_parameter('restart_after', 2)
        self._restart_after = int(self.get_parameter('restart_after').value)
        self._no_reply = {}
        self._termed = set()          # pids already sent SIGTERM: a repeat gets SIGKILL
        self._robots = list(self.get_parameter('robots').value)
        self._grace = float(self.get_parameter('grace_s').value)
        self._max = int(self.get_parameter('max_retries').value)
        self._period = float(self.get_parameter('period_s').value)
        self._t0 = self.get_clock().now()
        self._active = {r: False for r in self._robots}
        self._retries = {r: 0 for r in self._robots}
        self._busy = {r: False for r in self._robots}
        self._busy_since = {r: 0.0 for r in self._robots}
        self._unready_warned = {r: 0.0 for r in self._robots}
        self._is_active = {r: self.create_client(Trigger, f'/{r}/lifecycle_manager_navigation/is_active')
                           for r in self._robots}
        self._manage = {r: self.create_client(ManageLifecycleNodes, f'/{r}/lifecycle_manager_navigation/manage_nodes')
                        for r in self._robots}
        self.create_timer(float(self.get_parameter('period_s').value), self._tick)
        self.get_logger().info(f'Nav2 bringup watchdog for {self._robots} (grace {self._grace:.0f}s, '
                               f'max {self._max} retries)')

    def _tick(self):
        up = (self.get_clock().now() - self._t0).nanoseconds / 1e9
        now = time.monotonic()
        for r in self._robots:
            if self._active[r]:
                continue
            if self._busy[r]:
                if now - self._busy_since[r] < self._call_timeout:
                    continue
                self._no_reply[r] = self._no_reply.get(r, 0) + 1
                self.get_logger().warn(f'[{r}] lifecycle call got no reply in {self._call_timeout:.0f}s '
                                       f'(#{self._no_reply[r]}) - abandoning it, will retry')
                self._busy[r] = False
                if self._no_reply[r] >= self._restart_after:
                    self._no_reply[r] = 0
                    self._restart_manager(r)
                    continue
            if not self._is_active[r].service_is_ready():
                # Manager not up yet (Nav2 still starting) or its services never got discovered.
                if up >= self._grace and now - self._unready_warned[r] >= self._period * 3:
                    self._unready_warned[r] = now
                    self.get_logger().warn(f'[{r}] lifecycle_manager_navigation services not visible '
                                           f'after {up:.0f}s - Nav2 for {r} is probably not up')
                    if up >= self._grace + 3 * self._period:
                        self._restart_manager(r, quiet=True)   # only acts if its process exists
                continue
            self._busy[r] = True
            self._busy_since[r] = now
            f = self._is_active[r].call_async(Trigger.Request())
            f.add_done_callback(lambda fut, ns=r, t=up: self._on_status(ns, fut, t))

    def _on_status(self, r, fut, up):
        try:
            ok = bool(fut.result().success)
        except Exception:
            ok = False
        if fut.done():
            self._no_reply[r] = 0
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

    def _manager_pids(self, r):
        """Exact PIDs of /<r>/lifecycle_manager_navigation in OUR ROS domain (never anyone else's)."""
        domain = os.environ.get('ROS_DOMAIN_ID', '')
        pids = []
        for d in os.listdir('/proc'):
            if not d.isdigit():
                continue
            try:
                with open(f'/proc/{d}/cmdline', 'rb') as f:
                    args = f.read().decode(errors='replace').split('\0')
                if f'__node:=lifecycle_manager_navigation' not in args or f'__ns:=/{r}' not in args:
                    continue
                with open(f'/proc/{d}/environ', 'rb') as f:
                    env = dict(kv.split('=', 1) for kv in f.read().decode(errors='replace').split('\0')
                               if '=' in kv)
                if env.get('ROS_DOMAIN_ID', '') == domain:
                    pids.append(int(d))
            except OSError:
                continue
        return pids

    def _restart_manager(self, r, quiet=False):
        pids = self._manager_pids(r)
        if not pids:
            if not quiet:
                self.get_logger().warn(f'[{r}] manager unresponsive but its process was not found '
                                       f'(composed or another host?) - cannot restart it')
            return
        for pid in pids:
            sig = signal.SIGKILL if pid in self._termed else signal.SIGTERM
            self._termed.add(pid)
            self.get_logger().warn(f'[{r}] lifecycle_manager_navigation (pid {pid}) is unresponsive - '
                                   f'restarting it with {sig.name} (launch respawns it)')
            try:
                os.kill(pid, sig)
            except OSError as e:
                self.get_logger().warn(f'[{r}] could not signal pid {pid}: {e}')
        self._retries[r] = 0      # fresh manager: allow the full RESET + STARTUP budget again

    def _call(self, r, command, then):
        if not self._manage[r].service_is_ready():
            self.get_logger().warn(f'[{r}] manage_nodes service not ready - will retry next tick')
            self._busy[r] = False
            return
        self._busy_since[r] = time.monotonic()
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
