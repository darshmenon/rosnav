#!/usr/bin/env python3
"""
slam_stall_watchdog.py — restart slam_toolbox when it hangs and stops publishing map->odom.

Observed 2026-10-12 (hospital/cafe, slam_toolbox online_async): a run sometimes stalls ~30-35 s in.
slam_toolbox logs "Message Filter dropping message ... queue is full", the map stays at its first
~1,100 cells, map->odom stops updating, the node no longer answers services, and everything
downstream (Nav2 "Transform data too old", explorer "no frontiers") fails. Unlike the controller-side
TF freeze this one IS visible from outside: map->odom goes stale while odom->base_link stays live.

Only if /scan is still arriving (a silent /scan means the fault is upstream and a restart cannot help),
if map->odom is older than `stale_sec` (sim time) while odom->base_link is fresh, this node sends
SIGKILL to the slam_toolbox process and the launch's respawn=True brings a fresh one up.
Only a direct child of this node's own launch process is ever killed (never a name/pattern kill,
the machine is shared with other ROS instances).

Parameters:
  stale_sec        (float, 8.0)   map->odom age that counts as hung
  startup_grace    (float, 90.0)  sim seconds before a never-seen map->odom counts as hung
  cooldown_sec     (float, 40.0)  wall seconds to wait after a restart before judging again
  max_restarts     (int,   5)     give up after this many (0 = unlimited)
  process_match    (str, 'slam_toolbox_node')  substring of the child's cmdline
  posegraph_path   (str, '')     if set (slam_posegraph_autosave's output prefix), the last saved pose graph
                                 is reloaded at the robot's last known pose after a restart, so the map
                                 built so far is not lost
"""

import os
import signal
import time

import math

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
import tf2_ros
from slam_toolbox.srv import DeserializePoseGraph


def _children_of(ppid: int):
    out = []
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open(f'/proc/{pid}/stat') as f:
                stat = f.read()
            # field 4 (after the ")" that closes comm) is the parent pid
            parent = int(stat.rsplit(')', 1)[1].split()[1])
            if parent != ppid:
                continue
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode(errors='ignore')
            out.append((int(pid), cmd))
        except (OSError, ValueError, IndexError):
            continue
    return out


class SlamStallWatchdog(Node):
    def __init__(self):
        super().__init__('slam_stall_watchdog')
        self.declare_parameter('stale_sec', 8.0)
        self.declare_parameter('startup_grace', 90.0)
        self.declare_parameter('cooldown_sec', 40.0)
        self.declare_parameter('max_restarts', 5)
        self.declare_parameter('process_match', 'slam_toolbox_node')
        self.declare_parameter('posegraph_path', '')
        self._stale = float(self.get_parameter('stale_sec').value)
        self._grace = float(self.get_parameter('startup_grace').value)
        self._cool = float(self.get_parameter('cooldown_sec').value)
        self._max = int(self.get_parameter('max_restarts').value)
        self._match = str(self.get_parameter('process_match').value)
        self._pg = str(self.get_parameter('posegraph_path').value).strip()
        self._last_pose = None            # (x, y, yaw) of the odom frame in the map frame, while SLAM was healthy
        self._restore_at = None           # wall time after which to reload the pose graph
        self._restore_done_by = 0.0
        self._deser = self.create_client(DeserializePoseGraph, '/slam_toolbox/deserialize_map')
        # A stale map->odom is only slam_toolbox's fault if scans are still reaching it. If /scan is silent
        # (e.g. scan_quality_gate rejecting everything) a restart cannot help and just throws the map away.
        self._last_scan_wall = 0.0
        self._warned_no_scan = 0.0
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)

        self._buf = tf2_ros.Buffer()
        self._tl = tf2_ros.TransformListener(self._buf, self)
        self._start = self.get_clock().now()
        self._seen = False
        self._restarts = 0
        self._last_restart = 0.0
        self._last_log = 0.0
        self.create_timer(2.0, self._tick)
        self.get_logger().info(
            f'watching map->odom: restart slam_toolbox if older than {self._stale:.0f}s '
            f'(grace {self._grace:.0f}s, cooldown {self._cool:.0f}s, max {self._max or "unlimited"})')

    def _on_scan(self, _msg):
        self._last_scan_wall = time.monotonic()

    def _age(self, parent, child):
        try:
            t = self._buf.lookup_transform(parent, child, Time())
        except Exception:
            return None
        return (self.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds / 1e9

    def _tick(self):
        now_wall = time.monotonic()
        self._maybe_restore(now_wall)
        if now_wall - self._last_restart < self._cool:
            return
        up = (self.get_clock().now() - self._start).nanoseconds / 1e9
        a_mo = self._age('map', 'odom')
        a_ob = self._age('odom', 'base_link')
        if a_mo is not None:
            self._seen = True
            if a_mo < 3.0:
                self._remember_pose()
        if a_ob is None or a_ob > 5.0:
            return                       # odometry itself is not live — not a SLAM hang
        hung = (a_mo is not None and a_mo > self._stale) or (not self._seen and up > self._grace)
        if now_wall - self._last_log > 60.0:
            self._last_log = now_wall
            self.get_logger().info(
                f'ok: map->odom age={"none" if a_mo is None else f"{a_mo:.1f}s"}, up={up:.0f}s, '
                f'restarts={self._restarts}')
        if not hung:
            return
        if time.monotonic() - self._last_scan_wall > 4.0:
            if now_wall - self._warned_no_scan > 30.0:
                self._warned_no_scan = now_wall
                self.get_logger().warn(
                    f'map->odom stale ({a_mo}) but /scan has been silent for >4 s — the problem is upstream of '
                    'slam_toolbox (scan filter/gate, bridge), NOT restarting it')
            return
        if self._max and self._restarts >= self._max:
            if self._restarts == self._max:
                self.get_logger().error(f'slam_toolbox hung again but already restarted {self._max}x — giving up')
                self._restarts += 1
            return
        self._kill_slam(a_mo, up)

    @staticmethod
    def _yaw(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def _remember_pose(self):
        """Keep the last healthy map->odom. The robot keeps driving while slam_toolbox is hung, so after
        a restart its map pose is rebuilt as (this map->odom) x (live odom->base_link), not the stale pose."""
        try:
            t = self._buf.lookup_transform('map', 'odom', Time()).transform
        except Exception:
            return
        self._last_pose = (t.translation.x, t.translation.y, self._yaw(t.rotation))

    def _pose_in_old_map(self):
        mx, my, myaw = self._last_pose
        try:
            t = self._buf.lookup_transform('odom', 'base_link', Time()).transform
        except Exception:
            return None
        ox, oy, oyaw = t.translation.x, t.translation.y, self._yaw(t.rotation)
        c, s = math.cos(myaw), math.sin(myaw)
        return (mx + c * ox - s * oy, my + s * ox + c * oy, myaw + oyaw)

    def _maybe_restore(self, now_wall):
        if self._restore_at is None or now_wall < self._restore_at:
            return
        if now_wall > self._restore_done_by:
            self.get_logger().error('pose-graph restore timed out — continuing with a fresh map')
            self._restore_at = None
            return
        if not self._deser.wait_for_service(timeout_sec=0.2):
            return                       # new slam_toolbox not up yet
        a_new = self._age('map', 'odom')
        if a_new is None or a_new > 2.0:
            return                       # service is advertised but the new node is not processing scans yet
        pose = self._pose_in_old_map()
        if pose is None:
            return                       # odom->base_link not available yet, retry next tick
        x, y, yaw = pose
        req = DeserializePoseGraph.Request()
        req.filename = self._pg
        req.match_type = DeserializePoseGraph.Request.START_AT_GIVEN_POSE
        req.initial_pose = Pose2D(x=x, y=y, theta=yaw)
        self._restore_at = None
        self.get_logger().info(
            f'restoring pose graph {self._pg} at map pose ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} deg)')
        self._deser.call_async(req).add_done_callback(
            lambda f: self.get_logger().info('pose-graph restore call returned' if f.result() is not None
                                             else 'pose-graph restore call FAILED'))

    def _kill_slam(self, a_mo, up):
        victims = [(p, c) for p, c in _children_of(os.getppid()) if self._match in c]
        if not victims:
            self.get_logger().error(
                f'map->odom stale ({a_mo}) but no "{self._match}" child of launch pid {os.getppid()} found')
            self._last_restart = time.monotonic()
            return
        self._restarts += 1
        self._last_restart = time.monotonic()
        if self._pg and self._last_pose is not None and os.path.exists(self._pg + '.posegraph'):
            self._restore_at = time.monotonic() + 3.0         # then polled until the new node publishes map->odom
            self._restore_done_by = time.monotonic() + 90.0
        elif self._pg:
            self.get_logger().warn('no saved pose graph / last pose yet — new slam_toolbox starts with a fresh map')
        for pid, cmd in victims:
            self.get_logger().warn(
                f'SLAM HANG: map->odom age={"none" if a_mo is None else f"{a_mo:.0f}s"} at sim t={up:.0f}s '
                f'-> SIGKILL slam_toolbox pid {pid} (restart #{self._restarts}); launch respawn restarts it')
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError as e:
                self.get_logger().error(f'kill failed: {e}')


def main():
    rclpy.init()
    node = SlamStallWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
