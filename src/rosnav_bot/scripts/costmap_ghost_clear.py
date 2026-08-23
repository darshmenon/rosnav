#!/usr/bin/env python3
"""
costmap_ghost_clear.py — Clear ghost/duplicate obstacles after a SLAM loop closure.

slam_toolbox's own /map self-corrects on loop closure (regenerated from the
corrected pose graph). Nav2's costmaps don't: static_layer picks up the
corrected /map within one map_update_interval, but obstacle_layer's live scan
marks were written at the *pre-correction* map-frame cell addresses and stay
there — until the robot re-observes that same physical spot and a raytrace
clears it, a stale/duplicate wall can sit offset from the real one. This is
most visible right after a big map->odom jump (a loop closure), so: watch
map->odom for a jump past a threshold and clear the costmap(s) when one hits,
via Nav2's ClearEntireCostmap service (same one 'ros2 run nav2_costmap_2d ...'
recovery behaviors use) — cheap, and freshly-visible cells get re-marked from
the next few scans anyway.

Usage (global costmap only, the default):
  ros2 run rosnav_bot costmap_ghost_clear.py --ros-args -p use_sim_time:=true

Usage (also clear the rolling local costmap on a jump):
  ros2 run rosnav_bot costmap_ghost_clear.py --ros-args -p use_sim_time:=true \\
      -p clear_local:=true

Namespaced (multi-robot):
  ros2 run rosnav_bot costmap_ghost_clear.py --ros-args -p use_sim_time:=true \\
      -r __ns:=/robot1 -p odom_frame:=robot1/odom -p base_frame:=robot1/base_link
"""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from nav2_msgs.srv import ClearEntireCostmap


def _yaw_deg(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


class CostmapGhostClear(Node):
    def __init__(self):
        super().__init__('costmap_ghost_clear')

        self.declare_parameter('fixed_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('check_period_sec', 1.0)
        self.declare_parameter('jump_threshold_m', 0.15)
        self.declare_parameter('jump_threshold_deg', 8.0)
        self.declare_parameter('cooldown_sec', 3.0)
        self.declare_parameter('clear_local', False)
        self.declare_parameter('global_costmap_service',
                                'global_costmap/clear_entirely_global_costmap')
        self.declare_parameter('local_costmap_service',
                                'local_costmap/clear_entirely_local_costmap')

        self._fixed_frame = str(self.get_parameter('fixed_frame').value)
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._jump_m = float(self.get_parameter('jump_threshold_m').value)
        self._jump_deg = float(self.get_parameter('jump_threshold_deg').value)
        self._cooldown = float(self.get_parameter('cooldown_sec').value)
        self._clear_local = bool(self.get_parameter('clear_local').value)

        self._tf_buffer = tf2_ros.Buffer(node=self)
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._global_client = self.create_client(
            ClearEntireCostmap, str(self.get_parameter('global_costmap_service').value))
        self._local_client = (
            self.create_client(
                ClearEntireCostmap, str(self.get_parameter('local_costmap_service').value))
            if self._clear_local else None)

        self._last_xyz_yaw: tuple[float, float, float] | None = None
        self._last_clear_time = 0.0
        self._n_clears = 0

        period = max(0.2, float(self.get_parameter('check_period_sec').value))
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f'costmap_ghost_clear watching {self._fixed_frame}->{self._odom_frame} '
            f'jump_threshold={self._jump_m}m/{self._jump_deg}deg cooldown={self._cooldown}s '
            f'clear_local={self._clear_local}')

    def _lookup(self, target: str, source: str):
        try:
            return self._tf_buffer.lookup_transform(
                target, source, Time(), timeout=Duration(seconds=0.05))
        except Exception:
            return None

    def _call_clear(self, client, label: str) -> None:
        if not client.service_is_ready():
            self.get_logger().warn(f'{label} clear service not ready — skipping this trigger')
            return
        client.call_async(ClearEntireCostmap.Request())
        self.get_logger().info(f'{label} costmap cleared')

    def _tick(self) -> None:
        tf = self._lookup(self._fixed_frame, self._odom_frame)
        if tf is None:
            return
        tr = tf.transform.translation
        yaw = _yaw_deg(tf.transform.rotation)
        xyz_yaw = (tr.x, tr.y, yaw)

        if self._last_xyz_yaw is None:
            self._last_xyz_yaw = xyz_yaw
            return

        lx, ly, lyaw = self._last_xyz_yaw
        dist = math.hypot(tr.x - lx, tr.y - ly)
        dyaw = abs((yaw - lyaw + 180.0) % 360.0 - 180.0)
        self._last_xyz_yaw = xyz_yaw

        if dist < self._jump_m and dyaw < self._jump_deg:
            return

        now = time.monotonic()
        if now - self._last_clear_time < self._cooldown:
            return
        self._last_clear_time = now
        self._n_clears += 1

        self.get_logger().info(
            f'map->odom jumped {dist:.3f}m / {dyaw:.1f}deg (loop closure correction) — '
            f'clearing costmap(s) to drop stale marks (clear #{self._n_clears})')
        self._call_clear(self._global_client, 'global')
        if self._clear_local:
            self._call_clear(self._local_client, 'local')


def main(args=None):
    rclpy.init(args=args)
    node = CostmapGhostClear()
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
