#!/usr/bin/env python3
"""
frontier_coordinator.py — Centralized multi-robot frontier exploration.

Single node assigns each robot a unique frontier so no two robots ever target
the same unexplored area. When a robot finishes or fails, it is immediately
reassigned. Replaces the per-robot frontier_explorer in multi-robot mode.

Parameters
----------
robot_namespaces  Comma-separated robot names, e.g. "robot1,robot2,robot3"
min_frontier_size Minimum cluster size to consider a frontier (default 5)
revisit_radius    Radius (m) within which a frontier counts as visited (default 0.5)
assign_radius     Radius (m) within which a frontier counts as taken (default 1.0)
poll_period       Seconds between assignment cycles (default 2.0)
map_topic         OccupancyGrid topic (default /map)
min_goal_distance Ignore frontiers closer than this to the robot (default 0.35)
map_save_path     File prefix for final map save, e.g. /path/to/maps/map (default '')
"""

import math
import os
import subprocess
from collections import deque
from enum import Enum, auto

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import tf2_ros

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose


class _State(Enum):
    IDLE = auto()
    NAVIGATING = auto()


class FrontierCoordinator(Node):
    def __init__(self):
        super().__init__('frontier_coordinator')

        self.declare_parameter('robot_namespaces', 'robot1,robot2')
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('revisit_radius', 0.5)
        self.declare_parameter('assign_radius', 1.0)
        self.declare_parameter('poll_period', 2.0)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('min_goal_distance', 0.35)
        self.declare_parameter('map_save_path', '')

        raw = self.get_parameter('robot_namespaces').value
        self._robots = [ns.strip() for ns in raw.split(',') if ns.strip()]
        self._min_size    = self.get_parameter('min_frontier_size').value
        self._revisit_r   = self.get_parameter('revisit_radius').value
        self._assign_r    = self.get_parameter('assign_radius').value
        self._min_goal_d  = self.get_parameter('min_goal_distance').value
        self._save_path   = self.get_parameter('map_save_path').value.strip()
        map_topic         = self.get_parameter('map_topic').value
        poll_period       = self.get_parameter('poll_period').value

        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self)

        self._map: OccupancyGrid | None = None
        self._map_saved = False

        # Per-robot tracking
        self._state:    dict[str, _State]                    = {r: _State.IDLE for r in self._robots}
        self._assigned: dict[str, tuple[float, float] | None] = {r: None        for r in self._robots}
        self._visited:  list[tuple[float, float]]            = []

        # One Nav2 action client per robot
        self._nav_clients: dict[str, ActionClient] = {
            r: ActionClient(self, NavigateToPose, f'/{r}/navigate_to_pose')
            for r in self._robots
        }

        self.get_logger().info(f'Waiting for Nav2 servers: {self._robots}')
        for r, client in self._nav_clients.items():
            client.wait_for_server()
            self.get_logger().info(f'  /{r}/navigate_to_pose ready')

        self.create_subscription(OccupancyGrid, map_topic, self._map_cb, 1)
        self.create_timer(poll_period, self._cycle)
        self.get_logger().info('Frontier coordinator running.')

    # ------------------------------------------------------------------
    def _map_cb(self, msg: OccupancyGrid):
        self._map = msg

    # ------------------------------------------------------------------
    def _cycle(self):
        if self._map is None:
            return

        frontiers = self._find_frontiers()
        idle = [r for r in self._robots if self._state[r] == _State.IDLE]

        if not idle:
            return

        if not frontiers:
            if all(s == _State.IDLE for s in self._state.values()):
                self.get_logger().info('Exploration complete — no frontiers remain.')
                self._save_map()
            return

        for r in idle:
            pos = self._robot_pos(r)
            if pos is None:
                continue
            goal = self._pick(frontiers, pos, r)
            if goal is None:
                continue
            self._assigned[r] = goal
            self._state[r] = _State.NAVIGATING
            self.get_logger().info(f'[{r}] assigned ({goal[0]:.2f}, {goal[1]:.2f})')
            self._send_goal(r, goal[0], goal[1])

    # ------------------------------------------------------------------
    def _pick(self, frontiers, robot_pos, ns):
        """Nearest frontier not already taken by another robot or visited."""
        rx, ry = robot_pos
        taken = [g for r, g in self._assigned.items() if r != ns and g is not None]

        best, best_d = None, float('inf')
        for fx, fy in frontiers:
            if _near_any(fx, fy, self._visited, self._revisit_r):
                continue
            if _near_any(fx, fy, taken, self._assign_r):
                continue
            d = math.hypot(fx - rx, fy - ry)
            if d < self._min_goal_d:
                continue
            if d < best_d:
                best_d, best = d, (fx, fy)
        return best

    # ------------------------------------------------------------------
    def _robot_pos(self, ns):
        try:
            tf = self._tf.lookup_transform('map', f'{ns}/base_link', rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:
            return None

    # ------------------------------------------------------------------
    def _send_goal(self, ns, x, y):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        f = self._nav_clients[ns].send_goal_async(goal)
        f.add_done_callback(lambda fut, r=ns: self._on_accepted(fut, r))

    def _on_accepted(self, future, ns):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn(f'[{ns}] goal rejected — freeing.')
            self._assigned[ns] = None
            self._state[ns] = _State.IDLE
            return
        handle.get_result_async().add_done_callback(lambda fut, r=ns: self._on_result(fut, r))

    def _on_result(self, future, ns):
        status = future.result().status
        goal = self._assigned[ns]
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'[{ns}] reached ({goal[0]:.2f}, {goal[1]:.2f})')
            if goal:
                self._visited.append(goal)
        else:
            self.get_logger().warn(f'[{ns}] failed status={status} — will reassign.')
            # Do NOT add to visited: let another robot retry this frontier
        self._assigned[ns] = None
        self._state[ns] = _State.IDLE

    # ------------------------------------------------------------------
    def _find_frontiers(self) -> list[tuple[float, float]]:
        msg = self._map
        w, h  = msg.info.width, msg.info.height
        res   = msg.info.resolution
        ox    = msg.info.origin.position.x
        oy    = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        free    = data == 0
        unknown = data == -1

        adj = np.zeros_like(unknown, dtype=bool)
        adj[:-1, :] |= unknown[1:, :]
        adj[1:,  :] |= unknown[:-1, :]
        adj[:,  :-1] |= unknown[:, 1:]
        adj[:,   1:] |= unknown[:, :-1]
        mask = free & adj

        if not mask.any():
            return []

        seen  = np.zeros_like(mask, dtype=bool)
        result = []
        for sy, sx in np.argwhere(mask):
            sy, sx = int(sy), int(sx)
            if seen[sy, sx]:
                continue
            queue = deque([(sy, sx)])
            seen[sy, sx] = True
            cluster = []
            while queue:
                y, x = queue.popleft()
                cluster.append((y, x))
                for ny, nx in ((y-1,x),(y+1,x),(y,x-1),(y,x+1)):
                    if 0 <= ny < h and 0 <= nx < w and not seen[ny, nx] and mask[ny, nx]:
                        seen[ny, nx] = True
                        queue.append((ny, nx))
            if len(cluster) < self._min_size:
                continue
            cells = np.array(cluster, dtype=np.float32)
            cy, cx = cells.mean(axis=0)
            result.append((ox + (cx + 0.5) * res, oy + (cy + 0.5) * res))
        return result

    # ------------------------------------------------------------------
    def _save_map(self):
        if self._map_saved or not self._save_path:
            return
        save_dir = os.path.dirname(self._save_path)
        if save_dir:
            os.makedirs(save_dir, exist_ok=True)
        self.get_logger().info(f'Saving map → {self._save_path}')
        try:
            subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', self._save_path],
                check=True)
            self._map_saved = True
            self.get_logger().info('Map saved.')
        except Exception as e:
            self.get_logger().error(f'Map save failed: {e}')


# ──────────────────────────────────────────────────────────────────────────────
def _near_any(fx, fy, points, radius):
    return any(math.hypot(fx - px, fy - py) < radius for px, py in points)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyError, Exception):
            pass  # Humble rclpy KeyError on ActionClient cleanup
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
