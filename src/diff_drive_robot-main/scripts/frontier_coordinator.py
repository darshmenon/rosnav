#!/usr/bin/env python3
"""
frontier_coordinator.py — Centralized multi-robot frontier exploration.

Single node assigns each robot a unique frontier so no two robots ever target
the same unexplored area. When a robot finishes or fails, it is immediately
reassigned. Replaces the per-robot frontier_explorer in multi-robot mode.

Parameters
----------
robot_namespaces  Comma-separated robot names, e.g. "robot1,robot2,robot3"
frontier_detector Frontier detector plugin: classic or wfd (default wfd)
frontier_scorer   Goal scorer plugin: nearest or weighted (default weighted)
min_frontier_size Minimum cluster size to consider a frontier (default 5)
revisit_radius    Radius (m) within which a frontier counts as visited (default 0.5)
assign_radius     Radius (m) within which a frontier counts as taken (default 1.0)
poll_period       Seconds between assignment cycles (default 2.0)
map_topic         OccupancyGrid topic (default /map)
min_goal_distance Ignore frontiers closer than this to the robot (default 0.35)
frontier_clearance_radius Minimum map clearance around frontier goals (default 0.30)
failed_goal_radius Radius for matching failed frontier goals (default 0.75)
failed_goal_cooldown Seconds to avoid a failed frontier area (default 45)
distance_weight   Weighted scorer distance penalty (default 1.0)
info_gain_weight  Weighted scorer information gain reward (default 3.0)
hysteresis_radius Radius for robot-continuity scoring bonus (default 2.0)
hysteresis_gain   Weighted scorer current assignment bonus (default 1.5)
publish_markers   Publish RViz MarkerArray debug overlays (default true)
nav_wait_warn_sec Seconds between warnings for missing Nav2 action servers (default 15)
tf_wait_warn_sec  Seconds between warnings for missing robot map TF (default 15)
map_save_path     File prefix for final map save, e.g. /path/to/maps/map (default '')

Published topics
----------------
/exploration/stats    std_msgs/String JSON status snapshot
/exploration/markers  visualization_msgs/MarkerArray debug markers
"""

import json
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
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray


class _State(Enum):
    IDLE = auto()
    NAVIGATING = auto()


class FrontierCoordinator(Node):
    def __init__(self):
        super().__init__('frontier_coordinator')

        self.declare_parameter('robot_namespaces', 'robot1,robot2')
        self.declare_parameter('frontier_detector', 'wfd')
        self.declare_parameter('frontier_scorer', 'weighted')
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('revisit_radius', 0.5)
        self.declare_parameter('assign_radius', 1.0)
        self.declare_parameter('poll_period', 2.0)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('min_goal_distance', 0.35)
        self.declare_parameter('frontier_clearance_radius', 0.30)
        self.declare_parameter('failed_goal_radius', 0.75)
        self.declare_parameter('failed_goal_cooldown', 45.0)
        self.declare_parameter('distance_weight', 1.0)
        self.declare_parameter('info_gain_weight', 3.0)
        self.declare_parameter('hysteresis_radius', 2.0)
        self.declare_parameter('hysteresis_gain', 1.5)
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('nav_wait_warn_sec', 15.0)
        self.declare_parameter('tf_wait_warn_sec', 15.0)
        self.declare_parameter('map_save_path', '')

        raw = self.get_parameter('robot_namespaces').value
        self._robots = [ns.strip() for ns in raw.split(',') if ns.strip()]
        self._detector    = self.get_parameter('frontier_detector').value.strip().lower()
        self._scorer      = self.get_parameter('frontier_scorer').value.strip().lower()
        self._min_size    = self.get_parameter('min_frontier_size').value
        self._revisit_r   = self.get_parameter('revisit_radius').value
        self._assign_r    = self.get_parameter('assign_radius').value
        self._min_goal_d  = self.get_parameter('min_goal_distance').value
        self._frontier_clearance = self.get_parameter('frontier_clearance_radius').value
        self._failed_goal_r = self.get_parameter('failed_goal_radius').value
        self._failed_goal_cooldown = self.get_parameter('failed_goal_cooldown').value
        self._dist_w      = self.get_parameter('distance_weight').value
        self._info_w      = self.get_parameter('info_gain_weight').value
        self._hyst_r      = self.get_parameter('hysteresis_radius').value
        self._hyst_gain   = self.get_parameter('hysteresis_gain').value
        self._publish_markers_enabled = self.get_parameter('publish_markers').value
        self._nav_wait_warn_sec = self.get_parameter('nav_wait_warn_sec').value
        self._tf_wait_warn_sec = self.get_parameter('tf_wait_warn_sec').value
        self._save_path   = self.get_parameter('map_save_path').value.strip()
        map_topic         = self.get_parameter('map_topic').value
        poll_period       = self.get_parameter('poll_period').value

        if self._detector not in ('classic', 'wfd'):
            self.get_logger().warn(
                f'Unknown frontier_detector={self._detector!r}; using wfd.')
            self._detector = 'wfd'
        if self._scorer not in ('nearest', 'weighted'):
            self.get_logger().warn(
                f'Unknown frontier_scorer={self._scorer!r}; using weighted.')
            self._scorer = 'weighted'

        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self)

        self._map: OccupancyGrid | None = None
        self._map_saved = False
        self._start_time = self.get_clock().now()
        self._last_frontier_count = 0
        self._failed_goals = 0
        self._succeeded_goals = 0
        self._nav_ready: dict[str, bool] = {r: False for r in self._robots}
        self._last_nav_wait_warn: dict[str, float] = {r: 0.0 for r in self._robots}
        self._tf_ready: dict[str, bool] = {r: False for r in self._robots}
        self._last_tf_wait_warn: dict[str, float] = {r: 0.0 for r in self._robots}
        self._last_assignment_cycle_log = 0.0

        # Per-robot tracking
        self._state:    dict[str, _State]                    = {r: _State.IDLE for r in self._robots}
        self._assigned: dict[str, tuple[float, float] | None] = {r: None        for r in self._robots}
        self._visited:  list[tuple[float, float]]            = []
        self._failed_frontiers: list[dict]                    = []

        # One Nav2 action client per robot
        self._nav_clients: dict[str, ActionClient] = {
            r: ActionClient(self, NavigateToPose, f'/{r}/navigate_to_pose')
            for r in self._robots
        }
        self._stats_pub = self.create_publisher(String, '/exploration/stats', 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/exploration/frontiers', 10)

        self.get_logger().info(f'Watching Nav2 servers: {self._robots}')
        self.create_subscription(OccupancyGrid, map_topic, self._map_cb, 1)
        self.create_timer(poll_period, self._cycle)
        self.get_logger().info(
            f'Frontier coordinator running '
            f'(detector={self._detector}, scorer={self._scorer}, map_topic={map_topic}).')

    # ------------------------------------------------------------------
    def _map_cb(self, msg: OccupancyGrid):
        self._map = msg

    # ------------------------------------------------------------------
    def _cycle(self):
        if self._map is None:
            self.get_logger().debug('Waiting for occupancy grid map.')
            return

        elapsed = self._elapsed_sec()
        robot_positions = self._robot_positions(elapsed)
        self._refresh_nav_servers()
        if self._detector == 'wfd' and not robot_positions:
            self.get_logger().debug('Waiting for robot TF before WFD frontier search.')
            self._publish_stats()
            self._publish_markers([], robot_positions)
            return

        frontiers = self._find_frontiers(robot_positions.values())
        self._last_frontier_count = len(frontiers)

        ready_idle = [
            r for r in self._robots
            if self._state[r] == _State.IDLE and self._nav_ready.get(r, False)
        ]
        waiting_nav = [
            r for r in self._robots
            if self._state[r] == _State.IDLE and not self._nav_ready.get(r, False)
        ]
        waiting_tf = [
            r for r in self._robots
            if self._state[r] == _State.IDLE and r not in robot_positions
        ]
        if elapsed - self._last_assignment_cycle_log >= 10.0:
            self.get_logger().info(
                f'cycle: frontiers={len(frontiers)} ready_idle={ready_idle} '
                f'waiting_nav={waiting_nav} waiting_tf={waiting_tf}')
            self._last_assignment_cycle_log = elapsed

        idle = [
            r for r in self._robots
            if self._state[r] == _State.IDLE and self._nav_ready.get(r, False)
        ]
        if idle and frontiers:
            for r in idle:
                pos = robot_positions.get(r)
                if pos is None:
                    continue
                goal = self._pick(frontiers, pos, r)
                if goal is None:
                    continue
                self._assigned[r] = goal
                self._state[r] = _State.NAVIGATING
                self.get_logger().info(f'[{r}] assigned ({goal[0]:.2f}, {goal[1]:.2f})')
                self._send_goal(r, goal[0], goal[1])
        elif not frontiers and all(s == _State.IDLE for s in self._state.values()):
            self.get_logger().info('Exploration complete — no frontiers remain.')
            self._save_map()

        self._publish_markers(frontiers, robot_positions)
        self._publish_stats()

    # ------------------------------------------------------------------
    def _refresh_nav_servers(self):
        elapsed = self._elapsed_sec()
        for robot, client in self._nav_clients.items():
            ready = client.server_is_ready()
            if ready and not self._nav_ready[robot]:
                self.get_logger().info(f'/{robot}/navigate_to_pose ready')
            elif not ready and self._nav_ready[robot]:
                self.get_logger().warn(f'/{robot}/navigate_to_pose disappeared')
            elif not ready and elapsed - self._last_nav_wait_warn[robot] >= self._nav_wait_warn_sec:
                self.get_logger().warn(
                    f'Waiting for /{robot}/navigate_to_pose '
                    f'({elapsed:.0f}s since coordinator start)')
                self._last_nav_wait_warn[robot] = elapsed
            self._nav_ready[robot] = ready

    # ------------------------------------------------------------------
    def _robot_positions(self, elapsed):
        positions = {}
        for robot in self._robots:
            pos = self._robot_pos(robot)
            if pos is not None:
                positions[robot] = pos
                if not self._tf_ready[robot]:
                    self.get_logger().info(f'map -> {robot}/base_link TF ready')
                self._tf_ready[robot] = True
                continue

            if self._tf_ready[robot]:
                self.get_logger().warn(f'map -> {robot}/base_link TF disappeared')
            self._tf_ready[robot] = False
            if elapsed - self._last_tf_wait_warn[robot] >= self._tf_wait_warn_sec:
                self.get_logger().warn(
                    f'Waiting for map -> {robot}/base_link TF '
                    f'({elapsed:.0f}s since coordinator start)')
                self._last_tf_wait_warn[robot] = elapsed
        return positions

    def _elapsed_sec(self):
        return (self.get_clock().now() - self._start_time).nanoseconds / 1e9

    # ------------------------------------------------------------------
    def _pick(self, frontiers, robot_pos, ns):
        """Pick a frontier through the selected scorer plugin."""
        rx, ry = robot_pos
        taken = [g for r, g in self._assigned.items() if r != ns and g is not None]

        best, best_score = None, float('-inf')
        for frontier in frontiers:
            fx, fy = frontier['point']
            if _near_any(fx, fy, self._visited, self._revisit_r):
                continue
            if self._recently_failed(fx, fy):
                continue
            if _near_any(fx, fy, taken, self._assign_r):
                continue
            d = math.hypot(fx - rx, fy - ry)
            if d < self._min_goal_d:
                continue

            score = -d
            if self._scorer == 'weighted':
                score = self._score_weighted(frontier, d, ns)
            if score > best_score:
                best_score, best = score, (fx, fy)
        return best

    def _score_weighted(self, frontier, distance, ns):
        score = self._info_w * frontier['info_gain'] - self._dist_w * distance
        current = self._assigned.get(ns)
        if current is not None:
            fx, fy = frontier['point']
            if math.hypot(fx - current[0], fy - current[1]) <= self._hyst_r:
                score += self._hyst_gain
        return score

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
            self._failed_goals += 1
            self._assigned[ns] = None
            self._state[ns] = _State.IDLE
            self._publish_stats()
            return
        handle.get_result_async().add_done_callback(lambda fut, r=ns: self._on_result(fut, r))

    def _on_result(self, future, ns):
        status = future.result().status
        goal = self._assigned[ns]
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'[{ns}] reached ({goal[0]:.2f}, {goal[1]:.2f})')
            self._succeeded_goals += 1
            if goal:
                self._visited.append(goal)
        else:
            self.get_logger().warn(f'[{ns}] failed status={status} — will reassign.')
            self._failed_goals += 1
            if goal:
                self._remember_failed_frontier(goal)
        self._assigned[ns] = None
        self._state[ns] = _State.IDLE
        self._publish_stats()

    # ------------------------------------------------------------------
    def _find_frontiers(self, robot_positions) -> list[dict]:
        msg = self._map
        w, h  = msg.info.width, msg.info.height
        res   = msg.info.resolution
        ox    = msg.info.origin.position.x
        oy    = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        free    = data == 0
        unknown = data == -1
        occupied = data >= 50

        mask = self._frontier_mask(free, unknown)
        if self._detector == 'wfd':
            reachable = self._reachable_mask(free, robot_positions, msg.info)
            mask &= reachable

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
            goal_cell, clearance = self._best_goal_cell(cluster, occupied, res)
            if goal_cell is None:
                continue
            cy, cx = goal_cell
            result.append({
                'point': (ox + (cx + 0.5) * res, oy + (cy + 0.5) * res),
                'size': len(cluster),
                'info_gain': self._info_gain(cluster, unknown, res),
                'clearance': clearance,
            })
        return result

    def _best_goal_cell(self, cluster, occupied, resolution):
        radius_cells = max(1, math.ceil(self._frontier_clearance / resolution))
        best = None
        best_clearance = -1.0
        for y, x in cluster:
            clearance = self._cell_clearance(y, x, occupied, radius_cells, resolution)
            if clearance >= self._frontier_clearance and clearance > best_clearance:
                best = (float(y), float(x))
                best_clearance = clearance
        return best, best_clearance

    @staticmethod
    def _cell_clearance(y, x, occupied, radius_cells, resolution):
        h, w = occupied.shape
        y0, y1 = max(0, y - radius_cells), min(h, y + radius_cells + 1)
        x0, x1 = max(0, x - radius_cells), min(w, x + radius_cells + 1)
        occ = np.argwhere(occupied[y0:y1, x0:x1])
        if occ.size == 0:
            return (radius_cells + 1) * resolution
        dy = occ[:, 0] + y0 - y
        dx = occ[:, 1] + x0 - x
        return math.sqrt(float(np.min(dx * dx + dy * dy))) * resolution

    @staticmethod
    def _frontier_mask(free, unknown):
        adj = np.zeros_like(unknown, dtype=bool)
        adj[:-1, :] |= unknown[1:, :]
        adj[1:,  :] |= unknown[:-1, :]
        adj[:,  :-1] |= unknown[:, 1:]
        adj[:,   1:] |= unknown[:, :-1]
        return free & adj

    def _reachable_mask(self, free, robot_positions, info):
        h, w = free.shape
        seen = np.zeros_like(free, dtype=bool)
        queue = deque()
        for wx, wy in robot_positions:
            y, x = _world_to_cell(wx, wy, info)
            if 0 <= y < h and 0 <= x < w and free[y, x] and not seen[y, x]:
                seen[y, x] = True
                queue.append((y, x))

        while queue:
            y, x = queue.popleft()
            for ny, nx in ((y-1,x),(y+1,x),(y,x-1),(y,x+1)):
                if 0 <= ny < h and 0 <= nx < w and free[ny, nx] and not seen[ny, nx]:
                    seen[ny, nx] = True
                    queue.append((ny, nx))
        return seen

    @staticmethod
    def _info_gain(cluster, unknown, resolution):
        h, w = unknown.shape
        cells = set()
        for y, x in cluster:
            for ny in range(y - 2, y + 3):
                for nx in range(x - 2, x + 3):
                    if 0 <= ny < h and 0 <= nx < w and unknown[ny, nx]:
                        cells.add((ny, nx))
        return len(cells) * resolution * resolution

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

    # ------------------------------------------------------------------
    def _remember_failed_frontier(self, goal):
        now = self._elapsed_sec()
        gx, gy = goal
        self._failed_frontiers = [
            f for f in self._failed_frontiers
            if now - f['time'] <= self._failed_goal_cooldown
        ]
        self._failed_frontiers.append({'point': goal, 'time': now})
        self.get_logger().warn(
            f'Blacklisting failed frontier near ({gx:.2f}, {gy:.2f}) '
            f'for {self._failed_goal_cooldown:.0f}s')

    def _recently_failed(self, fx, fy):
        now = self._elapsed_sec()
        kept = []
        blocked = False
        for item in self._failed_frontiers:
            if now - item['time'] > self._failed_goal_cooldown:
                continue
            kept.append(item)
            gx, gy = item['point']
            if math.hypot(fx - gx, fy - gy) <= self._failed_goal_r:
                blocked = True
        self._failed_frontiers = kept
        return blocked

    # ------------------------------------------------------------------
    def _publish_stats(self):
        active = [r for r, s in self._state.items() if s == _State.NAVIGATING]
        idle = [r for r, s in self._state.items() if s == _State.IDLE]
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        explored_pct, explored_area, map_area = self._map_progress()
        msg = String()
        msg.data = json.dumps({
            'time_elapsed_s': round(elapsed, 1),
            'robots': self._robots,
            'active_robots': len(active),
            'active': active,
            'idle_robots': len(idle),
            'idle': idle,
            'nav_ready': [r for r, ready in self._nav_ready.items() if ready],
            'nav_waiting': [r for r, ready in self._nav_ready.items() if not ready],
            'tf_ready': [r for r, ready in self._tf_ready.items() if ready],
            'tf_waiting': [r for r, ready in self._tf_ready.items() if not ready],
            'frontier_count': self._last_frontier_count,
            'visited_count': len(self._visited),
            'succeeded_goals': self._succeeded_goals,
            'failed_goals': self._failed_goals,
            'failed_frontier_blacklist': len(self._failed_frontiers),
            'explored_pct': explored_pct,
            'explored_area_m2': explored_area,
            'map_area_m2': map_area,
            'assignments': {
                r: [round(g[0], 3), round(g[1], 3)]
                for r, g in self._assigned.items() if g is not None
            },
            'detector': self._detector,
            'scorer': self._scorer,
        }, sort_keys=True)
        self._stats_pub.publish(msg)

    def _map_progress(self):
        if self._map is None:
            return 0.0, 0.0, 0.0
        data = np.array(self._map.data, dtype=np.int8)
        total = data.size
        if total == 0:
            return 0.0, 0.0, 0.0
        known = int(np.count_nonzero(data != -1))
        cell_area = self._map.info.resolution * self._map.info.resolution
        return (
            round(100.0 * known / total, 2),
            round(known * cell_area, 2),
            round(total * cell_area, 2),
        )

    def _publish_markers(self, frontiers, robot_positions):
        if not self._publish_markers_enabled:
            return

        markers = MarkerArray()
        markers.markers.append(_delete_all_marker())
        now = self.get_clock().now().to_msg()

        for i, frontier in enumerate(frontiers):
            fx, fy = frontier['point']
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = now
            marker.ns = 'frontiers'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = fx
            marker.pose.position.y = fy
            marker.pose.position.z = 0.05
            marker.pose.orientation.w = 1.0
            scale = max(0.12, min(0.45, 0.08 + math.sqrt(frontier['size']) * 0.015))
            marker.scale.x = scale
            marker.scale.y = scale
            marker.scale.z = 0.08
            marker.color = ColorRGBA(r=0.1, g=0.55, b=1.0, a=0.65)
            markers.markers.append(marker)

        for i, (vx, vy) in enumerate(self._visited):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = now
            marker.ns = 'visited'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = vx
            marker.pose.position.y = vy
            marker.pose.position.z = 0.04
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.18
            marker.color = ColorRGBA(r=0.55, g=0.55, b=0.55, a=0.55)
            markers.markers.append(marker)

        for i, (robot, goal) in enumerate(self._assigned.items()):
            if goal is None:
                continue
            gx, gy = goal
            color = _robot_color(robot)

            goal_m = Marker()
            goal_m.header.frame_id = 'map'
            goal_m.header.stamp = now
            goal_m.ns = 'goals'
            goal_m.id = i
            goal_m.type = Marker.CYLINDER
            goal_m.action = Marker.ADD
            goal_m.pose.position.x = gx
            goal_m.pose.position.y = gy
            goal_m.pose.position.z = 0.12
            goal_m.pose.orientation.w = 1.0
            goal_m.scale.x = 0.45
            goal_m.scale.y = 0.45
            goal_m.scale.z = 0.16
            goal_m.color = color
            markers.markers.append(goal_m)

            pos = robot_positions.get(robot)
            if pos is None:
                continue
            line = Marker()
            line.header.frame_id = 'map'
            line.header.stamp = now
            line.ns = 'assignments'
            line.id = i
            line.type = Marker.LINE_LIST
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.06
            line.color = color
            line.points = [
                Point(x=pos[0], y=pos[1], z=0.15),
                Point(x=gx, y=gy, z=0.15),
            ]
            markers.markers.append(line)

        self._marker_pub.publish(markers)


# ──────────────────────────────────────────────────────────────────────────────
def _near_any(fx, fy, points, radius):
    return any(math.hypot(fx - px, fy - py) < radius for px, py in points)


def _world_to_cell(wx, wy, info):
    return (math.floor((wy - info.origin.position.y) / info.resolution),
            math.floor((wx - info.origin.position.x) / info.resolution))


def _delete_all_marker():
    marker = Marker()
    marker.action = Marker.DELETEALL
    return marker


def _robot_color(robot):
    palette = [
        ColorRGBA(r=0.95, g=0.2, b=0.15, a=0.9),
        ColorRGBA(r=0.1, g=0.75, b=0.3, a=0.9),
        ColorRGBA(r=0.95, g=0.65, b=0.1, a=0.9),
        ColorRGBA(r=0.55, g=0.25, b=0.95, a=0.9),
    ]
    idx = sum(ord(c) for c in robot) % len(palette)
    return palette[idx]



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
