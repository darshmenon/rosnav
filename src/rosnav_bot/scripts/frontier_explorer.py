#!/usr/bin/env python3
# frontier_explorer.py — WFD + weighted scoring + explore BT
"""
Frontier-based autonomous exploration.

Uses the same detector/scorer plugins as frontier_coordinator:
  frontier_detector: classic | wfd              (default wfd)
  frontier_scorer:   nearest | weighted | utility (default utility)

utility scorer mirrors explore_lite:
  score = gain_scale * size_m - potential_scale * distance
  (equivalent to minimizing explore_lite cost).

Goals are validated against Nav2 global_costmap when available, and sent
with config/bt/explore_nav.xml unless behavior_tree is overridden.

All tuning values are ROS 2 parameters — override at launch:
  ros2 run rosnav_bot frontier_explorer.py --ros-args \
      -p frontier_detector:=wfd -p frontier_scorer:=utility \
      -p potential_scale:=3.0 -p gain_scale:=1.0

Run alongside slam.launch.py (mapping mode):
  ros2 launch rosnav_bot slam.launch.py
  ros2 run rosnav_bot frontier_explorer.py
"""

import math
import os
import subprocess
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
import tf2_ros

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose

_MAP_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _pkg_share() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('rosnav_bot')
    except Exception:
        return os.path.join(os.path.expanduser('~'), 'rosnav', 'src', 'rosnav_bot')


def _resolve_bt(name_or_path: str) -> str:
    """Resolve a BT stem (e.g. 'explore_nav') or absolute path to a file."""
    raw = (name_or_path or '').strip()
    if not raw:
        return ''
    if os.path.isfile(raw):
        return raw
    stem = raw[:-4] if raw.endswith('.xml') else raw
    share = _pkg_share()
    candidates = [
        os.path.join(share, 'config', 'bt', f'{stem}.xml'),
        os.path.join(
            os.path.expanduser('~'), 'rosnav', 'src',
            'rosnav_bot', 'config', 'bt', f'{stem}.xml'),
    ]
    for path in candidates:
        if os.path.isfile(path):
            return path
    return candidates[0]


def _world_to_cell(wx, wy, info):
    return (
        math.floor((wy - info.origin.position.y) / info.resolution),
        math.floor((wx - info.origin.position.x) / info.resolution),
    )


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('revisit_radius', 0.3)
        self.declare_parameter('poll_period', 1.5)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('action_name', 'navigate_to_pose')
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('min_goal_distance', 0.50)
        self.declare_parameter('map_save_path', '')
        self.declare_parameter('max_frontier_retries', 2)
        self.declare_parameter('goal_timeout', 60.0)
        self.declare_parameter('frontier_detector', 'wfd')
        self.declare_parameter('frontier_scorer', 'utility')
        self.declare_parameter('frontier_clearance_radius', 0.55)
        self.declare_parameter('distance_weight', 1.0)
        self.declare_parameter('info_gain_weight', 3.0)
        self.declare_parameter('potential_scale', 3.0)
        self.declare_parameter('gain_scale', 1.0)
        self.declare_parameter('hysteresis_radius', 2.0)
        self.declare_parameter('hysteresis_gain', 1.5)
        self.declare_parameter('behavior_tree', 'explore_nav')
        self.declare_parameter('costmap_topic', 'global_costmap/costmap')
        self.declare_parameter('validate_on_costmap', True)
        # Nav2 OccupancyGrid: 0=free, 1-98=inflation, 99=inscribed, 100=lethal.
        # Goals must be nearly free or Nav2 refuses / stalls in the inflation layer.
        self.declare_parameter('costmap_max_cost', 1)
        self.declare_parameter('costmap_abort_cost', 99)
        self.declare_parameter('allow_unknown_costmap', True)
        # Stand this far back from unknown cells into free space (m).
        # Must be >= global inflation_radius (0.5) or goals sit in inflated cells.
        self.declare_parameter('goal_pullback', 0.55)
        self.declare_parameter('goal_search_radius', 1.25)

        self._min_size = self.get_parameter('min_frontier_size').value
        self._revisit_r = self.get_parameter('revisit_radius').value
        self._goal_frame = self.get_parameter('goal_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._min_goal_dist = self.get_parameter('min_goal_distance').value
        self._map_save_path = self.get_parameter('map_save_path').value.strip()
        self._max_retries = self.get_parameter('max_frontier_retries').value
        self._goal_timeout = self.get_parameter('goal_timeout').value
        self._map_topic = self.get_parameter('map_topic').value
        self._detector = self.get_parameter('frontier_detector').value.strip().lower()
        self._scorer = self.get_parameter('frontier_scorer').value.strip().lower()
        self._frontier_clearance = self.get_parameter('frontier_clearance_radius').value
        self._dist_w = self.get_parameter('distance_weight').value
        self._info_w = self.get_parameter('info_gain_weight').value
        self._potential_scale = self.get_parameter('potential_scale').value
        self._gain_scale = self.get_parameter('gain_scale').value
        self._hyst_r = self.get_parameter('hysteresis_radius').value
        self._hyst_gain = self.get_parameter('hysteresis_gain').value
        self._bt_xml = _resolve_bt(self.get_parameter('behavior_tree').value)
        self._costmap_topic = self.get_parameter('costmap_topic').value.strip()
        self._validate_costmap = self.get_parameter('validate_on_costmap').value
        self._costmap_max_cost = int(self.get_parameter('costmap_max_cost').value)
        self._costmap_abort_cost = int(self.get_parameter('costmap_abort_cost').value)
        self._allow_unknown_costmap = self.get_parameter('allow_unknown_costmap').value
        self._goal_pullback = float(self.get_parameter('goal_pullback').value)
        self._goal_search_radius = float(self.get_parameter('goal_search_radius').value)
        map_topic = self._map_topic
        action_name = self.get_parameter('action_name').value
        poll_period = self.get_parameter('poll_period').value

        if self._detector not in ('classic', 'wfd'):
            self.get_logger().warn(
                f'Unknown frontier_detector={self._detector!r}; using wfd.')
            self._detector = 'wfd'
        if self._scorer not in ('nearest', 'weighted', 'utility'):
            self.get_logger().warn(
                f'Unknown frontier_scorer={self._scorer!r}; using utility.')
            self._scorer = 'utility'
        if self._bt_xml and not os.path.isfile(self._bt_xml):
            self.get_logger().warn(
                f'behavior_tree not found at {self._bt_xml!r}; '
                'Nav2 default BT will be used.')
            self._bt_xml = ''

        # ------------------------------------------------------------------
        # TF buffer for robot pose lookup
        # ------------------------------------------------------------------
        self._tf_buffer = tf2_ros.Buffer(node=self)
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self._map: OccupancyGrid | None = None
        self._costmap: OccupancyGrid | None = None
        self._navigating = False
        self._visited: list[tuple[float, float]] = []
        self._fail_counts: list[list] = []  # [x, y, count]
        self._current_goal: tuple[float, float] | None = None
        self._current_goal_handle = None
        self._ignore_next_result = False
        self._goal_sent_time: float = 0.0
        self._iteration = 0
        self._map_saved = False
        self._logged_init_wait = False
        self._logged_costmap = False
        self._last_status_log = 0.0

        # ------------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------------
        self._nav_client = ActionClient(self, NavigateToPose, action_name)
        self._map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self._map_callback, _MAP_QOS)
        if self._validate_costmap and self._costmap_topic:
            self.create_subscription(
                OccupancyGrid, self._costmap_topic,
                self._costmap_callback, _MAP_QOS)

        self.get_logger().info('Waiting for Nav2 action server...')
        self._nav_client.wait_for_server()
        bt_note = self._bt_xml or '(nav2 default)'
        cm_note = self._costmap_topic if self._validate_costmap else 'off'
        self.get_logger().info(
            f'Ready | detector={self._detector} scorer={self._scorer} '
            f'bt={bt_note} costmap={cm_note} '
            f'potential={self._potential_scale} gain={self._gain_scale} '
            f'costmap_max={self._costmap_max_cost} pullback={self._goal_pullback:.2f}m '
            f'clearance={self._frontier_clearance:.2f}m '
            f'min_size={self._min_size} revisit_r={self._revisit_r:.2f}m')
        self.get_logger().info('Waiting for map...')

        self.create_timer(poll_period, self._explore)

    # ------------------------------------------------------------------
    # Map / costmap callbacks
    # ------------------------------------------------------------------
    def _map_callback(self, msg: OccupancyGrid):
        self._map = msg

    def _costmap_callback(self, msg: OccupancyGrid):
        first = self._costmap is None
        self._costmap = msg
        if first and not self._logged_costmap:
            self._logged_costmap = True
            self.get_logger().info(
                f'Costmap ready on {self._costmap_topic} '
                f'({msg.info.width}x{msg.info.height} @ '
                f'{msg.info.resolution:.3f}m)')

    # ------------------------------------------------------------------
    # Main exploration loop
    # ------------------------------------------------------------------
    def _explore(self):
        if self._navigating:
            elapsed = time.monotonic() - self._goal_sent_time
            timed_out = elapsed > self._goal_timeout
            invalidated = (
                not timed_out and self._current_goal is not None
                and not self._costmap_allows(
                    *self._current_goal, max_cost=self._costmap_abort_cost)
            )
            if timed_out or invalidated:
                reason = (f'exceeded {self._goal_timeout:.0f}s timeout' if timed_out
                          else 'costmap now lethal/inscribed at goal')
                self.get_logger().warn(
                    f'Goal to ({self._current_goal[0]:.2f}, {self._current_goal[1]:.2f}) '
                    f'{reason} — cancelling and blacklisting as stuck.')
                if self._current_goal_handle is not None:
                    self._current_goal_handle.cancel_goal_async()
                    self._ignore_next_result = True
                if self._current_goal is not None:
                    self._register_failure(*self._current_goal)
                self._navigating = False
                self._current_goal_handle = None
            return

        if self._map is None:
            now = time.monotonic()
            if now - self._last_status_log >= 10.0:
                self.get_logger().info('Waiting for occupancy map...')
                self._last_status_log = now
            return

        pos = self._robot_position()
        frontiers = self._find_frontiers(pos)
        if not frontiers:
            if self._iteration == 0:
                now = time.monotonic()
                if not self._logged_init_wait:
                    self.get_logger().info(
                        'No frontiers yet — map still initializing, waiting before '
                        'treating this as exploration-complete.')
                    self._logged_init_wait = True
                    self._last_status_log = now
                elif now - self._last_status_log >= 10.0:
                    reason = 'TF (map->base_link) unavailable' if pos is None else \
                        f'0 candidate cells (detector={self._detector}, robot at ' \
                        f'({pos[0]:.2f}, {pos[1]:.2f}))'
                    self.get_logger().warn(
                        f'Still no frontiers after {self._iteration} goals sent — {reason}. '
                        f'Still waiting.')
                    self._last_status_log = now
                return
            self.get_logger().info('No frontiers — exploration complete.')
            self._save_map_once()
            return

        goal, pick = self._best_frontier(frontiers, pos)
        if goal is None:
            if pos is None:
                self.get_logger().debug('TF not ready yet, retrying...')
                return
            self.get_logger().info(
                f'No usable frontier '
                f'(candidates={pick["n_total"]} visited={pick["n_visited"]} '
                f'costmap_block={pick["n_costmap"]} too_near={pick["n_near"]} '
                f'scorer={self._scorer}) — exploration complete.')
            self._finish_exploration()
            return

        self._iteration += 1
        if self._map_save_path and self._iteration % 10 == 0:
            self.get_logger().info(
                f'Progressively auto-saving map to {self._map_save_path} '
                f'(topic={self._map_topic}) ...')
            subprocess.Popen(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-t', self._map_topic, '-f', self._map_save_path],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )

        self.get_logger().info(
            f'#{self._iteration} → ({goal[0]:.2f}, {goal[1]:.2f}) '
            f'| scorer={self._scorer} score={pick["score"]:.2f} '
            f'dist={pick["distance"]:.2f}m size={pick["size_m"]:.2f}m '
            f'info={pick["info_gain"]:.2f}m² clear={pick["clearance"]:.2f}m '
            f'cost={self._costmap_value(*goal)} '
            f'| pool={pick["n_ok"]}/{pick["n_total"]} '
            f'(skip visited={pick["n_visited"]} costmap={pick["n_costmap"]} '
            f'near={pick["n_near"]})')
        self._current_goal = goal
        self._send_goal(*goal)

    # ------------------------------------------------------------------
    # Map saving and shutdown
    # ------------------------------------------------------------------
    def _finish_exploration(self):
        if self._map_save_path:
            self.get_logger().info(f'Final map save to {self._map_save_path} ...')
            try:
                subprocess.run(
                    ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                     '-t', self._map_topic, '-f', self._map_save_path],
                    check=True
                )
                self.get_logger().info('Map saved successfully.')
            except subprocess.CalledProcessError as e:
                self.get_logger().error(f'Failed to save map: {e}')
        else:
            self.get_logger().info('No map_save_path provided. Skipping auto-save.')

        self.get_logger().info('Shutting down explorer.')
        raise SystemExit(0)

    # ------------------------------------------------------------------
    # Frontier detection (classic or WFD)
    # ------------------------------------------------------------------
    def _find_frontiers(self, robot_pos) -> list[dict]:
        msg = self._map
        width, height = msg.info.width, msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        free = data == 0
        unknown = data == -1
        occupied = data >= 50

        mask = self._frontier_mask(free, unknown)
        raw_count = int(mask.sum())
        if self._detector == 'wfd':
            if robot_pos is None:
                return []
            reachable = self._reachable_mask(free, [robot_pos], msg.info)
            mask &= reachable
            if raw_count > 0 and not mask.any():
                now = time.monotonic()
                if now - getattr(self, '_last_reach_warn', 0.0) >= 10.0:
                    self.get_logger().warn(
                        f'{raw_count} raw frontier cell(s) exist but 0 are reachable '
                        f'from robot pos {robot_pos} — robot\'s own map cell is likely '
                        f'not marked free (free_cells={int(reachable.sum())}), so the '
                        f'flood-fill has nothing to expand from.')
                    self._last_reach_warn = now

        if not mask.any():
            return []

        centroids = []
        visited = np.zeros_like(mask, dtype=bool)
        for sy, sx in np.argwhere(mask):
            sy, sx = int(sy), int(sx)
            if visited[sy, sx]:
                continue

            queue = deque([(sy, sx)])
            visited[sy, sx] = True
            cluster = []

            while queue:
                y, x = queue.popleft()
                cluster.append((y, x))
                for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                    if ny < 0 or ny >= height or nx < 0 or nx >= width:
                        continue
                    if visited[ny, nx] or not mask[ny, nx]:
                        continue
                    visited[ny, nx] = True
                    queue.append((ny, nx))

            if len(cluster) < self._min_size:
                continue

            goal_cell, clearance = self._place_safe_goal(
                cluster, free, unknown, occupied, res)
            if goal_cell is None:
                continue
            cy, cx = goal_cell
            centroids.append({
                'point': (ox + (cx + 0.5) * res, oy + (cy + 0.5) * res),
                'size': len(cluster),
                'size_m': len(cluster) * res,
                'info_gain': self._info_gain(cluster, unknown, res),
                'clearance': clearance,
            })
        return centroids

    @staticmethod
    def _frontier_mask(free, unknown):
        adj = np.zeros_like(unknown, dtype=bool)
        adj[:-1, :] |= unknown[1:, :]
        adj[1:, :] |= unknown[:-1, :]
        adj[:, :-1] |= unknown[:, 1:]
        adj[:, 1:] |= unknown[:, :-1]
        return free & adj

    def _reachable_mask(self, free, robot_positions, info):
        h, w = free.shape
        seen = np.zeros_like(free, dtype=bool)
        queue = deque()
        for wx, wy in robot_positions:
            y, x = _world_to_cell(wx, wy, info)
            seed = (y, x) if (0 <= y < h and 0 <= x < w and free[y, x]) else \
                self._nearest_free_cell(y, x, free, info.resolution)
            if seed is not None and not seen[seed]:
                seen[seed] = True
                queue.append(seed)

        while queue:
            y, x = queue.popleft()
            for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                if 0 <= ny < h and 0 <= nx < w and free[ny, nx] and not seen[ny, nx]:
                    seen[ny, nx] = True
                    queue.append((ny, nx))
        return seen

    @staticmethod
    def _nearest_free_cell(cy, cx, free, resolution, max_radius_m=1.0):
        """Cold-start fallback: the robot's own cell can still read as
        unknown (not yet observed with confidence, e.g. the lidar's
        min-range blind spot) even though the robot is physically standing
        on free ground. Search outward in expanding rings for the nearest
        cell the map *does* consider free, so WFD has somewhere to seed
        from instead of deadlocking forever on a fresh map."""
        h, w = free.shape
        max_radius_cells = max(1, math.ceil(max_radius_m / resolution))
        for r in range(0, max_radius_cells + 1):
            y0, y1 = max(0, cy - r), min(h, cy + r + 1)
            x0, x1 = max(0, cx - r), min(w, cx + r + 1)
            window = free[y0:y1, x0:x1]
            if not window.any():
                continue
            ys, xs = np.nonzero(window)
            ys, xs = ys + y0, xs + x0
            dists = (ys - cy) ** 2 + (xs - cx) ** 2
            i = np.argmin(dists)
            return (int(ys[i]), int(xs[i]))
        return None

    def _place_safe_goal(self, cluster, free, unknown, occupied, resolution):
        """Place a Nav2-reachable goal *inside* free space, not on the frontier.

        Frontier cells sit on free∩unknown and almost always fall inside the
        global costmap inflation layer (robot_radius + inflation_radius), so
        NavigateToPose stalls. We BFS into free cells, require pullback from
        unknown + clearance from occupied, and prefer cells the costmap marks
        as free.
        """
        h, w = free.shape
        pull_cells = max(1, math.ceil(self._goal_pullback / resolution))
        clear_cells = max(1, math.ceil(self._frontier_clearance / resolution))
        search_cells = max(pull_cells, math.ceil(self._goal_search_radius / resolution))

        # Seed BFS from frontier cluster into surrounding free space, scoring
        # every free cell that clears the *occupied*-clearance requirement.
        # Cold-start note: right after spawn the known-free region can be too
        # small for any cell to clear the full goal_pullback from unknown
        # space (chicken-and-egg — the region only grows once the robot
        # moves, which needs a goal first). Rather than hard-rejecting below
        # goal_pullback, track the best candidate at each cell's *actual*
        # unk_clear and relax the threshold afterward if nothing meets it.
        seen = np.zeros_like(free, dtype=bool)
        queue = deque()
        for y, x in cluster:
            y, x = int(y), int(x)
            if not seen[y, x]:
                seen[y, x] = True
                queue.append((y, x, 0))

        candidates = []  # (unk_clear, occ_clear, dist, y, x)
        while queue:
            y, x, dist = queue.popleft()
            if dist > search_cells:
                continue
            for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                if 0 <= ny < h and 0 <= nx < w and free[ny, nx] and not seen[ny, nx]:
                    seen[ny, nx] = True
                    queue.append((ny, nx, dist + 1))

            occ_clear = self._cell_clearance(y, x, occupied, clear_cells, resolution)
            if occ_clear < self._frontier_clearance:
                continue
            wx = self._map.info.origin.position.x + (x + 0.5) * resolution
            wy = self._map.info.origin.position.y + (y + 0.5) * resolution
            if not self._costmap_allows(wx, wy):
                continue
            unk_clear = self._cell_clearance(y, x, unknown, pull_cells + 1, resolution)
            candidates.append((unk_clear, occ_clear, dist, y, x))

        if not candidates:
            return None, -1.0

        qualifying = [c for c in candidates if c[0] >= self._goal_pullback]
        pool = qualifying if qualifying else candidates
        if not qualifying:
            best_available = max(c[0] for c in candidates)
            now = time.monotonic()
            if now - getattr(self, '_last_pullback_relax_warn', 0.0) >= 10.0:
                self.get_logger().warn(
                    f'No cell clears the full goal_pullback={self._goal_pullback:.2f}m '
                    f'from unknown space (best available={best_available:.2f}m) — known-free '
                    f'region is likely still too small (cold start). Relaxing pullback for '
                    f'this pick instead of rejecting the frontier outright.')
                self._last_pullback_relax_warn = now

        best = None
        best_score = float('-inf')
        for unk_clear, occ_clear, dist, y, x in pool:
            score = unk_clear + occ_clear - 0.15 * dist * resolution
            if score > best_score:
                best_score = score
                best = (float(y), float(x), occ_clear)

        return (best[0], best[1]), best[2]

    @staticmethod
    def _cell_clearance(y, x, mask, radius_cells, resolution):
        """Min distance (m) from (y,x) to any True cell in mask."""
        h, w = mask.shape
        y0, y1 = max(0, y - radius_cells), min(h, y + radius_cells + 1)
        x0, x1 = max(0, x - radius_cells), min(w, x + radius_cells + 1)
        hits = np.argwhere(mask[y0:y1, x0:x1])
        if hits.size == 0:
            return (radius_cells + 1) * resolution
        dy = hits[:, 0] + y0 - y
        dx = hits[:, 1] + x0 - x
        return math.sqrt(float(np.min(dx * dx + dy * dy))) * resolution

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
    # Pick frontier (nearest | weighted | utility)
    # ------------------------------------------------------------------
    def _best_frontier(self, frontiers, pos):
        empty = {
            'n_total': len(frontiers), 'n_ok': 0, 'n_visited': 0,
            'n_costmap': 0, 'n_near': 0, 'score': float('-inf'),
            'distance': 0.0, 'size_m': 0.0, 'info_gain': 0.0, 'clearance': 0.0,
        }
        if pos is None:
            return None, empty
        rx, ry = pos
        best, best_score, best_meta = None, float('-inf'), empty
        n_visited = n_costmap = n_near = n_ok = 0
        for frontier in frontiers:
            fx, fy = frontier['point']
            if self._already_visited(fx, fy):
                n_visited += 1
                continue
            if not self._costmap_allows(fx, fy):
                n_costmap += 1
                continue
            d = math.hypot(fx - rx, fy - ry)
            if d < self._min_goal_dist:
                n_near += 1
                continue
            n_ok += 1
            score = self._score_frontier(frontier, d)
            if score > best_score:
                best_score = score
                best = (fx, fy)
                best_meta = {
                    'n_total': len(frontiers),
                    'n_ok': n_ok,
                    'n_visited': n_visited,
                    'n_costmap': n_costmap,
                    'n_near': n_near,
                    'score': score,
                    'distance': d,
                    'size_m': frontier['size_m'],
                    'info_gain': frontier['info_gain'],
                    'clearance': frontier.get('clearance', 0.0),
                }
        # Refresh skip counts on the chosen meta (counts finish after loop)
        if best is not None:
            best_meta = {
                **best_meta,
                'n_ok': n_ok,
                'n_visited': n_visited,
                'n_costmap': n_costmap,
                'n_near': n_near,
            }
        else:
            best_meta = {
                'n_total': len(frontiers),
                'n_ok': n_ok,
                'n_visited': n_visited,
                'n_costmap': n_costmap,
                'n_near': n_near,
                'score': float('-inf'),
                'distance': 0.0,
                'size_m': 0.0,
                'info_gain': 0.0,
                'clearance': 0.0,
            }
        return best, best_meta

    def _score_frontier(self, frontier, distance):
        if self._scorer == 'utility':
            # explore_lite: minimize potential*dist - gain*size  →  maximize this
            score = (
                self._gain_scale * frontier['size_m']
                - self._potential_scale * distance)
        elif self._scorer == 'weighted':
            score = self._info_w * frontier['info_gain'] - self._dist_w * distance
        else:
            score = -distance
        if self._current_goal is not None and self._scorer != 'nearest':
            fx, fy = frontier['point']
            cx, cy = self._current_goal
            if math.hypot(fx - cx, fy - cy) <= self._hyst_r:
                score += self._hyst_gain
        return score

    def _costmap_allows(self, x, y, costmap: OccupancyGrid | None = None,
                        max_cost: int | None = None) -> bool:
        """Reject goals that land in inflated/lethal Nav2 costmap cells."""
        if not self._validate_costmap:
            return True
        msg = self._costmap if costmap is None else costmap
        if msg is None:
            return True
        limit = self._costmap_max_cost if max_cost is None else max_cost
        cy, cx = _world_to_cell(x, y, msg.info)
        h, w = msg.info.height, msg.info.width
        if cy < 0 or cy >= h or cx < 0 or cx >= w:
            return self._allow_unknown_costmap
        val = int(msg.data[cy * w + cx])
        if val < 0:
            return self._allow_unknown_costmap
        return val < limit

    def _costmap_value(self, x, y) -> int | None:
        if self._costmap is None:
            return None
        cy, cx = _world_to_cell(x, y, self._costmap.info)
        h, w = self._costmap.info.height, self._costmap.info.width
        if cy < 0 or cy >= h or cx < 0 or cx >= w:
            return None
        return int(self._costmap.data[cy * w + cx])

    def _already_visited(self, fx, fy):
        return any(
            math.hypot(fx - vx, fy - vy) < self._revisit_r
            for vx, vy in self._visited)

    def _robot_position(self):
        """Return (x, y) of base_link in the map frame via TF lookup."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._goal_frame, self._base_frame, rclpy.time.Time())
            return (
                tf.transform.translation.x,
                tf.transform.translation.y,
            )
        except Exception:
            self.get_logger().debug(
                f'Waiting for TF {self._goal_frame} -> {self._base_frame}')
            return None

    # ------------------------------------------------------------------
    # Nav2 goal
    # ------------------------------------------------------------------
    def _send_goal(self, x: float, y: float):
        self._navigating = True
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self._goal_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0
        if self._bt_xml:
            goal_msg.behavior_tree = self._bt_xml

        self._goal_sent_time = time.monotonic()
        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_cb)

    def _save_map_once(self):
        if self._map_saved or not self._map_save_path:
            return

        save_prefix = os.path.expanduser(self._map_save_path)
        save_dir = os.path.dirname(save_prefix)
        if save_dir:
            os.makedirs(save_dir, exist_ok=True)

        self.get_logger().info(
            f'Auto-saving map to: {save_prefix} (topic={self._map_topic})')
        try:
            subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-t', self._map_topic, '-f', save_prefix],
                check=True,
            )
            self._map_saved = True
            self.get_logger().info('Map saved successfully.')
        except Exception as exc:
            self.get_logger().error(f'Failed to save map: {exc}')

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            gx, gy = self._current_goal or (0.0, 0.0)
            self.get_logger().warn(
                f'Nav2 rejected goal ({gx:.2f}, {gy:.2f}) — trying next frontier.')
            self._navigating = False
            return
        gx, gy = self._current_goal or (0.0, 0.0)
        self.get_logger().info(
            f'Nav2 accepted goal ({gx:.2f}, {gy:.2f})'
            + (f' bt={os.path.basename(self._bt_xml)}' if self._bt_xml else ''))
        self._current_goal_handle = handle
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        self._current_goal_handle = None
        if self._ignore_next_result:
            self._ignore_next_result = False
            return
        elapsed = time.monotonic() - self._goal_sent_time
        status = future.result().status
        gx, gy = self._current_goal or (0.0, 0.0)
        if status == GoalStatus.STATUS_SUCCEEDED:
            if elapsed < 0.5:
                self.get_logger().warn(
                    f'Spurious success at ({gx:.2f}, {gy:.2f}) in {elapsed:.2f}s '
                    f'— not counting as visited.')
                if self._current_goal is not None:
                    self._register_failure(*self._current_goal)
            else:
                self.get_logger().info(
                    f'Reached ({gx:.2f}, {gy:.2f}) in {elapsed:.1f}s '
                    f'(visited={len(self._visited) + 1}). Searching next...')
                if self._current_goal is not None:
                    self._visited.append(self._current_goal)
        else:
            self.get_logger().warn(
                f'Nav failed ({gx:.2f}, {gy:.2f}) status={status} '
                f'after {elapsed:.1f}s.')
            if self._current_goal is not None:
                self._register_failure(*self._current_goal)
        self._navigating = False

    def _register_failure(self, fx, fy):
        """Track repeated failures at (roughly) the same frontier and
        blacklist it once it exceeds max_frontier_retries, so exploration
        doesn't retry an unreachable point forever."""
        for entry in self._fail_counts:
            if math.hypot(fx - entry[0], fy - entry[1]) < self._revisit_r:
                entry[2] += 1
                if entry[2] >= self._max_retries:
                    self.get_logger().warn(
                        f'Blacklist ({fx:.2f}, {fy:.2f}) after {entry[2]} fails '
                        f'(max_retries={self._max_retries}).')
                    self._visited.append((fx, fy))
                else:
                    self.get_logger().warn(
                        f'Fail count ({fx:.2f}, {fy:.2f}) = {entry[2]}/'
                        f'{self._max_retries}')
                return
        self._fail_counts.append([fx, fy, 1])
        self.get_logger().warn(
            f'Fail count ({fx:.2f}, {fy:.2f}) = 1/{self._max_retries}')


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
