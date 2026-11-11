#!/usr/bin/env python3
"""
frontier_coordinator.py — Centralized multi-robot frontier exploration.

Single node assigns each robot a unique frontier so no two robots ever target
the same unexplored area. When a robot finishes or fails, it is immediately
reassigned. Replaces the per-robot frontier_explorer in multi-robot mode.

Parameters
----------
robot_namespaces  Comma-separated robot names, e.g. "robot1,robot2,robot3"
frontier_detector Frontier detector plugin: classic, wfd, or rrt (default wfd)
frontier_scorer   Goal scorer: nearest | weighted | utility (default utility)
                  utility = gain*size_m - potential*distance
min_frontier_size Minimum cluster size to consider a frontier (default 5)
revisit_radius    Radius (m) within which a frontier counts as visited (default 0.5)
assign_radius     Radius (m) within which a frontier counts as taken (default 1.0)
poll_period       Seconds between assignment cycles (default 2.0)
map_topic         OccupancyGrid topic (default /map)
min_goal_distance Ignore frontiers closer than this to the robot (default 0.35)
frontier_clearance_radius Minimum map clearance around frontier goals (default 0.40).
                  Must exceed the largest robot_radius in use (0.34 for mppi configs)
                  or goal cells can land inside the costmap's lethal inscribed zone,
                  causing "GridBased: failed to create plan" even though the raw map
                  shows the cell as free.
failed_goal_radius Radius for matching failed frontier goals (default 0.75)
failed_goal_cooldown Seconds to avoid a failed frontier area (default 45)
distance_weight   Weighted scorer distance penalty (default 1.0)
info_gain_weight  Weighted scorer information gain reward (default 3.0)
potential_scale   Utility scorer distance penalty (default 3.0)
gain_scale        Utility scorer frontier-size reward (default 1.0)
hysteresis_radius Radius for robot-continuity scoring bonus (default 2.0)
hysteresis_gain   Weighted/utility current assignment bonus (default 1.5)
costmap_topic_suffix Per-robot Nav2 costmap topic suffix (default global_costmap/costmap)
validate_on_costmap Reject goals in inflated/lethal costmap cells (default true)
costmap_max_cost  Reject OccupancyGrid values >= this (default 50)
allow_unknown_costmap Allow goals on unknown/-1 costmap cells (default true)
publish_markers   Publish RViz MarkerArray debug overlays (default true)
nav_wait_warn_sec Seconds between warnings for missing Nav2 action servers (default 15)
tf_wait_warn_sec  Seconds between warnings for missing robot map TF (default 15)
map_save_path     File prefix for final map save, e.g. /path/to/maps/map (default '')
                  Uses map_saver_cli -t <map_topic> so /map_merged works in slam_mode:=multi
behavior_tree     BT XML stem/path for NavigateToPose (default explore_nav)

Published topics
----------------
/exploration/stats    std_msgs/String JSON status snapshot
/exploration/markers  visualization_msgs/MarkerArray debug markers
"""

import json
import math
import os
import subprocess
import time
from collections import deque
from enum import Enum, auto

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
import tf2_ros

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import ColorRGBA, String

_MAP_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
from visualization_msgs.msg import Marker, MarkerArray



def split_frontier(cluster, res, max_len_m=3.0):
    """Cut a long frontier cluster into pieces of about max_len_m each (cells are (row, col)).

    Ring-like clusters (centroid far from every cell) are ordered by angle around the centroid, others by
    their projection on the principal axis. Short clusters come back unchanged.
    """
    n_parts = int(math.ceil(len(cluster) * res / max_len_m))
    if n_parts <= 1:
        return [cluster]
    pts = np.asarray(cluster, dtype=float)
    c = pts.mean(axis=0)
    d = pts - c
    radius = np.hypot(d[:, 0], d[:, 1])
    if radius.min() > 0.3 * radius.max():                       # ring / arc
        key = np.arctan2(d[:, 0], d[:, 1])
    else:                                                        # elongated: principal axis
        _, _, vt = np.linalg.svd(d, full_matrices=False)
        key = d @ vt[0]
    order = np.argsort(key)
    return [[cluster[i] for i in idx] for idx in np.array_split(order, n_parts) if len(idx)]

NUDGE_MIN_FRONT_M = 0.8   # stop the nudge when the lidar sees anything this close dead ahead


def front_clearance(scan, half_angle=0.5):
    """Smallest valid range within +-half_angle rad of straight ahead (9.9 when nothing is seen)."""
    best = 9.9
    a = scan.angle_min
    for r in scan.ranges:
        if abs(math.atan2(math.sin(a), math.cos(a))) <= half_angle and math.isfinite(r) \
                and max(scan.range_min, 0.05) <= r < best:
            best = r
        a += scan.angle_increment
    return best


NUDGE_DISTANCE_M = 1.5   # cold-start forward nudge: real travel needed to pass slam_toolbox's travel gate


class _State(Enum):
    IDLE = auto()
    NAVIGATING = auto()


class FrontierCoordinator(Node):
    def __init__(self):
        super().__init__('frontier_coordinator')

        self.declare_parameter('robot_namespaces', 'robot1,robot2')
        self.declare_parameter('frontier_detector', 'wfd')
        self.declare_parameter('frontier_scorer', 'utility')
        # greedy: each idle robot takes its best frontier in turn (order-biased).
        # hungarian: optimal joint robot<->frontier assignment over total score.
        self.declare_parameter('allocation', 'greedy')
        self.declare_parameter('rrt_iterations', 300)
        self.declare_parameter('rrt_step_size', 0.5)
        self.declare_parameter('min_frontier_size', 5)
        self.declare_parameter('revisit_radius', 0.5)
        self.declare_parameter('assign_radius', 1.0)
        self.declare_parameter('poll_period', 2.0)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('min_goal_distance', 0.35)
        self.declare_parameter('frontier_clearance_radius', 0.55)
        self.declare_parameter('failed_goal_radius', 0.75)
        self.declare_parameter('failed_goal_cooldown', 45.0)
        self.declare_parameter('distance_weight', 1.0)
        self.declare_parameter('info_gain_weight', 3.0)
        self.declare_parameter('potential_scale', 3.0)
        self.declare_parameter('gain_scale', 1.0)
        self.declare_parameter('hysteresis_radius', 2.0)
        self.declare_parameter('hysteresis_gain', 1.5)
        # Soft spread: penalise frontiers near OTHER robots' current goals so
        # the fleet fans out (0 = off). Complements the hard assign_radius.
        # Per-robot min goal distance for mixed fleets, e.g. "robot2:0.8,robot3:0.5"
        # (ackermann robots cannot reach goals inside their turning radius).
        self.declare_parameter('min_goal_distance_overrides', '')
        # Tiny frontiers (a few wall-edge cells) have almost no unknown area behind them; chasing
        # them sent robots to useless far corners. Skipped while any larger frontier exists.
        self.declare_parameter('min_info_gain', 0.05)
        # A map->base_link transform older than this (s) is stale (SLAM / map->odom stopped
        # updating): treat the robot position as unknown instead of judging motion from it. 0 = off.
        self.declare_parameter('max_tf_age', 3.0)
        # Unknown map cells within this radius (m) of a robot count as traversable when measuring
        # path distances: a robot that spawned beside the explored area stands in cells the shared
        # map has not seen yet, and its own lidar clears them in Nav2. 0 = off.
        self.declare_parameter('unknown_reach', 1.5)
        # A goal Nav2 accepted but never answered (its stack was reset mid-goal, or the action
        # server vanished) would pin the robot in NAVIGATING forever. After this many seconds, or
        # as soon as its action server is gone for more than 5 s, the goal is cancelled and counted
        # as failed. 0 = off.
        self.declare_parameter('goal_timeout_s', 150.0)
        self.declare_parameter('separation_weight', 0.0)
        self.declare_parameter('separation_radius', 6.0)
        self.declare_parameter('suspicious_frontier_ratio', 3.0)
        self.declare_parameter('suspicious_frontier_penalty', 8.0)
        self.declare_parameter('suspicious_frontier_buffer', 1.0)
        self.declare_parameter('suspicious_frontier_hard_ratio', 6.0)
        self.declare_parameter('failed_frontier_min_radius', 0.75)
        self.declare_parameter('failed_frontier_max_radius', 3.0)
        self.declare_parameter('failed_frontier_radius_scale', 0.35)
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('nav_wait_warn_sec', 15.0)
        self.declare_parameter('tf_wait_warn_sec', 15.0)
        self.declare_parameter('map_save_path', '')
        self.declare_parameter('behavior_tree', 'explore_nav')
        self.declare_parameter('costmap_topic_suffix', 'global_costmap/costmap')
        self.declare_parameter('validate_on_costmap', True)
        self.declare_parameter('costmap_max_cost', 1)
        self.declare_parameter('allow_unknown_costmap', True)
        self.declare_parameter('goal_pullback', 0.55)
        self.declare_parameter('goal_search_radius', 1.25)

        raw = self.get_parameter('robot_namespaces').value
        self._robots = [ns.strip() for ns in raw.split(',') if ns.strip()]
        self._detector    = self.get_parameter('frontier_detector').value.strip().lower()
        self._scorer      = self.get_parameter('frontier_scorer').value.strip().lower()
        self._rrt_iterations = int(self.get_parameter('rrt_iterations').value)
        self._rrt_step    = float(self.get_parameter('rrt_step_size').value)
        self._min_size    = self.get_parameter('min_frontier_size').value
        self._revisit_r   = self.get_parameter('revisit_radius').value
        self._assign_r    = self.get_parameter('assign_radius').value
        self._allocation  = str(self.get_parameter('allocation').value).strip().lower()
        if self._allocation not in ('greedy', 'hungarian'):
            self.get_logger().warn(f'Unknown allocation={self._allocation!r}; using greedy.')
            self._allocation = 'greedy'
        self._min_goal_d  = self.get_parameter('min_goal_distance').value
        self._min_goal_by_robot = parse_robot_floats(
            self.get_parameter('min_goal_distance_overrides').value)
        self._frontier_clearance = self.get_parameter('frontier_clearance_radius').value
        self._failed_goal_r = self.get_parameter('failed_goal_radius').value
        self._failed_goal_cooldown = self.get_parameter('failed_goal_cooldown').value
        self._dist_w      = self.get_parameter('distance_weight').value
        self._info_w      = self.get_parameter('info_gain_weight').value
        self._potential_scale = self.get_parameter('potential_scale').value
        self._gain_scale  = self.get_parameter('gain_scale').value
        self._hyst_r      = self.get_parameter('hysteresis_radius').value
        self._hyst_gain   = self.get_parameter('hysteresis_gain').value
        self._min_gain    = self.get_parameter('min_info_gain').value
        self._max_tf_age  = self.get_parameter('max_tf_age').value
        self._unknown_reach = float(self.get_parameter('unknown_reach').value)
        self._goal_timeout = float(self.get_parameter('goal_timeout_s').value)
        self._sep_w       = self.get_parameter('separation_weight').value
        self._sep_r       = self.get_parameter('separation_radius').value
        self._suspect_ratio = float(self.get_parameter('suspicious_frontier_ratio').value)
        self._suspect_penalty = float(
            self.get_parameter('suspicious_frontier_penalty').value)
        self._suspect_buffer = float(self.get_parameter('suspicious_frontier_buffer').value)
        self._suspect_hard_ratio = float(
            self.get_parameter('suspicious_frontier_hard_ratio').value)
        self._failed_frontier_min_radius = float(
            self.get_parameter('failed_frontier_min_radius').value)
        self._failed_frontier_max_radius = float(
            self.get_parameter('failed_frontier_max_radius').value)
        self._failed_frontier_radius_scale = float(
            self.get_parameter('failed_frontier_radius_scale').value)
        self._publish_markers_enabled = self.get_parameter('publish_markers').value
        self._nav_wait_warn_sec = self.get_parameter('nav_wait_warn_sec').value
        self._tf_wait_warn_sec = self.get_parameter('tf_wait_warn_sec').value
        self._save_path   = self.get_parameter('map_save_path').value.strip()
        self._map_topic   = self.get_parameter('map_topic').value
        self._bt_xml      = _resolve_bt(self.get_parameter('behavior_tree').value)
        self._costmap_suffix = self.get_parameter('costmap_topic_suffix').value.strip()
        self._validate_costmap = self.get_parameter('validate_on_costmap').value
        self._costmap_max_cost = int(self.get_parameter('costmap_max_cost').value)
        self._allow_unknown_costmap = self.get_parameter('allow_unknown_costmap').value
        self._goal_pullback = float(self.get_parameter('goal_pullback').value)
        self._goal_search_radius = float(self.get_parameter('goal_search_radius').value)
        map_topic         = self._map_topic
        poll_period       = self.get_parameter('poll_period').value

        if self._detector not in ('classic', 'wfd', 'rrt'):
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

        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self)

        self._map: OccupancyGrid | None = None
        self._costmaps: dict[str, OccupancyGrid | None] = {r: None for r in self._robots}
        self._map_saved = False
        self._ever_assigned_goal = False
        self._start_time = self.get_clock().now()
        self._last_frontier_count = 0
        self._failed_goals = 0
        self._succeeded_goals = 0
        self._cold_spin_at, self._cold_spin_proc, self._cold_spin_n = {}, {}, {}
        self._nudges, self._nudge_pubs, self._nudge_timer = {}, {}, None
        self._nudge_front = {}
        self._nav_ready: dict[str, bool] = {r: False for r in self._robots}
        self._last_nav_wait_warn: dict[str, float] = {r: 0.0 for r in self._robots}
        self._tf_ready: dict[str, bool] = {r: False for r in self._robots}
        self._last_tf_wait_warn: dict[str, float] = {r: 0.0 for r in self._robots}
        self._last_assignment_cycle_log = 0.0

        # Per-robot tracking
        self._state:    dict[str, _State]                    = {r: _State.IDLE for r in self._robots}
        self._assigned: dict[str, tuple[float, float] | None] = {r: None        for r in self._robots}
        self._assigned_meta: dict[str, dict | None]           = {r: None        for r in self._robots}
        self._visited:  list[tuple[float, float]]            = []
        self._failed_frontiers: list[dict]                    = []
        # Spurious-success guard (as in frontier_explorer.py): Nav2 can report SUCCEEDED
        # without the robot moving when the controller has no TF yet. Such a goal must
        # not be marked visited, or the whole frontier ring is swallowed in seconds.
        self._goal_sent_t:   dict[str, float]                 = {r: 0.0   for r in self._robots}
        self._goal_handle:   dict[str, object]                = {r: None  for r in self._robots}
        self._goal_gen:      dict[str, int]                   = {r: 0     for r in self._robots}
        self._goal_start:    dict[str, tuple | None]          = {r: None  for r in self._robots}
        self._hold_until:    dict[str, float]                 = {r: 0.0   for r in self._robots}
        self._spurious:      dict[str, int]                   = {r: 0     for r in self._robots}

        # One Nav2 action client per robot
        self._nav_clients: dict[str, ActionClient] = {
            r: ActionClient(self, NavigateToPose, f'/{r}/navigate_to_pose')
            for r in self._robots
        }
        self._stats_pub = self.create_publisher(String, '/exploration/stats', 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/exploration/frontiers', 10)

        self.get_logger().info(f'Watching Nav2 servers: {self._robots}')
        self.create_subscription(OccupancyGrid, map_topic, self._map_cb, _MAP_QOS)
        if self._validate_costmap and self._costmap_suffix:
            for r in self._robots:
                topic = f'/{r}/{self._costmap_suffix}'
                self.create_subscription(
                    OccupancyGrid, topic,
                    lambda msg, robot=r: self._costmap_cb(msg, robot),
                    _MAP_QOS)
        self.create_timer(poll_period, self._cycle)
        bt_note = self._bt_xml or '(nav2 default)'
        self.get_logger().info(
            f'Frontier coordinator ready | robots={self._robots} '
            f'detector={self._detector} scorer={self._scorer} bt={bt_note} '
            f'map={map_topic} potential={self._potential_scale} '
            f'gain={self._gain_scale} suspect_ratio={self._suspect_ratio:.2f} '
            f'suspect_penalty={self._suspect_penalty:.2f} '
            f'suspect_hard={self._suspect_hard_ratio:.2f} '
            f'costmap_validate={self._validate_costmap} '
            f'costmap_max={self._costmap_max_cost} pullback={self._goal_pullback:.2f}m '
            f'clearance={self._frontier_clearance:.2f}m')

    # ------------------------------------------------------------------
    def _map_cb(self, msg: OccupancyGrid):
        self._map = msg

    def _costmap_cb(self, msg: OccupancyGrid, robot: str):
        self._costmaps[robot] = msg

    # ------------------------------------------------------------------
    def _cycle(self):
        if self._map is None:
            self.get_logger().debug('Waiting for occupancy grid map.')
            return

        elapsed = self._elapsed_sec()
        robot_positions = self._robot_positions(elapsed)
        self._refresh_nav_servers()
        self._expire_stale_goals()
        if self._detector in ('wfd', 'rrt') and not robot_positions:
            self.get_logger().debug('Waiting for robot TF before frontier search.')
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
            costmap_ready = [
                r for r in self._robots if self._costmaps.get(r) is not None]
            self.get_logger().info(
                f'cycle t={elapsed:.0f}s frontiers={len(frontiers)} '
                f'ready_idle={ready_idle} waiting_nav={waiting_nav} '
                f'waiting_tf={waiting_tf} costmaps={costmap_ready} '
                f'visited={len(self._visited)} '
                f'ok={self._succeeded_goals} fail={self._failed_goals}')
            self._last_assignment_cycle_log = elapsed

        now_m = time.monotonic()
        idle = [
            r for r in self._robots
            if self._state[r] == _State.IDLE and self._nav_ready.get(r, False)
            and now_m >= self._hold_until[r]
        ]
        if idle and frontiers:
            plan = {}
            if self._allocation == 'hungarian' and len(idle) > 1:
                plan = self._hungarian_plan(idle, frontiers, robot_positions)
            for r in idle:
                pos = robot_positions.get(r)
                if pos is None:
                    continue
                if r in plan:
                    goal, pick = plan[r]
                else:
                    goal, pick = self._pick(frontiers, pos, r)
                if goal is None:
                    self.get_logger().info(
                        f'[{r}] no usable frontier '
                        f'(pool={pick["n_total"]} ok={pick["n_ok"]} '
                        f'visited={pick["n_visited"]} failed={pick["n_failed"]} '
                        f'taken={pick["n_taken"]} costmap={pick["n_costmap"]} '
                        f'near={pick["n_near"]} suspicious={pick.get("n_suspicious", 0)} '
                        f'unreachable={pick.get("n_unreachable", 0)})')
                    self._cold_spin(r)
                    continue
                self._assigned[r] = goal
                self._assigned_meta[r] = pick
                self._state[r] = _State.NAVIGATING
                self._ever_assigned_goal = True
                self.get_logger().info(
                    f'[{r}] → ({goal[0]:.2f}, {goal[1]:.2f}) '
                    f'| scorer={self._scorer} score={pick["score"]:.2f} '
                    f'dist={pick["distance"]:.2f}m size={pick["size_m"]:.2f}m '
                    f'info={pick["info_gain"]:.2f}m² clear={pick["clearance"]:.2f}m '
                    f'suspect={pick["suspicious_ratio"]:.2f} '
                    f'| pool={pick["n_ok"]}/{pick["n_total"]} '
                    f'(skip visited={pick["n_visited"]} failed={pick["n_failed"]} '
                    f'taken={pick["n_taken"]} costmap={pick["n_costmap"]} '
                    f'near={pick["n_near"]} suspicious={pick.get("n_suspicious", 0)} '
                    f'unreachable={pick.get("n_unreachable", 0)})')
                self._send_goal(r, goal[0], goal[1])
        elif (not frontiers and self._ever_assigned_goal
              and all(s == _State.IDLE for s in self._state.values())):
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
                self.get_logger().warn(
                    f'map -> {robot}/base_link TF disappeared ({getattr(self, "_tf_fail", {}).get(robot, "?")})')
            self._tf_ready[robot] = False
            if elapsed - self._last_tf_wait_warn[robot] >= self._tf_wait_warn_sec:
                self.get_logger().warn(
                    f'Waiting for map -> {robot}/base_link TF '
                    f'({elapsed:.0f}s since coordinator start)')
                self._last_tf_wait_warn[robot] = elapsed
        return positions

    def _cold_spin(self, ns):
        """Fleet still cold (fewer than 3 goals reached) and this robot has nothing to go to: the known map is
        only the first lidar integration. slam_toolbox integrates a new scan only after real forward travel
        (a hand test: rotating changed nothing, 2 m forward took the map from 3.6 to 38 m2), and the first cold
        goal can be under 0.5 m away - so the map never grows. Nudge the robot ~0.8 m forward (open loop,
        /<ns>/cmd_vel; the collision monitor still guards it). At most 2 nudges per robot."""
        if getattr(self, '_succeeded_goals', 99) >= 3:
            return False
        now = time.monotonic()
        if self._cold_spin_n.get(ns, 0) >= 2:
            return False
        if now - self._cold_spin_at.setdefault(ns, -1e9) < 45.0 or ns in self._nudges:
            return False
        self._cold_spin_at[ns] = now
        self._cold_spin_n[ns] = self._cold_spin_n.get(ns, 0) + 1
        self.get_logger().warn(f'[{ns}] no frontier yet and the map is still tiny - nudging ~1.5 m forward so SLAM integrates new scans.')
        try:
            self._start_nudge(ns)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'[{ns}] cold-start nudge could not start: {e}')
            return False
        return True

    def _start_nudge(self, ns):
        from geometry_msgs.msg import Twist
        if ns not in self._nudge_pubs:
            self._nudge_pubs[ns] = self.create_publisher(Twist, f'/{ns}/cmd_vel', 10)
            from sensor_msgs.msg import LaserScan
            self.create_subscription(
                LaserScan, f'/{ns}/scan',
                lambda m, ns=ns: self._nudge_front.__setitem__(ns, front_clearance(m)),
                qos_profile_sensor_data)
        if self._nudge_timer is None:
            self._nudge_timer = self.create_timer(0.1, self._nudge_tick)
        # drive until ~1.5 m of real travel (measured on TF) or 10 s: a fixed 3.2 s command only moved the
        # robot ~0.3 m (acceleration + collision monitor), which never reached slam_toolbox's travel gate
        self._nudges[ns] = {'deadline': time.monotonic() + 10.0, 'start': None}

    def _nudge_tick(self):
        from geometry_msgs.msg import Twist
        now = time.monotonic()
        for ns, st in list(self._nudges.items()):
            msg = Twist()
            done = now >= st['deadline']
            try:
                p = self._tf.lookup_transform('map', f'{ns}/base_link', rclpy.time.Time()).transform.translation
                if st['start'] is None:
                    st['start'] = (p.x, p.y)
                elif math.hypot(p.x - st['start'][0], p.y - st['start'][1]) >= NUDGE_DISTANCE_M:
                    done = True
            except Exception:  # noqa: BLE001 - TF hiccup: fall back to the deadline
                pass
            if self._nudge_front.get(ns, 9.9) < NUDGE_MIN_FRONT_M:
                done = True                                 # something (often the next robot in the spawn row) is ahead
            if done:
                del self._nudges[ns]                        # one zero command to stop
            else:
                msg.linear.x = 0.25
            self._nudge_pubs[ns].publish(msg)

    def _elapsed_sec(self):
        return (self.get_clock().now() - self._start_time).nanoseconds / 1e9

    # ------------------------------------------------------------------
    def _pick(self, frontiers, robot_pos, ns, collect=None, allow_tiny=False, allow_suspicious=False):
        """Pick a frontier through the selected scorer plugin.

        Returns (goal_xy | None, diagnostics dict).
        """
        taken = [g for r, g in self._assigned.items() if r != ns and g is not None]
        path_distances = self._path_distance_map(robot_pos)

        best, best_score = None, float('-inf')
        best_meta = {}
        n_visited = n_failed = n_taken = n_costmap = n_near = n_ok = n_unreachable = n_suspicious = 0
        n_tiny = 0
        for frontier in frontiers:
            fx, fy = frontier['point']
            if _near_any(fx, fy, self._visited, self._revisit_r):
                n_visited += 1
                continue
            if self._recently_failed(fx, fy):
                n_failed += 1
                continue
            # Cold start: before any goal has been assigned or finished the known-free bubble is
            # tiny and every frontier looks "large vs clearance" - don't hard-reject then (the
            # soft penalty in the scorer still applies), or the fleet never leaves the spawn.
            cold = (getattr(self, '_succeeded_goals', 1) + getattr(self, '_failed_goals', 1) == 0
                    and not any(g is not None for g in getattr(self, '_assigned', {1: 1}).values()))
            if (not cold and not allow_suspicious
                    and frontier.get('suspicious_ratio', 0.0) >= self._suspect_hard_ratio):
                n_suspicious += 1
                continue
            if (not allow_tiny and getattr(self, '_min_gain', 0.0) > 0.0
                    and frontier.get('info_gain', 0.0) < self._min_gain):
                n_tiny += 1
                continue
            if _near_any(fx, fy, taken, self._assign_r):
                n_taken += 1
                continue
            if not self._costmap_allows(fx, fy, ns):
                n_costmap += 1
                continue
            d = self._frontier_path_distance(frontier, path_distances)
            if not math.isfinite(d):
                n_unreachable += 1
                continue
            if d < self._min_goal_for(ns):
                n_near += 1
                continue

            n_ok += 1
            score = self._score_frontier(frontier, d, ns)
            if collect is not None:
                collect.append((score, (fx, fy), {
                    'score': score, 'distance': d, 'size_m': frontier['size_m'],
                    'info_gain': frontier['info_gain'],
                    'clearance': frontier.get('clearance', 0.0),
                    'suspicious_ratio': frontier.get('suspicious_ratio', 0.0)}))
            if score > best_score:
                best_score = score
                best = (fx, fy)
                best_meta = {
                    'score': score,
                    'distance': d,
                    'size_m': frontier['size_m'],
                    'info_gain': frontier['info_gain'],
                    'clearance': frontier.get('clearance', 0.0),
                    'suspicious_ratio': frontier.get('suspicious_ratio', 0.0),
                }

        if best is None and n_tiny and not allow_tiny:
            # only tiny frontiers are left: take them rather than stop exploring early
            return self._pick(frontiers, robot_pos, ns, collect=collect, allow_tiny=True,
                              allow_suspicious=allow_suspicious)

        if best is None and n_suspicious and not allow_suspicious:
            # every remaining frontier looks suspicious (e.g. one big perimeter of a small known bubble,
            # seen in the cafe fleet run: pool=1 suspicious=1 forever after the first goal). Rather than
            # idle, take the best of them; failures are blacklisted so a bad one cannot loop.
            return self._pick(frontiers, robot_pos, ns, collect=collect, allow_tiny=allow_tiny,
                              allow_suspicious=True)

        diag = {
            'n_total': len(frontiers),
            'n_tiny': n_tiny,
            'n_ok': n_ok,
            'n_visited': n_visited,
            'n_failed': n_failed,
            'n_taken': n_taken,
            'n_costmap': n_costmap,
            'n_near': n_near,
            'n_unreachable': n_unreachable,
            'n_suspicious': n_suspicious,
            'score': best_meta.get('score', float('-inf')),
            'distance': best_meta.get('distance', 0.0),
            'size_m': best_meta.get('size_m', 0.0),
            'info_gain': best_meta.get('info_gain', 0.0),
            'clearance': best_meta.get('clearance', 0.0),
            'suspicious_ratio': best_meta.get('suspicious_ratio', 0.0),
        }
        return best, diag

    def _hungarian_plan(self, idle, frontiers, robot_positions):
        """Jointly assign idle robots to distinct frontiers maximizing total score.

        Returns {robot: (goal_xy, pick_diag)}. Frontiers within assign_radius of
        each other can't both be used, so winners that conflict are dropped and
        those robots fall back to the greedy pick.
        """
        import numpy as np
        from scipy.optimize import linear_sum_assignment
        cands, diags = {}, {}
        for r in idle:
            pos = robot_positions.get(r)
            if pos is None:
                continue
            c = []
            _, diags[r] = self._pick(frontiers, pos, r, collect=c)
            if c:
                cands[r] = c
        robots = list(cands)
        if len(robots) < 2:
            return {}
        # unique frontier columns across robots
        cols = []
        for r in robots:
            for _, xy, _ in cands[r]:
                if not any(math.hypot(xy[0] - q[0], xy[1] - q[1]) < 1e-6 for q in cols):
                    cols.append(xy)
        big = -1e6
        gain = np.full((len(robots), len(cols)), big)
        meta = {}
        for i, r in enumerate(robots):
            for score, xy, m in cands[r]:
                j = next(k for k, q in enumerate(cols)
                         if math.hypot(xy[0] - q[0], xy[1] - q[1]) < 1e-6)
                gain[i, j] = score
                meta[(i, j)] = m
        rows, cs = linear_sum_assignment(-gain)
        plan, used = {}, []
        for i, j in sorted(zip(rows, cs), key=lambda t: -gain[t[0], t[1]]):
            if gain[i, j] <= big / 2:
                continue
            xy = cols[j]
            if _near_any(xy[0], xy[1], used, self._assign_r):
                continue
            used.append(xy)
            pick = dict(diags[robots[i]])
            pick.update(meta[(i, j)])
            plan[robots[i]] = (xy, pick)
        self.get_logger().info(
            f'hungarian: {len(plan)}/{len(robots)} robots assigned over {len(cols)} frontiers '
            f'total_score={sum(p[1]["score"] for p in plan.values()):.2f}')
        return plan

    def _score_frontier(self, frontier, distance, ns):
        suspicious_ratio = self._suspicious_frontier_ratio(
            frontier['size_m'], frontier.get('clearance', 0.0))
        suspicious_penalty = 0.0
        if self._scorer != 'nearest' and suspicious_ratio > self._suspect_ratio:
            suspicious_penalty = (
                suspicious_ratio - self._suspect_ratio) * self._suspect_penalty
        if self._scorer == 'utility':
            score = (
                self._gain_scale * frontier['size_m']
                - self._potential_scale * distance)
        elif self._scorer == 'weighted':
            score = self._info_w * frontier['info_gain'] - self._dist_w * distance
        else:
            score = -distance
        if self._scorer != 'nearest':
            current = self._assigned.get(ns)
            if current is not None:
                fx, fy = frontier['point']
                if math.hypot(fx - current[0], fy - current[1]) <= self._hyst_r:
                    score += self._hyst_gain
        return score - suspicious_penalty - self._separation_penalty(frontier, ns)

    def _tf_is_stale(self, tf) -> bool:
        max_age = getattr(self, '_max_tf_age', 0.0)
        if not max_age or max_age <= 0.0:
            return False
        stamp = tf.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return False                      # static-only chain: no meaningful age
        now = self.get_clock().now().nanoseconds / 1e9
        age = now - (stamp.sec + stamp.nanosec / 1e9)
        self._last_tf_age = (age, now, stamp.sec + stamp.nanosec / 1e9)
        return age > max_age

    def _min_goal_for(self, ns):
        return getattr(self, '_min_goal_by_robot', {}).get(ns, self._min_goal_d)

    def _separation_penalty(self, frontier, ns):
        """Linear penalty (max separation_weight) for frontiers close to
        another robot's current goal; 0 beyond separation_radius."""
        if self._sep_w <= 0.0 or self._sep_r <= 0.0:
            return 0.0
        fx, fy = frontier['point']
        near = min(
            (math.hypot(fx - g[0], fy - g[1])
             for r, g in self._assigned.items() if r != ns and g is not None),
            default=None)
        if near is None or near >= self._sep_r:
            return 0.0
        return self._sep_w * (1.0 - near / self._sep_r)

    def _suspicious_frontier_ratio(self, size_m: float, clearance: float) -> float:
        """Heuristic leak detector for giant frontier clusters behind thin seams."""
        depth = max(clearance + self._suspect_buffer, 1e-3)
        return size_m / depth

    def _path_distance_map(self, robot_pos):
        msg = self._map
        data = np.array(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
        free = data == 0
        h, w = free.shape
        dist = np.full((h, w), np.inf, dtype=np.float32)
        seed = _world_to_cell(robot_pos[0], robot_pos[1], msg.info)
        reach = getattr(self, '_unknown_reach', 0.0)
        # Only when the robot is NOT standing in known free space. A robot whose own map already
        # covers its cell (e.g. each robot's SLAM map in slam_mode:=multi) keeps the strict
        # known-free search: carving unknown cells there picked frontiers Nav2 then could not reach.
        in_known_free = 0 <= seed[0] < h and 0 <= seed[1] < w and bool(free[seed])
        if reach > 0.0 and not in_known_free and 0 <= seed[0] < h and 0 <= seed[1] < w:
            # unknown cells right around the robot are passable (not occupied ones)
            rc = int(math.ceil(reach / msg.info.resolution))
            y0, y1 = max(0, seed[0] - rc), min(h, seed[0] + rc + 1)
            x0, x1 = max(0, seed[1] - rc), min(w, seed[1] + rc + 1)
            yy, xx = np.ogrid[y0:y1, x0:x1]
            near = (yy - seed[0]) ** 2 + (xx - seed[1]) ** 2 <= rc * rc
            free[y0:y1, x0:x1] |= near & (data[y0:y1, x0:x1] == -1)
        resolved = (seed if (0 <= seed[0] < h and 0 <= seed[1] < w and free[seed])
                    else self._nearest_free_cell(seed[0], seed[1], free, msg.info.resolution))
        if resolved is None:
            return dist
        sy, sx = resolved
        queue = deque([(sy, sx)])
        dist[sy, sx] = 0.0
        while queue:
            y, x = queue.popleft()
            base = dist[y, x] + msg.info.resolution
            for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
                if 0 <= ny < h and 0 <= nx < w and free[ny, nx] and not math.isfinite(dist[ny, nx]):
                    dist[ny, nx] = base
                    queue.append((ny, nx))
        return dist

    @staticmethod
    def _frontier_path_distance(frontier, path_distances):
        cell = frontier.get('goal_cell')
        if cell is None:
            return float('inf')
        y, x = cell
        if y < 0 or x < 0 or y >= path_distances.shape[0] or x >= path_distances.shape[1]:
            return float('inf')
        return float(path_distances[y, x])

    def _costmap_allows(self, x, y, ns) -> bool:
        if not self._validate_costmap:
            return True
        msg = self._costmaps.get(ns)
        if msg is None:
            return True
        return self._costmap_point_ok(x, y, msg)

    def _costmap_point_ok(self, x, y, msg: OccupancyGrid) -> bool:
        cy, cx = _world_to_cell(x, y, msg.info)
        h, w = msg.info.height, msg.info.width
        if cy < 0 or cy >= h or cx < 0 or cx >= w:
            return self._allow_unknown_costmap
        val = int(msg.data[cy * w + cx])
        if val < 0:
            return self._allow_unknown_costmap
        return val < self._costmap_max_cost

    # ------------------------------------------------------------------
    def _robot_pos(self, ns):
        try:
            tf = self._tf.lookup_transform('map', f'{ns}/base_link', rclpy.time.Time())
            if self._tf_is_stale(tf):
                self._tf_fail = getattr(self, '_tf_fail', {})
                self._tf_fail[ns] = (f'stale: age {self._last_tf_age[0]:.2f}s > {self._max_tf_age:.1f}s '
                                        f'(clock now={self._last_tf_age[1]:.2f} stamp={self._last_tf_age[2]:.2f})')
                return None
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception as e:  # noqa: BLE001
            self._tf_fail = getattr(self, '_tf_fail', {})
            self._tf_fail[ns] = f'{type(e).__name__}: {str(e)[:120]}'
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
        if self._bt_xml:
            goal.behavior_tree = self._bt_xml
        self._goal_sent_t[ns] = time.monotonic()
        self._goal_start[ns] = self._robot_pos(ns)

        self._goal_gen[ns] += 1
        self._goal_handle[ns] = None
        f = self._nav_clients[ns].send_goal_async(goal)
        f.add_done_callback(lambda fut, r=ns, g=self._goal_gen[ns]: self._on_accepted(fut, r, g))

    def _on_accepted(self, future, ns, gen=None):
        handle = future.result()
        if gen is not None and gen != self._goal_gen[ns]:
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()      # a goal we already gave up on
            return
        goal = self._assigned.get(ns)
        gx, gy = goal if goal else (0.0, 0.0)
        if not handle.accepted:
            self.get_logger().warn(
                f'[{ns}] Nav2 rejected ({gx:.2f}, {gy:.2f}) — freeing.')
            self._failed_goals += 1
            if goal:
                self._remember_failed_frontier(goal, self._assigned_meta.get(ns))
            self._assigned[ns] = None
            self._assigned_meta[ns] = None
            self._state[ns] = _State.IDLE
            self._publish_stats()
            return
        self.get_logger().info(f'[{ns}] Nav2 accepted ({gx:.2f}, {gy:.2f})')
        self._goal_handle[ns] = handle
        handle.get_result_async().add_done_callback(
            lambda fut, r=ns, g=self._goal_gen[ns]: self._on_result(fut, r, g))

    def _expire_stale_goals(self):
        """Free robots stuck NAVIGATING on a goal Nav2 will never answer."""
        timeout = getattr(self, '_goal_timeout', 0.0)
        if timeout <= 0.0:
            return
        now = time.monotonic()
        freed = False
        for ns in self._robots:
            if self._state[ns] != _State.NAVIGATING:
                continue
            elapsed = now - self._goal_sent_t[ns]
            server_gone = not self._nav_ready.get(ns, False) and elapsed > 5.0
            if elapsed < timeout and not server_gone:
                continue
            goal = self._assigned.get(ns)
            why = 'its Nav2 action server is gone' if server_gone else f'no result after {elapsed:.0f}s'
            self.get_logger().warn(
                f'[{ns}] goal {goal} abandoned: {why} - cancelling, counting it failed and freeing {ns}.')
            handle = self._goal_handle.get(ns)
            if handle is not None:
                try:
                    handle.cancel_goal_async()
                except Exception:
                    pass
            self._goal_gen[ns] += 1             # any late reply for this goal is now ignored
            self._goal_handle[ns] = None
            self._failed_goals += 1
            if goal:
                self._remember_failed_frontier(goal, self._assigned_meta.get(ns))
            self._assigned[ns] = None
            self._assigned_meta[ns] = None
            self._state[ns] = _State.IDLE
            freed = True
        if freed:
            self._publish_stats()

    def _spurious_success(self, ns, goal) -> bool:
        """True when Nav2 said SUCCEEDED but the robot never really went anywhere."""
        gx, gy = goal
        start = self._goal_start.get(ns)
        pos = self._robot_pos(ns)
        elapsed = time.monotonic() - self._goal_sent_t[ns]
        if start is None or pos is None:
            return False
        # Goal already inside Nav2's tolerance when sent: success without motion is right.
        if math.hypot(gx - start[0], gy - start[1]) < self._min_goal_for(ns):
            return False
        moved = math.hypot(pos[0] - start[0], pos[1] - start[1])
        need = min(0.3, 0.5 * math.hypot(gx - start[0], gy - start[1]))
        far = math.hypot(pos[0] - gx, pos[1] - gy) > 1.0
        if not (elapsed < 0.5 or far or moved < need):
            return False
        self._spurious[ns] += 1
        backoff = min(30.0, 2.0 * self._spurious[ns])
        self._hold_until[ns] = time.monotonic() + backoff
        self.get_logger().warn(
            f'[{ns}] spurious success at ({gx:.2f}, {gy:.2f}) in {elapsed:.2f}s, robot moved '
            f'{moved:.2f} m (< {need:.2f} m) - not marking visited; {self._spurious[ns]} in a '
            f'row, holding goals {backoff:.0f}s (controller TF likely not ready).')
        return True

    def _on_result(self, future, ns, gen=None):
        if gen is not None and gen != getattr(self, '_goal_gen', {}).get(ns, gen):
            return                              # reply for a goal we already expired
        if getattr(self, '_goal_handle', None) is not None:
            self._goal_handle[ns] = None
        status = future.result().status
        goal = self._assigned[ns]
        gx, gy = goal if goal else (0.0, 0.0)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'[{ns}] reached ({gx:.2f}, {gy:.2f}) '
                f'(visited={len(self._visited) + (1 if goal else 0)} '
                f'ok={self._succeeded_goals + 1})')
            if goal and self._spurious_success(ns, goal):
                self._assigned[ns] = None
                self._assigned_meta[ns] = None
                self._state[ns] = _State.IDLE
                self._publish_stats()
                return
            self._spurious[ns] = 0
            self._succeeded_goals += 1
            if goal:
                self._visited.append(goal)
        else:
            self.get_logger().warn(
                f'[{ns}] failed ({gx:.2f}, {gy:.2f}) status={status} — will reassign '
                f'(fail={self._failed_goals + 1}).')
            self._failed_goals += 1
            if goal:
                self._remember_failed_frontier(goal, self._assigned_meta.get(ns))
        self._assigned[ns] = None
        self._assigned_meta[ns] = None
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
        seed_cells = None
        if self._detector == 'wfd':
            reachable = self._reachable_mask(free, robot_positions, msg.info)
            mask &= reachable
        elif self._detector == 'rrt':
            seed_cells = self._rrt_seed_cells(free, mask, msg.info, robot_positions)

        if not mask.any():
            return []
        if seed_cells is not None and not seed_cells:
            return []

        candidate_cells = (
            seed_cells if seed_cells is not None
            else ((int(y), int(x)) for y, x in np.argwhere(mask)))

        seen  = np.zeros_like(mask, dtype=bool)
        result = []
        for sy, sx in candidate_cells:
            sy, sx = int(sy), int(sx)
            if seen[sy, sx] or not mask[sy, sx]:
                continue
            queue = deque([(sy, sx)])
            seen[sy, sx] = True
            cluster = []
            while queue:
                y, x = queue.popleft()
                cluster.append((y, x))
                # 8-connected: a diagonal sensor-FOV edge is a staircase of cells that touch only at corners;
                # 4-connectivity shatters it into fragments below min_frontier_size (=> 'no frontiers').
                for ny, nx in ((y-1,x),(y+1,x),(y,x-1),(y,x+1),(y-1,x-1),(y-1,x+1),(y+1,x-1),(y+1,x+1)):
                    if 0 <= ny < h and 0 <= nx < w and not seen[ny, nx] and mask[ny, nx]:
                        seen[ny, nx] = True
                        queue.append((ny, nx))
            if len(cluster) < self._min_size:
                continue
            # A long frontier (e.g. the whole ring around the first lidar view) would otherwise offer ONE
            # goal; once that is visited nothing else is offered and the fleet idles. Offer several.
            for part in split_frontier(cluster, res, getattr(self, '_split_len_m', 3.0)):
                if len(part) < self._min_size:
                    continue
                goal_cell, clearance = self._place_safe_goal(
                    part, free, unknown, occupied, res)
                if goal_cell is None:
                    continue
                cy, cx = goal_cell
                result.append({
                    'point': (ox + (cx + 0.5) * res, oy + (cy + 0.5) * res),
                    'goal_cell': (int(cy), int(cx)),
                    'size': len(part),
                    'size_m': len(part) * res,
                    'info_gain': self._info_gain(part, unknown, res),
                    'clearance': clearance,
                    'suspicious_ratio': self._suspicious_frontier_ratio(len(part) * res, clearance),
                })
        return result

    def _place_safe_goal(self, cluster, free, unknown, occupied, resolution):
        """Place a goal inside free space, pulled back off the frontier.

        Cold-start note: right after spawn the known-free region can be too
        small for any cell to clear the full goal_pullback from unknown
        space (chicken-and-egg — the region only grows once a robot moves,
        which needs a goal first). Rather than hard-rejecting below
        goal_pullback, track every candidate that clears *occupied*
        clearance and relax the pullback requirement afterward if nothing
        meets it, instead of silently reporting zero frontiers forever.
        """
        h, w = free.shape
        pull_cells = max(1, math.ceil(self._goal_pullback / resolution))
        clear_cells = max(1, math.ceil(self._frontier_clearance / resolution))
        search_cells = max(pull_cells, math.ceil(self._goal_search_radius / resolution))
        ref_costmap = next(
            (c for c in self._costmaps.values() if c is not None), None)

        seen = np.zeros_like(free, dtype=bool)
        queue = deque()
        for y, x in cluster:
            y, x = int(y), int(x)
            if not seen[y, x]:
                seen[y, x] = True
                queue.append((y, x, 0))

        ox = self._map.info.origin.position.x
        oy = self._map.info.origin.position.y
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
            wx = ox + (x + 0.5) * resolution
            wy = oy + (y + 0.5) * resolution
            if ref_costmap is not None and not self._costmap_point_ok(
                    wx, wy, ref_costmap):
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

        # Rank by proximity to the cluster first, clearance only as a
        # tiebreaker — scoring by clearance first makes every cluster's BFS
        # converge on whichever cell in reach has the best clearance
        # workspace-wide, which falsely marks all of them "visited" together
        # when the known-free region is small.
        best = None
        best_key = None
        for unk_clear, occ_clear, dist, y, x in pool:
            key = (dist, -(unk_clear + occ_clear))
            if best_key is None or key < best_key:
                best_key = key
                best = (float(y), float(x), occ_clear)

        return (best[0], best[1]), best[2]

    def _best_goal_cell(self, cluster, occupied, resolution):
        # Kept for compatibility; prefer _place_safe_goal.
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
    def _cell_clearance(y, x, mask, radius_cells, resolution):
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
            seed = (y, x) if (0 <= y < h and 0 <= x < w and free[y, x]) else \
                self._nearest_free_cell(y, x, free, info.resolution)
            if seed is not None and not seen[seed]:
                seen[seed] = True
                queue.append(seed)

        while queue:
            y, x = queue.popleft()
            for ny, nx in ((y-1,x),(y+1,x),(y,x-1),(y,x+1)):
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

    def _rrt_seed_cells(self, free, mask, info, robot_positions):
        """Multi-root RRT: grow a random tree through free space from every
        robot's cell; any step landing on a frontier cell is a cluster seed
        (not extended further). Same seed→flood-fill contract as wfd/classic,
        just a sampling-based (not full-grid scan) way to find the seeds."""
        h, w = free.shape
        res = info.resolution
        step_cells = max(1, round(self._rrt_step / res))

        tree_y, tree_x = [], []
        for wx, wy in robot_positions:
            ry, rx = _world_to_cell(wx, wy, info)
            seed = (ry, rx) if (0 <= ry < h and 0 <= rx < w and free[ry, rx]) else \
                self._nearest_free_cell(ry, rx, free, res)
            if seed is not None:
                tree_y.append(float(seed[0]))
                tree_x.append(float(seed[1]))
        if not tree_y:
            return set()

        seeds = set()
        rng = np.random.default_rng()
        for _ in range(self._rrt_iterations):
            sy = rng.integers(0, h)
            sx = rng.integers(0, w)
            ty = np.asarray(tree_y)
            tx = np.asarray(tree_x)
            i = int(np.argmin((ty - sy) ** 2 + (tx - sx) ** 2))
            ny0, nx0 = ty[i], tx[i]
            dy, dx = sy - ny0, sx - nx0
            dist = math.hypot(dy, dx)
            if dist < 1e-6:
                continue
            scale = min(step_cells, dist) / dist
            cy = int(round(ny0 + dy * scale))
            cx = int(round(nx0 + dx * scale))
            if not (0 <= cy < h and 0 <= cx < w):
                continue
            iy0, ix0 = int(round(ny0)), int(round(nx0))
            if mask[cy, cx]:
                if self._line_free(iy0, ix0, cy, cx, free):
                    seeds.add((cy, cx))
                continue
            if free[cy, cx] and self._line_free(iy0, ix0, cy, cx, free):
                tree_y.append(float(cy))
                tree_x.append(float(cx))
        return seeds

    @staticmethod
    def _line_free(y0, x0, y1, x1, free):
        """Bresenham rasterization; True iff every cell on the segment is free."""
        h, w = free.shape
        dy, dx = abs(y1 - y0), abs(x1 - x0)
        sy = 1 if y1 >= y0 else -1
        sx = 1 if x1 >= x0 else -1
        err = dx - dy
        y, x = y0, x0
        while True:
            if not (0 <= y < h and 0 <= x < w) or not free[y, x]:
                return False
            if y == y1 and x == x1:
                return True
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    # ------------------------------------------------------------------
    def _save_map(self):
        if self._map_saved or not self._save_path:
            return
        save_dir = os.path.dirname(self._save_path)
        if save_dir:
            os.makedirs(save_dir, exist_ok=True)
        self.get_logger().info(
            f'Saving map → {self._save_path} (topic={self._map_topic})')
        try:
            subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-t', self._map_topic, '-f', self._save_path],
                check=True)
            self._map_saved = True
            self.get_logger().info('Map saved.')
        except Exception as e:
            self.get_logger().error(f'Map save failed: {e}')

    # ------------------------------------------------------------------
    def _remember_failed_frontier(self, goal, frontier_meta=None):
        now = self._elapsed_sec()
        gx, gy = goal
        self._failed_frontiers = [
            f for f in self._failed_frontiers
            if now - f['time'] <= self._failed_goal_cooldown
        ]
        size_m = 0.0 if frontier_meta is None else float(frontier_meta.get('size_m', 0.0))
        suspicious = 1.0 if frontier_meta is None else max(
            1.0, float(frontier_meta.get('suspicious_ratio', 1.0)) / max(self._suspect_ratio, 1e-3))
        radius = size_m * self._failed_frontier_radius_scale * min(suspicious, 2.0)
        radius = max(self._failed_frontier_min_radius,
                     min(self._failed_frontier_max_radius, radius))
        self._failed_frontiers.append({'point': goal, 'time': now, 'radius': radius})
        self.get_logger().warn(
            f'Blacklisting failed frontier near ({gx:.2f}, {gy:.2f}) '
            f'for {self._failed_goal_cooldown:.0f}s radius={radius:.2f}m')

    def _recently_failed(self, fx, fy):
        now = self._elapsed_sec()
        kept = []
        blocked = False
        for item in self._failed_frontiers:
            if now - item['time'] > self._failed_goal_cooldown:
                continue
            kept.append(item)
            gx, gy = item['point']
            radius = float(item.get('radius', self._failed_goal_r))
            if math.hypot(fx - gx, fy - gy) <= radius:
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
def parse_robot_floats(raw) -> dict:
    """'robot2:0.8, robot3:0.5' -> {'robot2': 0.8, 'robot3': 0.5}; bad items skipped."""
    out = {}
    for item in str(raw or '').split(','):
        name, _, val = item.partition(':')
        try:
            out[name.strip()] = float(val)
        except ValueError:
            continue
    out.pop('', None)
    return out


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
    import faulthandler
    import signal
    faulthandler.register(signal.SIGUSR1, all_threads=True)   # `kill -USR1 <pid>` dumps every thread's stack (hang diagnosis)
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
