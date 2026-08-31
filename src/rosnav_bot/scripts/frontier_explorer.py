#!/usr/bin/env python3
# frontier_explorer.py — WFD + weighted scoring + explore BT
"""
Frontier-based autonomous exploration.

Uses the same detector/scorer plugins as frontier_coordinator:
  frontier_detector: classic | wfd | rrt        (default wfd)
  frontier_scorer:   nearest | weighted | utility (default utility)

rrt detector:
  Sampling-based alternative to wfd's full-grid flood fill. Grows a
  random tree (RRT) through known-free space from the robot's position;
  any sampled step landing on a frontier cell is recorded as a cluster
  seed instead of extended further. Seeds are then flood-filled into
  full clusters exactly like wfd/classic, so scoring/goal-safety/BT
  logic downstream is unchanged — only *how frontiers are found* differs.
  Cheaper than wfd on large maps (no full-grid connected-component scan)
  at the cost of being probabilistic (may miss small/thin frontiers on
  a given cycle; self-corrects next poll).

utility scorer:
  score = gain_scale * size_m - potential_scale * distance
  (maximize frontier size, minimize travel distance).

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

import json
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


def _yaw_deg(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


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
        self.declare_parameter('min_viable_goal_distance', 0.30)
        self.declare_parameter('map_save_path', '')
        self.declare_parameter('max_frontier_retries', 2)
        self.declare_parameter('empty_frontier_max_wait', 15)
        self.declare_parameter('max_visited_relax_streak', 8)
        self.declare_parameter('goal_timeout', 60.0)
        self.declare_parameter('frontier_detector', 'wfd')
        self.declare_parameter('frontier_scorer', 'utility')
        self.declare_parameter('rrt_iterations', 300)
        self.declare_parameter('rrt_step_size', 0.5)
        self.declare_parameter('frontier_clearance_radius', 0.55)
        self.declare_parameter('distance_weight', 1.0)
        self.declare_parameter('info_gain_weight', 3.0)
        self.declare_parameter('potential_scale', 3.0)
        self.declare_parameter('gain_scale', 1.0)
        self.declare_parameter('hysteresis_radius', 2.0)
        self.declare_parameter('hysteresis_gain', 1.5)
        self.declare_parameter('suspicious_frontier_ratio', 3.0)
        self.declare_parameter('suspicious_frontier_penalty', 8.0)
        self.declare_parameter('suspicious_frontier_buffer', 1.0)
        self.declare_parameter('suspicious_frontier_hard_ratio', 6.0)
        self.declare_parameter('failed_frontier_min_radius', 0.75)
        self.declare_parameter('failed_frontier_max_radius', 3.0)
        self.declare_parameter('failed_frontier_radius_scale', 0.35)
        self.declare_parameter('failed_frontier_cooldown', 120.0)
        self.declare_parameter('local_minimum_timeout', 20.0)
        self.declare_parameter('local_minimum_min_time', 15.0)
        self.declare_parameter('local_minimum_min_translation', 0.20)
        self.declare_parameter('local_minimum_min_free_cell_growth', 20)
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
        # Flat [x1, y1, x2, y2, ...] polygon (map frame); < 3 points (incl.
        # the [0.0, 0.0] default) = unbounded. Frontier goals outside it are
        # rejected before scoring — keeps exploration inside a survey area
        # on a map that extends further.
        # NOTE: rclpy can't type-infer an *empty* double-array default or
        # override (both collapse to BYTE_ARRAY and raise
        # InvalidParameterTypeException) — the default must be non-empty to
        # lock in DOUBLE_ARRAY, so "disabled" is spelled [0.0, 0.0], not [].
        self.declare_parameter('exploration_boundary', [0.0, 0.0])
        # Checkpoint of visited frontiers, written periodically + on finish.
        # Empty path = disabled. resume_session=true loads it at startup so
        # a restarted explorer doesn't immediately re-walk already-covered
        # ground.
        self.declare_parameter('session_state_path', '')
        self.declare_parameter('resume_session', False)
        # 'ring' (default, unchanged): fixed-radius unknown-cell count around
        # the whole cluster, independent of vantage point/heading — cheap,
        # but overcounts cells the sensor wouldn't actually see from the
        # chosen goal. 'fov': cast a depth/angle-limited sensor cone from the
        # goal cell along the robot's approach heading, stopping at the
        # first occupied cell per ray (occlusion) — a more physically
        # grounded gain estimate. Opt-in: changing the default would shift
        # utility-scorer behavior and invalidate the explorer-backend
        # coverage numbers already recorded for 'ring'.
        self.declare_parameter('info_gain_mode', 'ring')
        self.declare_parameter('info_gain_fov', 1.04)
        self.declare_parameter('info_gain_max_depth', 2.0)
        self.declare_parameter('info_gain_angular_res', 0.10)

        self._min_size = self.get_parameter('min_frontier_size').value
        self._revisit_r = self.get_parameter('revisit_radius').value
        self._goal_frame = self.get_parameter('goal_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._min_goal_dist = self.get_parameter('min_goal_distance').value
        # Nav2's general_goal_checker xy_goal_tolerance (nav2_params*.yaml) is
        # 0.25m — any goal inside that radius of the robot's *current* pose
        # is reported "reached" instantly without the robot moving at all.
        # Cold-start relaxation must never hand out a goal closer than this,
        # or exploration orbits the same spot forever without the map ever
        # growing. Keep a small margin above 0.25m since the two configs
        # aren't wired together and can drift.
        self._min_viable_dist = float(self.get_parameter('min_viable_goal_distance').value)
        self._map_save_path = self.get_parameter('map_save_path').value.strip()
        self._max_retries = self.get_parameter('max_frontier_retries').value
        self._max_empty_wait = int(self.get_parameter('empty_frontier_max_wait').value)
        self._max_visited_relax_streak = int(
            self.get_parameter('max_visited_relax_streak').value)
        self._goal_timeout = self.get_parameter('goal_timeout').value
        self._map_topic = self.get_parameter('map_topic').value
        self._detector = self.get_parameter('frontier_detector').value.strip().lower()
        self._scorer = self.get_parameter('frontier_scorer').value.strip().lower()
        self._rrt_iterations = int(self.get_parameter('rrt_iterations').value)
        self._rrt_step = float(self.get_parameter('rrt_step_size').value)
        self._frontier_clearance = self.get_parameter('frontier_clearance_radius').value
        self._dist_w = self.get_parameter('distance_weight').value
        self._info_w = self.get_parameter('info_gain_weight').value
        self._potential_scale = self.get_parameter('potential_scale').value
        self._gain_scale = self.get_parameter('gain_scale').value
        self._hyst_r = self.get_parameter('hysteresis_radius').value
        self._hyst_gain = self.get_parameter('hysteresis_gain').value
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
        self._failed_frontier_cooldown = float(
            self.get_parameter('failed_frontier_cooldown').value)
        self._local_min_timeout = float(
            self.get_parameter('local_minimum_timeout').value)
        self._local_min_min_time = float(
            self.get_parameter('local_minimum_min_time').value)
        self._local_min_translation = float(
            self.get_parameter('local_minimum_min_translation').value)
        self._local_min_free_growth = int(
            self.get_parameter('local_minimum_min_free_cell_growth').value)
        self._bt_xml = _resolve_bt(self.get_parameter('behavior_tree').value)
        self._costmap_topic = self.get_parameter('costmap_topic').value.strip()
        self._validate_costmap = self.get_parameter('validate_on_costmap').value
        self._costmap_max_cost = int(self.get_parameter('costmap_max_cost').value)
        self._costmap_abort_cost = int(self.get_parameter('costmap_abort_cost').value)
        self._allow_unknown_costmap = self.get_parameter('allow_unknown_costmap').value
        self._goal_pullback = float(self.get_parameter('goal_pullback').value)
        self._goal_search_radius = float(self.get_parameter('goal_search_radius').value)
        self._boundary = self._parse_boundary(
            list(self.get_parameter('exploration_boundary').value))
        self._session_path = self.get_parameter('session_state_path').value.strip()
        self._resume_session = bool(self.get_parameter('resume_session').value)
        self._info_gain_mode = self.get_parameter('info_gain_mode').value.strip().lower()
        self._info_gain_fov = float(self.get_parameter('info_gain_fov').value)
        self._info_gain_max_depth = float(self.get_parameter('info_gain_max_depth').value)
        self._info_gain_angular_res = float(self.get_parameter('info_gain_angular_res').value)
        map_topic = self._map_topic
        action_name = self.get_parameter('action_name').value
        poll_period = self.get_parameter('poll_period').value

        if self._detector not in ('classic', 'wfd', 'rrt'):
            self.get_logger().warn(
                f'Unknown frontier_detector={self._detector!r}; using wfd.')
            self._detector = 'wfd'
        if self._scorer not in ('nearest', 'weighted', 'utility'):
            self.get_logger().warn(
                f'Unknown frontier_scorer={self._scorer!r}; using utility.')
            self._scorer = 'utility'
        if self._info_gain_mode not in ('ring', 'fov'):
            self.get_logger().warn(
                f'Unknown info_gain_mode={self._info_gain_mode!r}; using ring.')
            self._info_gain_mode = 'ring'
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
        self._current_frontier_meta: dict | None = None
        self._current_goal_handle = None
        self._ignore_next_result = False
        self._goal_sent_time: float = 0.0
        self._iteration = 0
        self._map_saved = False
        self._logged_init_wait = False
        self._empty_streak = 0
        self._last_map_update = 0.0
        self._last_goal_done = 0.0
        self._visited_relax_streak = 0
        self._failed_frontiers: list[dict] = []
        self._last_free_cells = None
        self._map_free_cells = None
        self._nav_progress_time: float = 0.0
        self._nav_progress_pose: tuple[float, float] | None = None
        self._nav_progress_free_cells: int | None = None
        self._last_growth_log = 0.0
        self._logged_costmap = False
        self._last_status_log = 0.0
        self._save_proc: subprocess.Popen | None = None

        if self._resume_session:
            self._load_session()

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
        rrt_note = (f' rrt_iters={self._rrt_iterations} rrt_step={self._rrt_step:.2f}m'
                    if self._detector == 'rrt' else '')
        boundary_note = (f' boundary={len(self._boundary)}pts' if self._boundary else '')
        session_note = (f' session={self._session_path}'
                        f'(resume={self._resume_session})' if self._session_path else '')
        self.get_logger().info(
            f'Ready | detector={self._detector} scorer={self._scorer} '
            f'bt={bt_note} costmap={cm_note} '
            f'potential={self._potential_scale} gain={self._gain_scale} '
            f'suspect_ratio={self._suspect_ratio:.2f} '
            f'suspect_penalty={self._suspect_penalty:.2f} '
            f'suspect_hard={self._suspect_hard_ratio:.2f} '
            f'costmap_max={self._costmap_max_cost} pullback={self._goal_pullback:.2f}m '
            f'clearance={self._frontier_clearance:.2f}m '
            f'local_min_timeout={self._local_min_timeout:.1f}s '
            f'min_size={self._min_size} revisit_r={self._revisit_r:.2f}m{rrt_note}'
            f'{boundary_note}{session_note}')
        self.get_logger().info('Waiting for map...')

        self.create_timer(poll_period, self._explore)

    # ------------------------------------------------------------------
    # Map / costmap callbacks
    # ------------------------------------------------------------------
    def _map_callback(self, msg: OccupancyGrid):
        self._map = msg
        self._last_map_update = time.monotonic()
        self._map_free_cells = int(np.count_nonzero(np.asarray(msg.data, dtype=np.int8) == 0))

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
        if self._save_proc is not None and self._save_proc.poll() is not None:
            self._save_proc = None
        if self._navigating:
            elapsed = time.monotonic() - self._goal_sent_time
            timed_out = elapsed > self._goal_timeout
            invalidated = (
                not timed_out and self._current_goal is not None
                and not self._costmap_allows(
                    *self._current_goal, max_cost=self._costmap_abort_cost)
            )
            local_minimum = (
                not timed_out and not invalidated
                and self._navigation_in_local_minimum(elapsed)
            )
            if timed_out or invalidated or local_minimum:
                if timed_out:
                    reason = f'exceeded {self._goal_timeout:.0f}s timeout'
                elif invalidated:
                    reason = 'costmap now lethal/inscribed at goal'
                else:
                    reason = (
                        f'made no map-growth/translation progress for '
                        f'{self._local_min_timeout:.0f}s')
                self.get_logger().warn(
                    f'Goal to ({self._current_goal[0]:.2f}, {self._current_goal[1]:.2f}) '
                    f'{reason} — cancelling and blacklisting as stuck.')
                if self._current_goal_handle is not None:
                    self._current_goal_handle.cancel_goal_async()
                    self._ignore_next_result = True
                if self._current_goal is not None:
                    self._register_failure(*self._current_goal, self._current_frontier_meta)
                self._navigating = False
                self._current_goal_handle = None
                self._current_frontier_meta = None
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
            if not self._confirm_empty('no frontier cells detected'):
                return
            self.get_logger().info('No frontiers — exploration complete.')
            self._save_map_once()
            return

        goal, pick = self._best_frontier(frontiers, pos)
        if goal is None:
            if pos is None:
                self.get_logger().debug('TF not ready yet, retrying...')
                return
            reason = (
                f'candidates={pick["n_total"]} visited={pick["n_visited"]} '
                f'costmap_block={pick["n_costmap"]} too_near={pick["n_near"]} '
                f'failed={pick.get("n_failed", 0)} '
                f'suspicious={pick.get("n_suspicious", 0)} '
                f'unreachable={pick.get("n_unreachable", 0)} scorer={self._scorer}')
            if not self._confirm_empty(reason):
                return
            self.get_logger().info(f'No usable frontier ({reason}) — exploration complete.')
            self._finish_exploration()
            return

        self._empty_streak = 0
        self._iteration += 1
        if self._map_save_path and self._iteration % 10 == 0:
            if self._save_proc is not None and self._save_proc.poll() is None:
                self.get_logger().warn(
                    'Previous progressive map-save still running — skipping '
                    'this cycle\'s autosave to avoid piling up map_saver_cli processes.')
            else:
                if self._save_proc is not None:
                    self._save_proc.wait()
                self.get_logger().info(
                    f'Progressively auto-saving map to {self._map_save_path} '
                    f'(topic={self._map_topic}) ...')
                self._save_proc = subprocess.Popen(
                    ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                     '-t', self._map_topic, '-f', self._map_save_path],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                )
        if self._session_path and self._iteration % 10 == 0:
            self._save_session_checkpoint()

        self.get_logger().info(
            f'#{self._iteration} → ({goal[0]:.2f}, {goal[1]:.2f}) '
            f'| scorer={self._scorer} score={pick["score"]:.2f} '
            f'dist={pick["distance"]:.2f}m size={pick["size_m"]:.2f}m '
            f'info={pick["info_gain"]:.2f}m² clear={pick["clearance"]:.2f}m '
            f'suspect={pick.get("suspicious_ratio", 0.0):.2f} '
            f'cost={self._costmap_value(*goal)} '
            f'| pool={pick["n_ok"]}/{pick["n_total"]} '
            f'(skip visited={pick["n_visited"]} costmap={pick["n_costmap"]} '
            f'near={pick["n_near"]} failed={pick.get("n_failed", 0)} '
            f'suspicious={pick.get("n_suspicious", 0)} '
            f'unreachable={pick.get("n_unreachable", 0)})')
        self._current_goal = goal
        self._current_frontier_meta = pick
        self._send_goal(*goal)

    def _confirm_empty(self, reason: str) -> bool:
        """Debounce the "no usable frontier" verdict until /map is fresh.

        Right after a goal completes, slam_toolbox's map_update_interval
        (commonly 5s) means /map may not have republished the newly-explored
        area yet — this poll can rediscover the exact same (now-visited)
        frontier cluster and look like completion when the robot has really
        just paused between hops. Require at least one map update newer than
        the last goal's completion before trusting "no usable frontier".
        A streak cap is a safety valve in case /map stops publishing.
        """
        self._empty_streak += 1
        map_is_fresh = self._last_map_update > self._last_goal_done
        if not map_is_fresh and self._empty_streak < self._max_empty_wait:
            self.get_logger().debug(
                f'No usable frontier ({reason}) but /map has not refreshed '
                f'since the last goal completed — waiting '
                f'(streak {self._empty_streak}/{self._max_empty_wait}).')
            return False
        return True

    # ------------------------------------------------------------------
    # Map saving and shutdown
    # ------------------------------------------------------------------
    def _finish_exploration(self):
        if self._save_proc is not None and self._save_proc.poll() is None:
            self.get_logger().info('Waiting for in-flight progressive autosave to finish...')
            self._save_proc.wait()
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

        if self._session_path:
            self.get_logger().info(f'Final session checkpoint to {self._session_path} ...')
            self._save_session_checkpoint()

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

        free_count = int(free.sum())
        unknown_count = int(unknown.sum())
        occupied_count = int(occupied.sum())
        total_cells = max(1, width * height)
        known_count = free_count + occupied_count
        explored_pct = 100.0 * known_count / total_cells
        now = time.monotonic()
        if now - self._last_growth_log >= 2.0:
            delta = '' if self._last_free_cells is None else f' (Δ{free_count - self._last_free_cells:+d})'
            robot_note = ''
            if robot_pos is not None:
                robot_note = f' robot=({robot_pos[0]:.2f},{robot_pos[1]:.2f})'
            tf_note = self._localization_summary()
            self.get_logger().info(
                f'/map free cells: {free_count}{delta} '
                f'occupied={occupied_count} unknown={unknown_count} '
                f'explored={explored_pct:.1f}% '
                f'({width}x{height} @ {res:.3f}m '
                f'origin=({ox:.2f},{oy:.2f}))'
                f'{robot_note}{tf_note}')
            self._last_free_cells = free_count
            self._last_growth_log = now

        mask = self._frontier_mask(free, unknown)
        raw_count = int(mask.sum())
        seed_cells = None
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
        elif self._detector == 'rrt':
            if robot_pos is None:
                return []
            seed_cells = self._rrt_seed_cells(free, mask, msg.info, robot_pos)
            if raw_count > 0 and not seed_cells:
                now = time.monotonic()
                if now - getattr(self, '_last_reach_warn', 0.0) >= 10.0:
                    self.get_logger().warn(
                        f'{raw_count} raw frontier cell(s) exist but {self._rrt_iterations} '
                        f'RRT samples from robot pos {robot_pos} found 0 — probabilistic '
                        f'miss, will resample next cycle.')
                    self._last_reach_warn = now

        if not mask.any():
            return []
        if seed_cells is not None and not seed_cells:
            return []

        candidate_cells = (
            seed_cells if seed_cells is not None
            else ((int(y), int(x)) for y, x in np.argwhere(mask)))

        centroids = []
        visited = np.zeros_like(mask, dtype=bool)
        for sy, sx in candidate_cells:
            sy, sx = int(sy), int(sx)
            if visited[sy, sx] or not mask[sy, sx]:
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
            point = (ox + (cx + 0.5) * res, oy + (cy + 0.5) * res)
            if not self._in_boundary(*point):
                continue
            if self._info_gain_mode == 'fov' and robot_pos is not None:
                heading = math.atan2(point[1] - robot_pos[1], point[0] - robot_pos[0])
                info_gain = self._info_gain_fov_cast(
                    (int(cy), int(cx)), heading, unknown, occupied, res)
            else:
                info_gain = self._info_gain(cluster, unknown, res)
            centroids.append({
                'point': point,
                'goal_cell': (int(cy), int(cx)),
                'size': len(cluster),
                'size_m': len(cluster) * res,
                'info_gain': info_gain,
                'clearance': clearance,
                'suspicious_ratio': self._suspicious_frontier_ratio(len(cluster) * res, clearance),
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

    def _rrt_seed_cells(self, free, mask, info, robot_pos):
        """RRT sampling: grow a random tree through free space from the
        robot's cell; any step that lands on a frontier cell is recorded
        as a cluster seed (not extended further). Returns a set of (y, x)
        seed cells that downstream flood-fill clusters exactly like the
        wfd/classic seeds — this only changes *how* seeds are found."""
        h, w = free.shape
        res = info.resolution
        step_cells = max(1, round(self._rrt_step / res))
        ry, rx = _world_to_cell(robot_pos[0], robot_pos[1], info)
        if not (0 <= ry < h and 0 <= rx < w and free[ry, rx]):
            seed = self._nearest_free_cell(ry, rx, free, res)
            if seed is None:
                return set()
            ry, rx = seed

        tree_y = [float(ry)]
        tree_x = [float(rx)]
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

        # Rank by proximity to this cluster first, clearance only as a
        # tiebreaker. Scoring by clearance first (as this used to) makes
        # every cluster's BFS converge on whichever cell in reach has the
        # best clearance workspace-wide — when the known-free region is
        # small (cold start, tight corridors) that's the *same* cell for
        # every cluster, so one visit falsely marks all of them "visited".
        # Picking the closest qualifying cell keeps each goal anchored to
        # the frontier that produced it.
        best = None
        best_key = None
        for unk_clear, occ_clear, dist, y, x in pool:
            key = (dist, -(unk_clear + occ_clear))
            if best_key is None or key < best_key:
                best_key = key
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

    def _info_gain_fov_cast(self, goal_cell, heading_rad, unknown, occupied, resolution):
        """info_gain_mode='fov': count unknown cells actually visible from
        goal_cell — a sensor cone of info_gain_fov width, out to
        info_gain_max_depth, centered on heading_rad (the approach direction
        robot_pos -> goal). Each ray stops at the first occupied cell
        (occlusion), so a frontier behind a wall from this vantage point
        doesn't inflate the score the way the fixed omnidirectional 'ring'
        mode can.
        """
        h, w = unknown.shape
        gy, gx = goal_cell
        depth_cells = max(1, math.ceil(self._info_gain_max_depth / resolution))
        n_rays = max(3, int(self._info_gain_fov / max(self._info_gain_angular_res, 1e-3)) + 1)
        half_fov = self._info_gain_fov / 2.0
        seen = set()
        for i in range(n_rays):
            theta = heading_rad - half_fov + (self._info_gain_fov * i / (n_rays - 1))
            dy, dx = math.sin(theta), math.cos(theta)
            for r in range(1, depth_cells + 1):
                cy = int(round(gy + dy * r))
                cx = int(round(gx + dx * r))
                if not (0 <= cy < h and 0 <= cx < w):
                    break
                if occupied[cy, cx]:
                    break
                if unknown[cy, cx]:
                    seen.add((cy, cx))
        return len(seen) * resolution * resolution

    # ------------------------------------------------------------------
    # Exploration boundary (optional survey-area polygon, map frame)
    # ------------------------------------------------------------------
    @staticmethod
    def _parse_boundary(flat: list) -> list:
        if not flat:
            return []
        if len(flat) % 2 != 0:
            flat = flat[:-1]
        pts = [(float(flat[i]), float(flat[i + 1])) for i in range(0, len(flat), 2)]
        return pts if len(pts) >= 3 else []

    def _in_boundary(self, x: float, y: float) -> bool:
        """Ray-casting point-in-polygon test; True (unbounded) if no boundary set."""
        if not self._boundary:
            return True
        inside = False
        n = len(self._boundary)
        x1, y1 = self._boundary[-1]
        for x2, y2 in self._boundary:
            if (y1 > y) != (y2 > y):
                x_cross = (x2 - x1) * (y - y1) / (y2 - y1) + x1
                if x < x_cross:
                    inside = not inside
            x1, y1 = x2, y2
        return inside

    # ------------------------------------------------------------------
    # Session checkpoint (visited-frontier list — save/resume)
    # ------------------------------------------------------------------
    def _load_session(self):
        if not self._session_path or not os.path.isfile(self._session_path):
            if self._session_path:
                self.get_logger().info(
                    f'resume_session=true but no checkpoint at {self._session_path} '
                    '— starting fresh.')
            return
        try:
            with open(self._session_path) as f:
                data = json.load(f)
            self._visited = [(float(x), float(y)) for x, y in data.get('visited', [])]
            self.get_logger().info(
                f'Resumed session from {self._session_path}: '
                f'{len(self._visited)} previously-visited frontier(s) loaded.')
        except Exception as exc:
            self.get_logger().warn(f'Failed to load session checkpoint: {exc}')

    def _save_session_checkpoint(self):
        if not self._session_path:
            return
        save_dir = os.path.dirname(os.path.expanduser(self._session_path))
        if save_dir:
            os.makedirs(save_dir, exist_ok=True)
        try:
            with open(os.path.expanduser(self._session_path), 'w') as f:
                json.dump({'visited': self._visited}, f)
        except Exception as exc:
            self.get_logger().warn(f'Failed to write session checkpoint: {exc}')

    # ------------------------------------------------------------------
    # Pick frontier (nearest | weighted | utility)
    # ------------------------------------------------------------------
    def _best_frontier(self, frontiers, pos):
        empty = {
            'n_total': len(frontiers), 'n_ok': 0, 'n_visited': 0,
            'n_costmap': 0, 'n_near': 0, 'n_unreachable': 0,
            'n_failed': 0, 'n_suspicious': 0, 'score': float('-inf'),
            'distance': 0.0, 'size_m': 0.0, 'info_gain': 0.0, 'clearance': 0.0,
        }
        if pos is None:
            return None, empty
        n_visited = n_costmap = n_near = n_ok = n_unreachable = n_failed = n_suspicious = 0
        far_candidates = []
        near_candidates = []
        visited_candidates = []  # (frontier, fx, fy) — already-visited but otherwise untried
        path_distances = self._path_distance_map(pos)
        for frontier in frontiers:
            fx, fy = frontier['point']
            if self._recently_failed(fx, fy):
                n_failed += 1
                continue
            if frontier.get('suspicious_ratio', 0.0) >= self._suspect_hard_ratio:
                n_suspicious += 1
                continue
            d = self._frontier_path_distance(frontier, path_distances)
            if not math.isfinite(d):
                n_unreachable += 1
                continue
            if d <= self._min_viable_dist:
                # Inside Nav2's own goal tolerance — sending this goal is a
                # guaranteed no-op (instant "reached" without moving), which
                # is how cold-start hops used to get stuck orbiting one spot.
                # Excluded outright, before the visited/costmap/near checks.
                n_unreachable += 1
                continue
            if self._already_visited(fx, fy):
                n_visited += 1
                visited_candidates.append((frontier, fx, fy))
                continue
            if not self._costmap_allows(fx, fy):
                n_costmap += 1
                continue
            if d < self._min_goal_dist:
                n_near += 1
                near_candidates.append((frontier, d))
                continue
            n_ok += 1
            far_candidates.append((frontier, d))

        # Cold start: right after spawn the known-free region is only as big
        # as the robot's immediate surroundings, so every frontier can fall
        # within min_goal_distance of the robot — there's nothing farther to
        # pick yet. Relax onto the near pool (same fallback pattern as the
        # goal_pullback relax in _place_safe_goal) instead of letting the
        # explorer conclude "exploration complete" before it has ever moved.
        pool, relaxed = (far_candidates, False) if far_candidates else (near_candidates, True)

        best, best_score, best_meta = None, float('-inf'), None
        for frontier, d in pool:
            score = self._score_frontier(frontier, d)
            if score > best_score:
                best_score = score
                best = frontier['point']
                best_meta = {
                    'n_total': len(frontiers),
                    'n_ok': n_ok,
                    'n_visited': n_visited,
                    'n_costmap': n_costmap,
                    'n_near': n_near,
                    'n_unreachable': n_unreachable,
                    'n_failed': n_failed,
                    'n_suspicious': n_suspicious,
                    'score': score,
                    'distance': d,
                    'size_m': frontier['size_m'],
                    'info_gain': frontier['info_gain'],
                    'clearance': frontier.get('clearance', 0.0),
                    'suspicious_ratio': frontier.get('suspicious_ratio', 0.0),
                }

        if best is None and visited_candidates:
            # Last-resort tier: every remaining frontier already looks
            # visited. This happens when the cold-start known-free bubble is
            # smaller than revisit_radius + min_goal_distance combined, so
            # any short hop lands within revisit range of everything nearby
            # — not because exploration is actually done. Break out by
            # retargeting whichever frontier sits farthest from the nearest
            # visited point, instead of concluding "complete" while
            # unexplored space almost certainly still lies behind it.
            #
            # Circuit breaker: if this tier keeps firing many times in a row
            # with no intervening normal pick, the map has stopped growing
            # (e.g. spawned in a pocket too small to escape) and retargeting
            # forever just spins the robot in place. Give up for real once
            # the streak is too long, instead of looping indefinitely.
            self._visited_relax_streak += 1
            if self._visited_relax_streak > self._max_visited_relax_streak:
                self.get_logger().warn(
                    f'Visited-relax fired {self._visited_relax_streak} times in a row '
                    f'with no real progress — the known-free region likely stopped '
                    f'growing (cold-start pocket too small to escape). Giving up instead '
                    f'of spinning in place forever.')
                return None, {**empty, 'n_ok': n_ok, 'n_visited': n_visited,
                              'n_costmap': n_costmap, 'n_near': n_near,
                              'n_unreachable': n_unreachable,
                              'n_failed': n_failed, 'n_suspicious': n_suspicious}

            def _dist_to_nearest_visited(fx, fy):
                if not self._visited:
                    return float('inf')
                return min(math.hypot(fx - vx, fy - vy) for vx, vy in self._visited)

            frontier, fx, fy = max(
                visited_candidates, key=lambda t: _dist_to_nearest_visited(t[1], t[2]))
            rx, ry = pos
            d = math.hypot(fx - rx, fy - ry)
            now = time.monotonic()
            if now - getattr(self, '_last_visited_relax_warn', 0.0) >= 10.0:
                self.get_logger().warn(
                    f'All {n_visited} frontier(s) already look visited (cold-start '
                    f'known-free region smaller than revisit_radius={self._revisit_r:.2f}m '
                    f'+ min_goal_distance={self._min_goal_dist:.2f}m) — retargeting the '
                    f'one farthest from any visited point instead of declaring complete '
                    f'(streak {self._visited_relax_streak}/{self._max_visited_relax_streak}).')
                self._last_visited_relax_warn = now
            return (fx, fy), {
                'n_total': len(frontiers), 'n_ok': n_ok, 'n_visited': n_visited,
                'n_costmap': n_costmap, 'n_near': n_near, 'n_unreachable': n_unreachable,
                'n_failed': n_failed, 'n_suspicious': n_suspicious,
                'score': self._score_frontier(frontier, d), 'distance': d,
                'size_m': frontier['size_m'], 'info_gain': frontier['info_gain'],
                'clearance': frontier.get('clearance', 0.0),
                'suspicious_ratio': frontier.get('suspicious_ratio', 0.0),
            }

        if best is None:
            return None, {**empty, 'n_ok': n_ok, 'n_visited': n_visited,
                          'n_costmap': n_costmap, 'n_near': n_near,
                          'n_unreachable': n_unreachable,
                          'n_failed': n_failed, 'n_suspicious': n_suspicious}

        self._visited_relax_streak = 0
        if relaxed:
            now = time.monotonic()
            if now - getattr(self, '_last_min_dist_relax_warn', 0.0) >= 10.0:
                self.get_logger().warn(
                    f'All {n_near} frontier(s) are within min_goal_distance='
                    f'{self._min_goal_dist:.2f}m of the robot (cold start / small '
                    f'known-free region) — relaxing to take a first short hop '
                    f'instead of declaring exploration complete.')
                self._last_min_dist_relax_warn = now
        return best, best_meta

    def _score_frontier(self, frontier, distance):
        suspicious_ratio = self._suspicious_frontier_ratio(
            frontier['size_m'], frontier.get('clearance', 0.0))
        suspicious_penalty = 0.0
        if self._scorer != 'nearest' and suspicious_ratio > self._suspect_ratio:
            suspicious_penalty = (
                suspicious_ratio - self._suspect_ratio) * self._suspect_penalty
        if self._scorer == 'utility':
            # minimize potential*dist - gain*size  →  maximize this
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
        return score - suspicious_penalty

    def _suspicious_frontier_ratio(self, size_m: float, clearance: float) -> float:
        """Large clusters with very small safe pullback are often wall-leak artifacts.

        Use the reachable free-space depth from the chosen goal cell as a proxy for
        how much of the cluster is likely real. Genuine frontiers can be large, but
        they usually also admit goals with meaningful clearance into the explored
        region; seam leaks tend to produce huge clusters while every reachable goal
        stays pinned close to the same thin wall opening.
        """
        depth = max(clearance + self._suspect_buffer, 1e-3)
        return size_m / depth

    def _path_distance_map(self, robot_pos):
        msg = self._map
        data = np.array(msg.data, dtype=np.int16).reshape((msg.info.height, msg.info.width))
        free = data == 0
        h, w = free.shape
        dist = np.full((h, w), np.inf, dtype=np.float32)
        seed = _world_to_cell(robot_pos[0], robot_pos[1], msg.info)
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

    def _localization_summary(self) -> str:
        parts = []
        for label, target, source in (
            ('map_odom', self._goal_frame, 'odom'),
            ('map_base', self._goal_frame, self._base_frame),
        ):
            try:
                tf = self._tf_buffer.lookup_transform(
                    target, source, rclpy.time.Time())
            except Exception:
                parts.append(f'{label}=missing')
                continue
            tr = tf.transform.translation
            yaw = _yaw_deg(tf.transform.rotation)
            parts.append(f'{label}=({tr.x:.2f},{tr.y:.2f},yaw={yaw:.1f}deg)')
        return ' ' + ' '.join(parts) if parts else ''

    # ------------------------------------------------------------------
    # Nav2 goal
    # ------------------------------------------------------------------
    def _send_goal(self, x: float, y: float):
        self._navigating = True
        self._reset_navigation_progress()
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

    def _reset_navigation_progress(self):
        now = time.monotonic()
        self._nav_progress_time = now
        self._nav_progress_pose = self._robot_position()
        self._nav_progress_free_cells = self._map_free_cells

    def _navigation_in_local_minimum(self, elapsed: float) -> bool:
        if self._local_min_timeout <= 0.0 or elapsed < self._local_min_min_time:
            return False

        now = time.monotonic()
        pos = self._robot_position()
        free_cells = self._map_free_cells
        progressed = self._navigation_made_progress(pos, free_cells)
        if progressed:
            self._nav_progress_time = now
            self._nav_progress_pose = pos
            self._nav_progress_free_cells = free_cells
            return False

        return now - self._nav_progress_time >= self._local_min_timeout

    def _navigation_made_progress(self, pos, free_cells) -> bool:
        if pos is not None and self._nav_progress_pose is not None:
            px, py = self._nav_progress_pose
            if math.hypot(pos[0] - px, pos[1] - py) >= self._local_min_translation:
                return True
        elif pos is not None and self._nav_progress_pose is None:
            return True

        if free_cells is not None and self._nav_progress_free_cells is not None:
            if free_cells - self._nav_progress_free_cells >= self._local_min_free_growth:
                return True
        elif free_cells is not None and self._nav_progress_free_cells is None:
            return True

        return False

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
            if self._current_goal is not None:
                self._register_failure(*self._current_goal, self._current_frontier_meta)
            self._current_frontier_meta = None
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
                    self._register_failure(*self._current_goal, self._current_frontier_meta)
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
                self._register_failure(*self._current_goal, self._current_frontier_meta)
        self._current_frontier_meta = None
        self._last_goal_done = time.monotonic()
        self._navigating = False

    def _register_failure(self, fx, fy, frontier_meta=None):
        """Track repeated failures at (roughly) the same frontier and
        blacklist it once it exceeds max_frontier_retries, so exploration
        doesn't retry an unreachable point forever."""
        for entry in self._fail_counts:
            if math.hypot(fx - entry[0], fy - entry[1]) < self._revisit_r:
                entry[2] += 1
                if entry[2] >= self._max_retries:
                    self._remember_failed_frontier((fx, fy), frontier_meta)
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

    def _remember_failed_frontier(self, goal, frontier_meta=None):
        now = time.monotonic()
        gx, gy = goal
        self._failed_frontiers = [
            f for f in self._failed_frontiers
            if now - f['time'] <= self._failed_frontier_cooldown
        ]
        size_m = 0.0 if frontier_meta is None else float(frontier_meta.get('size_m', 0.0))
        suspicious = 1.0 if frontier_meta is None else max(
            1.0, float(frontier_meta.get('suspicious_ratio', 1.0)) / max(self._suspect_ratio, 1e-3))
        radius = size_m * self._failed_frontier_radius_scale * min(suspicious, 2.0)
        radius = max(self._failed_frontier_min_radius,
                     min(self._failed_frontier_max_radius, radius))
        self._failed_frontiers.append({'point': goal, 'time': now, 'radius': radius})
        self.get_logger().warn(
            f'Suppressing failed frontier near ({gx:.2f}, {gy:.2f}) '
            f'for {self._failed_frontier_cooldown:.0f}s radius={radius:.2f}m')

    def _recently_failed(self, fx, fy):
        now = time.monotonic()
        kept = []
        blocked = False
        for item in self._failed_frontiers:
            if now - item['time'] > self._failed_frontier_cooldown:
                continue
            kept.append(item)
            gx, gy = item['point']
            if math.hypot(fx - gx, fy - gy) <= item['radius']:
                blocked = True
        self._failed_frontiers = kept
        return blocked


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
