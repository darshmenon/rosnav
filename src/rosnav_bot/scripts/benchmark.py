#!/usr/bin/env python3
"""
benchmark.py — SLAM / navigation / localization benchmarking harness.

Point this at an already-running stack (`slam_nav.launch.py` or
`multi_robot.launch.py`) and it measures one of three things, writing a JSON
report you can later diff with `mode:=report`:

  slam          Map growth over time: coverage %, time-to-converge.
                Run against a live SLAM instance (slam:=true).
  nav           Goal-to-goal runs via Nav2: time-to-goal, planned vs actual
                path length (speed/efficiency), goal pose error (accuracy),
                recovery count. Needs Nav2 active. Default assumes AMCL
                (slam:=false); pass -p localizer:=none when running against
                a live SLAM instance instead (slam:=true).
  localization  AMCL pose-covariance trace over time, as an accuracy/
                confidence proxy (no ground truth available in sim without
                extra plumbing, so this is relative, not absolute error).
  accuracy      SLAM drift-correction stats over time (map->odom magnitude —
                works with zero setup). If a ground-truth pose topic is
                bridged (e.g. Gazebo's own pose for the robot model), also
                computes absolute position/yaw error, RMSE, and max error
                against the SLAM estimate (map->base_link), self-aligning on
                the first sample since ground truth and the map frame don't
                share an origin/heading in general. Same math as
                slam_accuracy_monitor.py, wrapped as a timed benchmark run.
  report        Offline: load 2+ JSON reports and print a comparison table
                (e.g. dwb.json vs mppi.json, or slam2d.json vs slam3d.json).
                Also writes comparison bar/line charts (PNG, needs matplotlib)
                and a single self-contained HTML dashboard next to them.
                Doesn't touch ROS.

Examples
────────
  ros2 run rosnav_bot benchmark.py --ros-args -p mode:=slam \\
      -p label:=cafe_2d -p duration_sec:=120 -p out_dir:=~/rosnav_benchmarks

  ros2 run rosnav_bot benchmark.py --ros-args -p mode:=nav \\
      -p label:=cafe_mppi -p goals_file:=src/rosnav_bot/config/waypoints.yaml \\
      -p localizer:=none   # omit (defaults to amcl) for slam:=false runs

  ros2 run rosnav_bot benchmark.py --ros-args -p mode:=localization \\
      -p label:=cafe_amcl -p duration_sec:=60

  # Drift-only, no ground truth needed:
  ros2 run rosnav_bot benchmark.py --ros-args -p mode:=accuracy \\
      -p label:=cafe_headless -p duration_sec:=120

  # With ground truth bridged as nav_msgs/Odometry or geometry_msgs/PoseStamped:
  ros2 run rosnav_bot benchmark.py --ros-args -p mode:=accuracy \\
      -p label:=cafe_headless -p duration_sec:=120 \\
      -p ground_truth_topic:=/test/ground_truth_pose -p ground_truth_type:=pose_stamped

  ros2 run rosnav_bot benchmark.py --ros-args -p mode:=report \\
      -p inputs:="['~/rosnav_benchmarks/cafe_dwb.json','~/rosnav_benchmarks/cafe_mppi.json']" \\
      -p out_html:=~/rosnav_benchmarks/controllers.html

All modes write <out_dir>/<label>_<mode>.json (out_dir default
~/rosnav_benchmarks). Report mode reads whatever paths you give it and
writes charts + an HTML dashboard next to the first input unless
-p out_dir:=... / -p out_html:=... override that; -p charts:=false skips
chart/dashboard generation and only prints the table.
"""

import base64
import json
import math
import os
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import tf2_ros
from rclpy.duration import Duration
from rclpy.time import Time

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False


def _out_dir(raw: str) -> str:
    d = os.path.expanduser(raw or '~/rosnav_benchmarks')
    os.makedirs(d, exist_ok=True)
    return d


def _dist(x1, y1, x2, y2) -> float:
    return math.hypot(x2 - x1, y2 - y1)


def _declare_numeric(node, name, default):
    # dynamic_typing so `-p duration_sec:=20` (parsed as int by the ROS CLI)
    # doesn't hard-fail against a float default — read side always float()s.
    node.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))


def _write_report(out_dir: str, label: str, mode: str, data: dict, logger):
    path = os.path.join(out_dir, f'{label}_{mode}.json')
    data['label'] = label
    data['mode'] = mode
    data['generated_at'] = time.time()
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)
    logger.info(f'Report written: {path}')
    return path


# ─────────────────────────── slam mode ──────────────────────────────────
class SlamBenchmark(Node):
    """Tracks /map growth over time: cell coverage % and convergence time."""

    def __init__(self):
        super().__init__('slam_benchmark')
        self.declare_parameter('label', 'run')
        _declare_numeric(self, 'duration_sec', 120.0)
        self.declare_parameter('out_dir', '~/rosnav_benchmarks')
        _declare_numeric(self, 'converge_threshold', 0.98)

        self._label = self.get_parameter('label').value
        self._duration = float(self.get_parameter('duration_sec').value)
        self._out_dir = _out_dir(self.get_parameter('out_dir').value)
        self._converge_threshold = float(self.get_parameter('converge_threshold').value)

        self._start = time.time()
        self._samples = []  # (t, free, occupied, unknown, total)
        self._converged_at = None
        self.done = False

        qos = QoSProfile(depth=5)
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, qos)

        self.get_logger().info(
            f'[slam] label={self._label} duration={self._duration}s — '
            f'waiting for /map updates...')
        self.create_timer(1.0, self._tick)

    def _map_cb(self, msg: OccupancyGrid):
        data = msg.data
        total = len(data)
        if total == 0:
            return
        unknown = sum(1 for c in data if c == -1)
        occupied = sum(1 for c in data if c >= 65)
        free = total - unknown - occupied
        t = time.time() - self._start
        self._samples.append((t, free, occupied, unknown, total))

        if self._converged_at is None and self._samples:
            final_known = free + occupied
            frac = final_known / total
            # Converged once known-cell fraction stops changing meaningfully
            # vs the last sample taken >5s ago.
            for pt, pfree, pocc, _pu, ptotal in reversed(self._samples[:-1]):
                if t - pt >= 5.0:
                    prev_frac = (pfree + pocc) / ptotal
                    if prev_frac > 0 and frac / max(prev_frac, 1e-6) < 1.0 + (1 - self._converge_threshold):
                        self._converged_at = t
                    break

    def _tick(self):
        elapsed = time.time() - self._start
        if self._samples:
            t, free, occ, unk, total = self._samples[-1]
            coverage = (free + occ) / total * 100.0
            self.get_logger().info(
                f'[slam] t={elapsed:5.1f}s coverage={coverage:5.1f}% '
                f'free={free} occ={occ} unknown={unk}')
        if elapsed >= self._duration:
            self._finish()

    def _finish(self):
        if not self._samples:
            self.get_logger().error('[slam] No /map messages received — is SLAM running?')
            self.done = True
            return
        t, free, occ, unk, total = self._samples[-1]
        report = {
            'duration_sec': self._duration,
            'samples': len(self._samples),
            'final_coverage_pct': round((free + occ) / total * 100.0, 2),
            'final_free_cells': free,
            'final_occupied_cells': occ,
            'final_unknown_cells': unk,
            'total_cells': total,
            'time_to_converge_sec': round(self._converged_at, 1) if self._converged_at else None,
            'converge_threshold': self._converge_threshold,
            'coverage_timeline': [
                {'t': round(s[0], 1), 'coverage_pct': round((s[1] + s[2]) / s[4] * 100.0, 2)}
                for s in self._samples
            ],
        }
        _write_report(self._out_dir, self._label, 'slam', report, self.get_logger())
        self.done = True


# ─────────────────────────── nav mode ───────────────────────────────────
def _load_goals(path: str):
    if not HAS_YAML or not path or not os.path.isfile(os.path.expanduser(path)):
        # Generic fallback square, matches waypoint_nav.py's default.
        return [(2.0, 0.0, 0.0), (2.0, 2.0, 90.0), (0.0, 2.0, 180.0), (0.0, 0.0, -90.0)]
    with open(os.path.expanduser(path)) as f:
        data = yaml.safe_load(f) or {}
    raw = data.get('waypoints', [])
    return [(float(w[0]), float(w[1]), float(w[2]) if len(w) > 2 else 0.0) for w in raw] or \
        [(2.0, 0.0, 0.0), (2.0, 2.0, 90.0), (0.0, 2.0, 180.0), (0.0, 0.0, -90.0)]


class NavBenchmark(Node):
    """Sends a sequence of Nav2 goals, timing + measuring each run."""

    def __init__(self):
        super().__init__('nav_benchmark')
        self.declare_parameter('label', 'run')
        self.declare_parameter('goals_file', '')
        self.declare_parameter('out_dir', '~/rosnav_benchmarks')
        self.declare_parameter('odom_topic', '/odom')
        # 'amcl' for nav-on-map (slam:=false, the default). Pass 'none' when
        # benchmarking nav performance against a live SLAM instance instead —
        # this repo's slam_toolbox isn't lifecycle-managed (no get_state
        # service), so waitUntilNav2Active(localizer='slam_toolbox') hangs;
        # 'none' waits on bt_navigator directly and skips the localizer check.
        self.declare_parameter('localizer', 'amcl')
        # Gazebo ground truth (world frame) is the arbiter for goal error: the
        # map frame starts at the spawn pose (AMCL initial pose = spawn), so
        # map<-world is a fixed rigid transform given the spawn pose. Defaults
        # are the cafe spawn from slam_nav.launch.py's SPAWN table.
        self.declare_parameter('gt_topic', '/ground_truth')
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', -2.0)
        self.declare_parameter('spawn_yaw', -1.57)
        # max measured goal error that still counts as a success
        self.declare_parameter('success_tol_m', 0.5)
        # max AMCL-vs-ground-truth start error before a trial is declared invalid
        self.declare_parameter('loc_tol_m', 0.5)

        self._label = self.get_parameter('label').value
        self._success_tol_m = float(self.get_parameter('success_tol_m').value)
        self._loc_tol_m = float(self.get_parameter('loc_tol_m').value)
        self._out_dir = _out_dir(self.get_parameter('out_dir').value)
        self._goals = _load_goals(self.get_parameter('goals_file').value)
        odom_topic = self.get_parameter('odom_topic').value
        self._localizer = self.get_parameter('localizer').value

        self._odom_dist = 0.0
        self._last_odom_xy = None
        # Goals are in the map frame, so goal error and the getPath start
        # pose must come from a map-frame pose (AMCL), not /odom: odom's
        # origin is wherever the robot spawned and only coincides with map
        # by luck, which made goal_error_m meaningless.
        self._map_xy = None
        amcl_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, amcl_qos)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        # /amcl_pose only republishes after update_min_d/a of motion, so it
        # can lag the true final pose by ~0.25 m; map->base_link TF is live.
        self._tf_buffer = tf2_ros.Buffer(node=self)
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._gt_map_xy = None
        self._spawn = (self.get_parameter('spawn_x').value,
                       self.get_parameter('spawn_y').value,
                       self.get_parameter('spawn_yaw').value)
        self.create_subscription(
            Odometry, self.get_parameter('gt_topic').value, self._gt_cb, 10)

        self.get_logger().info(f'[nav] label={self._label} goals={len(self._goals)}')

    def _gt_cb(self, msg: Odometry):
        sx, sy, syaw = self._spawn
        dx = msg.pose.pose.position.x - sx
        dy = msg.pose.pose.position.y - sy
        self._gt_map_xy = (dx * math.cos(syaw) + dy * math.sin(syaw),
                           -dx * math.sin(syaw) + dy * math.cos(syaw))

    def _map_pose_xy(self):
        """Latest map-frame (x, y): TF map->base_link, else /amcl_pose, else None."""
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_link', Time(), timeout=Duration(seconds=0.2)).transform.translation
            return (t.x, t.y)
        except Exception:
            return self._map_xy

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        self._map_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._last_odom_xy is not None:
            self._odom_dist += _dist(*self._last_odom_xy, x, y)
        self._last_odom_xy = (x, y)

    def run(self):
        from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

        # Our own executor for this node's /odom subscription, kept separate
        # from BasicNavigator's internal spinning. rclpy.spin_once(node) with
        # no explicit executor falls back to a process-wide *shared* global
        # executor (rclpy.get_global_executor()) — and BasicNavigator's own
        # methods (isTaskComplete, getFeedback, ...) already spin themselves
        # via that same implicit global executor internally. Alternating
        # rclpy.spin_once(nav, ...) / rclpy.spin_once(self, ...) both hit that
        # one shared executor and never reliably drained this node's own
        # queued /odom messages — _odom_dist/_last_odom_xy stayed frozen at
        # their initial values, so every distance/error metric came back
        # 0/null. A dedicated executor for `self` sidesteps that entirely.
        own_executor = rclpy.executors.SingleThreadedExecutor()
        own_executor.add_node(self)

        nav = BasicNavigator()
        if self._localizer in ('', 'none'):
            # slam_toolbox in this repo's launch isn't lifecycle-managed
            # (no <node>/get_state service) — waitUntilNav2Active() would
            # hang forever waiting on it. Wait for bt_navigator directly.
            nav._waitForNodeToActivate('bt_navigator')
            nav.info('Nav2 is ready for use!')
        else:
            nav.waitUntilNav2Active(localizer=self._localizer)
        results = []
        # Spin our own subscriptions (odom/amcl/tf/ground truth) before the
        # first goal; otherwise every start pose below is still None.
        t_spin = time.time()
        while time.time() - t_spin < 2.0:
            own_executor.spin_once(timeout_sec=0.1)

        for i, (gx, gy, gyaw_deg) in enumerate(self._goals):
            own_executor.spin_once(timeout_sec=0.1)
            start_xy = self._map_pose_xy() or self._last_odom_xy
            start_dist = self._odom_dist
            # AMCL can converge on a wrong pose; Nav2 then believes it is already
            # at (or far from) the goal and returns instant SUCCEEDED/FAIL. Compare
            # the map-frame TF pose with Gazebo ground truth before each goal and
            # mark the trial invalid (not a controller result) if they disagree.
            start_map = self._map_pose_xy()
            start_gt = self._gt_map_xy
            loc_err = (_dist(start_map[0], start_map[1], start_gt[0], start_gt[1])
                       if start_map and start_gt else None)
            if loc_err is not None and loc_err > self._loc_tol_m:
                self.get_logger().warn(
                    f'[nav] goal {i+1}: INVALID start — AMCL pose is {loc_err:.2f} m from '
                    f'ground truth; skipping (rerun this trial)')
                results.append({
                    'goal_index': i, 'goal': {'x': gx, 'y': gy, 'yaw_deg': gyaw_deg},
                    'success': False, 'invalid': True,
                    'invalid_reason': f'start localization error {loc_err:.2f} m',
                    'start_localization_error_m': round(loc_err, 3),
                    'elapsed_sec': 0.0, 'num_recoveries': 0, 'goal_error_m': None,
                    'actual_path_len_m': 0.0, 'planned_path_len_m': None,
                    'avg_speed_mps': None, 'path_efficiency': None,
                    'goal_error_frame': 'gt'})
                continue

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = nav.get_clock().now().to_msg()
            goal.pose.position.x = gx
            goal.pose.position.y = gy
            yaw = math.radians(gyaw_deg)
            goal.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.orientation.w = math.cos(yaw / 2.0)

            planned_len = None
            try:
                start_pose = PoseStamped()
                start_pose.header.frame_id = 'map'
                start_pose.header.stamp = nav.get_clock().now().to_msg()
                if start_xy:
                    start_pose.pose.position.x, start_pose.pose.position.y = start_xy
                    start_pose.pose.orientation.w = 1.0
                    path = nav.getPath(start_pose, goal)
                    if path and path.poses:
                        planned_len = sum(
                            _dist(path.poses[k].pose.position.x, path.poses[k].pose.position.y,
                                  path.poses[k + 1].pose.position.x, path.poses[k + 1].pose.position.y)
                            for k in range(len(path.poses) - 1))
            except Exception as exc:
                self.get_logger().warn(f'[nav] getPath failed: {exc}')

            t0 = time.time()
            accepted = nav.goToPose(goal)
            num_recoveries = 0
            track_len, track_last, track_t = 0.0, start_xy, 0.0
            gt_len, gt_last = 0.0, self._gt_map_xy
            track = []  # per-0.5s [t, map_xy, gt_map_xy, odom_xy] for post-hoc diagnosis
            while not nav.isTaskComplete():
                if time.time() - track_t > 0.5:
                    track_t = time.time()
                    cur = self._map_pose_xy()
                    if cur and track_last:
                        track_len += _dist(track_last[0], track_last[1], cur[0], cur[1])
                    track_last = cur or track_last
                    track.append([round(time.time() - t0, 2),
                                  [round(v, 3) for v in cur] if cur else None,
                                  [round(v, 3) for v in self._gt_map_xy] if self._gt_map_xy else None,
                                  [round(v, 3) for v in self._last_odom_xy] if self._last_odom_xy else None])
                    g = self._gt_map_xy
                    if g and gt_last:
                        gt_len += _dist(gt_last[0], gt_last[1], g[0], g[1])
                    gt_last = g or gt_last
                fb = nav.getFeedback()
                if fb is not None:
                    num_recoveries = max(num_recoveries, getattr(fb, 'number_of_recoveries', 0))
                rclpy.spin_once(nav, timeout_sec=0.1)
                own_executor.spin_once(timeout_sec=0.05)
            elapsed = time.time() - t0

            result = nav.getResult()
            # Nav2's own verdict, then cross-checked against measured error below.
            nav_success = accepted is not False and result == TaskResult.SUCCEEDED
            success = nav_success
            actual_len = self._odom_dist - start_dist
            own_executor.spin_once(timeout_sec=0.2)  # drain a final /amcl_pose
            map_xy = self._map_pose_xy()
            # Primary error = Gazebo ground truth (frame 'gt'). The map->base_link
            # TF is only valid up to the last AMCL update (it publishes future-
            # dated map->odom, so a latest-time lookup lags the robot by up to
            # ~update_min_d), which showed up as 0.3-0.5 m phantom goal error.
            if self._gt_map_xy:
                error_frame = 'gt'
                final_xy = self._gt_map_xy
            elif map_xy:
                error_frame = 'map'
                final_xy = map_xy
            else:
                error_frame = 'odom'
                final_xy = self._last_odom_xy
                self.get_logger().warn(
                    '[nav] no ground truth or /amcl_pose — goal_error_m falls back to '
                    '/odom and is only valid if odom==map')
            goal_error = _dist(final_xy[0], final_xy[1], gx, gy) if final_xy else None
            # A rejected/instant goal reports SUCCEEDED with the robot nowhere near
            # the target (seen: t=0.06s, speed 0, 3.56 m error). Require the measured
            # error to be within a loose bound (>= Nav2's 0.25 m xy tolerance plus
            # localisation noise) before counting a success.
            if success and goal_error is not None and goal_error > self._success_tol_m:
                success = False

            run = {
                'goal_index': i,
                'goal': {'x': gx, 'y': gy, 'yaw_deg': gyaw_deg},
                'success': success,
                'elapsed_sec': round(elapsed, 2),
                'planned_path_len_m': round(planned_len, 3) if planned_len else None,
                'actual_path_len_m': round(actual_len, 3),
                'path_efficiency': (
                    round(planned_len / actual_len, 3)
                    if planned_len and actual_len > 0.01 else None),
                'avg_speed_mps': round(actual_len / elapsed, 3) if elapsed > 0 else None,
                'num_recoveries': num_recoveries,
                'goal_error_m': round(goal_error, 3) if goal_error is not None else None,
                'goal_error_frame': error_frame,
                'goal_accepted': accepted is not False,
                'start_localization_error_m': round(loc_err, 3) if loc_err is not None else None,
                'nav2_reported_success': nav_success,
                'map_path_len_m': round(track_len, 3),
                'gt_path_len_m': round(gt_len, 3),
                'track': track,
                'goal_error_map_tf_m': (round(_dist(map_xy[0], map_xy[1], gx, gy), 3)
                                        if map_xy else None),
                'straight_line_m': round(_dist(start_xy[0], start_xy[1], gx, gy), 3) if start_xy else None,
            }
            results.append(run)
            self.get_logger().info(
                f"[nav] goal {i+1}/{len(self._goals)} "
                f"{'OK' if success else 'FAIL'} t={run['elapsed_sec']}s "
                f"speed={run['avg_speed_mps']}m/s err={run['goal_error_m']}m "
                f"recoveries={num_recoveries}")

        n_invalid = sum(1 for r in results if r.get('invalid'))
        valid = [r for r in results if not r.get('invalid')]
        n_ok = sum(1 for r in valid if r['success'])
        speeds = [r['avg_speed_mps'] for r in results if r['avg_speed_mps']]
        errors = [r['goal_error_m'] for r in results if r['success'] and r['goal_error_m'] is not None]
        report = {
            'goals_total': len(valid),
            'goals_invalid': n_invalid,  # AMCL mislocalised at start; excluded from the rate
            'goals_succeeded': n_ok,
            'success_rate': round(n_ok / len(valid), 3) if valid else None,
            'avg_speed_mps': round(sum(speeds) / len(speeds), 3) if speeds else None,
            'avg_goal_error_m': round(sum(errors) / len(errors), 3) if errors else None,
            'total_recoveries': sum(r['num_recoveries'] for r in results),
            'runs': results,
        }
        _write_report(self._out_dir, self._label, 'nav', report, self.get_logger())


# ─────────────────────── localization mode ──────────────────────────────
class LocalizationBenchmark(Node):
    """Tracks AMCL pose-covariance trace over time as an accuracy proxy."""

    def __init__(self):
        super().__init__('localization_benchmark')
        self.declare_parameter('label', 'run')
        _declare_numeric(self, 'duration_sec', 60.0)
        self.declare_parameter('out_dir', '~/rosnav_benchmarks')
        self.declare_parameter('pose_topic', '/amcl_pose')

        self._label = self.get_parameter('label').value
        self._duration = float(self.get_parameter('duration_sec').value)
        self._out_dir = _out_dir(self.get_parameter('out_dir').value)
        pose_topic = self.get_parameter('pose_topic').value

        self._start = time.time()
        self._samples = []  # (t, x, y, cov_trace)
        self.done = False
        self.create_subscription(
            PoseWithCovarianceStamped, pose_topic, self._pose_cb, 10)

        self.get_logger().info(
            f'[localization] label={self._label} topic={pose_topic} '
            f'duration={self._duration}s')
        self.create_timer(1.0, self._tick)

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        cov = msg.pose.covariance
        # trace of the x,y,yaw block (indices 0,7,35 in the 6x6 row-major cov)
        trace = cov[0] + cov[7] + cov[35]
        self._samples.append((time.time() - self._start, x, y, trace))

    def _tick(self):
        elapsed = time.time() - self._start
        if self._samples:
            t, x, y, trace = self._samples[-1]
            self.get_logger().info(f'[localization] t={elapsed:5.1f}s cov_trace={trace:.5f}')
        if elapsed >= self._duration:
            self._finish()

    def _finish(self):
        if not self._samples:
            self.get_logger().error(
                '[localization] No pose messages received — is AMCL running '
                'and has an initial pose been set?')
            self.done = True
            return
        traces = [s[3] for s in self._samples]
        jumps = [
            _dist(self._samples[i][1], self._samples[i][2],
                  self._samples[i + 1][1], self._samples[i + 1][2])
            for i in range(len(self._samples) - 1)
        ]
        report = {
            'duration_sec': self._duration,
            'samples': len(self._samples),
            'avg_cov_trace': round(sum(traces) / len(traces), 6),
            'max_cov_trace': round(max(traces), 6),
            'final_cov_trace': round(traces[-1], 6),
            'max_pose_jump_m': round(max(jumps), 3) if jumps else None,
            'note': ('cov_trace is x+y+yaw covariance diagonal sum from AMCL — '
                     'a confidence proxy, not ground-truth position error.'),
        }
        _write_report(self._out_dir, self._label, 'localization', report, self.get_logger())
        self.done = True


# ─────────────────────── accuracy mode ───────────────────────────────────
def _yaw_deg(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def _wrap_deg(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


class AccuracyBenchmark(Node):
    """SLAM drift-correction stats over time; absolute error/RMSE too if a
    ground-truth pose topic is bridged. Same math as
    slam_accuracy_monitor.py, wrapped as a timed benchmark run."""

    def __init__(self):
        super().__init__('accuracy_benchmark')
        self.declare_parameter('label', 'run')
        _declare_numeric(self, 'duration_sec', 120.0)
        self.declare_parameter('out_dir', '~/rosnav_benchmarks')
        self.declare_parameter('fixed_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ground_truth_topic', '')
        self.declare_parameter('ground_truth_type', 'odometry')

        self._label = self.get_parameter('label').value
        self._duration = float(self.get_parameter('duration_sec').value)
        self._out_dir = _out_dir(self.get_parameter('out_dir').value)
        self._fixed_frame = self.get_parameter('fixed_frame').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value

        self._tf_buffer = tf2_ros.Buffer(node=self)
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        gt_topic = str(self.get_parameter('ground_truth_topic').value).strip()
        gt_type = str(self.get_parameter('ground_truth_type').value).strip().lower()
        self._gt_enabled = bool(gt_topic)
        self._gt_xyz_yaw = None
        if self._gt_enabled:
            if gt_type not in ('odometry', 'pose_stamped'):
                raise SystemExit(
                    f"ground_truth_type must be odometry|pose_stamped, got {gt_type!r}")
            if gt_type == 'odometry':
                self.create_subscription(Odometry, gt_topic, self._on_gt_odom, 10)
            else:
                self.create_subscription(PoseStamped, gt_topic, self._on_gt_pose, 10)

        self._aligned = False
        self._align_dx = self._align_dy = self._align_dyaw = 0.0
        self._acc_samples = []  # (t, pos_err, yaw_err)

        self._start = time.time()
        self._drift_samples = []  # (t, dx, dy, dyaw)
        self.done = False
        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f'[accuracy] label={self._label} duration={self._duration}s '
            + (f'ground_truth={gt_topic} ({gt_type})' if self._gt_enabled
               else 'ground_truth=off (drift-only)'))

    def _on_gt_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._gt_xyz_yaw = (p.x, p.y, _yaw_deg(msg.pose.pose.orientation))

    def _on_gt_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._gt_xyz_yaw = (p.x, p.y, _yaw_deg(msg.pose.orientation))

    def _lookup(self, target, source):
        try:
            return self._tf_buffer.lookup_transform(
                target, source, Time(), timeout=Duration(seconds=0.05))
        except Exception:
            return None

    def _tick(self):
        elapsed = time.time() - self._start
        map_odom = self._lookup(self._fixed_frame, self._odom_frame)
        map_base = self._lookup(self._fixed_frame, self._base_frame)

        if map_odom is not None:
            tr = map_odom.transform.translation
            yaw = _yaw_deg(map_odom.transform.rotation)
            self._drift_samples.append((elapsed, tr.x, tr.y, yaw))

        if self._gt_enabled and self._gt_xyz_yaw is not None and map_base is not None:
            gx, gy, gyaw = self._gt_xyz_yaw
            mb = map_base.transform.translation
            myaw = _yaw_deg(map_base.transform.rotation)
            if not self._aligned:
                self._align_dyaw = myaw - gyaw
                rad = math.radians(self._align_dyaw)
                cos_a, sin_a = math.cos(rad), math.sin(rad)
                self._align_dx = mb.x - (gx * cos_a - gy * sin_a)
                self._align_dy = mb.y - (gx * sin_a + gy * cos_a)
                self._aligned = True
            else:
                rad = math.radians(self._align_dyaw)
                cos_a, sin_a = math.cos(rad), math.sin(rad)
                ax = gx * cos_a - gy * sin_a + self._align_dx
                ay = gx * sin_a + gy * cos_a + self._align_dy
                ayaw = _wrap_deg(gyaw + self._align_dyaw)
                pos_err = math.hypot(mb.x - ax, mb.y - ay)
                yaw_err = _wrap_deg(myaw - ayaw)
                self._acc_samples.append((elapsed, pos_err, yaw_err))

        if self._drift_samples:
            t, dx, dy, dyaw = self._drift_samples[-1]
            msg = f'[accuracy] t={elapsed:5.1f}s drift=({dx:.2f},{dy:.2f}) yaw={dyaw:.1f}deg'
            if self._acc_samples:
                pos_err, yaw_err = self._acc_samples[-1][1], self._acc_samples[-1][2]
                msg += f' pos_err={pos_err:.3f}m yaw_err={yaw_err:.1f}deg'
            self.get_logger().info(msg)

        if elapsed >= self._duration:
            self._finish()

    def _finish(self):
        if not self._drift_samples:
            self.get_logger().error('[accuracy] No map->odom TF available — is SLAM running?')
            self.done = True
            return
        drift_mags = [math.hypot(dx, dy) for _, dx, dy, _ in self._drift_samples]
        report = {
            'duration_sec': self._duration,
            'samples': len(self._drift_samples),
            'final_drift_m': round(drift_mags[-1], 4),
            'max_drift_m': round(max(drift_mags), 4),
            'final_drift_yaw_deg': round(self._drift_samples[-1][3], 2),
            'ground_truth_enabled': self._gt_enabled,
        }
        if self._acc_samples:
            pos_errs = [e[1] for e in self._acc_samples]
            yaw_errs = [e[2] for e in self._acc_samples]
            rmse = math.sqrt(sum(e * e for e in pos_errs) / len(pos_errs))
            report.update({
                'final_pos_err_m': round(pos_errs[-1], 4),
                'final_yaw_err_deg': round(yaw_errs[-1], 2),
                'rmse_pos_err_m': round(rmse, 4),
                'max_pos_err_m': round(max(pos_errs), 4),
                'accuracy_samples': len(self._acc_samples),
            })
        elif self._gt_enabled:
            report['note'] = 'ground_truth_topic set but no matching messages received'
        _write_report(self._out_dir, self._label, 'accuracy', report, self.get_logger())
        self.done = True


# ─────────────────────────── report mode ─────────────────────────────────
def _report_table_keys(rows):
    keys = set()
    for r in rows:
        keys.update(k for k, v in r.items() if not isinstance(v, (list, dict)))
    keys.discard('generated_at')
    return ['label', 'mode'] + sorted(k for k in keys if k not in ('label', 'mode'))


def _print_table(rows, ordered):
    widths = {k: max(len(k), *(len(str(r.get(k, ''))) for r in rows)) for k in ordered}
    header = ' | '.join(k.ljust(widths[k]) for k in ordered)
    print(header)
    print('-' * len(header))
    for r in rows:
        print(' | '.join(str(r.get(k, '')).ljust(widths[k]) for k in ordered))


def _try_import_pyplot():
    try:
        import matplotlib
        matplotlib.use('Agg')  # headless — report mode never opens a display
        import matplotlib.pyplot as plt
        return plt
    except ImportError:
        return None


def _numeric_bar_chart(plt, rows, keys, title, out_path):
    """One grouped bar chart: one x-tick per key, one bar per report (label)."""
    keys = [k for k in keys if any(isinstance(r.get(k), (int, float)) for r in rows)]
    if not keys:
        return False
    width = 0.8 / max(len(rows), 1)
    x = range(len(keys))
    fig, ax = plt.subplots(figsize=(max(6, len(keys) * 1.8), 4.5))
    for i, r in enumerate(rows):
        vals = [r.get(k) if isinstance(r.get(k), (int, float)) else 0 for k in keys]
        ax.bar([xi + i * width for xi in x], vals, width=width, label=r.get('label', f'run{i}'))
    ax.set_xticks([xi + width * (len(rows) - 1) / 2 for xi in x])
    ax.set_xticklabels(keys, rotation=20, ha='right')
    ax.set_title(title)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    return True


def _timeline_chart(plt, rows, title, out_path):
    fig, ax = plt.subplots(figsize=(7, 4.5))
    plotted = False
    for r in rows:
        timeline = r.get('coverage_timeline') or []
        if not timeline:
            continue
        ax.plot([p['t'] for p in timeline], [p['coverage_pct'] for p in timeline],
                 label=r.get('label', '?'))
        plotted = True
    if not plotted:
        plt.close(fig)
        return False
    ax.set_xlabel('time (s)')
    ax.set_ylabel('coverage %')
    ax.set_title(title)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)
    return True


def plot_reports(rows, out_dir):
    """Bar/line comparison charts per benchmark mode present in `rows`.
    Returns [(title, png_path), ...]; [] (with a log line) if matplotlib
    isn't installed — charts are a soft dependency, table output still works."""
    plt = _try_import_pyplot()
    if plt is None:
        print('(matplotlib not installed — skipping charts; `pip install matplotlib` to enable)')
        return []

    by_mode = {}
    for r in rows:
        by_mode.setdefault(r.get('mode'), []).append(r)

    charts = []
    if 'nav' in by_mode:
        path = os.path.join(out_dir, 'compare_nav.png')
        if _numeric_bar_chart(plt, by_mode['nav'],
                               ['avg_speed_mps', 'avg_goal_error_m', 'success_rate', 'total_recoveries'],
                               'Controller comparison (nav)', path):
            charts.append(('Controller comparison', path))
    if 'localization' in by_mode:
        path = os.path.join(out_dir, 'compare_localization.png')
        if _numeric_bar_chart(plt, by_mode['localization'],
                               ['avg_cov_trace', 'max_cov_trace', 'max_pose_jump_m'],
                               'Localization filter comparison', path):
            charts.append(('Localization comparison', path))
    if 'accuracy' in by_mode:
        keys = ['final_drift_m', 'max_drift_m']
        if any('rmse_pos_err_m' in r for r in by_mode['accuracy']):
            keys += ['rmse_pos_err_m', 'max_pos_err_m']
        path = os.path.join(out_dir, 'compare_accuracy.png')
        if _numeric_bar_chart(plt, by_mode['accuracy'], keys,
                               'Accuracy / drift comparison', path):
            charts.append(('Accuracy comparison', path))
    if 'slam' in by_mode:
        bar_path = os.path.join(out_dir, 'compare_slam_final.png')
        if _numeric_bar_chart(plt, by_mode['slam'], ['final_coverage_pct', 'time_to_converge_sec'],
                               'SLAM method comparison (final)', bar_path):
            charts.append(('SLAM final stats', bar_path))
        line_path = os.path.join(out_dir, 'compare_slam_timeline.png')
        if _timeline_chart(plt, by_mode['slam'], 'SLAM coverage over time', line_path):
            charts.append(('SLAM coverage over time', line_path))
    return charts


def write_html_dashboard(rows, ordered, charts, out_path):
    def esc(s):
        return str(s).replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')

    thead = ''.join(f'<th>{esc(k)}</th>' for k in ordered)
    tbody = ''.join(
        '<tr>' + ''.join(f'<td>{esc(r.get(k, ""))}</td>' for k in ordered) + '</tr>'
        for r in rows)

    imgs = []
    for title, path in charts:
        if not os.path.isfile(path):
            continue
        with open(path, 'rb') as f:
            b64 = base64.b64encode(f.read()).decode('ascii')
        imgs.append(f'<h2>{esc(title)}</h2><img src="data:image/png;base64,{b64}">')

    html = f"""<!doctype html>
<html><head><meta charset="utf-8"><title>rosnav_bot benchmark comparison</title>
<style>
body {{ font-family: sans-serif; margin: 2rem; background: #1e1e1e; color: #eee; }}
table {{ border-collapse: collapse; margin-bottom: 2rem; }}
th, td {{ border: 1px solid #555; padding: 4px 10px; text-align: right; font-size: 0.85rem; }}
th:first-child, td:first-child {{ text-align: left; }}
img {{ max-width: 100%; background: #fff; border-radius: 6px; margin-bottom: 2rem; }}
</style></head><body>
<h1>rosnav_bot benchmark comparison</h1>
<table><thead><tr>{thead}</tr></thead><tbody>{tbody}</tbody></table>
{''.join(imgs)}
</body></html>"""
    with open(out_path, 'w') as f:
        f.write(html)
    print(f'HTML dashboard written: {out_path}')


def run_report(inputs, out_dir=None, want_charts=True, out_html=None):
    rows = []
    first_dir = None
    for path in inputs:
        p = os.path.expanduser(path)
        if not os.path.isfile(p):
            print(f'  (missing: {p})')
            continue
        first_dir = first_dir or os.path.dirname(p)
        with open(p) as f:
            rows.append(json.load(f))

    if not rows:
        print('No valid report files given.')
        return

    modes = {r['mode'] for r in rows}
    if len(modes) > 1:
        print(f'Warning: comparing different modes {modes} — fields may not align.\n')

    ordered = _report_table_keys(rows)
    _print_table(rows, ordered)

    if not want_charts:
        return
    dest_dir = os.path.expanduser(out_dir) if out_dir else (first_dir or '.')
    os.makedirs(dest_dir, exist_ok=True)
    charts = plot_reports(rows, dest_dir)
    if charts:
        html_path = os.path.expanduser(out_html) if out_html else os.path.join(dest_dir, 'comparison.html')
        write_html_dashboard(rows, ordered, charts, html_path)


def main(args=None):
    rclpy.init(args=args)
    probe = Node('benchmark_probe')
    probe.declare_parameter('mode', 'nav')
    probe.declare_parameter('inputs', [''])
    probe.declare_parameter('out_dir', '')
    probe.declare_parameter('out_html', '')
    probe.declare_parameter('charts', True)
    mode = probe.get_parameter('mode').value
    inputs = probe.get_parameter('inputs').value
    report_out_dir = probe.get_parameter('out_dir').value
    report_out_html = probe.get_parameter('out_html').value
    report_charts = probe.get_parameter('charts').value
    probe.destroy_node()

    if mode == 'report':
        rclpy.shutdown()
        run_report([i for i in inputs if i], out_dir=report_out_dir or None,
                    want_charts=report_charts, out_html=report_out_html or None)
        return

    if mode in ('slam', 'localization', 'accuracy'):
        node = {'slam': SlamBenchmark, 'localization': LocalizationBenchmark,
                'accuracy': AccuracyBenchmark}[mode]()
        # rclpy.spin() doesn't reliably return after rclpy.shutdown() is
        # called from inside a timer callback (observed hang past report
        # write) — drive it manually and stop as soon as node.done flips.
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    elif mode == 'nav':
        node = NavBenchmark()
        node.run()
        node.destroy_node()
        rclpy.shutdown()
    else:
        print(f"Unknown mode {mode!r}. Use slam | nav | localization | accuracy | report.")
        rclpy.shutdown()


if __name__ == '__main__':
    main()
