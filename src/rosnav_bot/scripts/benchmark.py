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
      -p inputs:="['~/rosnav_benchmarks/cafe_dwb.json','~/rosnav_benchmarks/cafe_mppi.json']"

All modes write <out_dir>/<label>_<mode>.json (out_dir default
~/rosnav_benchmarks). Report mode reads whatever paths you give it.
"""

import json
import math
import os
import time

import rclpy
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

        self._label = self.get_parameter('label').value
        self._out_dir = _out_dir(self.get_parameter('out_dir').value)
        self._goals = _load_goals(self.get_parameter('goals_file').value)
        odom_topic = self.get_parameter('odom_topic').value
        self._localizer = self.get_parameter('localizer').value

        self._odom_dist = 0.0
        self._last_odom_xy = None
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)

        self.get_logger().info(f'[nav] label={self._label} goals={len(self._goals)}')

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._last_odom_xy is not None:
            self._odom_dist += _dist(*self._last_odom_xy, x, y)
        self._last_odom_xy = (x, y)

    def run(self):
        from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

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

        for i, (gx, gy, gyaw_deg) in enumerate(self._goals):
            start_xy = self._last_odom_xy
            start_dist = self._odom_dist

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
            nav.goToPose(goal)
            num_recoveries = 0
            while not nav.isTaskComplete():
                fb = nav.getFeedback()
                if fb is not None:
                    num_recoveries = max(num_recoveries, getattr(fb, 'number_of_recoveries', 0))
                rclpy.spin_once(nav, timeout_sec=0.2)
            elapsed = time.time() - t0

            result = nav.getResult()
            success = result == TaskResult.SUCCEEDED
            actual_len = self._odom_dist - start_dist
            final_xy = self._last_odom_xy
            goal_error = _dist(final_xy[0], final_xy[1], gx, gy) if final_xy else None

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
            }
            results.append(run)
            self.get_logger().info(
                f"[nav] goal {i+1}/{len(self._goals)} "
                f"{'OK' if success else 'FAIL'} t={run['elapsed_sec']}s "
                f"speed={run['avg_speed_mps']}m/s err={run['goal_error_m']}m "
                f"recoveries={num_recoveries}")

        n_ok = sum(1 for r in results if r['success'])
        speeds = [r['avg_speed_mps'] for r in results if r['avg_speed_mps']]
        errors = [r['goal_error_m'] for r in results if r['success'] and r['goal_error_m'] is not None]
        report = {
            'goals_total': len(results),
            'goals_succeeded': n_ok,
            'success_rate': round(n_ok / len(results), 3) if results else 0.0,
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
def run_report(inputs):
    rows = []
    for path in inputs:
        p = os.path.expanduser(path)
        if not os.path.isfile(p):
            print(f'  (missing: {p})')
            continue
        with open(p) as f:
            rows.append(json.load(f))

    if not rows:
        print('No valid report files given.')
        return

    modes = {r['mode'] for r in rows}
    if len(modes) > 1:
        print(f'Warning: comparing different modes {modes} — fields may not align.\n')

    keys = set()
    for r in rows:
        keys.update(k for k, v in r.items() if not isinstance(v, (list, dict)))
    keys.discard('generated_at')
    ordered = ['label', 'mode'] + sorted(k for k in keys if k not in ('label', 'mode'))

    widths = {k: max(len(k), *(len(str(r.get(k, ''))) for r in rows)) for k in ordered}
    header = ' | '.join(k.ljust(widths[k]) for k in ordered)
    print(header)
    print('-' * len(header))
    for r in rows:
        print(' | '.join(str(r.get(k, '')).ljust(widths[k]) for k in ordered))


def main(args=None):
    rclpy.init(args=args)
    probe = Node('benchmark_probe')
    probe.declare_parameter('mode', 'nav')
    probe.declare_parameter('inputs', [''])
    mode = probe.get_parameter('mode').value
    inputs = probe.get_parameter('inputs').value
    probe.destroy_node()

    if mode == 'report':
        rclpy.shutdown()
        run_report([i for i in inputs if i])
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
