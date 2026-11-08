#!/usr/bin/env python3
"""nav2_doctor - attach to a running Nav2 stack and say why the robot isn't moving.

Pure-logic `Doctor` (no ROS imports, unit-testable) + a thin rclpy `main()`.
"""
import argparse
import json
import time
from dataclasses import dataclass

__version__ = '0.4.0'

NAV2_NODES = ['controller_server', 'planner_server', 'bt_navigator',
              'behavior_server', 'smoother_server', 'waypoint_follower']
STALE_S = 2.0          # a stream is "stale" if silent this long
FLAP_WINDOW_S = 30.0   # goal-flapping look-back
STARTUP_S = 60.0       # grace period before inactive nodes count as errors
FLAP_COUNT = 4         # goals cancelled/aborted inside the window => flapping
PROGRESS_S = 15.0      # goal active this long with no movement => NO_PROGRESS
AMCL_COV_MAX = 0.5     # AMCL x/y covariance (m^2) above which localisation is uncertain
PROGRESS_M = 0.1       # minimum displacement (m) that counts as movement
DRIVE_S = 5.0          # cmd_vel non-zero this long with no odom movement => CMD_NO_MOTION
SKEW_S = 1.0           # TF stamp vs wall clock difference that counts as clock skew


HINTS = {
    'TF_NONE': 'check ROS_DOMAIN_ID/--ns; is robot_state_publisher or the driver running? try `ros2 topic list | grep tf`',
    'TF_FREEZE': 'check the TF publisher (sim paused? driver crashed?) with `ros2 run tf2_ros tf2_monitor`',
    'TF_EDGE_STALE': 'the named edge\'s publisher died; `ros2 run tf2_ros tf2_echo <parent> <child>` to confirm',
    'NO_MAP_ODOM': 'start AMCL/SLAM and publish /initialpose (RViz "2D Pose Estimate")',
    'ODOM_NONE': 'check the base driver/odom topic name and namespace',
    'ODOM_STALE': 'odometry source stopped; check the base driver or sim bridge',
    'SCAN_NONE': 'check the lidar driver/bridge and the scan topic name',
    'SCAN_STALE': 'lidar stopped; check the driver or sim bridge',
    'COSTMAP_STALE': 'fix TF/scan first; then `ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap {}`',
    'NODE_MISSING': 'launch the node or check lifecycle_manager node_names; `ros2 node list`',
    'NODE_STARTING': 'wait for startup to finish; if it persists the lifecycle_manager is stuck',
    'NODE_INACTIVE': 'see the lifecycle_manager log; usually a YAML param error. `ros2 lifecycle get /<node>`',
    'GOAL_FLAPPING': 'add hysteresis/min goal distance to the exploration or goal source',
    'NO_CMD_VEL': 'controller_server isn\'t producing output; check its log and that the goal checker/plugins loaded',
    'CMD_ZERO': 'planner or controller failed; check bt_navigator and controller_server logs, clear costmaps',
    'NO_PROGRESS': 'check progress_checker params and for an obstacle; see recovery behaviours in the BT log',
    'AMCL_UNCERTAIN': 'publish /initialpose in RViz, check the map matches the world, and that /scan is reaching AMCL',
    'NO_CMD_SUBSCRIBER': 'remap: `ros2 topic info /cmd_vel -v`; match the driver topic (cmd_vel_smoothed, velocity_smoother)',
    'CMD_NO_MOTION': 'check e-stop, motor enable and that the driver converts cmd_vel; `ros2 topic echo /odom`',
    'CLOCK_SKEW': 'sync clocks (chrony) or set use_sim_time consistently on every node',
    'INFLATION_SMALL': 'raise inflation_layer.inflation_radius above robot_radius (or footprint circumradius)',
    'RECOVERY_LOOP': 'find what keeps failing planning/control: read planner/controller logs, check costmap for phantom obstacles, loosen goal tolerance',
    'NO_NAV2_VISIBLE': 'match ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY, RMW_IMPLEMENTATION and --ns with the robot',
}


RECOVERY_WORDS = ('spin', 'backup', 'back_up', 'wait', 'clearentire', 'clear_entire', 'recovery')
RECOVERY_COUNT = 3     # recovery behaviours started inside the window => RECOVERY_LOOP
RECOVERY_WINDOW_S = 60.0


def inscribed_radius(footprint):
    """Smallest distance from the origin to the footprint polygon edges, from a Nav2
    string like '[[0.3,0.2],[0.3,-0.2],[-0.3,-0.2],[-0.3,0.2]]'. None if unparseable."""
    try:
        pts = [tuple(map(float, p)) for p in json.loads(footprint)]
    except (ValueError, TypeError):
        return None
    if len(pts) < 3 or any(len(p) != 2 for p in pts):
        return None
    best = None
    for (x1, y1), (x2, y2) in zip(pts, pts[1:] + pts[:1]):
        dx, dy = x2 - x1, y2 - y1
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, -(x1 * dx + y1 * dy) / L2))
        d = ((x1 + t * dx) ** 2 + (y1 + t * dy) ** 2) ** 0.5
        best = d if best is None else min(best, d)
    return best


@dataclass
class Finding:
    level: str   # ERROR | WARN | OK
    code: str
    msg: str


class Doctor:
    """Feed it events with `seen()` / `state()`; ask `diagnose(now)`."""

    def __init__(self, startup_s=STARTUP_S, stale_s=STALE_S, flap_window_s=FLAP_WINDOW_S,
                 flap_count=FLAP_COUNT, progress_s=PROGRESS_S, progress_m=PROGRESS_M):
        self.startup_s = startup_s
        self.stale_s = stale_s
        self.flap_window_s = flap_window_s
        self.flap_count = flap_count
        self.progress_s = progress_s
        self.progress_m = progress_m
        self.poses = []          # (time, x, y) odometry samples
        self.goal_since = None   # when the current goal became active
        self.last = {}           # stream name -> last receipt time
        self.tf_edges = {}       # (parent, child) -> last receipt time
        self.lifecycle = {}      # node -> state label or None (unreachable)
        self.goal_times = []     # times a goal id first appeared
        self.goal_active = False
        self.last_cmd_nonzero = None
        self.started = None
        self.cmd_since = None       # start of the current run of non-zero cmd_vel
        self.cmd_subscribers = None  # other nodes subscribed to cmd_vel (None = unknown)
        self.skew = None            # worst recent |TF stamp - clock| in seconds
        self.recoveries = []        # times a recovery behaviour started
        self.params = {}            # costmap params: inflation_radius, robot_radius
        self.amcl_cov = None        # worst of AMCL x/y variance (m^2) from /amcl_pose

    # ---- inputs -------------------------------------------------------
    def seen(self, stream, now):
        self.started = self.started if self.started is not None else now
        self.last[stream] = now

    def amcl(self, var_x, var_y, now):
        self.seen('amcl', now)
        self.amcl_cov = max(var_x, var_y)

    def tf(self, parent, child, now, skew=None):
        self.seen('tf', now)
        if skew is not None:
            self.skew = abs(skew)
        self.tf_edges[(parent.lstrip('/'), child.lstrip('/'))] = now

    def cmd(self, linear, angular, now):
        self.seen('cmd_vel', now)
        if abs(linear) > 1e-3 or abs(angular) > 1e-3:
            self.last_cmd_nonzero = now
            if self.cmd_since is None:
                self.cmd_since = now
        else:
            self.cmd_since = None

    def recovery(self, node_name, now):
        if any(w in node_name.lower() for w in RECOVERY_WORDS):
            self.recoveries.append(now)
            self.recoveries = [t for t in self.recoveries if now - t <= RECOVERY_WINDOW_S]

    def goal(self, now):
        self.goal_times.append(now)
        self.goal_times = [t for t in self.goal_times if now - t <= self.flap_window_s]

    def pose(self, x, y, now):
        self.seen('odom', now)
        self.poses.append((now, x, y))
        keep = max(self.progress_s, self.stale_s) * 2
        self.poses = [p for p in self.poses if now - p[0] <= keep]

    def set_goal_active(self, active, now):
        if active and not self.goal_active:
            self.goal_since = now
        elif not active:
            self.goal_since = None
        self.goal_active = active

    # ---- helpers ------------------------------------------------------
    def _age(self, stream, now):
        t = self.last.get(stream)
        return None if t is None else now - t

    def _edge_ages(self, now, suffixes):
        return {e: now - t for e, t in self.tf_edges.items()
                if e[1].endswith(suffixes[0]) or e[0].endswith(suffixes[1])}

    # ---- diagnosis ----------------------------------------------------
    def diagnose(self, now):
        f = []
        age_tf = self._age('tf', now)
        if age_tf is None:
            f.append(Finding('ERROR', 'TF_NONE', 'no /tf received - sim/driver not running or wrong namespace/domain'))
        elif age_tf > self.stale_s:
            f.append(Finding('ERROR', 'TF_FREEZE',
                             f'/tf silent for {age_tf:.1f}s - TF froze; costmap filters go inert and Nav2 stalls'))
        else:
            for (p, c), t in self.tf_edges.items():
                a = now - t
                if a > self.stale_s:
                    f.append(Finding('ERROR', 'TF_EDGE_STALE',
                                     f'transform {p}->{c} stopped {a:.1f}s ago while other TF is alive'))

        has_map_odom = any(p.endswith('map') and c.endswith('odom') for p, c in self.tf_edges)
        if age_tf is not None and not has_map_odom:
            f.append(Finding('WARN', 'NO_MAP_ODOM', 'no map->odom transform - SLAM/AMCL not localizing'))

        for stream, label in (('odom', 'odometry'), ('scan', 'laser scan')):
            a = self._age(stream, now)
            if a is None:
                f.append(Finding('WARN', f'{stream.upper()}_NONE', f'no {label} messages seen'))
            elif a > self.stale_s:
                f.append(Finding('ERROR', f'{stream.upper()}_STALE', f'{label} silent for {a:.1f}s'))

        a = self._age('local_costmap', now)
        if a is not None and a > self.stale_s:
            f.append(Finding('ERROR', 'COSTMAP_STALE',
                             f'local costmap not updating ({a:.1f}s) - usually a TF or scan problem upstream'))

        if self.amcl_cov is not None and self.amcl_cov > AMCL_COV_MAX:
            f.append(Finding('WARN', 'AMCL_UNCERTAIN',
                             f'AMCL pose covariance {self.amcl_cov:.2f} m^2 (> {AMCL_COV_MAX}) - robot is poorly localized, set /initialpose or check the map/scan'))

        for n, st in self.lifecycle.items():
            if st is None:
                f.append(Finding('WARN', 'NODE_MISSING', f'{n}: lifecycle service unreachable (not launched?)'))
            elif st != 'active':
                if self.started is not None and now - self.started < self.startup_s:
                    f.append(Finding('WARN', 'NODE_STARTING', f'{n} is "{st}" - Nav2 may still be starting up'))
                else:
                    f.append(Finding('ERROR', 'NODE_INACTIVE', f'{n} is "{st}", not active - lifecycle manager stuck'))

        if len(self.goal_times) >= self.flap_count:
            f.append(Finding('WARN', 'GOAL_FLAPPING',
                             f'{len(self.goal_times)} goals cancelled/aborted in {self.flap_window_s:.0f}s - explorer/goal source is oscillating (add hysteresis)'))

        if self.goal_active:
            a = self._age('cmd_vel', now)
            moving = self.last_cmd_nonzero is not None and now - self.last_cmd_nonzero < self.stale_s
            if a is None:
                f.append(Finding('ERROR', 'NO_CMD_VEL', 'goal active but nothing publishes cmd_vel - controller not producing output'))
            elif not moving:
                f.append(Finding('ERROR', 'CMD_ZERO', 'goal active but cmd_vel is zero - planner/controller failed or path blocked'))

        if self.goal_active and self.goal_since is not None and now - self.goal_since >= self.progress_s:
            win = [p for p in self.poses if now - p[0] <= self.progress_s]
            if len(win) >= 2 and win[0][0] - (now - self.progress_s) < self.stale_s:
                x0, y0 = win[0][1:]
                moved = max(((x - x0) ** 2 + (y - y0) ** 2) ** 0.5 for _, x, y in win)
                if moved < self.progress_m:
                    f.append(Finding('WARN', 'NO_PROGRESS',
                                     f'goal active for {now - self.goal_since:.0f}s but robot moved only {moved:.2f}m in {self.progress_s:.0f}s - stuck, recovering, or blocked'))

        if self.cmd_subscribers == 0 and self.last_cmd_nonzero is not None \
                and now - self.last_cmd_nonzero < self.stale_s:
            f.append(Finding('ERROR', 'NO_CMD_SUBSCRIBER',
                             'cmd_vel is being published but nothing subscribes - base driver listens on a different '
                             'topic (e.g. cmd_vel_smoothed) or is not running'))

        if self.cmd_since is not None and now - self.cmd_since >= DRIVE_S \
                and self.last_cmd_nonzero is not None and now - self.last_cmd_nonzero < self.stale_s:
            win = [p for p in self.poses if now - p[0] <= DRIVE_S]
            if len(win) >= 2 and win[0][0] - (now - DRIVE_S) < self.stale_s:
                x0, y0 = win[0][1:]
                moved = max(((x - x0) ** 2 + (y - y0) ** 2) ** 0.5 for _, x, y in win)
                if moved < self.progress_m:
                    f.append(Finding('ERROR', 'CMD_NO_MOTION',
                                     f'cmd_vel non-zero for {now - self.cmd_since:.0f}s but odom moved {moved:.2f}m - '
                                     'chassis/driver not obeying, e-stop, or odom frozen'))

        if self.skew is not None and self.skew > SKEW_S:
            f.append(Finding('WARN', 'CLOCK_SKEW',
                             f'/tf stamps differ from this machine\'s clock by {self.skew:.1f}s - clock skew or '
                             'use_sim_time mismatch ("extrapolation into the future/past" errors)'))

        infl, rad = self.params.get('inflation_radius'), self.params.get('robot_radius')
        if infl is not None and rad is not None and infl < rad:
            f.append(Finding('WARN', 'INFLATION_SMALL',
                             f'inflation_radius {infl:.2f}m < robot_radius {rad:.2f}m - planner can route through '
                             'cells the robot cannot fit, causing aborts near obstacles'))

        if len([t for t in self.recoveries if now - t <= RECOVERY_WINDOW_S]) >= RECOVERY_COUNT:
            f.append(Finding('WARN', 'RECOVERY_LOOP',
                             f'{len(self.recoveries)} recovery behaviours in {RECOVERY_WINDOW_S:.0f}s - robot is '
                             'spinning/backing up repeatedly instead of making progress'))

        if self.lifecycle and all(v is None for v in self.lifecycle.values()) and not self.last:
            f.append(Finding('WARN', 'NO_NAV2_VISIBLE',
                             'no Nav2 nodes or topics visible - check ROS_DOMAIN_ID, ROS_LOCALHOST_ONLY, RMW '
                             'implementation and --ns match the running stack'))

        if not any(x.level == 'ERROR' for x in f):
            f.append(Finding('OK', 'HEALTHY', 'no stall causes detected'))
        return f


def fmt(findings, hints=False):
    icon = {'ERROR': '\033[31mERROR\033[0m', 'WARN': '\033[33mWARN \033[0m', 'OK': '\033[32mOK   \033[0m'}
    out = []
    for x in findings:
        out.append(f'{icon[x.level]} [{x.code}] {x.msg}')
        if hints and x.code in HINTS and x.level != 'OK':
            out.append(f'      -> {HINTS[x.code]}')
    return '\n'.join(out)


def readiness_verdict(findings):
    errors = [x.code for x in findings if x.level == 'ERROR']
    warns = [x.code for x in findings if x.level == 'WARN']
    if errors:
        return f'\033[31mNOT READY\033[0m - {len(errors)} blocking: {", ".join(errors)}'
    return '\033[32mREADY\033[0m' + (f' - {len(warns)} warning(s): {", ".join(warns)}' if warns else '')


def to_json(findings, ns, now_wall):
    return json.dumps({'time': now_wall, 'ns': ns,
                       'ok': not any(x.level == 'ERROR' for x in findings),
                       'findings': [dict(x.__dict__, hint=HINTS.get(x.code)) for x in findings]})


def main(argv=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile
    from tf2_msgs.msg import TFMessage
    from nav_msgs.msg import Odometry, OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    from action_msgs.msg import GoalStatusArray
    from lifecycle_msgs.srv import GetState
    from rcl_interfaces.srv import GetParameters
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--version', action='version', version=f'nav2_doctor {__version__}')
    ap.add_argument('--ns', default='', help='robot namespace, e.g. robot1')
    ap.add_argument('--period', type=float, default=2.0, help='report period (s)')
    ap.add_argument('--startup-grace', type=float, default=STARTUP_S,
                    help='seconds before inactive lifecycle nodes count as errors')
    ap.add_argument('--stale', type=float, default=STALE_S, help='seconds of silence before a stream is stale')
    ap.add_argument('--flap-count', type=int, default=FLAP_COUNT, help='cancelled/aborted goals that count as flapping')
    ap.add_argument('--flap-window', type=float, default=FLAP_WINDOW_S, help='flapping look-back window (s)')
    ap.add_argument('--progress-time', type=float, default=PROGRESS_S,
                    help='seconds with an active goal and no movement before NO_PROGRESS')
    ap.add_argument('--progress-dist', type=float, default=PROGRESS_M, help='metres that count as movement')
    ap.add_argument('--json', action='store_true', help='print one JSON object per report')
    ap.add_argument('--hints', action='store_true', help='print a suggested next step under each finding')
    ap.add_argument('--diagnostics', action='store_true',
                    help='also publish findings as diagnostic_msgs/DiagnosticArray on /diagnostics')
    ap.add_argument('--once', type=float, default=0.0, help='watch N seconds, print one report, exit')
    ap.add_argument('--ready', nargs='?', type=float, const=10.0, default=None, metavar='SECS',
                    help='pre-flight: watch SECS (default 10), print hints and a READY / NOT READY verdict; '
                         'exit 1 if not ready (lifecycle nodes must already be active)')
    args, ros_args = ap.parse_known_args(argv)
    if args.ready is not None:
        args.once, args.hints, args.startup_grace = args.ready, True, 0.0

    rclpy.init(args=ros_args)
    node = Node('nav2_doctor')
    d = Doctor(args.startup_grace, args.stale, args.flap_window, args.flap_count,
               args.progress_time, args.progress_dist)
    ns = ('/' + args.ns.strip('/')) if args.ns else ''
    clock = time.monotonic
    sd = qos_profile_sensor_data
    rel = QoSProfile(depth=100)

    def on_tf(m):
        for t in m.transforms:
            st = t.header.stamp
            skew = None
            if st.sec or st.nanosec:   # zero stamp = unstamped, nothing to compare
                skew = node.get_clock().now().nanoseconds / 1e9 - (st.sec + st.nanosec / 1e9)
            d.tf(t.header.frame_id, t.child_frame_id, clock(), skew)
    node.create_subscription(TFMessage, f'{ns}/tf', on_tf, rel)
    node.create_subscription(Odometry, f'{ns}/odom', lambda m: d.pose(m.pose.pose.position.x, m.pose.pose.position.y, clock()), sd)
    node.create_subscription(PoseWithCovarianceStamped, f'{ns}/amcl_pose',
                             lambda m: d.amcl(m.pose.covariance[0], m.pose.covariance[7], clock()), rel)
    node.create_subscription(LaserScan, f'{ns}/scan', lambda m: d.seen('scan', clock()), sd)
    node.create_subscription(OccupancyGrid, f'{ns}/local_costmap/costmap',
                             lambda m: d.seen('local_costmap', clock()), rel)
    node.create_subscription(Twist, f'{ns}/cmd_vel',
                             lambda m: d.cmd(m.linear.x, m.angular.z, clock()), sd)

    seen_active, counted = set(), set()

    def on_status(m):
        # Flapping = goals CANCELED/ABORTED before finishing; a new goal after SUCCEEDED is normal exploring.
        active = False
        for s in m.status_list:
            gid = bytes(s.goal_info.goal_id.uuid).hex()
            if s.status in (1, 2):                       # ACCEPTED / EXECUTING
                seen_active.add(gid)
                active = True
            elif s.status in (5, 6) and gid in seen_active and gid not in counted:   # CANCELED / ABORTED
                counted.add(gid)
                d.goal(clock())
        d.set_goal_active(active, clock())
    node.create_subscription(GoalStatusArray, f'{ns}/navigate_to_pose/_action/status', on_status, rel)

    try:
        from nav2_msgs.msg import BehaviorTreeLog

        def on_bt(m):
            for e in m.event_log:
                if e.current_status == 'RUNNING' and e.previous_status != 'RUNNING':
                    d.recovery(e.node_name, clock())
        node.create_subscription(BehaviorTreeLog, f'{ns}/behavior_tree_log', on_bt, rel)
    except ImportError:    # nav2_msgs not installed: skip the recovery-loop check
        pass

    clients = {n: node.create_client(GetState, f'{ns}/{n}/get_state') for n in NAV2_NODES}

    def poll_lifecycle():
        for n, c in clients.items():
            if not c.service_is_ready():
                d.lifecycle[n] = None
                continue
            fut = c.call_async(GetState.Request())
            fut.add_done_callback(lambda f, n=n: d.lifecycle.__setitem__(n, f.result().current_state.label))
    node.create_timer(3.0, poll_lifecycle)

    def poll_cmd_subscribers():
        infos = node.get_subscriptions_info_by_topic(f'{ns}/cmd_vel')
        d.cmd_subscribers = sum(1 for i in infos if i.node_name != node.get_name())
    node.create_timer(1.0, poll_cmd_subscribers)

    param_cli = node.create_client(GetParameters, f'{ns}/local_costmap/local_costmap/get_parameters')
    names = ['inflation_layer.inflation_radius', 'robot_radius', 'footprint']

    def poll_params():
        if not param_cli.service_is_ready():
            return
        req = GetParameters.Request(names=names)

        def done(f):
            try:
                vals = f.result().values
            except Exception:
                return
            for key, v in zip(('inflation_radius', 'robot_radius', 'footprint'), vals):
                if v.type == 3:        # PARAMETER_DOUBLE
                    d.params[key] = v.double_value
                elif v.type == 4 and key == 'footprint' and v.string_value.strip('[] '):   # PARAMETER_STRING
                    r = inscribed_radius(v.string_value)
                    if r is not None:
                        d.params['robot_radius'] = r
        param_cli.call_async(req).add_done_callback(done)
    node.create_timer(10.0, poll_params)

    diag_pub = node.create_publisher(DiagnosticArray, '/diagnostics', 10) if args.diagnostics else None
    lvl = {'OK': DiagnosticStatus.OK, 'WARN': DiagnosticStatus.WARN, 'ERROR': DiagnosticStatus.ERROR}

    def publish_diag(found):
        msg = DiagnosticArray()
        msg.header.stamp = node.get_clock().now().to_msg()
        for x in found:
            msg.status.append(DiagnosticStatus(level=lvl[x.level], name=f'nav2_doctor/{x.code}',
                                               message=x.msg, hardware_id=args.ns or 'nav2'))
        diag_pub.publish(msg)

    exit_code = [0]

    def report():
        found = d.diagnose(clock())
        if args.json:
            print(to_json(found, args.ns, time.time()), flush=True)
        else:
            print(f'\n--- nav2_doctor {time.strftime("%H:%M:%S")} ns="{args.ns}" ---')
            print(fmt(found, args.hints), flush=True)
        if diag_pub:
            publish_diag(found)
        return found
    start = clock()

    def tick():
        if args.once and clock() - start >= args.once:
            found = report()
            errors = [x for x in found if x.level == 'ERROR']
            exit_code[0] = 1 if errors else 0
            if args.ready is not None and not args.json:
                print(readiness_verdict(found), flush=True)
            raise SystemExit
        if not args.once:
            report()
    node.create_timer(args.once or args.period, tick)
    node.get_logger().info(f'watching {ns or "/"} - Ctrl+C to stop')
    from rclpy.executors import ExternalShutdownException
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return exit_code[0]


if __name__ == '__main__':
    raise SystemExit(main())
