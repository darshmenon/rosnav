#!/usr/bin/env python3
"""nav2_doctor - attach to a running Nav2 stack and say why the robot isn't moving.

Pure-logic `Doctor` (no ROS imports, unit-testable) + a thin rclpy `main()`.
"""
import argparse
import time
from dataclasses import dataclass

__version__ = '0.2.0'

NAV2_NODES = ['controller_server', 'planner_server', 'bt_navigator',
              'behavior_server', 'smoother_server', 'waypoint_follower']
STALE_S = 2.0          # a stream is "stale" if silent this long
FLAP_WINDOW_S = 30.0   # goal-flapping look-back
STARTUP_S = 60.0       # grace period before inactive nodes count as errors
FLAP_COUNT = 4         # new goals inside window => flapping


@dataclass
class Finding:
    level: str   # ERROR | WARN | OK
    code: str
    msg: str


class Doctor:
    """Feed it events with `seen()` / `state()`; ask `diagnose(now)`."""

    def __init__(self, startup_s=STARTUP_S):
        self.startup_s = startup_s
        self.last = {}           # stream name -> last receipt time
        self.tf_edges = {}       # (parent, child) -> last receipt time
        self.lifecycle = {}      # node -> state label or None (unreachable)
        self.goal_times = []     # times a goal id first appeared
        self.goal_active = False
        self.last_cmd_nonzero = None
        self.started = None

    # ---- inputs -------------------------------------------------------
    def seen(self, stream, now):
        self.started = self.started if self.started is not None else now
        self.last[stream] = now

    def tf(self, parent, child, now):
        self.seen('tf', now)
        self.tf_edges[(parent.lstrip('/'), child.lstrip('/'))] = now

    def cmd(self, linear, angular, now):
        self.seen('cmd_vel', now)
        if abs(linear) > 1e-3 or abs(angular) > 1e-3:
            self.last_cmd_nonzero = now

    def goal(self, now):
        self.goal_times.append(now)
        self.goal_times = [t for t in self.goal_times if now - t <= FLAP_WINDOW_S]

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
        elif age_tf > STALE_S:
            f.append(Finding('ERROR', 'TF_FREEZE',
                             f'/tf silent for {age_tf:.1f}s - TF froze; costmap filters go inert and Nav2 stalls'))
        else:
            for (p, c), t in self.tf_edges.items():
                a = now - t
                if a > STALE_S:
                    f.append(Finding('ERROR', 'TF_EDGE_STALE',
                                     f'transform {p}->{c} stopped {a:.1f}s ago while other TF is alive'))

        has_map_odom = any(p.endswith('map') and c.endswith('odom') for p, c in self.tf_edges)
        if age_tf is not None and not has_map_odom:
            f.append(Finding('WARN', 'NO_MAP_ODOM', 'no map->odom transform - SLAM/AMCL not localizing'))

        for stream, label in (('odom', 'odometry'), ('scan', 'laser scan')):
            a = self._age(stream, now)
            if a is None:
                f.append(Finding('WARN', f'{stream.upper()}_NONE', f'no {label} messages seen'))
            elif a > STALE_S:
                f.append(Finding('ERROR', f'{stream.upper()}_STALE', f'{label} silent for {a:.1f}s'))

        a = self._age('local_costmap', now)
        if a is not None and a > STALE_S:
            f.append(Finding('ERROR', 'COSTMAP_STALE',
                             f'local costmap not updating ({a:.1f}s) - usually a TF or scan problem upstream'))

        for n, st in self.lifecycle.items():
            if st is None:
                f.append(Finding('WARN', 'NODE_MISSING', f'{n}: lifecycle service unreachable (not launched?)'))
            elif st != 'active':
                if self.started is not None and now - self.started < self.startup_s:
                    f.append(Finding('WARN', 'NODE_STARTING', f'{n} is "{st}" - Nav2 may still be starting up'))
                else:
                    f.append(Finding('ERROR', 'NODE_INACTIVE', f'{n} is "{st}", not active - lifecycle manager stuck'))

        if len(self.goal_times) >= FLAP_COUNT:
            f.append(Finding('WARN', 'GOAL_FLAPPING',
                             f'{len(self.goal_times)} new goals in {FLAP_WINDOW_S:.0f}s - explorer/goal source is oscillating (add hysteresis)'))

        if self.goal_active:
            a = self._age('cmd_vel', now)
            moving = self.last_cmd_nonzero is not None and now - self.last_cmd_nonzero < STALE_S
            if a is None:
                f.append(Finding('ERROR', 'NO_CMD_VEL', 'goal active but nothing publishes cmd_vel - controller not producing output'))
            elif not moving:
                f.append(Finding('ERROR', 'CMD_ZERO', 'goal active but cmd_vel is zero - planner/controller failed or path blocked'))

        if not any(x.level == 'ERROR' for x in f):
            f.append(Finding('OK', 'HEALTHY', 'no stall causes detected'))
        return f


def fmt(findings):
    icon = {'ERROR': '\033[31mERROR\033[0m', 'WARN': '\033[33mWARN \033[0m', 'OK': '\033[32mOK   \033[0m'}
    return '\n'.join(f'{icon[x.level]} [{x.code}] {x.msg}' for x in findings)


def main(argv=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile
    from tf2_msgs.msg import TFMessage
    from nav_msgs.msg import Odometry, OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import Twist
    from action_msgs.msg import GoalStatusArray
    from lifecycle_msgs.srv import GetState

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--version', action='version', version=f'nav2_doctor {__version__}')
    ap.add_argument('--ns', default='', help='robot namespace, e.g. robot1')
    ap.add_argument('--period', type=float, default=2.0, help='report period (s)')
    ap.add_argument('--startup-grace', type=float, default=STARTUP_S,
                    help='seconds before inactive lifecycle nodes count as errors')
    ap.add_argument('--once', type=float, default=0.0, help='watch N seconds, print one report, exit')
    args, ros_args = ap.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = Node('nav2_doctor')
    d = Doctor(args.startup_grace)
    ns = ('/' + args.ns.strip('/')) if args.ns else ''
    clock = time.monotonic
    sd = qos_profile_sensor_data
    rel = QoSProfile(depth=100)

    def on_tf(m):
        for t in m.transforms:
            d.tf(t.header.frame_id, t.child_frame_id, clock())
    node.create_subscription(TFMessage, f'{ns}/tf', on_tf, rel)
    node.create_subscription(Odometry, f'{ns}/odom', lambda m: d.seen('odom', clock()), sd)
    node.create_subscription(LaserScan, f'{ns}/scan', lambda m: d.seen('scan', clock()), sd)
    node.create_subscription(OccupancyGrid, f'{ns}/local_costmap/costmap',
                             lambda m: d.seen('local_costmap', clock()), rel)
    node.create_subscription(Twist, f'{ns}/cmd_vel',
                             lambda m: d.cmd(m.linear.x, m.angular.z, clock()), sd)

    known = set()

    def on_status(m):
        active = False
        for s in m.status_list:
            gid = bytes(s.goal_info.goal_id.uuid).hex()
            if gid not in known:
                known.add(gid)
                d.goal(clock())
            if s.status in (1, 2):   # ACCEPTED / EXECUTING
                active = True
        d.goal_active = active
    node.create_subscription(GoalStatusArray, f'{ns}/navigate_to_pose/_action/status', on_status, rel)

    clients = {n: node.create_client(GetState, f'{ns}/{n}/get_state') for n in NAV2_NODES}

    def poll_lifecycle():
        for n, c in clients.items():
            if not c.service_is_ready():
                d.lifecycle[n] = None
                continue
            fut = c.call_async(GetState.Request())
            fut.add_done_callback(lambda f, n=n: d.lifecycle.__setitem__(n, f.result().current_state.label))
    node.create_timer(3.0, poll_lifecycle)

    exit_code = [0]

    def report():
        found = d.diagnose(clock())
        print(f'\n--- nav2_doctor {time.strftime("%H:%M:%S")} ns="{args.ns}" ---')
        print(fmt(found), flush=True)
        return found
    start = clock()

    def tick():
        if args.once and clock() - start >= args.once:
            exit_code[0] = 1 if any(x.level == 'ERROR' for x in report()) else 0
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
