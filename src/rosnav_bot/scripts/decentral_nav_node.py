#!/usr/bin/env python3
"""
decentral_nav_node.py — per-robot ROS 2 wrapper around decentral_nav_sim's decentralized controllers
(`vo` sampled velocity obstacle, `mppi_prior` MPPI around a VO prior). Research controller, same
status as mpc_controller.py: it publishes `cmd_vel_topic` (default `cmd_vel_nav`, relative -> /robotN/cmd_vel_nav)
instead of Nav2's controller_server output (the launch sends that to an unused topic).

Inputs (all relative to the node namespace unless absolute):
  plan      nav_msgs/Path   Nav2 global plan; replaces the sim's BFS gradient as the path direction
                            (and, for mppi_prior, as the goal-distance field)
  odom      Odometry        own twist (neighbours' twists come from `/<name>/odom`, param `neighbors`)
  scan      LaserScan       optional static obstacles (param use_scan), dropped near the plan
  TF        <plan frame> -> <name>/base_link for self and every neighbour (positions)
Output: Twist. The sim's holonomic velocity is turned into diff-drive (v, w) by `track_unicycle`
(turn toward the desired heading, forward speed scaled by cos(error), slew-limited).

Our twist over the sim: the plan-derived distance field means the same math works on any Nav2 map,
and a watchdog zeroes the output when the plan or own pose goes stale.

The pure computation lives in ROS-free functions (`compute_velocity`, `track_unicycle`,
`PlanWorld`) so it is unit-testable (test/test_decentral_nav_node.py).

  ros2 run rosnav_bot decentral_nav_node.py --ros-args -r __ns:=/robot1 \
       -p neighbors:="['robot2']" -p ctrl:=vo -p use_sim_time:=true
"""
from __future__ import annotations

import importlib.util
import math
import os

import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree

_HERE = os.path.dirname(os.path.abspath(__file__))


def _load_sim():
    spec = importlib.util.spec_from_file_location('decentral_nav_sim', os.path.join(_HERE, 'decentral_nav_sim.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


sim = _load_sim()


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def resample(path_xy: np.ndarray, step: float = 0.25) -> np.ndarray:
    """Even arclength resampling of an Nx2 polyline."""
    path_xy = np.asarray(path_xy, float)
    if len(path_xy) < 2:
        return path_xy.copy()
    seg = np.linalg.norm(np.diff(path_xy, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] < 1e-6:
        return path_xy[:1].copy()
    q = np.arange(0.0, s[-1], step)
    q = np.append(q, s[-1])
    return np.stack([np.interp(q, s, path_xy[:, 0]), np.interp(q, s, path_xy[:, 1])], axis=1)


class PlanWorld(sim.World):
    hit_margin = 0.03        # m: lidar returns sit on the wall surface, not inside it
    """decentral_nav_sim.World built from a Nav2 plan + scan points (1 m cells, as in the sim).

    Coordinates are shifted by `origin` (integer metres) so the sim's index-by-rint code works on a small
    local grid. pref_dir/dist_field follow the plan instead of a BFS over a map.
    """

    def __init__(self, plan_xy, scan_xy, extra_pts, lookahead=1.0, margin=4, plan_clear=0.35):
        plan = resample(plan_xy)
        allp = np.vstack([plan, np.asarray(extra_pts, float).reshape(-1, 2)])
        lo = np.floor(allp.min(0)).astype(int) - margin
        hi = np.ceil(allp.max(0)).astype(int) + margin
        self.origin = lo.astype(float)
        w, h = (hi - lo + 1)
        free = np.ones((h, w), bool)
        self.plan = plan - self.origin
        self.tree = cKDTree(self.plan)
        seg = np.linalg.norm(np.diff(self.plan, axis=0), axis=1) if len(plan) > 1 else np.zeros(0)
        arc = np.concatenate([[0.0], np.cumsum(seg)])
        self.rem = arc[-1] - arc                       # remaining plan length at each plan point
        self.lookahead = lookahead
        n_blocked = 0
        if scan_xy is not None and len(scan_xy):
            s = np.asarray(scan_xy, float) - self.origin
            if len(s):
                d, _ = self.tree.query(s)
                s = s[d > plan_clear]                  # the plan is already known-free: do not wall it in
                ix = np.rint(s[:, 0]).astype(int)
                iy = np.rint(s[:, 1]).astype(int)
                m = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
                free[iy[m], ix[m]] = False
                n_blocked = int((~free).sum())
        self.n_blocked = n_blocked
        # exact scan points for collision checks: the 1 m blocked squares above are fine for the cost fields, but
        # as a collision test they turn a 1.5 m corridor into ~1 m (no room for two robots to pass).
        self.pts = None
        self.pts_tree = None
        if scan_xy is not None and len(scan_xy):
            s2 = np.asarray(scan_xy, float) - self.origin
            if len(s2):
                d2, _ = self.tree.query(s2)
                s2 = s2[d2 > plan_clear]
                if len(s2):
                    self.pts = s2
                    self.pts_tree = cKDTree(s2)
        super().__init__(free)

    def ok(self, p, r=sim.R):
        """Disc of radius r at p clear of every (exact) scan point; falls back to the grid test without a scan."""
        if self.pts_tree is None:
            return super().ok(p, r)
        return not self.pts_tree.query_ball_point(np.asarray(p, float), r + self.hit_margin)

    def dist_field(self, goal):
        if 'df' not in self._cache:
            ys, xs = np.mgrid[0:self.h, 0:self.w]
            cells = np.stack([xs.ravel(), ys.ravel()], axis=1).astype(float)
            d, k = self.tree.query(cells)
            df = (d + self.rem[k]).reshape(self.h, self.w)
            df[~self.free] = np.inf
            self._cache['df'] = df
        return self._cache['df']

    def pref_dir(self, p, goal):
        d, k = self.tree.query(p)
        tgt_rem = max(self.rem[k] - self.lookahead, 0.0)
        j = int(np.argmin(np.abs(self.rem - tgt_rem)))
        v = self.plan[j] - np.asarray(p, float)
        n = np.linalg.norm(v)
        if n < 1e-6:
            v = np.asarray(goal, float) - np.asarray(p, float)
            n = np.linalg.norm(v)
        return v / (n + 1e-9)


def compute_velocity(ctrl, i, pos, vel, plan_xy, scan_xy=None, rng=None, vmax=0.5, lookahead=1.0):
    """Desired holonomic world-frame velocity for robot i.

    pos, vel: (N, 2) arrays in the plan frame (row i = this robot); plan_xy: Nx2 global plan;
    scan_xy: optional Mx2 static obstacle points in the plan frame. Returns (vx, vy, info).
    ROS-free: the node and the unit tests call exactly this.
    """
    pos = np.asarray(pos, float)
    vel = np.asarray(vel, float)
    rng = rng or np.random.default_rng(0)
    sim.VMAX = float(vmax)
    plan_xy = np.asarray(plan_xy, float)
    goal = plan_xy[-1]
    gd = float(np.linalg.norm(goal - pos[i]))
    if gd < 0.1:
        return 0.0, 0.0, {'reason': 'at_goal', 'blocked_cells': 0}
    if gd < 0.25:
        # the sim stops at 0.25 m, which is exactly Nav2's xy_goal_tolerance: creep the last stretch so the goal
        # checker is satisfied with margin (neighbours are ignored for these final 25 cm)
        d = (goal - pos[i]) / gd
        sp = min(0.2, 2.0 * gd, float(vmax))
        return float(d[0] * sp), float(d[1] * sp), {'reason': 'creep', 'blocked_cells': 0}
    world = PlanWorld(plan_xy, scan_xy, pos, lookahead=lookahead)
    o = world.origin
    lp, lg = pos - o, goal - o
    goals = np.tile(lg, (len(pos), 1))
    v = sim.command(ctrl, i, lp, vel, goals, world, rng)
    return float(v[0]), float(v[1]), {'reason': 'ok', 'blocked_cells': world.n_blocked}


def track_unicycle(vd, yaw, v_prev=0.0, w_prev=0.0, v_max=0.5, w_max=1.0, k_w=2.0,
                   a_max=1.0, alpha_max=3.0, dt=0.1):
    """Holonomic velocity (vx, vy) in world frame -> diff-drive (v, w). Turn toward the desired heading,
    scale forward speed by cos(heading error) (zero beyond 90 deg: rotate in place), slew-limit both."""
    sp = math.hypot(vd[0], vd[1])
    if sp < 0.03:
        v_t, w_t = 0.0, 0.0
    else:
        err = wrap(math.atan2(vd[1], vd[0]) - yaw)
        w_t = max(-w_max, min(w_max, k_w * err))
        v_t = min(sp, v_max) * max(0.0, math.cos(err))
    v = v_prev + max(-a_max * dt, min(a_max * dt, v_t - v_prev))
    w = w_prev + max(-alpha_max * dt, min(alpha_max * dt, w_t - w_prev))
    return v, w


def goal_rotation(dist, yaw, goal_yaw, xy_tol=0.15, yaw_tol=0.12, w_max=0.6, k_w=1.5):
    """Angular speed to rotate in place at the goal, or None when not applicable.

    Nav2's goal checker also wants the goal heading (yaw_goal_tolerance): inside xy_tol of the goal with the heading
    off by more than yaw_tol, turn on the spot, otherwise the goal is never "reached" and the controller aborts."""
    err = wrap(goal_yaw - yaw)
    if dist < xy_tol and abs(err) > yaw_tol:
        return max(-w_max, min(w_max, k_w * err))
    return None


# ── ROS node ─────────────────────────────────────────────────────────────────
def main():
    import rclpy
    import tf2_ros
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry, Path
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    def yaw_of(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    class DecentralNav(Node):
        def __init__(self):
            super().__init__('decentral_nav')
            P = self.declare_parameter
            P('ctrl', 'vo')                       # vo | mppi_prior
            P('neighbors', [''])                  # other robot namespaces, e.g. ['robot2', 'robot3']
            P('base_frame', '{ns}/base_link')
            P('cmd_vel_topic', 'cmd_vel_nav')
            P('plan_topic', 'plan')
            P('odom_topic', 'odom')
            P('scan_topic', 'scan')
            P('use_scan', True)
            P('scan_stride', 4)
            P('scan_range', 4.0)
            P('v_max', 0.5)
            P('w_max', 1.0)
            P('lookahead', 1.0)
            P('rate_hz', 10.0)
            P('plan_timeout', 15.0)               # s without a fresh plan -> zero (0 disables)
            P('pose_timeout', 1.0)
            P('seed', 0)
            g = lambda n: self.get_parameter(n).value  # noqa: E731
            self.ctrl = str(g('ctrl')).lower()
            if self.ctrl not in ('vo', 'mppi_prior'):
                raise ValueError(f"ctrl must be vo or mppi_prior, got {self.ctrl!r}")
            self.ns = self.get_namespace().strip('/') or 'robot1'
            self.neigh = [n for n in g('neighbors') if n and n != self.ns]
            self.base = str(g('base_frame'))
            self.vmax, self.wmax = float(g('v_max')), float(g('w_max'))
            self.lookahead = float(g('lookahead'))
            self.plan_timeout, self.pose_timeout = float(g('plan_timeout')), float(g('pose_timeout'))
            self.use_scan, self.stride, self.srange = bool(g('use_scan')), int(g('scan_stride')), float(g('scan_range'))
            self.rng = np.random.default_rng(int(g('seed')))
            self.tf = tf2_ros.Buffer()
            self.tfl = tf2_ros.TransformListener(self.tf, self)
            self.plan = None
            self.plan_frame = 'map'
            self.plan_t = None
            self.scan = None
            self.odom_twist = {}                  # name -> (vx_body, wz)
            self.v = self.w = 0.0
            self.n_ticks = self.n_zero = 0
            self.goal_yaw = 0.0
            self.goal_xy_tol, self.goal_yaw_tol = 0.12, 0.12
            self.last_reason = 'startup'
            self.create_subscription(Path, g('plan_topic'), self.on_plan, 1)
            self.create_subscription(Odometry, g('odom_topic'), lambda m: self.on_odom(self.ns, m), 10)
            for n in self.neigh:
                self.create_subscription(Odometry, f'/{n}/odom', lambda m, n=n: self.on_odom(n, m), 10)
            if self.use_scan:
                self.create_subscription(LaserScan, g('scan_topic'), self.on_scan, qos_profile_sensor_data)
            self.pub = self.create_publisher(Twist, g('cmd_vel_topic'), 10)
            self.Twist = Twist
            self.create_timer(1.0 / float(g('rate_hz')), self.tick)
            self.create_timer(2.0, self.status)
            self.get_logger().info(
                f'[decentral_nav] {self.ns}: ctrl={self.ctrl} neighbors={self.neigh} v_max={self.vmax} '
                f'w_max={self.wmax} use_scan={self.use_scan} plan_timeout={self.plan_timeout}s -> '
                f'{g("cmd_vel_topic")} (resolved /{self.ns}/{g("cmd_vel_topic")})')

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def on_plan(self, msg):
            if len(msg.poses) < 2:
                self.get_logger().warn('[decentral_nav] ignoring plan with <2 poses')
                return
            self.plan = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
            q = msg.poses[-1].pose.orientation
            self.goal_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.plan_frame = msg.header.frame_id or 'map'
            first = self.plan_t is None
            self.plan_t = self.now()
            if first:
                self.get_logger().info(f'[decentral_nav] first plan: {len(self.plan)} poses in {self.plan_frame}, '
                                       f'goal=({self.plan[-1][0]:.2f},{self.plan[-1][1]:.2f})')

        def on_odom(self, name, msg):
            self.odom_twist[name] = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

        def on_scan(self, msg):
            self.scan = msg

        def pose(self, name):
            """(x, y, yaw) of <name>/base_link in the plan frame, or None."""
            try:
                t = self.tf.lookup_transform(self.plan_frame, self.base.format(ns=name), rclpy.time.Time())
            except Exception:
                return None
            age = self.now() - (t.header.stamp.sec + t.header.stamp.nanosec * 1e-9)
            if age > self.pose_timeout:
                return None
            tr = t.transform
            return tr.translation.x, tr.translation.y, yaw_of(tr.rotation)

        def scan_points(self):
            if not self.use_scan or self.scan is None:
                return None
            s = self.scan
            r = np.asarray(s.ranges, float)[::self.stride]
            a = (s.angle_min + np.arange(len(s.ranges)) * s.angle_increment)[::self.stride]
            ok = np.isfinite(r) & (r > s.range_min) & (r < min(s.range_max, self.srange))
            if not ok.any():
                return np.empty((0, 2))
            try:
                t = self.tf.lookup_transform(self.plan_frame, s.header.frame_id, rclpy.time.Time())
            except Exception:
                return None
            yw = yaw_of(t.transform.rotation)
            lx, ly = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
            c, sn = math.cos(yw), math.sin(yw)
            return np.stack([t.transform.translation.x + c * lx - sn * ly,
                             t.transform.translation.y + sn * lx + c * ly], axis=1)

        def stop(self, reason):
            if reason != self.last_reason:
                self.get_logger().warn(f'[decentral_nav] {self.ns}: zero output ({reason})')
                self.last_reason = reason
            self.v = self.w = 0.0
            self.n_zero += 1
            self.pub.publish(self.Twist())

        def tick(self):
            self.n_ticks += 1
            if self.plan is None:
                return self.stop('no plan yet')            # watchdog: never command without a plan
            if self.plan_timeout > 0 and self.now() - self.plan_t > self.plan_timeout:
                return self.stop(f'plan stale >{self.plan_timeout:.0f}s')
            me = self.pose(self.ns)
            if me is None:
                return self.stop(f'no TF {self.plan_frame}->{self.base.format(ns=self.ns)}')
            names, pos, vel = [self.ns], [[me[0], me[1]]], [[0.0, 0.0]]
            for n in [self.ns] + self.neigh:
                p = me if n == self.ns else self.pose(n)
                if p is None:
                    continue
                vx_b, _ = self.odom_twist.get(n, (0.0, 0.0))
                v = [vx_b * math.cos(p[2]), vx_b * math.sin(p[2])]   # body-forward twist -> plan frame
                if n == self.ns:
                    vel[0] = v
                else:
                    names.append(n)
                    pos.append([p[0], p[1]])
                    vel.append(v)
            vx, vy, info = compute_velocity(self.ctrl, 0, np.array(pos), np.array(vel), self.plan,
                                            self.scan_points(), self.rng, self.vmax, self.lookahead)
            self.last_info = dict(info, n_neigh=len(names) - 1)
            gd = float(np.linalg.norm(self.plan[-1] - np.array(pos[0])))
            w_rot = goal_rotation(gd, me[2], self.goal_yaw, self.goal_xy_tol, self.goal_yaw_tol)
            if w_rot is not None:
                self.v, self.w = 0.0, w_rot
                self.last_vd, self.last_goal_d, self.last_reason = (0.0, 0.0), gd, 'rotate to goal yaw'
                msg = self.Twist()
                msg.angular.z = float(self.w)
                self.pub.publish(msg)
                return
            self.v, self.w = track_unicycle((vx, vy), me[2], self.v, self.w, self.vmax, self.wmax,
                                            dt=1.0 / float(self.get_parameter('rate_hz').value))
            self.last_reason = info['reason']
            self.last_vd = (vx, vy)
            self.last_goal_d = float(np.linalg.norm(self.plan[-1] - np.array(pos[0])))
            msg = self.Twist()
            msg.linear.x, msg.angular.z = float(self.v), float(self.w)
            self.pub.publish(msg)

        def status(self):
            g = getattr(self, 'last_info', None)
            if g is None:
                self.get_logger().info(f'[decentral_nav] {self.ns}: waiting ({self.last_reason}); '
                                       f'ticks={self.n_ticks}')
                return
            self.get_logger().info(
                f'[decentral_nav] {self.ns}: cmd v={self.v:.2f} w={self.w:.2f} desired=({self.last_vd[0]:.2f},'
                f'{self.last_vd[1]:.2f}) goal_d={self.last_goal_d:.2f} neighbours_seen={g["n_neigh"]}/{len(self.neigh)} '
                f'blocked_cells={g["blocked_cells"]} state={g["reason"]} zero_ticks={self.n_zero}/{self.n_ticks}')

    rclpy.init()
    node = DecentralNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(node.Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
