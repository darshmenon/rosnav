#!/usr/bin/env python3
"""Leader-follower (convoy) swarm behaviour.

The leader (default robot1) is driven by teleop or its own Nav2 goals. Each follower keeps a fixed
path-distance `spacing` behind the one in front: the leader's map-frame pose is recorded as a
breadcrumb trail, follower k aims at the trail point k*spacing (metres along the trail) behind the
leader's head, and a Nav2 NavigateToPose goal is sent to that follower whenever its target moved more
than `goal_tol`. Our additions: the gap widens with leader speed (`speed_gain`), and if the leader's TF goes silent for
`leader_timeout` s the first follower is promoted to leader (convoy survives a dead leader).
Trail-following (not a straight offset) keeps followers on ground the leader already
drove, so walls/doorways do not trap them.

  ros2 run rosnav_bot leader_follower.py --ros-args -p use_sim_time:=true \
      -p leader:=robot1 -p followers:=robot2,robot3 -p spacing:=1.0
"""
import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

try:  # same-directory import (works for the installed script and for tests)
    from cbf_safety_filter import cbf_filter
except Exception:  # noqa: BLE001
    cbf_filter = None


def se2_apply(tx, ty, tyaw, x, y, yaw=0.0):
    """Apply the 2D transform (tx, ty, tyaw) to the pose (x, y, yaw): returns the pose in the parent frame."""
    c, sn = math.cos(tyaw), math.sin(tyaw)
    return (tx + c * x - sn * y, ty + sn * x + c * y, math.atan2(math.sin(yaw + tyaw), math.cos(yaw + tyaw)))


def pursuit_cmd(fx, fy, fyaw, tx, ty, v, max_w=1.0):
    """Pure pursuit: (v, w) that steers the follower pose (fx, fy, fyaw) towards the lookahead point (tx, ty).
    Target behind the robot -> rotate in place first (v=0)."""
    dx, dy = tx - fx, ty - fy
    c, sn = math.cos(fyaw), math.sin(fyaw)
    lx, ly = c * dx + sn * dy, -sn * dx + c * dy
    l2 = lx * lx + ly * ly
    if l2 < 1e-6:
        return 0.0, 0.0
    if lx < 0.05:
        return 0.0, max_w if ly >= 0 else -max_w
    w = v * 2.0 * ly / l2
    return v, max(-max_w, min(max_w, w))


def gap_speed(v_leader, err, kp, vmax, hold_err=-0.15):
    """Speed that holds the gap constant: leader speed (feed-forward) + kp * gap error (feedback).
    err = path distance to the head minus the wanted distance (> 0: too far behind). Too close -> stop."""
    if err < hold_err:
        return 0.0
    return max(0.0, min(vmax, v_leader + kp * err))


FORMATIONS = ('column', 'line', 'wedge', 'diamond', 'echelon', 'circle')


def formation_slots(name, n, s):
    """(back, lateral) offsets in metres of followers 1..n relative to the leader (back > 0 = behind it,
    lateral > 0 = to its left). `s` is the base spacing. 'column' is trail-following, not slot-based."""
    out = []
    for k in range(1, n + 1):
        r = (k + 1) // 2                      # rank: 1,1,2,2,3,3...
        side = 1 if k % 2 else -1             # alternate left / right
        if name == 'line':                    # abreast
            out.append((0.0, side * r * s))
        elif name == 'wedge':                 # V / arrowhead
            out.append((r * s * 0.9, side * r * s * 0.8))
        elif name == 'diamond':               # left, right, then rear, then outer ranks
            out.append(((s, side * s) if k <= 2 else (r * s * 1.2, 0.0)) if k <= 3 else (r * s, side * s * 0.5))
        elif name == 'echelon':               # staggered diagonal, all on the left
            out.append((k * s * 0.8, k * s * 0.8))
        elif name == 'circle':                # ring around the leader
            a = 2.0 * math.pi * k / (n + 1) + math.pi / (n + 1)
            out.append((-1.5 * s * math.cos(a), 1.5 * s * math.sin(a)))
        else:                                 # column fallback: single file
            out.append((k * s, 0.0))
    return out


def slot_world(lx, ly, heading, back, lat):
    """World position of a formation slot: `back` m behind the leader along its heading, `lat` m to its left."""
    c, sn = math.cos(heading), math.sin(heading)
    return lx - back * c - lat * sn, ly - back * sn + lat * c


def keep_clear(off, r_min):
    """Stop a slot glide from passing through the leader: keep the (back, lateral) offset at least r_min away."""
    r = math.hypot(off[0], off[1])
    if r >= r_min:
        return off
    if r < 1e-6:
        return (r_min, 0.0)          # dead on the leader: push straight to its rear
    return (off[0] * r_min / r, off[1] * r_min / r)


def gap_color(err, tol=0.15):
    """RGB for a gap line: green within tol of the wanted gap, yellow within 3 tol, red beyond."""
    e = abs(err)
    return (0.1, 0.9, 0.2) if e <= tol else ((1.0, 0.8, 0.0) if e <= 3 * tol else (1.0, 0.15, 0.1))


def adaptive_spacing(base, gain, speed):
    """Convoy gap grows with leader speed (stopping distance): base + gain * v, capped at 3x base."""
    return min(base + gain * max(speed, 0.0), 3.0 * base)


class Trail:
    """Breadcrumb trail; `point_behind(d)` = point d metres of path length behind the head."""

    def __init__(self, step=0.1):
        self.step = step
        self.pts = []

    def add(self, x, y):
        if not self.pts or math.hypot(x - self.pts[-1][0], y - self.pts[-1][1]) >= self.step - 1e-9:
            self.pts.append((x, y))

    def point_behind(self, dist):
        """(x, y, yaw) on the trail `dist` m behind the newest point; None until the trail is long enough."""
        if len(self.pts) < 2:
            return None
        acc = 0.0
        for i in range(len(self.pts) - 1, 0, -1):
            (x1, y1), (x0, y0) = self.pts[i], self.pts[i - 1]
            seg = math.hypot(x1 - x0, y1 - y0)
            if acc + seg >= dist:
                f = (dist - acc) / seg if seg > 0 else 0.0
                return (x1 + (x0 - x1) * f, y1 + (y0 - y1) * f, math.atan2(y1 - y0, x1 - x0))
            acc += seg
        return None

    def project(self, x, y):
        """Index of the crumb nearest to (x, y)."""
        return min(range(len(self.pts)), key=lambda i: (self.pts[i][0] - x) ** 2 + (self.pts[i][1] - y) ** 2)

    def arclen_from(self, i):
        """Path length from crumb i to the head."""
        return sum(math.hypot(self.pts[j + 1][0] - self.pts[j][0], self.pts[j + 1][1] - self.pts[j][1])
                   for j in range(i, len(self.pts) - 1))

    def lookahead(self, i, dist):
        """Point `dist` m of path ahead of crumb i (towards the head); clamps at the head."""
        acc = 0.0
        for j in range(i, len(self.pts) - 1):
            seg = math.hypot(self.pts[j + 1][0] - self.pts[j][0], self.pts[j + 1][1] - self.pts[j][1])
            if acc + seg >= dist and seg > 0:
                f = (dist - acc) / seg
                return (self.pts[j][0] + (self.pts[j + 1][0] - self.pts[j][0]) * f,
                        self.pts[j][1] + (self.pts[j + 1][1] - self.pts[j][1]) * f)
            acc += seg
        return self.pts[-1]

    def trim(self, keep_m):
        """Drop crumbs older than keep_m of path length (bounded memory)."""
        acc = 0.0
        for i in range(len(self.pts) - 1, 0, -1):
            acc += math.hypot(self.pts[i][0] - self.pts[i - 1][0], self.pts[i][1] - self.pts[i - 1][1])
            if acc > keep_m:
                del self.pts[:i - 1]
                return


class LeaderFollower(Node):
    def __init__(self):
        super().__init__('leader_follower')
        self.declare_parameter('leader', 'robot1')
        self.declare_parameter('followers', 'robot2')
        self.declare_parameter('spacing', 1.0)
        self.declare_parameter('goal_tol', 0.4)
        self.declare_parameter('min_resend_s', 1.5)     # rate limit: Nav2 replans on every new goal
        self.declare_parameter('hold_frac', 0.6)        # hold (cancel goal) when closer than hold_frac * gap to the robot ahead
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('formation', 'column')  # column | line | wedge | diamond | echelon | circle (live-switchable)
        self.declare_parameter('transition_speed', 0.6)  # m/s slots glide to their new place when the formation changes
        self.declare_parameter('control', 'direct')   # direct = gap-regulated pursuit on /<ns>/cmd_vel; nav2 = NavigateToPose goals
        self.declare_parameter('kp', 0.8)             # gap-error gain (1/s)
        self.declare_parameter('vmax', 0.35)
        self.declare_parameter('accel', 0.4)          # m/s^2 limit: smooth speed changes keep the gap stable
        self.declare_parameter('speed_gain', 0.5)       # extra spacing (m) per m/s of leader speed
        self.declare_parameter('cbf', True)             # control-barrier-function filter on every follower command
        self.declare_parameter('cbf_safe_dist', 0.6)    # m, hard floor between any two robots (below the formation gap)
        self.declare_parameter('leader_timeout', 6.0)   # s of silent leader TF before a follower takes over
        self.leader = self.get_parameter('leader').value
        self.followers = [f for f in str(self.get_parameter('followers').value).split(',') if f]
        self.spacing = float(self.get_parameter('spacing').value)
        self.goal_tol = float(self.get_parameter('goal_tol').value)
        self.control = str(self.get_parameter('control').value)
        self.kp = float(self.get_parameter('kp').value)
        self.vmax = float(self.get_parameter('vmax').value)
        self.accel = float(self.get_parameter('accel').value)
        self.dt = 1.0 / float(self.get_parameter('rate').value)
        self.use_cbf = bool(self.get_parameter('cbf').value) and cbf_filter is not None
        self.cbf_d = float(self.get_parameter('cbf_safe_dist').value)
        self._nb = {}                # ns -> (x, y, t, vx, vy) for finite-difference neighbour velocity
        self.cbf_hits = 0            # ticks where the filter altered a command (logged)
        self.cmd_pub = {f: self.create_publisher(Twist, f'/{f}/cmd_vel', 10) for f in self.followers}
        self.v_prev = {f: 0.0 for f in self.followers}
        self.off_cur = {}            # smoothed (back, lateral) slot offset per follower
        self.slot_map = {}           # k -> (x, y) slot in map, for the RViz overlay
        self._lead = None            # (x, y, heading) of the leader in map
        self.trail_pub = self.create_publisher(Path, '/convoy/trail', 1)
        self.viz_pub = self.create_publisher(MarkerArray, '/convoy/markers', 1)
        self._viz_n = 0
        self.min_resend_s = float(self.get_parameter('min_resend_s').value)
        self.hold_frac = float(self.get_parameter('hold_frac').value)
        self.goal_handle = {f: None for f in self.followers}
        self.last_sent_t = {f: -1e9 for f in self.followers}
        self.speed_gain = float(self.get_parameter('speed_gain').value)
        self.leader_timeout = float(self.get_parameter('leader_timeout').value)
        self.trail = Trail()
        self.speed = 0.0          # leader speed estimate (m/s)
        self._last_xy = None
        self._last_t = None
        self._last_ok = None      # sim time the leader TF was last fresh
        self.last_goal = {f: None for f in self.followers}
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.nav_clients = {f: ActionClient(self, NavigateToPose, f'/{f}/navigate_to_pose') for f in self.followers}
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self.tick)
        self.get_logger().info(f'leader={self.leader} followers={self.followers} spacing={self.spacing} m')

    def xy_of(self, ns):
        try:
            t = self.tf.lookup_transform('map', f'{ns}/base_link', rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:  # noqa: BLE001
            return None

    def map_from_odom(self):
        """(tx, ty, yaw) of the leader's odom frame in map, or None. Jumps whenever localization corrects."""
        try:
            t = self.tf.lookup_transform('map', f'{self.leader}/odom', rclpy.time.Time())
            q = t.transform.rotation
            return (t.transform.translation.x, t.transform.translation.y,
                    math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        except Exception:  # noqa: BLE001
            return None

    def leader_xy(self):
        """Leader position in ITS ODOM FRAME (continuous, never jumps). The trail lives in this frame and is
        projected into map only when goals are sent, so an AMCL/SLAM correction moves the whole trail
        consistently instead of teleporting a crumb."""
        try:
            t = self.tf.lookup_transform(f'{self.leader}/odom', f'{self.leader}/base_link', rclpy.time.Time())
            age = (self.get_clock().now() - rclpy.time.Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
            if age > self.leader_timeout:
                return None  # stale TF: a dead leader must not look alive
            return t.transform.translation.x, t.transform.translation.y
        except Exception:  # noqa: BLE001
            return None

    def pose_of(self, ns):
        try:
            t = self.tf.lookup_transform('map', f'{ns}/base_link', rclpy.time.Time())
            q = t.transform.rotation
            return (t.transform.translation.x, t.transform.translation.y,
                    math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        except Exception:  # noqa: BLE001
            return None

    def neighbours(self, f):
        """[(x, y, vx, vy)] of every other robot in the convoy (map frame, finite-difference velocity)."""
        now = self.get_clock().now().nanoseconds * 1e-9
        out = []
        for ns in [self.leader] + [o for o in self.followers if o != f]:
            p = self.pose_of(ns)
            if p is None:
                continue
            old = self._nb.get(ns)
            vx = vy = 0.0
            if old is not None:
                dt = now - old[2]
                if dt < 0.05:
                    vx, vy = old[3], old[4]
                    out.append((p[0], p[1], vx, vy)); continue
                vx, vy = (p[0] - old[0]) / dt, (p[1] - old[1]) / dt
            self._nb[ns] = (p[0], p[1], now, vx, vy)
            out.append((p[0], p[1], vx, vy))
        return out

    def safe(self, f, fp, v, w):
        """CBF-filter a follower command so no pair of robots ever closes inside cbf_safe_dist."""
        if not self.use_cbf:
            return v, w
        v2, w2, n_inf = cbf_filter((v, w), fp, self.neighbours(f), safe_dist=self.cbf_d,
                                   max_v=max(self.vmax, 0.1))
        if abs(v2 - v) > 1e-3 or abs(w2 - w) > 1e-3:
            self.cbf_hits += 1
            if self.cbf_hits % 20 == 1:
                self.get_logger().info(f'CBF {f}: v {v:.2f}->{v2:.2f} w {w:.2f}->{w2:.2f} (hits={self.cbf_hits})')
        return (0.0, 0.0) if n_inf else (v2, w2)

    def drive(self, f, k, T, gap, prev_ahead):
        """Direct gap-regulated pursuit of the trail (trail projected to map with the current correction)."""
        fp = self.pose_of(f)
        if fp is None or len(self.trail.pts) < 2:
            return
        form = str(self.get_parameter('formation').value)
        if form != 'column' and form in FORMATIONS and self._lead is not None:
            return self.drive_formation(f, k, form, gap, fp)
        tm = Trail()
        tm.pts = [se2_apply(*T, x, y)[:2] for x, y in self.trail.pts]
        i = tm.project(fp[0], fp[1])
        slot = tm.point_behind(gap * k)
        if slot is not None:
            self.slot_map[k] = slot[:2]
        err = tm.arclen_from(i) - gap * k               # > 0: too far behind its slot
        too_close = math.hypot(fp[0] - prev_ahead[0], fp[1] - prev_ahead[1]) < self.hold_frac * gap
        v_des = 0.0 if too_close else gap_speed(self.speed, err, self.kp, self.vmax)
        dv = self.accel * self.dt
        v = self.v_prev[f] + max(-dv, min(dv, v_des - self.v_prev[f]))
        self.v_prev[f] = v
        tx, ty = tm.lookahead(i, max(0.5, min(1.2, 0.5 + 0.6 * v)))
        v_cmd, w = pursuit_cmd(fp[0], fp[1], fp[2], tx, ty, v)
        v_cmd, w = self.safe(f, fp, v_cmd, w)
        msg = Twist()
        msg.linear.x, msg.angular.z = v_cmd, w
        self.cmd_pub[f].publish(msg)

    def drive_formation(self, f, k, form, gap, fp):
        """Slot-based pursuit: hold this follower's slot in the leader-relative formation."""
        lx, ly, th = self._lead
        tgt = formation_slots(form, len(self.followers), gap)[k - 1]
        cur = self.off_cur.get(f, tgt)
        step = float(self.get_parameter('transition_speed').value) * self.dt   # glide, never snap
        d = math.hypot(tgt[0] - cur[0], tgt[1] - cur[1])
        if d > step:
            cur = (cur[0] + (tgt[0] - cur[0]) * step / d, cur[1] + (tgt[1] - cur[1]) * step / d)
        else:
            cur = tgt
        cur = keep_clear(cur, 0.8 * gap)
        self.off_cur[f] = cur
        sx, sy = slot_world(lx, ly, th, cur[0], cur[1])
        self.slot_map[k] = (sx, sy)
        dx, dy = sx - fp[0], sy - fp[1]
        dist = math.hypot(dx, dy)
        ahead = (dx * math.cos(fp[2]) + dy * math.sin(fp[2])) > 0.0
        v_des = max(0.0, min(self.vmax, self.speed + self.kp * dist)) if ahead and dist > 0.08 else 0.0
        if dist < 0.12:
            v_des = 0.0
        dv = self.accel * self.dt
        v = self.v_prev[f] + max(-dv, min(dv, v_des - self.v_prev[f]))
        self.v_prev[f] = v
        v_cmd, w = pursuit_cmd(fp[0], fp[1], fp[2], sx, sy, v)
        v_cmd, w = self.safe(f, fp, v_cmd, w)
        msg = Twist()
        msg.linear.x, msg.angular.z = v_cmd, w
        self.cmd_pub[f].publish(msg)

    def publish_viz(self, T, gap, robot_xy):
        """Overlays: leader trail, a gap line + distance label between consecutive robots, target-slot spheres."""
        now = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id, path.header.stamp = 'map', now
        for x, y in self.trail.pts[::3]:
            mx, my, _ = se2_apply(*T, x, y)
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x, ps.pose.position.y = mx, my
            path.poses.append(ps)
        self.trail_pub.publish(path)
        arr = MarkerArray()
        form = str(self.get_parameter('formation').value)
        for k in range(1, len(robot_xy)):
            if form != 'column' and form in FORMATIONS and self.followers[k - 1] in self.off_cur:
                a, b = robot_xy[0], robot_xy[k]            # leader -> follower, wanted distance = its slot offset
                want = math.hypot(*self.off_cur[self.followers[k - 1]])
            else:
                a, b = robot_xy[k - 1], robot_xy[k]        # column: consecutive robots, wanted distance = gap
                want = gap
            if a is None or b is None:
                continue
            d = math.hypot(a[0] - b[0], a[1] - b[1])
            r, g, bl = gap_color(d - want)
            line = Marker()
            line.header.frame_id, line.header.stamp, line.ns, line.id = 'map', now, 'gap', k
            line.type, line.action, line.scale.x = Marker.LINE_STRIP, Marker.ADD, 0.06
            line.color.r, line.color.g, line.color.b, line.color.a = r, g, bl, 1.0
            line.points = [Point(x=a[0], y=a[1], z=0.15), Point(x=b[0], y=b[1], z=0.15)]
            arr.markers.append(line)
            txt = Marker()
            txt.header.frame_id, txt.header.stamp, txt.ns, txt.id = 'map', now, 'gap_txt', k
            txt.type, txt.action, txt.scale.z = Marker.TEXT_VIEW_FACING, Marker.ADD, 0.28
            txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2 + 0.3, 0.4
            txt.color.r, txt.color.g, txt.color.b, txt.color.a = 1.0, 0.05, 0.05, 1.0   # red labels
            txt.text = f'{d:.2f} m'
            arr.markers.append(txt)
        for k, (mx, my) in sorted(self.slot_map.items()):
            if k > len(self.followers):
                continue
            sph = Marker()
            sph.header.frame_id, sph.header.stamp, sph.ns, sph.id = 'map', now, 'slot', k
            sph.type, sph.action = Marker.SPHERE, Marker.ADD
            sph.scale.x = sph.scale.y = sph.scale.z = 0.18
            sph.pose.position.x, sph.pose.position.y, sph.pose.position.z = mx, my, 0.1
            sph.color.r, sph.color.g, sph.color.b, sph.color.a = 1.0, 0.6, 0.1, 0.9   # orange slots (no blue)
            arr.markers.append(sph)
        self.viz_pub.publish(arr)

    def failover(self):
        """Leader lost: the follower nearest the head of the convoy becomes the new leader."""
        if not self.followers:
            return
        old, new = self.leader, self.followers.pop(0)
        self.leader = new
        self.last_goal.pop(new, None)
        self.nav_clients.pop(new, None)
        self.goal_handle.pop(new, None)
        self.cmd_pub.pop(new, None)
        self.v_prev.pop(new, None)
        self.trail = Trail()
        self._last_xy = self._last_t = self._last_ok = None
        self.get_logger().warn(f'leader {old} silent > {self.leader_timeout:.0f} s — {new} promoted to leader '
                               f'(followers now {self.followers})')

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        p = self.leader_xy()
        if p is None:
            if self._last_ok is None:
                self._last_ok = now  # grace period from first tick
            elif now - self._last_ok > self.leader_timeout:
                self.failover()
            self.get_logger().warn('no fresh map->%s/base_link TF' % self.leader, throttle_duration_sec=5.0)
            return
        self._last_ok = now
        if self._last_xy is not None and now > self._last_t:
            v = math.hypot(p[0] - self._last_xy[0], p[1] - self._last_xy[1]) / (now - self._last_t)
            self.speed = 0.7 * self.speed + 0.3 * v   # low-pass: tick jitter must not wobble the spacing
        self._last_xy, self._last_t = p, now
        gap = adaptive_spacing(self.spacing, self.speed_gain, self.speed)
        T = self.map_from_odom()
        if T is None:
            self.get_logger().warn('no map->%s/odom TF (localization not up)' % self.leader, throttle_duration_sec=5.0)
            return
        self.trail.add(*p)
        p_map = se2_apply(*T, p[0], p[1])[:2]
        hb = self.trail.point_behind(0.3)
        th = (hb[2] + T[2]) if hb is not None else (self._lead[2] if self._lead else 0.0)
        self._lead = (p_map[0], p_map[1], math.atan2(math.sin(th), math.cos(th)))
        self.trail.trim(gap * (len(self.followers) + 2) + 5.0)
        ahead = p_map
        robot_xy = [p_map]
        for k, f in enumerate(self.followers, start=1):
            fxy = self.xy_of(f)
            prev_ahead, ahead = ahead, fxy if fxy is not None else ahead
            robot_xy.append(fxy)
            if self.control == 'direct':
                self.drive(f, k, T, gap, prev_ahead)
                continue
            if fxy is not None and math.hypot(fxy[0] - prev_ahead[0], fxy[1] - prev_ahead[1]) < self.hold_frac * gap:
                gh = self.goal_handle.get(f)   # too close to the robot ahead: stop instead of pushing on
                if gh is not None:
                    gh.cancel_goal_async()
                    self.goal_handle[f] = None
                    self.last_goal[f] = None
                continue
            tgt = self.trail.point_behind(gap * k)
            if tgt is None or not self.nav_clients[f].server_is_ready():
                continue
            tgt = se2_apply(*T, *tgt)   # odom-frame trail point -> map frame, using the current correction
            last = self.last_goal[f]
            if last is not None and math.hypot(tgt[0] - last[0], tgt[1] - last[1]) < self.goal_tol:
                continue
            if now - self.last_sent_t[f] < self.min_resend_s:
                continue
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = tgt[0], tgt[1]
            goal.pose.pose.orientation.z = math.sin(tgt[2] / 2)
            goal.pose.pose.orientation.w = math.cos(tgt[2] / 2)
            fut = self.nav_clients[f].send_goal_async(goal)  # Nav2 replaces the active goal on a new one
            fut.add_done_callback(lambda r, f=f: self.goal_handle.__setitem__(f, r.result() if r.result() and r.result().accepted else None))
            self.last_goal[f] = tgt
            self.last_sent_t[f] = now
            self.get_logger().info(f'{f} -> ({tgt[0]:.2f}, {tgt[1]:.2f}) [{k * gap:.1f} m behind {self.leader}]',
                                   throttle_duration_sec=2.0)
        self._viz_n += 1
        if self._viz_n % 2 == 0:
            self.publish_viz(T, gap, robot_xy)


def main():
    rclpy.init()
    n = LeaderFollower()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
