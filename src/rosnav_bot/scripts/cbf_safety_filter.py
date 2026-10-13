#!/usr/bin/env python3
"""
cbf_safety_filter.py — control-barrier-function filter that keeps one robot clear of the others.

Sits between a robot's planner and its wheels: reads `cmd_vel_in`, outputs the closest safe `cmd_vel_out`.
For each neighbour j (position from TF, `map` -> `<ns>/base_link`), with p = x_i - x_j and
h = |p|^2 - D^2 (D = safety distance), the command must satisfy  2 p.(v_i - v_j) + alpha*h >= 0,
so the gap can shrink only as fast as alpha allows and never below D. Robot i is tracked at a point
`look` metres ahead so a differential-drive robot (v, w) can steer away: v_pt = A u, A = [[1,0],[0,look]].
Solved as a small QP by sequential projection onto each half-space.

Parameters: self_ns, neighbors (list of namespaces), safe_dist (0.8), alpha (1.0), look (0.25),
            max_v (0.5), max_w (1.5)
Not wired into the launch files yet; remap Nav2's cmd_vel into `cmd_vel_in` to use it.
  ros2 run rosnav_bot cbf_safety_filter.py --ros-args -p self_ns:=robot1 -p neighbors:="['robot2','robot3']" \\
      -r cmd_vel_in:=/robot1/cmd_vel_nav -r cmd_vel_out:=/robot1/cmd_vel
"""
import math


def cbf_filter(u_ref, self_pose, neighbors, safe_dist=0.8, alpha=1.0, look=0.25, max_v=0.5, max_w=1.5):
    """u_ref=(v,w); self_pose=(x,y,yaw); neighbors=[(x,y,vx,vy)] in the world frame. Returns (v,w, n_active)."""
    x, y, yaw = self_pose
    c, s = math.cos(yaw), math.sin(yaw)
    px, py = x + look * c, y + look * s                      # tracked point
    cons = []                                                # a1*v + a2*w >= b
    for nx, ny, nvx, nvy in neighbors:
        rx, ry = px - nx, py - ny
        h = rx * rx + ry * ry - safe_dist ** 2
        # point velocity = v*(c,s) + w*look*(-s,c)
        a1 = 2 * (rx * c + ry * s)
        a2 = 2 * look * (-rx * s + ry * c)
        b = 2 * (rx * nvx + ry * nvy) - alpha * h
        cons.append((a1, a2, b))
    v, w = u_ref
    active = 0
    for _ in range(20):
        ok = True
        for a1, a2, b in cons:
            g = a1 * v + a2 * w - b
            n2 = a1 * a1 + a2 * a2
            if g < -1e-9 and n2 > 1e-9:
                v, w = v - g * a1 / n2, w - g * a2 / n2
                ok = False
        v, w = max(-max_v, min(max_v, v)), max(-max_w, min(max_w, w))
        if ok:
            break
    for a1, a2, b in cons:
        if a1 * v + a2 * w - b < -1e-3:
            active += 1                                     # infeasible with limits: caller should stop
    return v, w, active


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    import tf2_ros

    class Node_(Node):
        def __init__(self):
            super().__init__('cbf_safety_filter')
            d = {'self_ns': 'robot1', 'neighbors': ['robot2'], 'safe_dist': 0.8, 'alpha': 1.0,
                 'look': 0.25, 'max_v': 0.5, 'max_w': 1.5}
            for k, v in d.items():
                self.declare_parameter(k, v)
            g = lambda k: self.get_parameter(k).value
            self.ns, self.nb = g('self_ns'), list(g('neighbors'))
            self.k = dict(safe_dist=g('safe_dist'), alpha=g('alpha'), look=g('look'),
                          max_v=g('max_v'), max_w=g('max_w'))
            self.buf = tf2_ros.Buffer()
            self.tl = tf2_ros.TransformListener(self.buf, self)
            self.prev = {}
            self.pub = self.create_publisher(Twist, 'cmd_vel_out', 10)
            self.create_subscription(Twist, 'cmd_vel_in', self.cb, 10)
            self.last_log = 0.0
            self.get_logger().info(f'CBF filter for {self.ns}, neighbours {self.nb}, {self.k}')

        def pose(self, ns):
            t = self.buf.lookup_transform('map', f'{ns}/base_link', rclpy.time.Time()).transform
            q = t.rotation
            return t.translation.x, t.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        def cb(self, msg):
            now = self.get_clock().now().nanoseconds / 1e9
            try:
                me = self.pose(self.ns)
                others = []
                for n in self.nb:
                    x, y, _ = self.pose(n)
                    px, py, pt = self.prev.get(n, (x, y, now))
                    dt = now - pt
                    vx, vy = ((x - px) / dt, (y - py) / dt) if dt > 0.05 else (0.0, 0.0)
                    if dt > 0.05 or n not in self.prev:
                        self.prev[n] = (x, y, now)
                    others.append((x, y, vx, vy))
            except Exception as e:
                self.pub.publish(msg)                       # no TF yet: pass through, Nav2's collision monitor still runs
                return
            v, w, bad = cbf_filter((msg.linear.x, msg.angular.z), me, others, **self.k)
            out = Twist(); out.linear.x, out.angular.z = v, w
            if bad:
                out.linear.x = 0.0
            if abs(v - msg.linear.x) + abs(w - msg.angular.z) > 0.02 and now - self.last_log > 5.0:
                self.last_log = now
                self.get_logger().info(f'{self.ns}: CBF adjusted ({msg.linear.x:.2f},{msg.angular.z:.2f}) -> ({v:.2f},{w:.2f})')
            self.pub.publish(out)

    rclpy.init()
    n = Node_()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
