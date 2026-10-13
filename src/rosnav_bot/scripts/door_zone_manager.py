#!/usr/bin/env python3
"""
door_zone_manager.py — one robot at a time through a doorway / narrow corridor.

Each zone has a centre, a radius (the doorway itself) and an approach radius. A robot that comes within the
approach radius queues (first come, first served); the front of the queue holds the zone and the others get
`hold=True` until the holder has left the zone plus `clear_margin` metres. A holder that has not moved for
`timeout` seconds loses the zone (stuck robot must not block the fleet forever).

Node: reads each robot's pose from TF (`map` -> `<ns>/base_link`), publishes
  /door_zones/status  (std_msgs/String JSON: per zone holder + queue)
  /<ns>/door_hold     (std_msgs/Bool) — True = stop before the zone (use as a speed limit / goal gate)
Parameters: robots ['robot1','robot2'], zones_json '[{"name":"living_door","x":-4.75,"y":1.5,"r":0.8,"approach":2.5}]',
            clear_margin 0.5, timeout 40.
Not wired into launch files. Offline test: python3 door_zone_manager.py --selftest
"""
import json
import math
import sys


class Zone:
    def __init__(self, name, x, y, r, approach, clear_margin=0.5, timeout=40.0):
        self.name, self.x, self.y, self.r, self.approach = name, x, y, r, approach
        self.clear, self.timeout = clear_margin, timeout
        self.holder, self.queue, self._moved_at, self._last, self.entered = None, [], 0.0, None, False
        self.passed = set()

    def d(self, p):
        return math.hypot(p[0] - self.x, p[1] - self.y)

    def update(self, poses, now):
        """poses {robot: (x, y)} -> {robot: hold(bool)} for robots affected by this zone."""
        # holder released after it has gone through (entered the zone, then > r + clear_margin away), if it turns
        # back (leaves the approach ring without entering), disappears, or has not moved for `timeout` seconds
        if self.holder is not None:
            p = poses.get(self.holder)
            if p is not None and self.d(p) <= self.r:
                self.entered = True
            if (p is None or (self.entered and self.d(p) > self.r + self.clear)
                    or (not self.entered and self.d(p) > self.approach) or now - self._moved_at > self.timeout):
                if self.entered:
                    self.passed.add(self.holder)           # ignore it until it leaves the approach ring
                self.holder, self.entered = None, False
            elif self._last is None or math.hypot(p[0] - self._last[0], p[1] - self._last[1]) > 0.05:
                self._last, self._moved_at = p, now
        # queue: everyone inside the approach ring, in arrival order
        self.passed = {n for n in self.passed if n in poses and self.d(poses[n]) <= self.approach}
        near = {n for n, p in poses.items() if self.d(p) <= self.approach and n not in self.passed}
        self.queue = [n for n in self.queue if n in near and n != self.holder]
        for n in sorted(near, key=lambda n: self.d(poses[n])):
            if n not in self.queue and n != self.holder:
                self.queue.append(n)
        if self.holder is None and self.queue:
            self.holder = self.queue.pop(0)
            self._moved_at, self._last, self.entered = now, poses[self.holder], False
        hold = {n: True for n in self.queue}
        if self.holder:
            hold[self.holder] = False
        return hold


def selftest():
    z = Zone('door', 0.0, 0.0, 0.8, 2.5, clear_margin=0.5, timeout=10)
    # robot A at x=-2 heading east, robot B at x=+2 heading west: both approach the door
    a, b, now, log, dmin = [-2.0, 0.0], [2.0, 0.0], 0.0, [], 9
    for i in range(300):
        now += 0.1
        hold = z.update({'A': tuple(a), 'B': tuple(b)}, now)
        if not hold.get('A', False) and a[0] < 3.5: a[0] += 0.05
        if not hold.get('B', False) and b[0] > -3.5: b[0] -= 0.05
        both_in = z.d(a) < z.r and z.d(b) < z.r
        assert not both_in, f'both robots inside the door at t={now:.1f}'
        log.append((z.holder, dict(hold)))
    holders = []
    for h, _ in log:
        if h and (not holders or holders[-1] != h): holders.append(h)
    print('holder order:', holders, '| B waited while A crossed:', any(hh.get('B') for _, hh in log))
    assert holders[0] in ('A', 'B') and len(holders) == 2
    # stuck holder is released after the timeout
    z2 = Zone('d', 0, 0, 0.8, 2.5, timeout=5)
    t = 0.0
    for i in range(100):
        t += 0.1; h = z2.update({'S': (0.1, 0.0), 'W': (1.5, 0.0)}, t)
    print('stuck holder released after timeout, holder now:', z2.holder, '(W goes next:', z2.holder == 'W' or 'S' not in (z2.holder or ''), ')')
    print('selftest OK')


def main():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
    import tf2_ros

    class N(Node):
        def __init__(self):
            super().__init__('door_zone_manager')
            self.declare_parameter('robots', ['robot1', 'robot2'])
            self.declare_parameter('zones_json', '[{"name":"living_door","x":-4.75,"y":1.5,"r":0.8,"approach":2.5}]')
            self.declare_parameter('clear_margin', 0.5)
            self.declare_parameter('timeout', 40.0)
            self.robots = list(self.get_parameter('robots').value)
            cm, to = self.get_parameter('clear_margin').value, self.get_parameter('timeout').value
            self.zones = [Zone(z['name'], z['x'], z['y'], z['r'], z['approach'], cm, to)
                          for z in json.loads(self.get_parameter('zones_json').value)]
            self.buf = tf2_ros.Buffer(); self.tl = tf2_ros.TransformListener(self.buf, self)
            self.holds = {r: self.create_publisher(Bool, f'/{r}/door_hold', 10) for r in self.robots}
            self.status = self.create_publisher(String, '/door_zones/status', 10)
            self.prev = {}
            self.create_timer(0.2, self.tick)
            self.get_logger().info(f'door zones {[z.name for z in self.zones]} for {self.robots}')

        def tick(self):
            now = self.get_clock().now().nanoseconds / 1e9
            poses = {}
            for r in self.robots:
                try:
                    t = self.buf.lookup_transform('map', f'{r}/base_link', rclpy.time.Time()).transform.translation
                    poses[r] = (t.x, t.y)
                except Exception:
                    pass
            hold = {r: False for r in self.robots}
            for z in self.zones:
                for r, h in z.update(poses, now).items():
                    hold[r] = hold[r] or h
            for r, h in hold.items():
                self.holds[r].publish(Bool(data=h))
                if self.prev.get(r) != h:
                    self.prev[r] = h
                    self.get_logger().info(f'{r}: {"HOLD before door" if h else "go"}')
            self.status.publish(String(data=json.dumps({z.name: {'holder': z.holder, 'queue': z.queue} for z in self.zones})))

    rclpy.init()
    n = N()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    selftest() if '--selftest' in sys.argv else main()
