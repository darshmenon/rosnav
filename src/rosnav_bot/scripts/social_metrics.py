#!/usr/bin/env python3
"""
social_metrics.py — scores a Nav2 run among pedestrians (ground truth from social_force_peds.py).

Metrics (the ones named in the evaluation plan): collision count (contact episodes,
centre distance < robot_r + ped_r), minimum clearance, personal-space intrusion
seconds (centre distance < 1.2 m, Hall's social distance), path length, mean speed,
smoothness (mean |d speed/dt|), run time. Writes JSON when `duration` elapses or on Ctrl-C.

  ros2 run rosnav_bot social_metrics.py --ros-args -p duration:=120.0 -p label:=plain_nav2 \
      -p out:=/tmp/run_plain.json
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String


class Metrics(Node):
    def __init__(self):
        super().__init__('social_metrics')
        dyn = ParameterDescriptor(dynamic_typing=True)     # accept 30 or 30.0 from the command line
        for k, v in dict(duration=120.0, label='run', out='social_metrics.json', robot_r=0.30,
                         ped_r=0.22, personal_space=1.2, map_frame='map',
                         base_frame='base_link', warmup=5.0).items():
            self.declare_parameter(k, v, dyn if isinstance(v, float) else ParameterDescriptor())
        g = lambda k: self.get_parameter(k).value
        self.p = {k: (float(g(k)) if k in ('duration', 'robot_r', 'ped_r', 'personal_space', 'warmup') else g(k))
                  for k in ('duration', 'label', 'out', 'robot_r', 'ped_r', 'personal_space', 'warmup')}
        import tf2_ros
        self.buf = tf2_ros.Buffer()
        self.tl = tf2_ros.TransformListener(self.buf, self)
        self.mf, self.bf = g('map_frame'), g('base_frame')
        self.peds = []
        self.create_subscription(String, '/social_peds/gt', self._on_gt, 10)
        self.t0 = time.monotonic()
        self.last_t, self.last_xy, self.last_speed = None, None, 0.0
        self.m = dict(collisions=0, moving_collisions=0, min_clearance=1e9, intrusion_s=0.0, path_m=0.0, jerk_sum=0.0,
                      samples=0, in_contact=False)
        self.create_timer(0.1, self._tick)
        self.get_logger().info(f'recording "{self.p["label"]}" for {self.p["duration"]} s -> {self.p["out"]}')

    def _on_gt(self, msg):
        try:
            self.peds = json.loads(msg.data)['peds']
        except (json.JSONDecodeError, KeyError):
            pass

    def _tick(self):
        el = time.monotonic() - self.t0
        if el >= self.p['duration'] + self.p['warmup']:
            self._finish()
            return
        if el < self.p['warmup'] or not self.peds:
            return
        try:
            t = self.buf.lookup_transform(self.mf, self.bf, rclpy.time.Time()).transform
        except Exception:  # noqa: BLE001
            return
        xy = (t.translation.x, t.translation.y)
        now = time.monotonic()
        m = self.m
        if self.last_xy is not None:
            dt = max(1e-3, now - self.last_t)
            d = math.hypot(xy[0] - self.last_xy[0], xy[1] - self.last_xy[1])
            m['path_m'] += d
            sp = d / dt
            m['jerk_sum'] += abs(sp - self.last_speed) / dt
            self.last_speed = sp
            m['intrusion_s'] += dt * (self._nearest(xy) < self.p['personal_space'])
        self.last_t, self.last_xy = now, xy
        near = self._nearest(xy)
        clear = near - self.p['robot_r'] - self.p['ped_r']
        m['min_clearance'] = min(m['min_clearance'], clear)
        hit = clear < 0.0
        if hit and not m['in_contact']:
            m['collisions'] += 1                               # one per contact episode
            if self.last_speed > 0.1:
                m['moving_collisions'] += 1                    # robot was driving into them (vs. being bumped while still)
        m['in_contact'] = hit
        m['samples'] += 1

    def _nearest(self, xy):
        return min(math.hypot(p['x'] - xy[0], p['y'] - xy[1]) for p in self.peds)

    def _finish(self):
        m = self.m
        n = max(1, m['samples'])
        dur = self.p['duration']
        res = dict(label=self.p['label'], duration_s=dur, collisions=m['collisions'], moving_collisions=m['moving_collisions'],
                   min_clearance_m=round(m['min_clearance'], 3) if m['min_clearance'] < 1e8 else None,
                   intrusion_s=round(m['intrusion_s'], 1), path_m=round(m['path_m'], 2),
                   mean_speed=round(m['path_m'] / dur, 3), smoothness=round(m['jerk_sum'] / n, 3),
                   samples=m['samples'])
        json.dump(res, open(self.p['out'], 'w'), indent=1)
        self.get_logger().info(f'RESULT {json.dumps(res)}')
        raise SystemExit(0)


def main():
    rclpy.init()
    n = Metrics()
    try:
        try:
            rclpy.spin(n)
        except Exception:  # noqa: BLE001 - a signal can invalidate the context mid-spin
            if rclpy.ok():
                raise
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
