#!/usr/bin/env python3
"""Send every robot its goal for a decentral scenario (swap/corridor/crowd) over Nav2 and score the run.

  ros2 run rosnav_bot decentral_gz_run.py --ros-args -p scenario:=corridor -p n:=4 -p use_sim_time:=true

Per robot: NavigateToPose to its goal in /<robot>/navigate_to_pose. While running, pose of every robot is read from TF
(map -> <robot>/base_link) at 10 Hz to get the minimum inter-robot distance and 'collision pairs' (< collide_dist).
Writes runs/decentral_gz/<scenario>_<n>_<ctrl>.json and prints one summary line. Timeout aborts unfinished robots.
"""
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import decentral_gz_scenarios as sc  # noqa: E402


def person_distances(robots, peds, limit):
    """robots {name: (x, y) | None}, peds [(id, x, y)] -> (min distance or inf, {(robot, ped id) closer than limit})."""
    best, hits = float('inf'), set()
    for nm, pos in robots.items():
        if not pos:
            continue
        for pid, px, py in peds:
            d = math.hypot(pos[0] - px, pos[1] - py)
            best = min(best, d)
            if d < limit:
                hits.add((nm, pid))
    return best, hits


class Runner(Node):
    def __init__(self):
        super().__init__('decentral_gz_run')
        self.declare_parameter('scenario', 'corridor')
        self.declare_parameter('n', 4)
        self.declare_parameter('timeout_s', 240.0)
        self.declare_parameter('collide_dist', 0.35)
        self.declare_parameter('label', 'vo')
        self.declare_parameter('out_dir', 'runs/decentral_gz')
        self.declare_parameter('swarm_form', 'wedge')
        self.declare_parameter('swarm_spacing', 1.5)
        kind = self.get_parameter('scenario').value
        n = int(self.get_parameter('n').value)
        self.s = sc.build(kind, n)
        self.names = [r['name'] for r in self.s['robots']]
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, self)
        self.ac = {nm: ActionClient(self, NavigateToPose, f'/{nm}/navigate_to_pose') for nm in self.names}
        self.res = {nm: dict(done=False, ok=False, t=None, retries=0) for nm in self.names}
        self.goals = {r['name']: g for r, g in zip(self.s['robots'], self.s['goals'])}
        self.retry_at = {}
        self.sent_at = {}                       # goals sent but not yet accepted/rejected (response can be lost)
        self.declare_parameter('retry_s', 3.0)
        self.min_pair = float('inf')
        self.coll_pairs = set()
        self.t0 = None
        self.peds = []                          # latest /social_peds/gt positions (empty without walking people)
        self.min_person = float('inf')
        self.person_hits = set()                # (robot, ped id) pairs closer than person_dist
        self.declare_parameter('person_dist', 0.5)
        self.create_subscription(String, '/social_peds/gt', self._gt, 10)
        self.create_timer(0.1, self._track)

    def _gt(self, msg):
        try:
            self.peds = [(int(q['id']), float(q['x']), float(q['y'])) for q in json.loads(msg.data)['peds']]
        except (ValueError, KeyError, TypeError):
            pass

    def _pos(self, nm):
        try:
            t = self.tf.lookup_transform('map', f'{nm}/base_link', rclpy.time.Time()).transform.translation
            return t.x, t.y
        except Exception:
            return None

    def _track(self):
        if self.t0 is None:
            return
        p = {nm: self._pos(nm) for nm in self.names}
        cd = float(self.get_parameter('collide_dist').value)
        for i, a in enumerate(self.names):
            for b in self.names[i + 1:]:
                if p[a] and p[b]:
                    d = math.hypot(p[a][0] - p[b][0], p[a][1] - p[b][1])
                    self.min_pair = min(self.min_pair, d)
                    if d < cd:
                        self.coll_pairs.add((a, b))
        pd = float(self.get_parameter('person_dist').value)
        d, hits = person_distances(p, self.peds, pd)
        self.min_person = min(self.min_person, d)
        self.person_hits |= hits

    def run(self):
        log = self.get_logger()
        for nm, ac in self.ac.items():
            if not ac.wait_for_server(timeout_sec=120.0):
                log.error(f'{nm}: navigate_to_pose not available')
                return 2
        log.info(f"{self.s['kind']} n={len(self.names)}: sending goals")
        self.t0 = time.monotonic()
        for nm in self.names:
            self._send(nm)
        end = self.t0 + float(self.get_parameter('timeout_s').value)
        while rclpy.ok() and time.monotonic() < end and not all(v['done'] and (v['ok'] or nm not in self.retry_at)
                                                                for nm, v in self.res.items()):
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.monotonic()
            for nm, at in list(self.retry_at.items()):
                if now >= at:                       # Nav2 aborted (planner/BT gave up): head-on blockage is transient
                    del self.retry_at[nm]
                    self.res[nm]['retries'] += 1
                    self._send(nm)
            for nm, at in list(self.sent_at.items()):
                if now - at > 10.0:                 # no accept/reject response (dropped under load): send again
                    self.get_logger().warn(f'{nm}: no goal response after 10 s, resending')
                    self.res[nm]['retries'] += 1
                    self._send(nm)
        return self._report()

    def _send(self, nm):
        g = self.goals[nm]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(g[0]), float(g[1])
        goal.pose.pose.orientation.w = 1.0
        self.res[nm]['done'] = False
        self.sent_at[nm] = time.monotonic()
        self.ac[nm].send_goal_async(goal).add_done_callback(lambda f, nm=nm: self._accepted(nm, f))

    def _accepted(self, nm, fut):
        self.sent_at.pop(nm, None)
        h = fut.result()
        if h is None or not h.accepted:
            self.retry_at[nm] = time.monotonic() + float(self.get_parameter('retry_s').value)
            return
        h.get_result_async().add_done_callback(lambda f, nm=nm: self._result(nm, f))

    def _result(self, nm, fut):
        r = self.res[nm]
        r['done'] = True
        r['ok'] = fut.result().status == 4
        r['t'] = round(time.monotonic() - self.t0, 1)
        if not r['ok']:
            self.retry_at[nm] = time.monotonic() + float(self.get_parameter('retry_s').value)

    def _swarm_metrics(self):
        """Final formation quality from TF: centroid-to-goal distance and mean slot error of the best-fit slots."""
        import numpy as np
        import decentral_nav_node as nd
        pts = [self._pos(nm) for nm in self.names]
        pts = [p for p in pts if p]
        if len(pts) < 2:
            return dict(swarm_ok=False)
        goal = np.array(self.s['goals'][0], float)
        cen = np.mean(pts, axis=0)
        h = math.atan2(goal[1] - self.s['robots'][0]['y'], goal[0] - self.s['robots'][0]['x'])
        form = self.get_parameter('swarm_form').value
        slots = nd.swarm_slots(tuple(cen), h, nd.swarm_offsets(form, len(pts), float(self.get_parameter('swarm_spacing').value)))
        asg = nd.assign_slots(pts, slots)
        err = [math.hypot(pts[i][0] - slots[asg[i]][0], pts[i][1] - slots[asg[i]][1]) for i in range(len(pts))]
        m = dict(swarm_form=form, centroid_goal_m=round(float(np.linalg.norm(cen - goal)), 2),
                 slot_err_mean_m=round(float(np.mean(err)), 2), slot_err_max_m=round(float(np.max(err)), 2),
                 final_xy=[[round(p[0], 2), round(p[1], 2)] for p in pts])
        m['swarm_ok'] = m['centroid_goal_m'] < 1.5 and self.min_pair > 0.35
        self.get_logger().info(f"SWARM {form}: centroid {m['centroid_goal_m']} m from goal, slot err mean/max "
                               f"{m['slot_err_mean_m']}/{m['slot_err_max_m']} m, min pair {round(self.min_pair, 2)} m, ok={m['swarm_ok']}")
        return m

    def _report(self):
        ok = sum(v['ok'] for v in self.res.values())
        n = len(self.names)
        mk = max((v['t'] for v in self.res.values() if v['ok']), default=None)
        out = dict(scenario=self.s['kind'], n=n, ctrl=self.get_parameter('label').value, success=ok,
                   makespan_s=mk, min_pair_m=None if self.min_pair == float('inf') else round(self.min_pair, 2),
                   collision_pairs=len(self.coll_pairs), robots=self.res,
                   min_person_m=None if self.min_person == float('inf') else round(self.min_person, 2),
                   person_collisions=len(self.person_hits))
        if self.s['kind'] == 'swarm':
            out.update(self._swarm_metrics())
        d = self.get_parameter('out_dir').value
        os.makedirs(d, exist_ok=True)
        path = os.path.join(d, f"{out['scenario']}_{n}_{out['ctrl']}.json")
        with open(path, 'w') as f:
            json.dump(out, f, indent=1)
        self.get_logger().info(f"RESULT {out['scenario']} n={n} ctrl={out['ctrl']}: {ok}/{n} reached, makespan={mk}s, "
                               f"min pair dist={out['min_pair_m']} m, collision pairs={out['collision_pairs']}, "
                               f"min person dist={out['min_person_m']} m, "
                               f"person hits={out['person_collisions']} -> {path}")
        return 0 if ok == n else 1


def main():
    rclpy.init()
    node = Runner()
    rc = node.run()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
