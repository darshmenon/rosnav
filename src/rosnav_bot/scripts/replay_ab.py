#!/usr/bin/env python3
"""
replay_ab.py — identical-scene A/B comparison of social-navigation planners from a RECORDED pedestrian stream.

Why: a live Gazebo run never repeats (people react to the robot, timing jitters), so "plain vs cv vs learned" on
two live runs compares two different crowds. Here the pedestrian stream is recorded ONCE (the /social_peds/gt
ground truth, optionally the robot's odom/scan too) and every planner is replayed against exactly that stream, from
the same start poses, scored with the same metrics as social_metrics.py (contact episodes at centre distance
< 0.30+0.22 m, minimum clearance, seconds inside the 1.2 m personal space, path length, mean speed, smoothness).

LIMITATION (be upfront about it): the replay is OPEN-LOOP. Recorded people do not react to the replayed robot, so
absolute collision numbers are not a closed-loop result; use it for same-scene RELATIVE comparison and regression.

  record    ros2 bag record of the ground truth + robot topics (run next to a live sim)
  extract   bag -> recording.json  (/social_peds/gt, optionally /odom as the robot's recorded path)
  synthetic social-force crowd on a map -> recording.json (no ROS / Gazebo: testing and CI)
  replay    run planners against the recording, print a table, write JSON
  score     score the robot path stored in a recording (a recorded live run) with the same metrics

  scripts/replay_ab.py synthetic --map maps/map_hospital.yaml --peds 10 --seconds 60 --out /tmp/rec.json
  scripts/replay_ab.py replay /tmp/rec.json --map maps/map_hospital.yaml --planners reactive cv-predictive cv-risk \\
        --starts 6 --out /tmp/replay.json
  scripts/replay_ab.py replay /tmp/rec.json --planners learned-risk --weights outputs/ped_predictor_n24.pt

recording.json: {"meta": {...}, "t": [..], "peds": [[{"id","x","y"}, ...] per frame], "robot": [[x,y,yaw]|null per frame]}
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import social_nav_core as s  # noqa: E402

DT = 0.1
ROBOT_R, PED_R, PERSONAL_SPACE = 0.30, 0.22, 1.2          # same constants as social_metrics.py
DEFAULT_TOPICS = ['/social_peds/gt', '/odom', '/scan', '/tf', '/tf_static', '/cmd_vel', '/clock']


# ------------------------------------------------------------------ recording container
class Recording:
    """Dense, resampled-to-0.1 s pedestrian stream. pos[k, i] = (x, y) of ped ids[i] at time t[k] (NaN = absent)."""

    def __init__(self, t, ids, pos, robot=None, meta=None):
        self.t, self.ids, self.meta = np.asarray(t, float), list(ids), meta or {}
        self.pos = np.asarray(pos, float)
        self.robot = robot                                  # (K,3) array with NaN rows, or None
        self.pos.setflags(write=False)                      # planners must not be able to change the scene
        if self.robot is not None:
            self.robot = np.asarray(self.robot, float)
            self.robot.setflags(write=False)

    @property
    def steps(self):
        return len(self.t)

    def vel(self, k, win=3):
        """Finite-difference velocity of every ped at step k over `win` frames (NaN-safe)."""
        a = max(0, k - win)
        if k == a:
            return np.zeros((len(self.ids), 2))
        v = (self.pos[k] - self.pos[a]) / ((k - a) * DT)
        return np.nan_to_num(v)

    def fingerprint(self):
        import hashlib
        return hashlib.sha1(np.nan_to_num(self.pos).tobytes()).hexdigest()[:12]

    # -- io
    def save(self, path):
        frames = []
        for k in range(self.steps):
            frames.append([dict(id=int(i), x=round(float(p[0]), 3), y=round(float(p[1]), 3))
                           for i, p in zip(self.ids, self.pos[k]) if np.isfinite(p).all()])
        robot = None if self.robot is None else [None if np.isnan(r).any() else [round(float(v), 3) for v in r]
                                                 for r in self.robot]
        json.dump(dict(meta=self.meta, t=[round(float(x), 3) for x in self.t], peds=frames, robot=robot), open(path, 'w'))

    @classmethod
    def from_frames(cls, t, frames, robot=None, meta=None):
        """frames: list (per timestamp) of [{id,x,y}] at arbitrary stamps; resampled to a uniform 0.1 s grid."""
        t = np.asarray(t, float)
        order = np.argsort(t)
        t = t[order]
        frames = [frames[i] for i in order]
        ids = sorted({int(p['id']) for f in frames for p in f})
        idx = {i: j for j, i in enumerate(ids)}
        raw = np.full((len(t), len(ids), 2), np.nan)
        for k, f in enumerate(frames):
            for p in f:
                raw[k, idx[int(p['id'])]] = (p['x'], p['y'])
        grid = np.arange(t[0], t[-1] + 1e-9, DT)
        pos = np.full((len(grid), len(ids), 2), np.nan)
        for j in range(len(ids)):
            ok = np.isfinite(raw[:, j, 0])
            if ok.sum() >= 2:
                inside = (grid >= t[ok][0]) & (grid <= t[ok][-1])
                for c in (0, 1):
                    pos[inside, j, c] = np.interp(grid[inside], t[ok], raw[ok, j, c])
        rob = None
        if robot is not None:
            R = np.full((len(t), 3), np.nan)
            for k, r in enumerate([robot[i] for i in order]):
                if r is not None:
                    R[k] = r
            ok = np.isfinite(R[:, 0])
            rob = np.full((len(grid), 3), np.nan)
            if ok.sum() >= 2:
                inside = (grid >= t[ok][0]) & (grid <= t[ok][-1])
                for c in range(3):
                    rob[inside, c] = np.interp(grid[inside], t[ok], np.unwrap(R[ok, 2]) if c == 2 else R[ok, c])
        return cls(grid - grid[0], ids, pos, rob, meta)

    @classmethod
    def load(cls, path):
        d = json.load(open(path))
        return cls.from_frames(d['t'], d['peds'], d.get('robot'), d.get('meta'))


# ------------------------------------------------------------------ metrics (mirrors social_metrics.Metrics)
class Metrics:
    def __init__(self, dt=DT):
        self.dt, self.collisions, self.in_contact = dt, 0, False
        self.min_clearance, self.intrusion_s, self.path_m, self.jerk_sum, self.n = 1e9, 0.0, 0.0, 0.0, 0
        self.last_xy, self.last_speed, self.elapsed = None, 0.0, 0.0

    def update(self, xy, peds_xy):
        self.elapsed += self.dt
        peds_xy = peds_xy[np.isfinite(peds_xy).all(1)] if len(peds_xy) else peds_xy
        if self.last_xy is not None:
            d = math.hypot(xy[0] - self.last_xy[0], xy[1] - self.last_xy[1])
            sp = d / self.dt
            self.path_m += d
            self.jerk_sum += abs(sp - self.last_speed) / self.dt
            self.last_speed = sp
            if len(peds_xy):
                self.intrusion_s += self.dt * (np.hypot(*(peds_xy - xy).T).min() < PERSONAL_SPACE)
        self.last_xy = (float(xy[0]), float(xy[1]))
        if len(peds_xy):
            near = float(np.hypot(*(peds_xy - xy).T).min())
            clear = near - ROBOT_R - PED_R
            self.min_clearance = min(self.min_clearance, clear)
            hit = clear < 0.0
            self.collisions += int(hit and not self.in_contact)
            self.in_contact = hit
        self.n += 1

    def result(self):
        n = max(1, self.n)
        return dict(collisions=self.collisions,
                    min_clearance_m=round(self.min_clearance, 3) if self.min_clearance < 1e8 else None,
                    intrusion_s=round(self.intrusion_s, 1), path_m=round(self.path_m, 2),
                    mean_speed=round(self.path_m / max(self.elapsed, 1e-9), 3),
                    smoothness=round(self.jerk_sum / n, 3), samples=self.n)


# ------------------------------------------------------------------ replay
def make_planner(name, world=None, conf=None):
    clear = world.clearance_many if world is not None else None
    if name == 'reactive':
        return s.ReactivePlanner()
    if name == 'cv-predictive':
        return s.PredictivePlanner()
    if name in ('cv-risk', 'learned-risk'):
        return s.RiskPlanner(conformal_radii=conf if name == 'learned-risk' else None, sigma_scale=0.5, clear_fn=clear)
    raise SystemExit(f'unknown planner {name}')


def _drive(robot, theta, v, world):
    """Differential drive: turn toward the plan, drive only when roughly aligned (same as the demo renderer)."""
    sp = float(np.linalg.norm(v))
    if sp > 0.02:
        err = (math.atan2(v[1], v[0]) - theta + math.pi) % (2 * math.pi) - math.pi
        ang = float(np.clip(3.0 * err, -1.8, 1.8))
        lin = sp * max(0.0, math.cos(err)) if abs(err) < 1.1 else 0.0
    else:
        lin = ang = 0.0
    theta += ang * DT
    new = robot + lin * DT * np.array([math.cos(theta), math.sin(theta)])
    if world is None or world.clearance(*new) > 0.3:
        robot = new
    return robot, theta


def run_episode(rec, planner_name, start, goals, world=None, predictor=None, conf=None, max_s=None):
    """Replay `rec` against one planner. The pedestrian stream is read-only (rec.pos is immutable)."""
    planner = make_planner(planner_name, world, conf)
    robot, theta = np.array(start, float), 0.0
    goals = [np.asarray(g, float) for g in goals]
    gi, reached = 0, 0
    route = world.route(tuple(robot), tuple(goals[0])) if world is not None else []
    m = Metrics()
    steps = rec.steps if max_s is None else min(rec.steps, int(max_s / DT))
    t_goal = None
    for k in range(steps):
        alive = np.isfinite(rec.pos[k]).all(1)
        P, V = rec.pos[k][alive], rec.vel(k)[alive]
        preds = None
        if planner_name == 'learned-risk' and predictor is not None and k >= 28 and len(P):
            import train_ped_predictor as tp
            hist = np.stack([rec.pos[max(0, k - 4 * j)][alive] for j in range(tp.OBS - 1, -1, -1)], 1)   # (n,8,2) @0.4 s
            preds = [p if p is not None else s.predict_cv(P[i], V[i], 3.0) for i, p in enumerate(predictor(hist, 3.0))]
        while len(route) > 1 and np.linalg.norm(np.array(route[0]) - robot) < 1.0:
            route.pop(0)
        tgt = np.array(route[min(2, len(route) - 1)]) if route else goals[gi]
        if np.linalg.norm(goals[gi] - robot) < 0.6:
            reached += 1
            t_goal = t_goal if t_goal is not None else k * DT
            gi = (gi + 1) % len(goals)
            route = world.route(tuple(robot), tuple(goals[gi])) if world is not None else []
        args = (robot, tgt, P, V)
        v = planner.plan(*args, preds=preds) if planner_name in ('cv-predictive', 'cv-risk', 'learned-risk') \
            else planner.plan(*args)
        v = s.safety_filter(v, robot, P)
        robot, theta = _drive(robot, theta, v, world)
        m.update(robot, rec.pos[k])
    out = m.result()
    out.update(goals_reached=reached, time_to_first_goal_s=t_goal)
    return out


def sample_starts(world, n, seed, goals):
    if world is None:
        rng = np.random.default_rng(seed)
        return [tuple(rng.uniform(-1, 1, 2)) for _ in range(n)]
    rng = np.random.default_rng(seed)
    out = []
    while len(out) < n:
        p = world.sample_free(rng, 0.8)
        if all(np.linalg.norm(np.array(p) - g) > 4.0 for g in goals) and world.route(p, tuple(goals[0])):
            out.append(p)
    return out


def summarize(rows):
    keys = ('collisions', 'min_clearance_m', 'intrusion_s', 'path_m', 'mean_speed', 'smoothness', 'goals_reached')
    agg = {}
    for name, rs in rows.items():
        agg[name] = dict(
            episodes=len(rs), collision_episodes_pct=round(100 * np.mean([r['collisions'] > 0 for r in rs]), 1),
            **{k: round(float(np.mean([r[k] for r in rs if r[k] is not None])), 3) for k in keys})
    return agg


def print_table(agg, fp):
    print(f'scene fingerprint {fp}  (identical for every row)')
    hdr = ['planner', 'eps', 'coll%', 'coll/ep', 'min_clr_m', 'intr_s', 'path_m', 'speed', 'smooth', 'goals']
    print(' | '.join(f'{h:>13}' for h in hdr))
    for n, a in agg.items():
        print(' | '.join(f'{x:>13}' for x in (n, a['episodes'], a['collision_episodes_pct'], a['collisions'],
                                              a['min_clearance_m'], a['intrusion_s'], a['path_m'], a['mean_speed'],
                                              a['smoothness'], a['goals_reached'])))


def cmd_replay(a):
    rec = Recording.load(a.recording)
    world = None
    mp = a.map or rec.meta.get('map')
    if mp and os.path.exists(mp):
        import social_force_core as sf
        world = sf.MapWorld(mp)
        world.remove_small_obstacles(0.9)
    goals = [np.array(a.goals[i:i + 2]) for i in range(0, len(a.goals), 2)]
    starts = sample_starts(world, a.starts, a.seed, goals)
    predictor = conf = None
    if 'learned-risk' in a.planners:
        import eval_learned_planner as ev
        predictor = ev.LearnedPredictor(a.weights)
        cj = a.weights + '.conformal.json'
        conf = json.load(open(cj))['radii'] if os.path.exists(cj) else None
    rows = {}
    for name in a.planners:
        t0 = time.time()
        rows[name] = [run_episode(rec, name, st, goals, world, predictor, conf, a.max_seconds) for st in starts]
        print(f'  {name}: {len(starts)} episodes in {time.time() - t0:.1f}s', flush=True)
    agg = summarize(rows)
    print_table(agg, rec.fingerprint())
    if a.out:
        json.dump(dict(fingerprint=rec.fingerprint(), starts=[list(map(float, p)) for p in starts], summary=agg,
                       episodes=rows, note='open-loop replay: recorded pedestrians do not react to the robot'),
                  open(a.out, 'w'), indent=1)
        print('wrote', a.out)


def cmd_score(a):
    rec = Recording.load(a.recording)
    if rec.robot is None or not np.isfinite(rec.robot).any():
        raise SystemExit('recording has no robot path (extract with --odom-topic /odom --odom-offset X Y)')
    m = Metrics()
    for k in range(rec.steps):
        if np.isfinite(rec.robot[k]).all():
            m.update(rec.robot[k][:2], rec.pos[k])
    r = m.result()
    print(json.dumps(r, indent=1))
    return r


# ------------------------------------------------------------------ record / extract / synthetic
def cmd_record(a):
    out = a.out or time.strftime('/tmp/social_bag_%Y%m%d_%H%M%S')
    cmd = ['ros2', 'bag', 'record', '-o', out] + a.topics
    print('recording:', ' '.join(cmd), f'(for {a.seconds}s)' if a.seconds else '(Ctrl-C to stop)')
    p = subprocess.Popen(cmd, start_new_session=True)            # own process group -> signal exactly this PID tree
    print('ros2 bag PID', p.pid)
    try:
        p.wait(timeout=a.seconds or None)
    except (subprocess.TimeoutExpired, KeyboardInterrupt):
        os.killpg(p.pid, signal.SIGINT)                          # graceful: lets rosbag2 finalise metadata
        p.wait(timeout=30)
    print('bag written to', out, '-> next: replay_ab.py extract', out)


def cmd_extract(a):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=a.bag, storage_id=a.storage), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    if a.gt_topic not in types:
        raise SystemExit(f'{a.gt_topic} not in bag (has: {sorted(types)})')
    msgs = {n: get_message(t) for n, t in types.items() if n in (a.gt_topic, a.odom_topic)}
    t_gt, fr_gt, t_od, od = [], [], [], []
    while r.has_next():
        topic, data, tns = r.read_next()
        if topic == a.gt_topic:
            d = json.loads(deserialize_message(data, msgs[topic]).data)
            t_gt.append(tns / 1e9)
            fr_gt.append([dict(id=p['id'], x=p['x'], y=p['y']) for p in d['peds']])
        elif topic == a.odom_topic and topic in msgs:
            m = deserialize_message(data, msgs[topic])
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            t_od.append(tns / 1e9)
            od.append([m.pose.pose.position.x + a.odom_offset[0], m.pose.pose.position.y + a.odom_offset[1], yaw])
    robot = None
    if od:                                                      # align odom samples to the gt stamps (nearest)
        t_od, od = np.asarray(t_od), np.asarray(od)
        robot = [list(od[int(np.argmin(np.abs(t_od - t)))]) if np.abs(t_od - t).min() < 0.5 else None for t in t_gt]
    rec = Recording.from_frames(t_gt, fr_gt, robot, dict(source=a.bag, gt_topic=a.gt_topic, map=a.map or ''))
    rec.save(a.out)
    print(f'wrote {a.out}: {rec.steps} steps ({rec.steps * DT:.0f}s), {len(rec.ids)} pedestrians, '
          f'robot path={"yes" if robot else "no"}, fingerprint {rec.fingerprint()}')


def cmd_synthetic(a):
    import social_force_core as sf
    world = sf.MapWorld(a.map)
    world.remove_small_obstacles(0.9)
    crowd = sf.Crowd(world, a.peds, seed=a.seed)
    t, frames = [], []
    for k in range(int(a.seconds / DT)):
        crowd.step(DT)                                         # no robot: people just live their life
        t.append(k * DT)
        frames.append([dict(id=i + 1, x=float(p[0]), y=float(p[1])) for i, p in enumerate(crowd.pos)])
    rec = Recording.from_frames(t, frames, None, dict(source='synthetic social-force crowd', map=a.map, seed=a.seed))
    rec.save(a.out)
    print(f'wrote {a.out}: {rec.steps} steps, {len(rec.ids)} pedestrians, '
          f'fingerprint {Recording.load(a.out).fingerprint()} (as replay will see it)')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('record'); p.add_argument('--out'); p.add_argument('--seconds', type=float, default=0)
    p.add_argument('--topics', nargs='+', default=DEFAULT_TOPICS)
    p = sub.add_parser('extract'); p.add_argument('bag'); p.add_argument('--out', default='recording.json')
    p.add_argument('--gt-topic', default='/social_peds/gt'); p.add_argument('--odom-topic', default='/odom')
    p.add_argument('--odom-offset', type=float, nargs=2, default=[0.0, 0.0],
                   help='map = odom + offset (the robot spawn pose when slam:=false)')
    p.add_argument('--map', default=''); p.add_argument('--storage', default='sqlite3')
    p = sub.add_parser('synthetic'); p.add_argument('--map', default=os.path.join(HERE, '..', 'maps', 'map_hospital.yaml'))
    p.add_argument('--peds', type=int, default=10); p.add_argument('--seconds', type=float, default=60)
    p.add_argument('--seed', type=int, default=0); p.add_argument('--out', default='recording.json')
    p = sub.add_parser('replay'); p.add_argument('recording'); p.add_argument('--map', default='')
    p.add_argument('--planners', nargs='+', default=['reactive', 'cv-predictive', 'cv-risk'])
    p.add_argument('--starts', type=int, default=5); p.add_argument('--seed', type=int, default=0)
    p.add_argument('--goals', type=float, nargs='+', default=[8.0, 0.4, -8.0, -0.4])
    p.add_argument('--weights', default=os.path.join(HERE, '..', '..', '..', 'outputs', 'ped_predictor_n24.pt'))
    p.add_argument('--max-seconds', type=float, default=None); p.add_argument('--out', default='')
    p = sub.add_parser('score'); p.add_argument('recording')
    a = ap.parse_args()
    dict(record=cmd_record, extract=cmd_extract, synthetic=cmd_synthetic, replay=cmd_replay, score=cmd_score)[a.cmd](a)


if __name__ == '__main__':
    main()
