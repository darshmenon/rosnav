#!/usr/bin/env python3
"""nav_explore_eval - score any exploration run on coverage, map accuracy and time-to-percent."""
import argparse
import json
import os
import subprocess
import sys
import time

from . import __version__
from .mapio import load_yaml_map
from .metrics import TIME_TO, path_length
from .runreport import Scorer, build_report, validate


def _scorer(a, truth):
    return Scorer(truth, do_align=a.align or a.rotate, rotate=a.rotate, max_shift=a.max_shift,
                  max_yaw=a.max_yaw, tol=a.tol)


def cmd_score(a):
    from .metrics import score
    live, truth = load_yaml_map(a.map), load_yaml_map(a.truth)
    if a.offset:
        res = score(live, truth, a.offset[0], a.offset[1], a.tol)
    else:
        res = _scorer(a, truth)(live)
    print(json.dumps(res, indent=2) if a.json else '\n'.join(f'{k:20s} {v:8.2f}' for k, v in res.items()))
    return 0


def _row(path):
    r = json.load(open(path))
    f, tt = r['final'], r['time_to_pct_s']
    return [r['label'], f"{f['coverage_pct']:.1f}", f"{f['wall_f1_pct']:.1f}", f"{f['false_free_pct']:.1f}",
            f"{r['path_length_m']:.1f}"] + [('-' if tt[str(p)] is None else f'{tt[str(p)]:.0f}') for p in TIME_TO]


def cmd_truth(a):
    from .worldmap import rasterize, shapes, write_map
    g = rasterize(shapes(open(a.world).read(), a.z, a.model_path, os.path.dirname(os.path.abspath(a.world))),
                  a.res, a.margin, tuple(a.seed))
    write_map(g, a.out)
    free = (g.data == 0).sum() * g.resolution ** 2
    print(f'wrote {a.out}.yaml / .pgm  ({g.shape[1]}x{g.shape[0]} @ {g.resolution} m, explorable area {free:.1f} m2)')
    return 0


def cmd_compare(a):
    head = ['label', 'cov%', 'f1%', 'leak%', 'path_m'] + [f't{p}%(s)' for p in TIME_TO]
    rows = [head] + [_row(p) for p in a.reports]
    w = [max(len(r[i]) for r in rows) for i in range(len(head))]
    for r in rows:
        print('  '.join(c.rjust(w[i]) for i, c in enumerate(r)))
    if a.png:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 4))
        for p in a.reports:
            r = json.load(open(p))
            ax.plot([s['t'] for s in r['series']], [s['coverage_pct'] for s in r['series']], label=r['label'])
        ax.set_xlabel('time (s)'); ax.set_ylabel('coverage (%)'); ax.legend(); ax.grid(alpha=.3)
        fig.tight_layout(); fig.savefig(a.png, dpi=140)
        print(f'wrote {a.png}')
    return 0


def _record_once(a, ros_args):
    import rclpy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from .mapio import from_msg

    rclpy.init(args=ros_args)
    node = Node('nav_explore_eval')
    truth = load_yaml_map(a.truth)
    scorer = _scorer(a, truth)
    robots = [r.strip() for r in a.robots.split(',')] if a.robots else [a.ns]
    maps, pts = [None] * len(robots), [[] for _ in robots]
    est_t, gt_t = [], []                                    # (t, x, y) for localisation error
    series, per_robot, t0 = [], [], time.monotonic()
    for k, r in enumerate(robots):
        ns = ('/' + r.strip('/')) if r else ''

        def on_map(m, k=k):
            maps[k] = from_msg(m)

        def on_odom(m, k=k):
            p = m.pose.pose.position
            q = pts[k]
            if not q or abs(p.x - q[-1][0]) + abs(p.y - q[-1][1]) > 0.02:
                q.append((p.x, p.y))
        node.create_subscription(OccupancyGrid, f'{ns}/map', on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(Odometry, f'{ns}/odom', on_odom, qos_profile_sensor_data)
        if k == 0:
            def on_est(m):
                p = m.pose.pose.position
                est_t.append((time.monotonic() - t0, p.x, p.y))
            node.create_subscription(Odometry, f'{ns}/odom', on_est, qos_profile_sensor_data)
    if a.gt_odom:
        def on_gt(m):
            p = m.pose.pose.position
            gt_t.append((time.monotonic() - t0, p.x, p.y))
        node.create_subscription(Odometry, a.gt_odom, on_gt, qos_profile_sensor_data)

    def tick():
        t = time.monotonic() - t0
        have = [g for g in maps if g is not None]
        if not have:
            if t >= a.duration:
                raise SystemExit
            return
        if len(robots) > 1:
            m, per = scorer.multi(have)
            per_robot[:] = per
        else:
            m = scorer(have[0])
        series.append((t, m))
        node.get_logger().info(f't={t:5.0f}s coverage={m["coverage_pct"]:.1f}% f1={m["wall_f1_pct"]:.1f}%')
        if t >= a.duration or m['coverage_pct'] >= a.stop_at:
            raise SystemExit
    node.create_timer(a.interval, tick)
    from rclpy.executors import ExternalShutdownException
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    node.destroy_node(); rclpy.try_shutdown()
    if not series:
        print('no /map received - nothing to score', file=sys.stderr)
        return 1
    extra = {}
    if len(robots) > 1:
        extra['robots'] = [dict(ns=robots[p['robot']], path_length_m=path_length(pts[p['robot']]), **p)
                           for p in per_robot]
    if a.gt_odom:
        from .metrics import localisation_error
        loc = localisation_error(est_t, gt_t)
        if loc:
            extra['localisation'] = loc
        else:
            print(f'warning: no overlapping poses on {a.gt_odom} - localisation error skipped', file=sys.stderr)
    rep = build_report(a.label, a.truth, series, [x for q in pts for x in q], extra)
    rep['path_length_m'] = sum(path_length(q) for q in pts)
    from .metrics import efficiency
    rep['efficiency'] = efficiency(rep['final'], rep['path_length_m'])
    json.dump(rep, open(a.out, 'w'), indent=2)
    print(f'wrote {a.out}')
    return 0


def cmd_record(a, ros_args):
    if a.repeat <= 1:
        return _record_once(a, ros_args)
    base, ext = os.path.splitext(a.out)
    outs, argv = [], [x for x in sys.argv[1:]]
    for n in range(1, a.repeat + 1):
        if a.between:
            print(f'[trial {n}/{a.repeat}] running: {a.between}')
            if subprocess.run(a.between, shell=True).returncode != 0:
                print('--between command failed; stopping', file=sys.stderr)
                break
        out = f'{base}_{n}{ext}'
        trial = [x for x in argv]
        for flag in ('--repeat', '--between', '--out'):
            while flag in trial:
                k = trial.index(flag)
                del trial[k:k + 2]
        cmd = [sys.executable, '-m', 'nav_explore_eval.cli'] + trial + ['--out', out]
        if subprocess.run(cmd).returncode == 0:
            outs.append(out)
    if not outs:
        return 1
    from .stats import aggregate, load_reports, stats_rows, text_table
    print(text_table(stats_rows(aggregate([r for _, r in load_reports(outs)]))))
    return 0


def cmd_bag(a, ros_args):
    from .bag import score_bag
    truth = load_yaml_map(a.truth)
    try:
        rep = score_bag(a.bag, truth, a.label, a.interval, dict(
            do_align=a.align or a.rotate, rotate=a.rotate, max_shift=a.max_shift, max_yaw=a.max_yaw, tol=a.tol),
            a.ns, a.stop_at, a.truth, a.gt_odom)
    except (ValueError, OSError, RuntimeError) as e:
        print(f'error: {e}', file=sys.stderr)
        return 1
    json.dump(rep, open(a.out, 'w'), indent=2)
    f = rep['final']
    if 'localisation' in rep:
        print(f"localisation: ATE rmse {rep['localisation']['ate_rmse_m']:.3f} m, drift {rep['localisation']['drift_pct'] or 0:.1f}%")
    print(f"scored {len(rep['series'])} samples over {rep['duration_s']:.0f}s: coverage {f['coverage_pct']:.1f}% "
          f"f1 {f['wall_f1_pct']:.1f}%  -> {a.out}")
    return 0


def cmd_multi(a):
    truth = load_yaml_map(a.truth)
    m, per = _scorer(a, truth).multi([load_yaml_map(p) for p in a.maps])
    if a.json:
        print(json.dumps({'merged': m, 'robots': per}, indent=2))
        return 0
    print('merged:'); print('\n'.join(f'  {k:20s} {v:8.2f}' for k, v in m.items()))
    for p, f in zip(a.maps, per):
        print(f"{p}: coverage {f['coverage_pct']:.1f}%  unique {f['unique_pct']:.1f}%  overlap {f['overlap_pct']:.1f}%")
    return 0


def cmd_stats(a):
    from .stats import aggregate, load_reports, markdown_table, stats_rows, text_table
    try:
        reps = [r for _, r in load_reports(a.reports)]
    except ValueError as e:
        print(f'error: {e}', file=sys.stderr)
        return 2
    rows = stats_rows(aggregate(reps))
    print(markdown_table(rows) if a.md else text_table(rows))
    return 0


def cmd_report(a):
    from .stats import html_report, load_reports, overlay_png
    try:
        reps = [r for _, r in load_reports(a.reports)]
    except ValueError as e:
        print(f'error: {e}', file=sys.stderr)
        return 2
    ov = None
    if a.map and a.truth:
        truth = load_yaml_map(a.truth)
        ov = overlay_png(load_yaml_map(a.map), truth, _scorer(a, truth))
    open(a.out, 'w').write(html_report(reps, ov, a.title))
    print(f'wrote {a.out}')
    return 0


def cmd_validate(a):
    bad = 0
    for p in a.files:
        try:
            errs = validate(json.load(open(p)))
        except (OSError, json.JSONDecodeError) as e:
            errs = [str(e)]
        print(f'{p}: ' + ('ok' if not errs else '; '.join(errs)))
        bad += bool(errs)
    return 1 if bad else 0


def cmd_leaderboard(a):
    from .stats import leaderboard
    try:
        md, problems = leaderboard(a.dir)
    except ValueError as e:
        print(f'error: {e}', file=sys.stderr)
        return 2
    for p in problems:
        print(f'invalid: {p}', file=sys.stderr)
    if not md:
        return 1
    if a.check:
        cur = open(a.out).read() if os.path.exists(a.out) else ''
        if cur != md:
            print(f'{a.out} is out of date: run `nav_explore_eval leaderboard {a.dir} --out {a.out}`', file=sys.stderr)
            return 1
    else:
        open(a.out, 'w').write(md)
        print(f'wrote {a.out}')
    return 1 if problems else 0


def _align_args(p):
    p.add_argument('--align', action='store_true', help='auto-find the translation between map frames')
    p.add_argument('--rotate', action='store_true', help='also search for a yaw offset (implies --align)')
    p.add_argument('--max-shift', type=float, default=3.0)
    p.add_argument('--max-yaw', type=float, default=15.0, help='yaw search range in degrees (+/-)')
    p.add_argument('--tol', type=float, default=0.10, help='wall match tolerance in metres')


def main(argv=None):
    ap = argparse.ArgumentParser(prog='nav_explore_eval', description=__doc__)
    ap.add_argument('--version', action='version', version=f'nav_explore_eval {__version__}')
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('score', help='score a saved map against a ground-truth map (offline)')
    s.add_argument('map'); s.add_argument('truth')
    s.add_argument('--offset', type=float, nargs=2, metavar=('DX', 'DY'))
    s.add_argument('--json', action='store_true')
    _align_args(s)
    t = sub.add_parser('truth-from-world', help='exact ground-truth map from a Gazebo/SDF world file')
    t.add_argument('world'); t.add_argument('--out', required=True, help='output prefix (writes .pgm + .yaml)')
    t.add_argument('--res', type=float, default=0.05); t.add_argument('--margin', type=float, default=1.0)
    t.add_argument('--z', type=float, default=0.2, help='lidar height: only collisions crossing it are walls')
    t.add_argument('--model-path', action='append', default=[], metavar='DIR',
                   help='extra directory to resolve model:// includes in (repeatable; also $GAZEBO_MODEL_PATH)')
    t.add_argument('--seed', type=float, nargs=2, default=(0.0, 0.0), metavar=('X', 'Y'),
                   help='a free point (robot spawn); free space = what is reachable from it')
    c = sub.add_parser('compare', help='table (and chart) comparing recorded runs')
    c.add_argument('reports', nargs='+'); c.add_argument('--png')
    r = sub.add_parser('record', help='watch a live /map and score it over time')
    r.add_argument('--truth', required=True, help='ground-truth map yaml')
    r.add_argument('--label', default='run'); r.add_argument('--out', default='run.json')
    r.add_argument('--ns', default=''); r.add_argument('--duration', type=float, default=600)
    r.add_argument('--robots', default='', help='comma-separated namespaces: score their merged maps (multi-robot)')
    r.add_argument('--repeat', type=int, default=1, help='N trials -> out_1.json .. out_N.json + mean/std table')
    r.add_argument('--between', default='', help='shell command run before each trial (restart your explorer)')
    r.add_argument('--stop-at', type=float, default=100.0, help='stop when coverage %% reached')
    r.add_argument('--interval', type=float, default=2.0)
    r.add_argument('--gt-odom', default='', metavar='TOPIC',
                   help='ground-truth nav_msgs/Odometry topic: adds localisation error (ATE) of /odom to the report')
    _align_args(r)
    b = sub.add_parser('bag', help='score a recorded rosbag2 offline (needs /map, optional /odom)')
    b.add_argument('bag'); b.add_argument('--truth', required=True)
    b.add_argument('--label', default='bag'); b.add_argument('--out', default='run.json')
    b.add_argument('--ns', default=''); b.add_argument('--interval', type=float, default=2.0)
    b.add_argument('--stop-at', type=float, default=100.0)
    b.add_argument('--gt-odom', default='', metavar='TOPIC', help='ground-truth Odometry topic in the bag (adds ATE)')
    _align_args(b)
    m = sub.add_parser('score-multi', help='merge several robots\' saved maps and score them (offline)')
    m.add_argument('maps', nargs='+'); m.add_argument('--truth', required=True); m.add_argument('--json', action='store_true')
    _align_args(m)
    st = sub.add_parser('stats', help='mean +/- std table, grouped by label (repeat trials)')
    st.add_argument('reports', nargs='+', help='run JSON files, globs or directories'); st.add_argument('--md', action='store_true')
    rp = sub.add_parser('report', help='one-file HTML report (+ map-vs-truth overlay)')
    rp.add_argument('reports', nargs='+'); rp.add_argument('--out', default='report.html')
    rp.add_argument('--title', default='Exploration report')
    rp.add_argument('--map', help='saved map yaml to overlay on the truth'); rp.add_argument('--truth')
    _align_args(rp)
    v = sub.add_parser('validate', help='check run JSON files against the report schema')
    v.add_argument('files', nargs='+')
    lb = sub.add_parser('leaderboard', help='build LEADERBOARD.md from a directory of run JSONs')
    lb.add_argument('dir', nargs='?', default='results'); lb.add_argument('--out', default='LEADERBOARD.md')
    lb.add_argument('--check', action='store_true', help='fail if --out is stale (for CI)')
    a, ros_args = ap.parse_known_args(argv)
    if a.cmd == 'score':
        return cmd_score(a)
    if a.cmd == 'compare':
        return cmd_compare(a)
    for name, fn in (('score-multi', cmd_multi), ('stats', cmd_stats), ('report', cmd_report),
                     ('validate', cmd_validate), ('leaderboard', cmd_leaderboard)):
        if a.cmd == name:
            return fn(a)
    if a.cmd == 'bag':
        return cmd_bag(a, ros_args)
    if a.cmd == 'truth-from-world':
        try:
            return cmd_truth(a)
        except (ValueError, OSError) as e:
            print(f'error: {e}', file=sys.stderr)
            return 2
    return cmd_record(a, ros_args)


if __name__ == '__main__':
    raise SystemExit(main())
