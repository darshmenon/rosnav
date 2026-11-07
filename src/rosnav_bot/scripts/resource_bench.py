#!/usr/bin/env python3
"""resource_bench.py — CPU / memory cost of a running ROS 2 stack, per process group.

Measures only processes whose environment has the given ROS_DOMAIN_ID (never name
patterns, so a second ROS instance on the machine is left alone). Use it to compare
`profile:=full` vs `profile:=lite`, or any two launch configs:

    scripts/resource_bench.py --domain 42 --duration 30 --csv /tmp/full.csv
    scripts/resource_bench.py --domain 42 --compare /tmp/full.csv   # delta vs a saved run
"""
import argparse
import csv
import os
import sys
import time

import psutil

GROUPS = [  # (label, substrings of the command line), first match wins
    ('gazebo', ('gz sim', 'gzserver', 'ruby', 'gz-sim')),
    ('nav2', ('controller_server', 'planner_server', 'bt_navigator', 'behavior_server',
              'smoother_server', 'waypoint_follower', 'lifecycle_manager', 'amcl',
              'map_server', 'nav2_', 'velocity_smoother')),
    ('slam', ('slam_toolbox', 'rtabmap', 'orb_slam', 'cartographer', 'kiss_icp', 'lio')),
    ('rviz', ('rviz2',)),
    ('bridge', ('ros_gz_bridge', 'parameter_bridge')),
    ('rosnav', ('rosnav_bot',)),
]


def classify(cmdline: str) -> str:
    for label, keys in GROUPS:
        if any(k in cmdline for k in keys):
            return label
    return 'other'


def domain_of(environ: dict):
    return environ.get('ROS_DOMAIN_ID')


def summarise(samples: list) -> dict:
    """samples: list of {group: (cpu_percent, rss_mb)} dicts -> {group: (mean_cpu, peak_rss)}."""
    out = {}
    groups = {g for s in samples for g in s}
    for g in groups:
        cpus = [s.get(g, (0.0, 0.0))[0] for s in samples]
        rss = [s.get(g, (0.0, 0.0))[1] for s in samples]
        out[g] = (sum(cpus) / len(cpus), max(rss))
    return out


def sample_once(domain: str, procs: dict) -> dict:
    cur = {}
    for p in psutil.process_iter(['pid', 'cmdline']):
        try:
            if domain_of(p.environ()) != domain:
                continue
            cmd = ' '.join(p.info['cmdline'] or [])
            if not cmd:
                continue
            if p.pid not in procs:
                procs[p.pid] = p
                p.cpu_percent(None)  # prime
                continue
            g = classify(cmd)
            cpu, rss = cur.get(g, (0.0, 0.0))
            cur[g] = (cpu + procs[p.pid].cpu_percent(None), rss + p.memory_info().rss / 1e6)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return cur


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--domain', required=True, help='ROS_DOMAIN_ID of the stack to measure')
    ap.add_argument('--duration', type=float, default=30.0)
    ap.add_argument('--period', type=float, default=1.0)
    ap.add_argument('--csv', help='save the summary here')
    ap.add_argument('--compare', help='CSV from an earlier run; print the delta')
    a = ap.parse_args()

    procs, samples = {}, []
    end = time.time() + a.duration
    print(f'[resource_bench] domain {a.domain}, {a.duration:.0f}s @ {a.period}s', file=sys.stderr)
    while time.time() < end:
        s = sample_once(a.domain, procs)
        if s:
            samples.append(s)
        time.sleep(a.period)
    if not samples:
        sys.exit(f'[resource_bench] no processes found with ROS_DOMAIN_ID={a.domain}')
    summ = summarise(samples)

    base = {}
    if a.compare:
        with open(a.compare) as f:
            base = {r['group']: (float(r['cpu_pct']), float(r['rss_mb'])) for r in csv.DictReader(f)}
    print(f'{"group":<10}{"cpu% (mean)":>13}{"rss MB (peak)":>15}' + ('   delta cpu / rss' if base else ''))
    tot = [0.0, 0.0]
    for g, (c, r) in sorted(summ.items(), key=lambda kv: -kv[1][0]):
        d = f'   {c - base[g][0]:+.0f} / {r - base[g][1]:+.0f}' if g in base else ''
        print(f'{g:<10}{c:>13.1f}{r:>15.0f}{d}')
        tot[0] += c
        tot[1] += r
    print(f'{"TOTAL":<10}{tot[0]:>13.1f}{tot[1]:>15.0f}')
    if a.csv:
        with open(a.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['group', 'cpu_pct', 'rss_mb'])
            for g, (c, r) in summ.items():
                w.writerow([g, f'{c:.1f}', f'{r:.0f}'])


if __name__ == '__main__':
    main()
