#!/usr/bin/env python3
"""
rosnav_top.py - live terminal dashboard for a rosnav fleet (like `top`, for the nav stack).

    ros2 run rosnav_bot rosnav_top.py            # needs rosnav_supervisor.py running
    ros2 run rosnav_bot rosnav_top.py --once     # print one frame and exit (for logs/CI)

Shows per robot: health level, odom/scan Hz, map->odom TF age, Nav2 lifecycle, mitigations done;
the supervisor's decision timeline (what it did and why); and CPU/RAM of the nav stack processes,
so "is it lightweight?" is visible live. Rendering is a pure function (`render`) - unit-tested.
"""
from __future__ import annotations

import argparse
import json
import re
import threading
import time

from rich.console import Console, Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table

LEVEL_STYLE = {'OK': 'bold green', 'WARN': 'bold yellow', 'ERROR': 'bold red'}
# name patterns of the processes that make up the stack, grouped for the resource table
STACK = [('Gazebo', r'gz|ign'), ('Nav2', r'controller_server|planner_server|bt_navigator|behavior_server|'
                                          r'smoother_server|waypoint_follower|lifecycle_manager|amcl|map_server|'
                                          r'velocity_smoother|collision_monitor'),
         ('SLAM', r'slam_toolbox|rtabmap|cartographer|kiss_icp'),
         ('rosnav scripts', r'frontier|fleet|supervisor|coordinator|mission|watchdog|\.py'),
         ('RViz', r'rviz')]


def group_procs(procs: list[dict]) -> dict[str, dict]:
    """procs: [{'name', 'cpu', 'rss_mb'}] -> {group: {'cpu', 'rss_mb', 'n'}}; first matching group wins."""
    out = {g: {'cpu': 0.0, 'rss_mb': 0.0, 'n': 0} for g, _ in STACK}
    for p in procs:
        for g, pat in STACK:
            if re.search(pat, p['name']):
                out[g]['cpu'] += p['cpu']
                out[g]['rss_mb'] += p['rss_mb']
                out[g]['n'] += 1
                break
    return {g: v for g, v in out.items() if v['n']}


def _fmt(v, unit='', nd=1):
    return '[dim]-[/]' if v is None else f'{v:.{nd}f}{unit}'


def render(snap: dict | None, procs: list[dict], age_s: float | None = None):
    if not snap:
        return Panel('[yellow]waiting for /rosnav/supervisor/status ...[/]\n'
                     'start it: ros2 run rosnav_bot rosnav_supervisor.py  (or launch with supervisor:=true)',
                     title='rosnav top')
    t = Table(title='Robots', expand=True)
    for c in ('robot', 'health', 'odom Hz', 'scan Hz', 'map->odom', 'Nav2', 'fixes', 'why'):
        t.add_column(c)
    for r, s in sorted(snap['robots'].items()):
        m = snap.get('metrics', {}).get(r, {})
        act = m.get('nav2_active')
        nav = '[green]active[/]' if act else '[red]DOWN[/]' if act is False else '[dim]?[/]'
        why = '; '.join(msg for lv, msg in s['findings'] if lv != 'OK')[:70] or '[dim]healthy[/]'
        t.add_row(r, f"[{LEVEL_STYLE[s['level']]}]{s['level']}[/]", _fmt(m.get('odom_hz'), nd=0),
                  _fmt(m.get('scan_hz'), nd=0), _fmt(m.get('tf_map_odom_age'), 's', 2), nav,
                  str(s['mitigations']), why)
    tl = Table(title='Supervisor timeline' + ('' if snap.get('mitigate', True) else ' (DRY RUN)'), expand=True)
    for c in ('t', 'robot', 'event'):
        tl.add_column(c)
    for e in list(snap.get('timeline', []))[-8:]:
        tl.add_row(f"{e['t']:.0f}", e['robot'], e['event'][:100])
    res = Table(title='Stack resources', expand=True)
    for c in ('group', 'procs', 'CPU %', 'RAM MB'):
        res.add_column(c)
    tot_c = tot_m = 0.0
    for g, v in group_procs(procs).items():
        res.add_row(g, str(v['n']), f"{v['cpu']:.0f}", f"{v['rss_mb']:.0f}")
        tot_c += v['cpu']
        tot_m += v['rss_mb']
    res.add_row('[bold]total[/]', '', f'[bold]{tot_c:.0f}[/]', f'[bold]{tot_m:.0f}[/]')
    stale = '' if age_s is None or age_s < 5 else f'  [red]supervisor silent {age_s:.0f}s[/]'
    return Group(t, tl, res, f'[dim]ctrl-c quits{stale}[/]')


def sample_procs() -> list[dict]:
    import psutil
    out = []
    for p in psutil.process_iter(['name', 'cmdline', 'cpu_percent', 'memory_info']):
        try:
            cmd = ' '.join(p.info['cmdline'] or [])
            if not re.search(r'ros|gz|ign|nav2|slam|rviz|rtabmap', cmd):
                continue
            out.append({'name': p.info['name'] + ' ' + cmd[-120:], 'cpu': p.info['cpu_percent'] or 0.0,
                        'rss_mb': p.info['memory_info'].rss / 1e6})
        except Exception:
            continue
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--once', action='store_true')
    ap.add_argument('--hz', type=float, default=2.0)
    a = ap.parse_args(argv)

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    rclpy.init()
    node = Node('rosnav_top')
    last = {'snap': None, 't': None}

    def cb(m):
        last['snap'], last['t'] = json.loads(m.data), time.monotonic()
    node.create_subscription(String, '/rosnav/supervisor/status', cb, 10)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    console = Console()
    sample_procs()                      # prime cpu_percent
    try:
        if a.once:
            time.sleep(3.0)
            console.print(render(last['snap'], sample_procs()))
            return 0 if last['snap'] else 1
        with Live(render(None, []), console=console, refresh_per_second=a.hz, screen=True) as live:
            while True:
                time.sleep(1.0 / a.hz)
                age = None if last['t'] is None else time.monotonic() - last['t']
                live.update(render(last['snap'], sample_procs(), age))
    except KeyboardInterrupt:
        return 0
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
