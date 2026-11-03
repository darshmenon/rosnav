#!/usr/bin/env python3
"""
rosnav_doctor.py - one-command health check for a running rosnav fleet.

    ros2 run rosnav_bot rosnav_doctor.py                # auto-detect robots, 5 s sample
    ros2 run rosnav_bot rosnav_doctor.py --robots robot1,robot2 --sample 8
    ros2 run rosnav_bot rosnav_doctor.py --json

Per robot it checks: map->odom and odom->base_link TF age, Nav2 lifecycle active, odom and scan
rates, and (fleet level) whether /map is growing. Then it names the likely cause, using the
failure modes met while debugging the fleet (stale map->odom, AMCL reset, aborted Nav2 bringup,
frozen SLAM). Exit code 0 = all OK, 1 = warnings, 2 = errors.

The diagnosis is a pure function (`diagnose`) so it is unit-tested without ROS.
"""
from __future__ import annotations

import argparse
import json
import sys

TF_STALE_S = 1.0       # a controller needs map->odom fresher than its transform_tolerance
MIN_ODOM_HZ = 5.0
MIN_SCAN_HZ = 3.0


def diagnose(robot: dict, map_growing: bool | None = None) -> list[tuple[str, str]]:
    """robot: {tf_map_odom_age, tf_odom_base_age (s or None = missing), nav2_active (bool|None),
    odom_hz, scan_hz, slam (bool)}. Returns [(level, message)], level in OK|WARN|ERROR."""
    out: list[tuple[str, str]] = []
    mo, ob = robot.get('tf_map_odom_age'), robot.get('tf_odom_base_age')
    odom, scan = robot.get('odom_hz', 0.0), robot.get('scan_hz', 0.0)

    if odom < 0.1:
        out.append(('ERROR', 'no odometry: bridge/EKF down or Gazebo stalled'))
    elif odom < MIN_ODOM_HZ:
        out.append(('WARN', f'odom only {odom:.1f} Hz: simulation overloaded (low real-time factor)'))
    if scan < 0.1:
        out.append(('ERROR', 'no laser scan: lidar bridge or laser filter down'))
    elif scan < MIN_SCAN_HZ:
        out.append(('WARN', f'scan only {scan:.1f} Hz: simulation overloaded'))

    if ob is None:
        out.append(('ERROR', 'odom->base_link TF missing: EKF/bridge not publishing'))
    elif ob > TF_STALE_S:
        out.append(('ERROR', f'odom->base_link TF stale ({ob:.1f}s): EKF stopped or overloaded'))

    if mo is None:
        who = 'slam_toolbox not running/localised' if robot.get('slam') else 'AMCL/static map->odom not publishing'
        out.append(('ERROR', f'map->odom TF missing: {who}'))
    elif mo > TF_STALE_S:
        if robot.get('slam'):
            out.append(('ERROR', f'map->odom stale ({mo:.1f}s): SLAM not integrating scans '
                                 '(queue full / odom TF gaps); controller will report "Transform data too old"'))
        else:
            out.append(('ERROR', f'map->odom stale ({mo:.1f}s): AMCL not updating (re-initialised by a '
                                 'republished /map while the robot stands still?) - use explore_localization:=static'))

    act = robot.get('nav2_active')
    if act is False:
        out.append(('ERROR', 'Nav2 lifecycle not active: bringup aborted (service race under load); '
                             'goals will be rejected - nav2_bringup_watchdog should reset it'))
    elif act is None:
        out.append(('WARN', 'Nav2 lifecycle manager not reachable'))

    if robot.get('slam') and map_growing is False:
        out.append(('WARN', '/map did not grow during the sample (fine if the robot is idle or exploration is done)'))
    if not out:
        out.append(('OK', 'healthy'))
    return out


def worst(findings: list[tuple[str, str]]) -> str:
    levels = [lv for lv, _ in findings]
    return 'ERROR' if 'ERROR' in levels else 'WARN' if 'WARN' in levels else 'OK'


# --------------------------------------------------------------------------- ROS part
def _collect(robots: list[str] | None, sample: float) -> tuple[dict, bool | None]:
    import rclpy
    import tf2_ros
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import LaserScan
    from std_srvs.srv import Trigger
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

    rclpy.init()
    node = Node('rosnav_doctor', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    import time

    def spin(t):
        # Wall-clock deadline, not a fixed number of spin_once calls: spin_once returns at once when a
        # callback is ready, so with /clock + TF + odom flooding it a count-based loop ends in well
        # under `t` seconds, starving slower topics (scan read as 0 Hz) and unanswered service calls
        # (Nav2 read as DOWN).
        end = time.monotonic() + t
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    spin(3.0)
    if not robots:
        robots = sorted({t.split('/')[1] for t, _ in node.get_topic_names_and_types()
                         if t.count('/') >= 2 and t.endswith('/cmd_vel')})
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    # frames published on /tf_static never refresh their stamp: they must not be reported as stale
    from tf2_msgs.msg import TFMessage
    static_children = set()
    node.create_subscription(
        TFMessage, '/tf_static',
        lambda m: static_children.update(t.child_frame_id.lstrip('/') for t in m.transforms),
        QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    stamps = {r: {'odom': [], 'scan': []} for r in robots}      # wall-clock arrivals
    for r in robots:
        node.create_subscription(Odometry, f'/{r}/odom', lambda m, r=r: stamps[r]['odom'].append(time.monotonic()),
                                 qos_profile_sensor_data)
        node.create_subscription(LaserScan, f'/{r}/scan', lambda m, r=r: stamps[r]['scan'].append(time.monotonic()),
                                 qos_profile_sensor_data)
    known = []
    node.create_subscription(
        OccupancyGrid, '/map', lambda m: known.append(sum(1 for v in m.data if v >= 0)),
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    clients = {r: node.create_client(Trigger, f'/{r}/lifecycle_manager_navigation/is_active') for r in robots}
    futs = {r: c.call_async(Trigger.Request()) for r, c in clients.items() if c.wait_for_service(timeout_sec=2.0)}
    spin(sample)

    def rate(ts):
        # Newly discovered subscriptions get a burst of queued messages, so only the second half of
        # the sample counts; first-to-last (not count/window) so discovery lag is not read as silence.
        if len(ts) < 3:
            return 0.0
        cut = ts[0] + 0.5 * (ts[-1] - ts[0])
        tail = [t for t in ts if t >= cut]
        return (len(tail) - 1) / (tail[-1] - tail[0]) if len(tail) >= 2 and tail[-1] > tail[0] else 0.0

    def age(target, source):
        if source in static_children:
            return 0.0
        try:
            tr = buf.lookup_transform(target, source, rclpy.time.Time())
        except Exception:
            return None
        # AMCL post-dates map->odom by its tolerance: a stamp in the future is fresh, not negative
        return max(0.0, (node.get_clock().now() - rclpy.time.Time.from_msg(tr.header.stamp)).nanoseconds / 1e9)

    # the SLAM robot is the one whose map->odom comes from slam_toolbox
    slam_nodes = {n for n, _ in node.get_node_names_and_namespaces() if 'slam_toolbox' in n}
    state = {}
    for i, r in enumerate(robots):
        fut = futs.get(r)
        active = (bool(fut.result().success) if fut is not None and fut.done() and fut.result() else
                  (None if fut is None else False))
        state[r] = {'tf_map_odom_age': age('map', f'{r}/odom'), 'tf_odom_base_age': age(f'{r}/odom', f'{r}/base_link'),
                    'nav2_active': active, 'odom_hz': rate(stamps[r]['odom']), 'scan_hz': rate(stamps[r]['scan']),
                    'slam': bool(slam_nodes) and i == 0}
    growing = None if len(known) < 2 else known[-1] > known[0]
    node.destroy_node()
    rclpy.try_shutdown()
    return state, growing


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--robots', default='', help='comma list; default: auto-detect from */cmd_vel')
    ap.add_argument('--sample', type=float, default=5.0, help='seconds to measure rates')
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args(argv)
    robots = [r for r in a.robots.split(',') if r] or None
    state, growing = _collect(robots, a.sample)
    report = {r: {'metrics': s, 'findings': diagnose(s, growing)} for r, s in state.items()}
    if a.json:
        print(json.dumps(report, indent=2))
    else:
        for r, d in report.items():
            m = d['metrics']
            fmt = lambda v: 'missing' if v is None else f'{v:.2f}s'
            print(f"{r}: {worst(d['findings'])}  odom {m['odom_hz']:.0f}Hz scan {m['scan_hz']:.0f}Hz  "
                  f"map->odom {fmt(m['tf_map_odom_age'])}  odom->base {fmt(m['tf_odom_base_age'])}  "
                  f"nav2 {'active' if m['nav2_active'] else 'DOWN' if m['nav2_active'] is False else '?'}")
            for lv, msg in d['findings']:
                print(f'   [{lv}] {msg}')
        if growing is not None:
            print(f"/map {'growing' if growing else 'static'} during sample")
    w = [worst(d['findings']) for d in report.values()] or ['ERROR']
    return 2 if 'ERROR' in w else 1 if 'WARN' in w else 0


if __name__ == '__main__':
    sys.exit(main())
