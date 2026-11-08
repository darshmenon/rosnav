#!/usr/bin/env python3
"""slam_doctor - attach to a running SLAM (any backend) and report its health: loop closures, tracking, map growth."""
import argparse
import json
import math
import time

from . import __version__
from .core import HINTS, SlamDoctor


def fmt(findings, hints=False):
    icon = {'ERROR': '\033[31mERROR\033[0m', 'WARN': '\033[33mWARN \033[0m', 'INFO': 'INFO ', 'OK': '\033[32mOK   \033[0m'}
    out = []
    for x in findings:
        out.append(f'{icon[x.level]} [{x.code}] {x.msg}')
        if hints and x.code in HINTS and x.level != 'OK':
            out.append(f'      -> {HINTS[x.code]}')
    return '\n'.join(out)


def to_json(findings, summary, ns, now_wall):
    return json.dumps({'time': now_wall, 'ns': ns, 'ok': not any(x.level == 'ERROR' for x in findings),
                       'summary': summary,
                       'findings': [dict(x.__dict__, hint=HINTS.get(x.code)) for x in findings]})


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def calibrate_main(directory):
    import glob
    import os
    from .features import KEYPOINTS_MIN, KEYPOINTS_MIN_PROXY, SHARPNESS_MIN, calibrate
    paths = sorted(p for e in ('png', 'jpg', 'jpeg') for p in glob.glob(os.path.join(directory, f'*.{e}')))
    if not paths:
        print(f'error: no .png/.jpg frames in {directory}')
        return 2
    paths = paths[::max(1, len(paths) // 60)]          # ~60 frames is plenty
    r = calibrate(paths)
    cur = KEYPOINTS_MIN if r['orb'] else KEYPOINTS_MIN_PROXY
    print(f'{r["frames"]} frames from {directory} ({"OpenCV ORB fast7" if r["orb"] else "NumPy proxy"})')
    for k in ('keypoints', 'sharpness'):
        print(f'  {k:10s} min {r[k]["min"]:7.0f}  p5 {r[k]["p5"]:7.0f}  median {r[k]["median"]:7.0f}')
    print(f'  brightness min {r["mean_brightness"]["min"]:.0f}  median {r["mean_brightness"]["median"]:.0f}  max {r["mean_brightness"]["max"]:.0f}')
    print(f'suggested KEYPOINTS_MIN={r["suggest"]["KEYPOINTS_MIN"]} (now {cur}), SHARPNESS_MIN={r["suggest"]["SHARPNESS_MIN"]} (now {SHARPNESS_MIN})')
    return 0


def main(argv=None):
    import sys
    args_list = list(sys.argv[1:] if argv is None else argv)
    if '--calibrate' in args_list:                       # no ROS needed
        i = args_list.index('--calibrate')
        if i + 1 >= len(args_list):
            print('error: --calibrate needs a directory of frames')
            return 2
        return calibrate_main(args_list[i + 1])
    import numpy as np
    import rclpy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from tf2_msgs.msg import TFMessage
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    from .features import image_stats, to_gray

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--version', action='version', version=f'slam_doctor {__version__}')
    ap.add_argument('--ns', default='', help='robot namespace, e.g. robot1')
    ap.add_argument('--map-frame', default='map')
    ap.add_argument('--odom-frame', default='odom')
    ap.add_argument('--image', default='', help='camera topic for feature-quality checks, e.g. /camera/image_raw')
    ap.add_argument('--period', type=float, default=2.0, help='report period (s)')
    ap.add_argument('--once', type=float, default=0.0, help='watch N seconds, print one report, exit 1 on ERROR')
    ap.add_argument('--json', action='store_true', help='print one JSON object per report')
    ap.add_argument('--hints', action='store_true', help='suggest a next step under each finding')
    ap.add_argument('--calibrate', metavar='DIR', help='measure your own camera frames and suggest thresholds, then exit')
    ap.add_argument('--events', help='append the correction/loop-closure timeline to this JSONL file')
    ap.add_argument('--diagnostics', action='store_true', help='also publish /diagnostics')
    args, ros_args = ap.parse_known_args(argv)

    rclpy.init(args=ros_args)
    node = Node('slam_doctor')
    d = SlamDoctor()
    ns = ('/' + args.ns.strip('/')) if args.ns else ''
    clock = time.monotonic
    rel = QoSProfile(depth=100)
    mp = ('' if not args.map_frame else args.map_frame).lstrip('/')
    od = args.odom_frame.lstrip('/')

    def on_tf(m):
        for t in m.transforms:
            if t.header.frame_id.lstrip('/') == mp and t.child_frame_id.lstrip('/') == od:
                tr, q = t.transform.translation, t.transform.rotation
                d.map_odom(tr.x, tr.y, yaw_of(q), clock())
    node.create_subscription(TFMessage, f'{ns}/tf', on_tf, rel)
    node.create_subscription(Odometry, f'{ns}/odom',
                             lambda m: d.odom(m.pose.pose.position.x, m.pose.pose.position.y, clock()), qos_profile_sensor_data)

    def on_map(m):
        a = np.asarray(m.data, dtype=np.int8)
        d.map_stats(int((a == 0).sum()), int((a > 50).sum()), int((a < 0).sum()), clock())
    node.create_subscription(OccupancyGrid, f'{ns}/map', on_map, QoSProfile(depth=1, durability=1, reliability=1))

    last_img = [0.0]
    if args.image:
        def on_img(m):
            if clock() - last_img[0] < 1.0:        # 1 Hz is plenty and keeps the doctor light
                return
            last_img[0] = clock()
            g = to_gray(bytes(m.data), m.width, m.height, m.step, m.encoding)
            if g is not None:
                d.frame(image_stats(g), clock())
        node.create_subscription(Image, args.image, on_img, qos_profile_sensor_data)

    diag_pub = node.create_publisher(DiagnosticArray, '/diagnostics', 10) if args.diagnostics else None
    lvl = {'OK': DiagnosticStatus.OK, 'INFO': DiagnosticStatus.OK, 'WARN': DiagnosticStatus.WARN,
           'ERROR': DiagnosticStatus.ERROR}
    written = [0]

    def flush_events():
        if args.events and len(d.events) > written[0]:
            with open(args.events, 'a') as fh:
                for e in d.events[written[0]:]:
                    fh.write(json.dumps(e) + '\n')
            written[0] = len(d.events)

    def report():
        now = clock()
        found = d.diagnose(now)
        if args.json:
            print(to_json(found, d.summary(now), args.ns, time.time()), flush=True)
        else:
            s = d.summary(now)
            print(f'\n--- slam_doctor {time.strftime("%H:%M:%S")} ns="{args.ns}" score={s["score"]} '
                  f'corrections={s["corrections"]} travel={s["travel_m"]}m ---')
            print(fmt(found, args.hints), flush=True)
        if diag_pub:
            msg = DiagnosticArray()
            msg.header.stamp = node.get_clock().now().to_msg()
            for x in found:
                msg.status.append(DiagnosticStatus(level=lvl[x.level], name=f'slam_doctor/{x.code}',
                                                   message=x.msg, hardware_id=args.ns or 'slam'))
            diag_pub.publish(msg)
        flush_events()
        return found

    exit_code, start = [0], clock()

    def tick():
        if args.once and clock() - start >= args.once:
            exit_code[0] = 1 if any(x.level == 'ERROR' for x in report()) else 0
            raise SystemExit
        if not args.once:
            report()
    node.create_timer(args.once or args.period, tick)
    node.get_logger().info(f'watching {ns or "/"} {mp}->{od} - Ctrl+C to stop')
    from rclpy.executors import ExternalShutdownException
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return exit_code[0]


if __name__ == '__main__':
    raise SystemExit(main())
