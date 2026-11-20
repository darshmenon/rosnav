#!/usr/bin/env python3
"""lio_doctor - is my lidar(-inertial) odometry healthy? Live checks, plus a KISS-ICP / RKO-LIO / LIO-SAM readiness preflight."""
import argparse
import json
import math
import sys
import time

from . import __version__
from .core import (ESTIMATORS, HINTS, CloudInfo, Finding, LioDoctor, check_cloud_fields, check_imu_yaw, field_range, health_overlay,
                   imu_yaw_offset, readiness, time_field)


def fmt(findings, hints=False):
    icon = {'ERROR': '\033[31mERROR\033[0m', 'WARN': '\033[33mWARN \033[0m', 'INFO': 'INFO ', 'OK': '\033[32mOK   \033[0m'}
    out = []
    for x in findings:
        out.append(f'{icon[x.level]} [{x.code}] {x.msg}')
        if hints and x.code in HINTS and x.level != 'OK':
            out.append(f'      -> {HINTS[x.code]}')
    return '\n'.join(out)


def fmt_readiness(res):
    col = {'READY': '\033[32mREADY   \033[0m', 'DEGRADED': '\033[33mDEGRADED\033[0m', 'BLOCKED': '\033[31mBLOCKED \033[0m'}
    lines = []
    for name in ESTIMATORS:
        v, reasons = res[name]
        lines.append(f'{col[v]} {name}')
        lines += [f'           - {r}' for r in reasons]
    can = [n for n in ESTIMATORS if res[n][0] != 'BLOCKED']
    lines.append('can run: ' + (', '.join(can) if can else 'none'))
    return '\n'.join(lines)


def to_json(findings, summary, now_wall):
    return json.dumps({'time': now_wall, 'ok': not any(x.level == 'ERROR' for x in findings), 'summary': summary,
                       'findings': [dict(x.__dict__, hint=HINTS.get(x.code)) for x in findings]})


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def check_cloud_main(path, as_json=False):
    """ROS-free: validate a cloud layout described in a JSON file (fields, point_step, width, height, row_step, data_len)."""
    with open(path) as fh:
        spec = json.load(fh)
    info = CloudInfo.from_dict(spec)
    findings = check_cloud_fields(info) or [Finding('OK', 'CLOUD_OK', 'cloud layout is valid')]
    yaw_off = spec.get('imu_yaw_offset_deg')
    if 'imu_yaw' in spec:    # {"imu": [[stamp, [x, y, z, w]], ...], "odom": [...]}: measure the offset from samples
        iy = spec['imu_yaw']
        imu, odom = [(t, tuple(q)) for t, q in iy.get('imu', [])], [(t, tuple(q)) for t, q in iy.get('odom', [])]
        findings = [x for x in findings if x.code != 'CLOUD_OK'] + check_imu_yaw(imu, odom, iy.get('max_gap', 0.05))
        r = imu_yaw_offset(imu, odom, iy.get('max_gap', 0.05))
        yaw_off = r[0] if r and r[2] >= 10 else yaw_off
    res = readiness(info, spec.get('stamp_position', 'unknown'), spec.get('imu_hz'), spec.get('imu_orientation'), yaw_off)
    if as_json:
        print(json.dumps({'findings': [x.__dict__ for x in findings], 'readiness': {k: {'verdict': v[0], 'reasons': v[1]} for k, v in res.items()}}))
    else:
        print(fmt(findings, hints=True))
        print(fmt_readiness(res))
    return 1 if any(x.level == 'ERROR' for x in findings) else 0


def main(argv=None):
    args_list = list(sys.argv[1:] if argv is None else argv)
    if '--check-cloud' in args_list:                      # no ROS needed
        i = args_list.index('--check-cloud')
        if i + 1 >= len(args_list):
            print('error: --check-cloud needs a JSON file')
            return 2
        return check_cloud_main(args_list[i + 1], '--json' in args_list)
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu, PointCloud2
    from tf2_msgs.msg import TFMessage

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--odom', default='/lio_sam/mapping/odometry', help='odometry topic under test')
    p.add_argument('--ref-odom', default='', help='reference odometry (e.g. /gt_odom) for windowed drift')
    p.add_argument('--cloud', default='/points', help='PointCloud2 topic fed to the estimator')
    p.add_argument('--imu', default='/imu', help='IMU topic')
    p.add_argument('--expected-hz', type=float, default=10.0, help='expected odometry rate')
    p.add_argument('--v-max', type=float, default=2.0, help='max real robot speed, m/s (pose-jump limit)')
    p.add_argument('--w-max', type=float, default=3.0, help='max real turn rate, rad/s')
    p.add_argument('--frames', default='', help='comma-separated TF frames that must exist (odom,base_link,laser_frame,imu_link)')
    p.add_argument('--tf-topics', default='/tf,/tf_static', help='comma-separated TF topics to read (a LIO stack with a private tree: /lio_tf,/lio_tf_static)')
    p.add_argument('--drift-window', type=float, default=30.0, help='seconds of history for the drift check')
    p.add_argument('--interval', type=float, default=5.0, help='report every N seconds')
    p.add_argument('--health-topic', default='', help='also publish an RViz text Marker with the score and findings (e.g. /lio_health)')
    p.add_argument('--health-frame', default='base_link', help='frame the health marker floats above (default base_link)')
    p.add_argument('--preflight', action='store_true', help='collect --duration s, print which of KISS-ICP/RKO-LIO/LIO-SAM can run, exit')
    p.add_argument('--duration', type=float, default=8.0, help='preflight collection time')
    p.add_argument('--once', action='store_true', help='report once after --duration and exit (exit 1 on ERROR)')
    p.add_argument('--json', action='store_true', help='one JSON object per report')
    p.add_argument('--hints', action='store_true', help='print a fix hint under each finding')
    p.add_argument('--version', action='version', version=__version__)
    a = p.parse_args(args_list)

    rclpy.init()
    node = Node('lio_doctor')
    doc = LioDoctor(expected_hz=a.expected_hz, v_max=a.v_max, w_max=a.w_max,
                    frames=tuple(x for x in a.frames.split(',') if x), drift_window_s=a.drift_window)
    sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    t0 = time.time()
    health_pub = None
    if a.health_topic:
        from visualization_msgs.msg import Marker
        health_pub = node.create_publisher(Marker, a.health_topic, 1)

    def stamp_s(m):
        return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def on_odom(m):
        pose = m.pose.pose
        doc.odom(stamp_s(m), pose.position.x, pose.position.y, yaw_of(pose.orientation), time.time())

    def on_ref(m):
        q = m.pose.pose.orientation
        doc.ref_odom(stamp_s(m), m.pose.pose.position.x, m.pose.pose.position.y, (q.x, q.y, q.z, q.w))

    def on_cloud(m):
        info = CloudInfo(fields=[(f.name, f.offset, f.datatype, f.count) for f in m.fields], point_step=m.point_step, width=m.width,
                         height=m.height, row_step=m.row_step, data_len=len(m.data), is_bigendian=m.is_bigendian, stamp=stamp_s(m))
        tf = time_field(info)   # per-point time is read relative to the header stamp
        rng = field_range(bytes(m.data), info, tf[0]) if tf else None
        doc.cloud(info, time.time(), rng)

    def on_imu(m):
        orient = not (m.orientation_covariance[0] == -1.0)
        q = m.orientation
        doc.imu(stamp_s(m), time.time(), m.angular_velocity.z, orient, (q.x, q.y, q.z, q.w))

    def on_tf(m):
        doc.tf([t.child_frame_id.lstrip('/') for t in m.transforms] + [t.header.frame_id.lstrip('/') for t in m.transforms])

    for topic, typ, cb in ((a.odom, Odometry, on_odom), (a.cloud, PointCloud2, on_cloud), (a.imu, Imu, on_imu)):
        node.create_subscription(typ, topic, cb, sensor_qos)
    if a.ref_odom:
        node.create_subscription(Odometry, a.ref_odom, on_ref, sensor_qos)
    for tf_topic in (x.strip() for x in a.tf_topics.split(',') if x.strip()):
        # *_static is latched (TRANSIENT_LOCAL): a VOLATILE subscriber never sees transforms published before it started
        durability = DurabilityPolicy.TRANSIENT_LOCAL if tf_topic.endswith('static') else DurabilityPolicy.VOLATILE
        node.create_subscription(TFMessage, tf_topic, on_tf, QoSProfile(depth=100, durability=durability))

    def report():
        now = time.time()
        if a.preflight:
            if doc.cloud_info is None:
                print(fmt([Finding('ERROR', 'CLOUD_NONE', f'no PointCloud2 on {a.cloud}')], a.hints))
                return 1
            print(fmt(check_cloud_fields(doc.cloud_info), a.hints))
            res = readiness(doc.cloud_info, doc.stamp_pos(), doc.imu_hz(), doc.imu_has_orientation, doc.imu_yaw_deg())
            print(fmt_readiness(res))
            return 0 if any(v[0] != 'BLOCKED' for v in res.values()) else 1
        findings = doc.diagnose(now)
        if health_pub is not None:
            text, (r, g, b) = health_overlay(findings, doc.summary(now))
            mk = Marker()
            mk.header.frame_id, mk.ns, mk.id, mk.type, mk.action = a.health_frame, 'lio_health', 0, Marker.TEXT_VIEW_FACING, Marker.ADD
            mk.pose.position.z, mk.pose.orientation.w, mk.scale.z = 1.2, 1.0, 0.25
            mk.color.r, mk.color.g, mk.color.b, mk.color.a, mk.text = r, g, b, 1.0, text
            health_pub.publish(mk)
        print(to_json(findings, doc.summary(now), now) if a.json else f'--- lio_doctor score {doc.score(now)}/100\n' + fmt(findings, a.hints), flush=True)
        return 1 if any(x.level == 'ERROR' for x in findings) else 0

    last = time.time()
    code = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.time()
            if (a.preflight or a.once) and now - t0 >= a.duration:
                code = report()
                break
            if not (a.preflight or a.once) and now - last >= a.interval:
                report()
                last = now
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
