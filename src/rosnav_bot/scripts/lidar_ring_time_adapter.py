#!/usr/bin/env python3
"""
lidar_ring_time_adapter.py - give the sim's 3D lidar cloud the `ring` and `time` fields LIO-SAM / FAST-LIO expect.

The Gazebo gpu_lidar cloud (/points) carries only x, y, z (+ intensity). Lidar-inertial odometry deskews each
scan with a per-point time and groups points by laser channel, so this node adds, per point:
  ring  (uint16)  channel index from the elevation angle: elevation in [-vfov, +vfov] -> 0 .. n_ring-1
  time  (float32) seconds since the start of the scan, from the azimuth: the sensor sweeps -pi -> +pi
                  once per `period` s, so t = (azimuth + pi) / 2pi * period
and republishes on `out_topic` as a Velodyne-style XYZIRT cloud (same frame and stamp as the input).

Parameters: in_topic (/points), out_topic (/points_lio), n_ring (16), vfov_deg (10, half-angle - must match the
sim's lidar3d_vfov_deg), period (0.1 s = 10 Hz), stamp_shift_s (-0.1: the sim stamps a cloud at the END of the
sweep, odometry wants the START).
The geometry is a pure function (`ring_and_time`) so it is unit-tested without ROS.
"""
import array
import math
import time

import numpy as np


def ring_and_time(xyz, n_ring=16, vfov_deg=10.0, period=0.1):
    """xyz: (N,3) array. Returns (ring uint16 (N,), time float32 (N,)) - see module docstring."""
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    elev = np.degrees(np.arctan2(z, np.hypot(x, y)))
    frac = (elev + vfov_deg) / (2.0 * vfov_deg)
    ring = np.clip(np.rint(frac * (n_ring - 1)), 0, n_ring - 1).astype(np.uint16)
    az = np.arctan2(y, x)                                   # -pi .. pi
    t = ((az + math.pi) / (2.0 * math.pi) * period).astype(np.float32)
    return ring, np.clip(t, 0.0, period).astype(np.float32)


_NP = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}   # sensor_msgs/PointField datatype -> numpy


def cloud_xyzi(fields, data, point_step, n_points, stride=1):
    """(M,4) float32 [x, y, z, intensity] from raw PointCloud2 bytes, NaN/inf rows dropped, every `stride`-th point kept.
    Reads the buffer with one numpy view instead of sensor_msgs_py.read_points (which was ~120 ms for 28.8k points)."""
    by = {f.name: f for f in fields}
    names = [n for n in ('x', 'y', 'z', 'intensity') if n in by]
    dt = np.dtype({'names': names, 'formats': [_NP[by[n].datatype] for n in names], 'offsets': [by[n].offset for n in names],
                   'itemsize': point_step})
    rec = np.frombuffer(data, dtype=dt, count=n_points)[::stride]
    out = np.zeros((len(rec), 4), np.float32)
    for i, n in enumerate(names):
        out[:, i] = rec[n]
    return out[np.isfinite(out[:, :3]).all(axis=1)]


# Velodyne-style XYZIRT record, explicit aligned layout: ring (u2) at 16, time (f4) at 20, 24 bytes per point.
# (A packed layout is 22 bytes and puts `time` at an unaligned offset - a message declaring point_step 20 for it
# made LIO-SAM read garbage timestamps and wait for IMU data forever.)
OUT_DTYPE = np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
                      'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
                      'offsets': [0, 4, 8, 12, 16, 20], 'itemsize': 24})
POINT_STEP = OUT_DTYPE.itemsize
FIELD_LAYOUT = [('x', 0, 7), ('y', 4, 7), ('z', 8, 7), ('intensity', 12, 7), ('ring', 16, 4), ('time', 20, 7)]  # name, offset, PointField datatype


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2, PointField

    class Adapter(Node):
        def __init__(self):
            super().__init__('lidar_ring_time_adapter')
            self.declare_parameter('in_topic', '/points')
            self.declare_parameter('out_topic', '/points_lio')
            self.declare_parameter('n_ring', 16)
            self.declare_parameter('vfov_deg', 10.0)
            self.declare_parameter('period', 0.1)
            self.declare_parameter('stride', 1)             # keep every k-th point (1 = all) if the estimator cannot keep up
            self.declare_parameter('stamp_shift_s', -0.1)   # added to the cloud stamp (end-of-sweep -> start-of-sweep)
            g = lambda k: self.get_parameter(k).value
            self.n_ring, self.vfov, self.period = int(g('n_ring')), float(g('vfov_deg')), float(g('period'))
            self.stride = max(1, int(g('stride')))
            self.proc_s = self.t_read = self.t_pub = 0.0
            self.stamp_shift_ns = int(float(g('stamp_shift_s')) * 1e9)
            self.pub = self.create_publisher(PointCloud2, g('out_topic'), qos_profile_sensor_data)
            self.create_subscription(PointCloud2, g('in_topic'), self.cb, qos_profile_sensor_data)
            self.fields = [PointField(name=n, offset=o, datatype=d, count=1) for n, o, d in FIELD_LAYOUT]
            self.n_in = self.n_out = 0
            self.create_timer(10.0, self.report)
            self.get_logger().info(f'{g("in_topic")} -> {g("out_topic")}: n_ring={self.n_ring} '
                                   f'vfov=+/-{self.vfov} deg period={self.period} s')

        def cb(self, msg):
            t_start = time.perf_counter()
            t_a = time.perf_counter()
            a = cloud_xyzi(msg.fields, msg.data, msg.point_step, msg.width * msg.height, self.stride)
            xyz = a[:, :3]
            inten = a[:, 3]
            t_b = time.perf_counter()
            ring, t = ring_and_time(xyz, self.n_ring, self.vfov, self.period)
            out = np.zeros(len(a), dtype=OUT_DTYPE)
            out['x'], out['y'], out['z'], out['intensity'], out['ring'], out['time'] = \
                xyz[:, 0], xyz[:, 1], xyz[:, 2], inten, ring, t
            m = PointCloud2()
            m.header = msg.header
            # gpu_lidar stamps a cloud at the END of its sweep; LIO-SAM/FAST-LIO expect the START (they need IMU
            # samples up to start+max(time), which only exist if the stamp is one sweep in the past).
            ns = max(0, msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec + self.stamp_shift_ns)
            m.header.stamp.sec, m.header.stamp.nanosec = divmod(ns, 1_000_000_000)
            m.height, m.width = 1, len(out)
            m.fields, m.is_bigendian, m.point_step = self.fields, False, POINT_STEP
            m.row_step, m.is_dense = POINT_STEP * len(out), True
            buf = array.array('B')       # an array('B') skips rclpy's per-element validation of bytes (~100 ms for 700 KB)
            buf.frombytes(out.tobytes())
            m.data = buf
            t_c = time.perf_counter()
            self.pub.publish(m)
            self.t_read += t_b - t_a
            self.t_pub += time.perf_counter() - t_c
            self.n_in += 1
            self.n_out += len(out)
            self.proc_s += time.perf_counter() - t_start

        def report(self):
            ms = 1000.0 * self.proc_s / self.n_in if self.n_in else 0.0
            self.get_logger().info(f'{self.n_in} clouds adapted ({self.n_out} points, stride {self.stride}) in the last 10 s, '
                                   f'{ms:.1f} ms/cloud ({self.n_in / 10.0:.1f} Hz; of which read {1000.0 * self.t_read / max(1, self.n_in):.1f} ms, '
                                   f'publish {1000.0 * self.t_pub / max(1, self.n_in):.1f} ms)')
            self.n_in = self.n_out = 0
            self.proc_s = self.t_read = self.t_pub = 0.0

    rclpy.init()
    rclpy.spin(Adapter())


if __name__ == '__main__':
    main()
