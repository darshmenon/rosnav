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
import math

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
    from sensor_msgs_py import point_cloud2

    class Adapter(Node):
        def __init__(self):
            super().__init__('lidar_ring_time_adapter')
            self.declare_parameter('in_topic', '/points')
            self.declare_parameter('out_topic', '/points_lio')
            self.declare_parameter('n_ring', 16)
            self.declare_parameter('vfov_deg', 10.0)
            self.declare_parameter('period', 0.1)
            self.declare_parameter('stamp_shift_s', -0.1)   # added to the cloud stamp (end-of-sweep -> start-of-sweep)
            g = lambda k: self.get_parameter(k).value
            self.n_ring, self.vfov, self.period = int(g('n_ring')), float(g('vfov_deg')), float(g('period'))
            self.stamp_shift_ns = int(float(g('stamp_shift_s')) * 1e9)
            self.pub = self.create_publisher(PointCloud2, g('out_topic'), qos_profile_sensor_data)
            self.create_subscription(PointCloud2, g('in_topic'), self.cb, qos_profile_sensor_data)
            self.fields = [PointField(name=n, offset=o, datatype=d, count=1) for n, o, d in FIELD_LAYOUT]
            self.n_in = self.n_out = 0
            self.create_timer(10.0, self.report)
            self.get_logger().info(f'{g("in_topic")} -> {g("out_topic")}: n_ring={self.n_ring} '
                                   f'vfov=+/-{self.vfov} deg period={self.period} s')

        def cb(self, msg):
            names = [f.name for f in msg.fields]
            pts = point_cloud2.read_points(msg, field_names=['x', 'y', 'z'] + (['intensity'] if 'intensity' in names else []),
                                           skip_nans=True)
            a = np.stack([pts[k].astype(np.float32) for k in pts.dtype.names], axis=1)
            xyz = a[:, :3]
            inten = a[:, 3] if a.shape[1] > 3 else np.zeros(len(a), np.float32)
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
            m.data = out.tobytes()
            self.pub.publish(m)
            self.n_in += 1
            self.n_out += len(out)

        def report(self):
            self.get_logger().info(f'{self.n_in} clouds adapted ({self.n_out} points) in the last 10 s')
            self.n_in = self.n_out = 0

    rclpy.init()
    rclpy.spin(Adapter())


if __name__ == '__main__':
    main()
