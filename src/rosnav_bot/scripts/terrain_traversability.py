#!/usr/bin/env python3
"""Slope-aware 3D-lidar -> costmap cloud.

Nav2's obstacle layer marks anything above min_obstacle_height as lethal, so on a slope the
ground itself becomes a wall (concentric red rings in RViz) and the robot can't make progress.
This node levels the cloud with the IMU, bins it into a height grid and publishes a cloud the
obstacle layer can use directly:

  * non-traversable cells (slope > max_slope_deg between neighbouring cells, or a
    step/vertical structure taller than step_height) -> points at z=mark_z  (marked)
  * traversable ground cells                           -> points at z=0     (clearing only)

Gentle slopes are therefore driven over, steep slopes / rocks / walls are avoided.
"""
import math

import numpy as np

try:  # ROS is optional so classify() can be unit-tested offline
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, PointCloud2
    from sensor_msgs_py import point_cloud2
    from tf2_ros import Buffer, TransformListener
except ImportError:  # pragma: no cover
    Node = object


def quat_to_mat(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def classify(p_base, R_level, cell, tan_slope, step):
    """Per-point blocked mask. p_base: Nx3 points in base frame; R_level: base -> gravity-up frame."""
    p_lvl = p_base @ R_level.T
    ij = np.floor(p_lvl[:, :2] / cell).astype(np.int64)
    keys, inv = np.unique(ij, axis=0, return_inverse=True)
    inv = inv.ravel()
    zmin = np.full(len(keys), np.inf)
    zmax = np.full(len(keys), -np.inf)
    np.minimum.at(zmin, inv, p_lvl[:, 2])
    np.maximum.at(zmax, inv, p_lvl[:, 2])
    index = {(int(a), int(b)): n for n, (a, b) in enumerate(keys)}

    bad = (zmax - zmin) > step  # vertical structure inside one cell
    for n, (a, b) in enumerate(keys):
        a, b = int(a), int(b)
        for da, db in ((1, 0), (0, 1), (1, 1), (1, -1)):
            m = index.get((a + da, b + db))
            if m is None:
                continue
            dz = zmin[m] - zmin[n]
            if abs(dz) / (cell * math.hypot(da, db)) > tan_slope:
                bad[n if dz > 0 else m] = True  # the uphill side is the "wall"
    return bad[inv], len(keys)


class TerrainTraversability(Node):
    def __init__(self):
        super().__init__('terrain_traversability')
        self.declare_parameter('in_topic', '/points')
        self.declare_parameter('out_topic', '/points_traversability')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('cell_size', 0.25)
        self.declare_parameter('max_range', 6.0)
        self.declare_parameter('max_slope_deg', 25.0)
        self.declare_parameter('step_height', 0.25)
        self.declare_parameter('mark_z', 0.3)
        self.declare_parameter('log_period', 5.0)
        g = lambda n: self.get_parameter(n).value
        self.base = g('base_frame')
        self.cell = float(g('cell_size'))
        self.max_range = float(g('max_range'))
        self.tan_slope = math.tan(math.radians(float(g('max_slope_deg'))))
        self.step = float(g('step_height'))
        self.mark_z = float(g('mark_z'))
        self.log_period = float(g('log_period'))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.R_level = np.eye(3)  # base -> heading-aligned gravity-up frame
        self.have_imu = False
        self.n_in = 0
        self.last_log = self.get_clock().now()
        self.create_subscription(Imu, g('imu_topic'), self.on_imu, qos_profile_sensor_data)
        self.pub = self.create_publisher(PointCloud2, g('out_topic'), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, g('in_topic'), self.on_cloud, qos_profile_sensor_data)
        self.get_logger().info(
            f"terrain_traversability: {g('in_topic')} -> {g('out_topic')} | cell={self.cell}m "
            f"range={self.max_range}m max_slope={g('max_slope_deg')}deg step={self.step}m "
            f"imu={g('imu_topic')} base={self.base}")

    def on_imu(self, msg):
        q = msg.orientation
        if abs(q.x) + abs(q.y) + abs(q.z) + abs(q.w) < 1e-6:
            return
        R = quat_to_mat(q.x, q.y, q.z, q.w)  # imu(base) -> world
        yaw = math.atan2(R[1, 0], R[0, 0])
        c, s = math.cos(-yaw), math.sin(-yaw)
        Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        self.R_level = Rz @ R
        if not self.have_imu:
            self.have_imu = True
            self.get_logger().info('IMU orientation received — levelling cloud with roll/pitch')

    def on_cloud(self, msg):
        try:
            tf = self.tf_buffer.lookup_transform(self.base, msg.header.frame_id, rclpy.time.Time())
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'TF {msg.header.frame_id}->{self.base} unavailable: {e}',
                                   throttle_duration_sec=5.0)
            return
        pts = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if len(pts) == 0:
            return
        p = np.column_stack([pts['x'], pts['y'], pts['z']]).astype(np.float64)
        t, q = tf.transform.translation, tf.transform.rotation
        p_base = p @ quat_to_mat(q.x, q.y, q.z, q.w).T + np.array([t.x, t.y, t.z])
        r = np.hypot(p_base[:, 0], p_base[:, 1])
        keep = r < self.max_range
        p_base = p_base[keep]
        if len(p_base) == 0:
            return

        bad_pt, ncells = classify(p_base, self.R_level, self.cell, self.tan_slope, self.step)

        z_out = np.where(bad_pt, self.mark_z, 0.0)
        out = np.column_stack([p_base[:, 0], p_base[:, 1], z_out]).astype(np.float32)
        hdr = msg.header
        hdr.frame_id = self.base
        self.pub.publish(point_cloud2.create_cloud_xyz32(hdr, out))

        self.n_in += 1
        now = self.get_clock().now()
        if (now - self.last_log).nanoseconds * 1e-9 >= self.log_period:
            self.last_log = now
            roll = math.degrees(math.atan2(self.R_level[2, 1], self.R_level[2, 2]))
            pitch = math.degrees(-math.asin(np.clip(self.R_level[2, 0], -1, 1)))
            self.get_logger().info(
                f'clouds={self.n_in} pts={len(p_base)} cells={ncells} '
                f'blocked_pts={int(bad_pt.sum())} ({100.0 * bad_pt.mean():.0f}%) '
                f'tilt roll={roll:.1f} pitch={pitch:.1f}deg imu={self.have_imu}')


def main():
    rclpy.init()
    node = TerrainTraversability()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
