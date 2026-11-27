#!/usr/bin/env python3
"""
occupancy_cloud.py -- republish an OccupancyGrid as two flat PointCloud2s (occupied / free) for RViz.

Why: RViz2's Map display draws EVERY colour scheme (map / costmap / raw) through the same GLSL program,
rviz/glsl120/indexed_8bit_image (it only swaps the palette texture), and that program fails to link on
some Mesa drivers ("active samplers with a different type refer to the same texture image unit"). The
PointCloud2 display uses different, working programs, so the map stays visible via this converter.
Unknown cells (-1) are skipped; one point per cell, sized to the map resolution on the RViz side.

Params: map_topic (/map), occupied_topic (/map_viz/occupied), free_topic (/map_viz/free),
        occupied_threshold (65), free_threshold (25), z_occupied (-0.01), z_free (-0.02)
"""
import numpy as np


def grid_to_points(data, width, height, resolution, origin_x, origin_y,
                   occupied_threshold=65, free_threshold=25, z_occupied=-0.01, z_free=-0.02):
    """Cell centres (map frame) of occupied and free cells as two (N,3) float32 arrays. Pure numpy."""
    a = np.asarray(data, dtype=np.int16).reshape(height, width)
    out = []
    for mask, z in ((a >= occupied_threshold, z_occupied), ((a >= 0) & (a <= free_threshold), z_free)):
        ys, xs = np.nonzero(mask)
        pts = np.empty((len(xs), 3), dtype=np.float32)
        pts[:, 0] = origin_x + (xs + 0.5) * resolution
        pts[:, 1] = origin_y + (ys + 0.5) * resolution
        pts[:, 2] = z
        out.append(pts)
    return out[0], out[1]


def main():
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header

    class OccupancyCloud(Node):
        def __init__(self):
            super().__init__('occupancy_cloud')
            for k, v in (('map_topic', '/map'), ('occupied_topic', '/map_viz/occupied'),
                         ('free_topic', '/map_viz/free'), ('occupied_threshold', 65),
                         ('free_threshold', 25), ('z_occupied', -0.01), ('z_free', -0.02)):
                self.declare_parameter(k, v)
            p = lambda k: self.get_parameter(k).value  # noqa: E731
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self._occ = self.create_publisher(point_cloud2.PointCloud2, p('occupied_topic'), qos)
            self._free = self.create_publisher(point_cloud2.PointCloud2, p('free_topic'), qos)
            self.create_subscription(OccupancyGrid, p('map_topic'), self._on_map, qos)
            self._n = 0
            self.get_logger().info(f'occupancy_cloud: {p("map_topic")} -> {p("occupied_topic")} + {p("free_topic")}')

        def _on_map(self, m):
            i = m.info
            occ, free = grid_to_points(
                m.data, i.width, i.height, i.resolution, i.origin.position.x, i.origin.position.y,
                self.get_parameter('occupied_threshold').value, self.get_parameter('free_threshold').value,
                self.get_parameter('z_occupied').value, self.get_parameter('z_free').value)
            hdr = Header(stamp=m.header.stamp, frame_id=m.header.frame_id)
            self._occ.publish(point_cloud2.create_cloud_xyz32(hdr, occ))
            self._free.publish(point_cloud2.create_cloud_xyz32(hdr, free))
            self._n += 1
            if self._n <= 3 or self._n % 20 == 0:
                self.get_logger().info(f'occupancy_cloud: map #{self._n} {i.width}x{i.height} '
                                       f'-> {len(occ)} occupied / {len(free)} free points')

    rclpy.init()
    node = OccupancyCloud()
    try:
        try:
            rclpy.spin(node)
        except Exception:  # noqa: BLE001 - a signal can invalidate the context mid-spin
            if rclpy.ok():
                raise
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
