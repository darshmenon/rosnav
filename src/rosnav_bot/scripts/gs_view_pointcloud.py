#!/usr/bin/env python3
"""
gs_view_pointcloud.py — publish a converted splat_points.npz (see
gs_splat_to_pointcloud.py) as a latched sensor_msgs/PointCloud2, so a trained
Gaussian Splat can be viewed in RViz — no gsplat renderer plugin needed,
just Gaussian centers colored by their learned RGB.

Usage:
  ros2 run rosnav_bot gs_view_pointcloud.py --ros-args \\
      -p npz_path:=/home/asimov/gs_data/cafe_points.npz -p frame_id:=world

Then in RViz: Fixed Frame = frame_id, add a PointCloud2 display on
/gs_capture/splat_points (Durability Policy: Transient Local).
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def make_cloud(xyz: np.ndarray, rgb: np.ndarray, frame_id: str) -> PointCloud2:
    n = xyz.shape[0]
    rgb_packed = (rgb[:, 0].astype(np.uint32) << 16 |
                  rgb[:, 1].astype(np.uint32) << 8 |
                  rgb[:, 2].astype(np.uint32)).astype(np.uint32)
    rgb_float = rgb_packed.view(np.float32)  # PCL-style packed-rgb-as-float convention

    data = np.zeros(n, dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.float32)])
    data['x'], data['y'], data['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    data['rgb'] = rgb_float

    msg = PointCloud2()
    msg.header = Header(frame_id=frame_id)
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * n
    msg.is_dense = True
    msg.data = data.tobytes()
    return msg


class SplatPointsPublisher(Node):
    def __init__(self):
        super().__init__('gs_view_pointcloud')
        self.declare_parameter('npz_path', '')
        self.declare_parameter('topic', '/gs_capture/splat_points')
        self.declare_parameter('frame_id', 'world')

        npz_path = str(self.get_parameter('npz_path').value)
        topic = str(self.get_parameter('topic').value)
        frame_id = str(self.get_parameter('frame_id').value)
        if not npz_path:
            self.get_logger().error("Required parameter 'npz_path' not set — see gs_splat_to_pointcloud.py")
            raise SystemExit(1)

        d = np.load(npz_path)
        self._msg = make_cloud(d['xyz'], d['rgb'], frame_id)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # latched — late RViz subscribers still get it
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(PointCloud2, topic, qos)
        self.create_timer(2.0, self._publish)
        self.get_logger().info(
            f"Publishing {d['xyz'].shape[0]} points on {topic} (frame_id={frame_id}), latched")

    def _publish(self):
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._msg)


def main(args=None):
    rclpy.init(args=args)
    node = SplatPointsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
