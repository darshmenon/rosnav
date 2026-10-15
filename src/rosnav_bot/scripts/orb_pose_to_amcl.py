#!/usr/bin/env python3
"""
orb_pose_to_amcl.py — expose the ORB-SLAM3 sidecar's pose as a map-frame
PoseWithCovarianceStamped that visual_amcl_fusion.py can consume
(visual_pose_topic:=/orb_localization_pose).

ORB-SLAM3 builds its own map frame, anchored wherever it initialised, so its
`robot_pose_slam` is NOT in the Nav2/AMCL map frame. This node estimates the
rigid 2D transform T (amcl_map <- orb_map) from AMCL while AMCL is confident
(covariance trace below `calib_cov_max`), keeps a slow-moving average of it,
and publishes T * orb_pose. If AMCL is lost, T is frozen, so the ORB pose is
the independent reference that can re-seed AMCL.

Subscribes: robot_pose_slam (PoseStamped, ORB frame), /amcl_pose
Publishes:  /orb_localization_pose (PoseWithCovarianceStamped, map frame),
            orb_loc/calibrated (std_msgs/Bool), orb_loc/lost_count (Int32)
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, Int32


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class OrbPoseToAmcl(Node):
    def __init__(self):
        super().__init__('orb_pose_to_amcl')
        self.declare_parameter('orb_pose_topic', 'robot_pose_slam')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('out_topic', '/orb_localization_pose')
        self.declare_parameter('calib_cov_max', 0.15)     # AMCL xy-cov trace to trust for calibration
        self.declare_parameter('calib_alpha', 0.02)       # EMA weight for new calibration samples
        self.declare_parameter('lost_gap_sec', 1.5)       # ORB pose gap counted as a tracking loss
        self.declare_parameter('out_cov', 0.02)           # reported xy variance while tracking

        self.amcl = None
        self.T = None            # (dx, dy, dyaw): amcl = R(dyaw) * orb + (dx, dy)
        self.last_orb_t = None
        self.lost = 0
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(PoseWithCovarianceStamped,
                                 self.get_parameter('amcl_pose_topic').value, self._on_amcl, qos)
        self.create_subscription(PoseStamped,
                                 self.get_parameter('orb_pose_topic').value, self._on_orb, 10)
        self.pub = self.create_publisher(PoseWithCovarianceStamped,
                                         self.get_parameter('out_topic').value, 1)
        self.cal_pub = self.create_publisher(Bool, 'orb_loc/calibrated', 1)
        self.lost_pub = self.create_publisher(Int32, 'orb_loc/lost_count', 1)
        self.get_logger().info('orb_pose_to_amcl up — waiting for ORB pose + confident AMCL to calibrate')

    def _on_amcl(self, msg):
        self.amcl = msg

    def _on_orb(self, msg):
        now = self.get_clock().now()
        if self.last_orb_t is not None and \
                (now - self.last_orb_t).nanoseconds * 1e-9 > self.get_parameter('lost_gap_sec').value:
            self.lost += 1
            self.get_logger().warn(f'ORB pose gap — tracking loss #{self.lost}')
        self.last_orb_t = now
        self.lost_pub.publish(Int32(data=self.lost))

        ox, oy = msg.pose.position.x, msg.pose.position.y
        oyaw = yaw_of(msg.pose.orientation)

        a = self.amcl
        if a is not None:
            c = a.pose.covariance
            if c[0] + c[7] < self.get_parameter('calib_cov_max').value:
                ayaw = yaw_of(a.pose.pose.orientation)
                dyaw = wrap(ayaw - oyaw)
                dx = a.pose.pose.position.x - (math.cos(dyaw) * ox - math.sin(dyaw) * oy)
                dy = a.pose.pose.position.y - (math.sin(dyaw) * ox + math.cos(dyaw) * oy)
                if self.T is None:
                    self.T = (dx, dy, dyaw)
                    self.get_logger().info(f'calibrated ORB->map T=({dx:.2f},{dy:.2f},{math.degrees(dyaw):.1f}deg)')
                else:
                    k = self.get_parameter('calib_alpha').value
                    px, py, pyaw = self.T
                    self.T = (px + k * (dx - px), py + k * (dy - py), wrap(pyaw + k * wrap(dyaw - pyaw)))
        self.cal_pub.publish(Bool(data=self.T is not None))
        if self.T is None:
            return

        tx, ty, tyaw = self.T
        out = PoseWithCovarianceStamped()
        out.header.frame_id = 'map'
        out.header.stamp = msg.header.stamp
        out.pose.pose.position.x = math.cos(tyaw) * ox - math.sin(tyaw) * oy + tx
        out.pose.pose.position.y = math.sin(tyaw) * ox + math.cos(tyaw) * oy + ty
        yaw = wrap(oyaw + tyaw)
        out.pose.pose.orientation.z = math.sin(yaw / 2)
        out.pose.pose.orientation.w = math.cos(yaw / 2)
        v = self.get_parameter('out_cov').value
        out.pose.covariance[0] = out.pose.covariance[7] = v
        out.pose.covariance[35] = 0.05
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(OrbPoseToAmcl())


if __name__ == '__main__':
    main()
