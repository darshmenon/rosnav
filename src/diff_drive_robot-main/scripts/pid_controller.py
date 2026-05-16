#!/usr/bin/env python3
"""
PID-based goal-seeking controller for the diff-drive robot.

No external control libraries — built on stdlib math only.

Two independent PID loops:
  - heading PID  → angular.z  (minimize angle-to-goal error)
  - distance P   → linear.x   (slow down near goal, scale with heading alignment)

All gains and limits are ROS 2 parameters so you can tune without recompiling:

  ros2 run diff_drive_robot pid_controller.py --ros-args \
      -p goal_x:=3.0 -p goal_y:=2.0 \
      -p heading_kp:=2.5 -p heading_ki:=0.01 -p heading_kd:=0.4
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class PID:
    """Discrete PID with anti-windup clamping."""

    def __init__(self, kp: float, ki: float, kd: float,
                 out_min: float, out_max: float, windup: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.windup = windup      # integral clamp (symmetric)

        self._integral = 0.0
        self._prev_error = 0.0
        self._first = True

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._first = True

    def compute(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0

        if self._first:
            self._prev_error = error
            self._first = False

        self._integral += error * dt
        # anti-windup
        self._integral = max(-self.windup, min(self.windup, self._integral))

        derivative = (error - self._prev_error) / dt
        self._prev_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(self.out_min, min(self.out_max, output))


class PIDGoalController(Node):
    def __init__(self):
        super().__init__('pid_goal_controller')

        # ── params ───────────────────────────────────────────────────────────
        self.declare_parameter('goal_x',          3.0)
        self.declare_parameter('goal_y',          3.0)
        self.declare_parameter('goal_tolerance',  0.15)

        # heading PID
        self.declare_parameter('heading_kp',    2.5)
        self.declare_parameter('heading_ki',    0.01)
        self.declare_parameter('heading_kd',    0.35)
        self.declare_parameter('heading_windup', 1.0)
        self.declare_parameter('max_angular',   2.0)

        # linear speed (simple P — overshooting isn't useful for position)
        self.declare_parameter('linear_kp',     0.6)
        self.declare_parameter('max_linear',    1.0)
        # reduce speed when heading error is large (degrees)
        self.declare_parameter('align_threshold_deg', 25.0)

        self.declare_parameter('cmd_vel_topic',  '/cmd_vel')
        self.declare_parameter('odom_topic',     '/odom')
        self.declare_parameter('timer_hz',       20.0)
        self.declare_parameter('log_every_n',    40)   # print every N ticks

        g = self.get_parameter
        self._goal = [g('goal_x').value, g('goal_y').value]
        self._tol  = g('goal_tolerance').value

        self._heading_pid = PID(
            kp=g('heading_kp').value,
            ki=g('heading_ki').value,
            kd=g('heading_kd').value,
            out_min=-g('max_angular').value,
            out_max= g('max_angular').value,
            windup=g('heading_windup').value,
        )
        self._linear_kp   = g('linear_kp').value
        self._max_linear  = g('max_linear').value
        self._align_rad   = math.radians(g('align_threshold_deg').value)
        self._log_every   = g('log_every_n').value

        # ── state ─────────────────────────────────────────────────────────────
        self._pos     = [0.0, 0.0]
        self._yaw     = 0.0
        self._arrived = False
        self._tick    = 0
        self._last_t  = None

        # ── ROS ───────────────────────────────────────────────────────────────
        self._pub = self.create_publisher(Twist, g('cmd_vel_topic').value, 10)
        self.create_subscription(Odometry, g('odom_topic').value,
                                 self._odom_cb, 10)
        period = 1.0 / g('timer_hz').value
        self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f'PID controller ready. Goal: ({self._goal[0]}, {self._goal[1]}) '
            f'tol={self._tol}m')

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self._pos[0] = msg.pose.pose.position.x
        self._pos[1] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y ** 2 + q.z ** 2),
        )

    def _control_loop(self):
        now = self.get_clock().now()

        # compute dt
        if self._last_t is None:
            self._last_t = now
            return
        dt = (now - self._last_t).nanoseconds * 1e-9
        self._last_t = now
        self._tick += 1

        if self._arrived:
            return

        dx   = self._goal[0] - self._pos[0]
        dy   = self._goal[1] - self._pos[1]
        dist = math.hypot(dx, dy)

        if dist < self._tol:
            self._pub.publish(Twist())
            self._arrived = True
            self.get_logger().info(
                f'Goal reached! final pos=({self._pos[0]:.3f},{self._pos[1]:.3f})')
            return

        # heading error in [-π, π]
        desired_yaw  = math.atan2(dy, dx)
        heading_err  = math.atan2(
            math.sin(desired_yaw - self._yaw),
            math.cos(desired_yaw - self._yaw),
        )

        # heading PID → angular velocity
        angular_z = self._heading_pid.compute(heading_err, dt)

        # linear speed: scale by dist and alignment
        alignment = max(0.0, 1.0 - abs(heading_err) / self._align_rad)
        linear_x  = min(self._linear_kp * dist, self._max_linear) * alignment

        twist = Twist()
        twist.linear.x  = float(linear_x)
        twist.angular.z = float(angular_z)
        self._pub.publish(twist)

        if self._tick % self._log_every == 0:
            self.get_logger().info(
                f'dist={dist:.2f}m  head_err={math.degrees(heading_err):.1f}°'
                f'  lin={linear_x:.2f}  ang={angular_z:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = PIDGoalController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
