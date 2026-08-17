#!/usr/bin/env python3
"""
dynamic_obstacle_driver.py — Patrols a dynamic_obstacle model back and forth.

Drives the model's VelocityControl plugin (see models/dynamic_obstacle/model.sdf)
over a /model/<name>/cmd_vel bridge, oscillating along one axis between
+-amplitude around its spawn point. Gives Nav2's dynamic-obstacle avoidance
and obstacle_tracker.py an actual moving object to detect in Gazebo.

Spawned by slam_nav.launch.py / multi_robot.launch.py when dynamic_obstacles > 0.

Run standalone against an already-spawned obstacle:
  ros2 run rosnav_bot dynamic_obstacle_driver.py --ros-args \
      -p obstacle_name:=dynamic_obstacle_1 -p axis:=y_axis -p amplitude:=3.0 -p speed:=0.4 \
      -r cmd_vel:=/model/dynamic_obstacle_1/cmd_vel

Note: axis defaults to "y_axis"/"x_axis" (not the bare letters "x"/"y") because
YAML 1.1 --params-file parsing reads unquoted "y"/"n" as booleans, which would
silently turn this parameter into a bool instead of a string.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

_PERIOD = 0.1


class DynamicObstacleDriver(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_driver')

        self.declare_parameter('obstacle_name', 'dynamic_obstacle_1')
        self.declare_parameter('axis', 'y_axis')
        self.declare_parameter('amplitude', 3.0)
        self.declare_parameter('speed', 0.4)

        self._name = self.get_parameter('obstacle_name').value
        raw_axis = self.get_parameter('axis').value.strip().lower()
        self._axis = 'x' if raw_axis.startswith('x') else 'y'
        if raw_axis not in ('x', 'y', 'x_axis', 'y_axis'):
            self.get_logger().warn(f'axis={raw_axis!r} unrecognised, defaulting to y')
        self._amplitude = float(self.get_parameter('amplitude').value)
        self._speed = abs(float(self.get_parameter('speed').value))

        self._pos = 0.0
        self._dir = 1.0

        self._pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._timer = self.create_timer(_PERIOD, self._tick)

        self.get_logger().info(
            f'{self._name}: patrolling +-{self._amplitude:.2f}m along {self._axis} '
            f'at {self._speed:.2f} m/s')

    def _tick(self):
        self._pos += self._dir * self._speed * _PERIOD
        if abs(self._pos) >= self._amplitude:
            self._pos = max(min(self._pos, self._amplitude), -self._amplitude)
            self._dir *= -1.0
            self.get_logger().info(
                f'{self._name}: reached {self._pos:+.2f}m, reversing to '
                f'{"+" if self._dir > 0 else "-"}{self._axis}')

        twist = Twist()
        velocity = self._dir * self._speed
        if self._axis == 'x':
            twist.linear.x = velocity
        else:
            twist.linear.y = velocity
        self._pub.publish(twist)


def main():
    rclpy.init()
    node = DynamicObstacleDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
