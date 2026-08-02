#!/usr/bin/env python3
"""
rmf_submit_task.py — Submit a task to rmf_task_dispatcher over the
/task_api_requests topic, so you can watch RMF's traffic scheduler negotiate
paths across the fleet registered by rmf_fleet_adapter.py.

Requires rmf_fleet.launch.py already running (rmf_traffic_schedule +
rmf_task_dispatcher + rmf_fleet_adapter.py, on top of a static-map
multi_robot.launch.py explore:=false).

Payload shape verified against open-rmf/rmf_api_msgs schemas
(dispatch_task_request.json / robot_task_request.json / task_request.json)
and open-rmf/rmf_demos's dispatch_patrol.py (Humble) reference implementation.

Usage
─────
  ros2 run diff_drive_robot rmf_submit_task.py patrol room_a room_b --rounds 2
  ros2 run diff_drive_robot rmf_submit_task.py patrol charging_dock room_a
  ros2 run diff_drive_robot rmf_submit_task.py patrol room_a room_b \\
      --fleet rosnav_fleet --robot robot1
"""

import argparse
import json
import sys
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)

from rmf_task_msgs.msg import ApiRequest, ApiResponse


class TaskSubmitter(Node):
    def __init__(self, use_sim_time: bool = True):
        super().__init__('rmf_submit_task')
        if use_sim_time:
            self.set_parameters([
                Parameter('use_sim_time', Parameter.Type.BOOL, True),
            ])

        transient_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(ApiRequest, '/task_api_requests', transient_qos)

        response_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._sub = self.create_subscription(
            ApiResponse, '/task_api_responses', self._on_response, response_qos)
        self._responses = {}

    def wait_for_clock(self, timeout_sec: float = 5.0):
        """Block until get_clock().now() is non-zero (sim time needs a /clock tick first)."""
        deadline = time.time() + timeout_sec
        while self.get_clock().now().nanoseconds == 0 and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.get_clock().now().nanoseconds == 0:
            self.get_logger().warning(
                f'no /clock tick within {timeout_sec:.1f}s — timestamps will read as 0 '
                f'(is use_sim_time correct? try --no-sim-time if not running against Gazebo)')

    def _on_response(self, msg: ApiResponse):
        self._responses[msg.request_id] = msg
        kind = {0: 'UNINITIALIZED', 1: 'ACKNOWLEDGE', 2: 'RESPONDING'}.get(msg.type, msg.type)
        print(f'[rmf_submit_task] response ({kind}) for {msg.request_id}: {msg.json_msg}')

    def submit(self, envelope: dict, wait_sec: float = 5.0) -> str:
        request_id = f'patrol_{uuid.uuid4().hex[:8]}'
        msg = ApiRequest()
        msg.request_id = request_id
        msg.json_msg = json.dumps(envelope)

        # Give late-joining TRANSIENT_LOCAL subscribers a moment to match.
        time.sleep(0.5)
        self._pub.publish(msg)
        print(f'[rmf_submit_task] submitted {request_id}:\n{json.dumps(envelope, indent=2)}')

        deadline = time.time() + wait_sec
        while time.time() < deadline and request_id not in self._responses:
            rclpy.spin_once(self, timeout_sec=0.2)
        if request_id not in self._responses:
            print(f'[rmf_submit_task] no response within {wait_sec:.1f}s — '
                  f'is rmf_task_dispatcher running? (ros2 node list)')
        return request_id


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'category', choices=['patrol'],
        help='Task category. Only "patrol" (loop between named locations) is wired up.')
    parser.add_argument('places', nargs='+', help='Named locations from locations.yaml, in order')
    parser.add_argument('--rounds', type=int, default=1, help='Number of times to loop the places')
    parser.add_argument('--wait', type=float, default=5.0, help='Seconds to wait for a response')
    parser.add_argument('--fleet', type=str, default='', help='Optional fleet name for robot_task_request')
    parser.add_argument('--robot', type=str, default='', help='Optional robot name for robot_task_request')
    parser.add_argument('--start-time', type=int, default=0,
                        help='Earliest start offset from now, in seconds')
    parser.add_argument('--requester', type=str, default='rosnav_fleet',
                        help='Identifier for the entity requesting this task')
    parser.add_argument('--no-sim-time', action='store_true',
                        help='Use wall clock instead of /clock')
    args = parser.parse_args()

    if bool(args.fleet) ^ bool(args.robot):
        print('[rmf_submit_task] --fleet and --robot must be set together')
        sys.exit(2)

    rclpy.init()
    node = TaskSubmitter(use_sim_time=not args.no_sim_time)
    try:
        if not args.no_sim_time:
            node.wait_for_clock()
        now = node.get_clock().now().to_msg()
        request_time_ms = now.sec * 1000 + round(now.nanosec / 1e6)
        earliest_start_ms = request_time_ms + args.start_time * 1000

        request = {
            'unix_millis_request_time': request_time_ms,
            'unix_millis_earliest_start_time': earliest_start_ms,
            'requester': args.requester,
            'category': args.category,
            'description': {
                'places': args.places,
                'rounds': args.rounds,
            },
        }

        if args.robot and args.fleet:
            envelope = {
                'type': 'robot_task_request',
                'robot': args.robot,
                'fleet': args.fleet,
                'request': request,
            }
        else:
            envelope = {
                'type': 'dispatch_task_request',
                'request': request,
            }

        node.submit(envelope, wait_sec=args.wait)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
