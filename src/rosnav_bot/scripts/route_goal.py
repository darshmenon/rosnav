#!/usr/bin/env python3
"""Drive to a goal along the Route Server lanes: ComputeRoute -> publish /route_path -> FollowPath.

  ros2 run rosnav_bot route_goal.py --ros-args -p goal_x:=2.0 -p goal_y:=-1.0 [-p follow:=false]

Needs slam_nav.launch.py route_graph:=auto (route_server active). With follow:=false it only
computes + publishes the lane path (view /route_path in RViz), so it is safe to try first.
"""
import sys

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputeRoute, FollowPath


class RouteGoal(Node):
    def __init__(self):
        super().__init__('route_goal')
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('frame', 'map')
        self.declare_parameter('follow', True)
        self.declare_parameter('controller_id', 'FollowPath')
        self.pub = self.create_publisher(Path, 'route_path', 1)
        self.route_ac = ActionClient(self, ComputeRoute, 'compute_route')
        self.follow_ac = ActionClient(self, FollowPath, 'follow_path')

    def run(self) -> int:
        log = self.get_logger()
        if not self.route_ac.wait_for_server(timeout_sec=15.0):
            log.error('compute_route action not available - launch with route_graph:=auto')
            return 2
        goal = ComputeRoute.Goal()
        goal.use_start = False          # start pose from TF
        goal.use_poses = True           # snap goal pose to the nearest graph node
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = self.get_parameter('frame').value
        goal.goal.pose.position.x = float(self.get_parameter('goal_x').value)
        goal.goal.pose.position.y = float(self.get_parameter('goal_y').value)
        goal.goal.pose.orientation.w = 1.0
        fut = self.route_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if handle is None or not handle.accepted:
            log.error('compute_route goal rejected')
            return 3
        rf = handle.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        outcome = rf.result()
        res = outcome.result
        # Humble's result has no error_code: failure = aborted status or an empty path
        if outcome.status != GoalStatus.STATUS_SUCCEEDED or not res.path.poses:
            log.error(f'no route (action status={outcome.status})')
            return 4
        n = len(res.path.poses)
        log.info(f'route: {len(res.route.nodes)} nodes, {n} path poses, '
                 f'planned in {res.planning_time.sec + res.planning_time.nanosec * 1e-9:.3f} s')
        self.pub.publish(res.path)
        if not self.get_parameter('follow').value:
            return 0
        if not self.follow_ac.wait_for_server(timeout_sec=10.0):
            log.error('follow_path action not available')
            return 5
        fg = FollowPath.Goal()
        fg.path = res.path
        fg.controller_id = self.get_parameter('controller_id').value
        ff = self.follow_ac.send_goal_async(fg)
        rclpy.spin_until_future_complete(self, ff)
        fh = ff.result()
        if fh is None or not fh.accepted:
            log.error('follow_path goal rejected')
            return 6
        fr = fh.get_result_async()
        rclpy.spin_until_future_complete(self, fr)
        log.info(f'follow_path finished, status={fr.result().status}')
        return 0


def main():
    rclpy.init()
    node = RouteGoal()
    rc = node.run()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
