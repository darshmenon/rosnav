"""Nav2-backed RMF RobotCommandHandle."""

from __future__ import annotations

import datetime
import math
import threading
from functools import partial
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose

import rmf_adapter as adpt
import rmf_adapter.plan as plan

from diff_drive_robot.rmf.config import RmfFleetConfig
from diff_drive_robot.rmf.docking import DockingPlugin


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ros_now_as_datetime(node: Node) -> datetime.datetime:
    return datetime.datetime.fromtimestamp(node.get_clock().now().nanoseconds / 1e9)


class Nav2RobotCommand(adpt.RobotCommandHandle):
    """Bridges RMF path-following / dock commands to Nav2 + a DockingPlugin."""

    def __init__(
        self,
        node: Node,
        ns: str,
        rmf_graph,
        cfg: RmfFleetConfig,
        docking: DockingPlugin,
    ):
        adpt.RobotCommandHandle.__init__(self)
        self.node = node
        self.ns = ns
        self.graph = rmf_graph
        self.map_name = cfg.map_name
        self.cfg = cfg
        self.docking = docking
        self.updater = None

        self._lock = threading.Lock()
        self._waypoints = []
        self._current_index = 0
        self._active = False
        self._next_arrival_estimator = None
        self._path_finished_cb = None
        self._goal_handle = None
        self._retries = 0
        self._last_pose = None

        self._nav_client = ActionClient(node, NavigateToPose, f'/{ns}/navigate_to_pose')

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pose_sub = node.create_subscription(
            PoseWithCovarianceStamped, f'/{ns}/amcl_pose',
            self._pose_cb, qos)

    def latest_start(self, max_wait_sec: Optional[float] = None):
        """Block briefly for one /{ns}/amcl_pose to seed the initial plan.Start."""
        wait = self.cfg.pose_wait_sec if max_wait_sec is None else max_wait_sec
        deadline = self.node.get_clock().now().nanoseconds / 1e9 + wait
        while self._last_pose is None and self.node.get_clock().now().nanoseconds / 1e9 < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if self._last_pose is None:
            self.node.get_logger().warning(
                f'[{self.ns}] no /{self.ns}/amcl_pose within {wait:.1f}s '
                f'(is AMCL running for this robot?)')
        return self._last_pose

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        self._last_pose = msg
        if self.updater is None:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        starts = plan.compute_plan_starts(
            self.graph, self.map_name, [[x], [y], [yaw]],
            ros_now_as_datetime(self.node))
        if starts:
            self.updater.update_position(starts)
        else:
            self.node.get_logger().warning(
                f'[{self.ns}] amcl_pose ({x:.2f},{y:.2f}) does not land on the '
                f'nav graph — check lanes / locations.yaml coverage')

    def follow_new_path(self, waypoints, next_arrival_estimator, path_finished_callback):
        self.stop()
        with self._lock:
            self._waypoints = waypoints
            self._current_index = 0
            self._active = True
            self._next_arrival_estimator = next_arrival_estimator
            self._path_finished_cb = path_finished_callback
            self._retries = 0
        self.node.get_logger().info(
            f'[{self.ns}] new RMF path: {len(waypoints)} waypoint(s)')
        self._send_next_goal()

    def stop(self):
        with self._lock:
            self._active = False
            self._path_finished_cb = None
            self._next_arrival_estimator = None
        self._cancel_nav_goal()
        self._cancel_dock()

    def _cancel_nav_goal(self):
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
        self._goal_handle = None

    def _cancel_dock(self):
        try:
            self.docking.cancel()
        except Exception as e:
            self.node.get_logger().warning(
                f'[{self.ns}] docking.cancel() failed: {e}')

    def _interrupt_rmf(self, reason: str):
        """Tell RMF this command failed so it can replan — do NOT claim success."""
        self.node.get_logger().error(f'[{self.ns}] {reason}')
        with self._lock:
            self._active = False
            self._path_finished_cb = None
            self._next_arrival_estimator = None
        self._cancel_nav_goal()
        self._cancel_dock()
        if self.updater is not None:
            try:
                self.updater.interrupted()
            except Exception as e:
                self.node.get_logger().warning(
                    f'[{self.ns}] updater.interrupted() failed: {e}')

    def dock(self, dock_name, docking_finished_callback):
        """Delegate final approach to the configured DockingPlugin."""
        self._cancel_dock()
        self.docking.start(
            namespace=self.ns,
            dock_name=dock_name,
            on_success=docking_finished_callback,
            on_failure=lambda reason: self._interrupt_rmf(
                f'{reason} — not signalling dock success'),
            log=self.node.get_logger(),
        )

    def _send_next_goal(self):
        with self._lock:
            if not self._active or self._current_index >= len(self._waypoints):
                return
            wp = self._waypoints[self._current_index]

        if not self._nav_client.wait_for_server(timeout_sec=self.cfg.goal_timeout_sec):
            self._interrupt_rmf('navigate_to_pose action server unavailable')
            return

        x, y, yaw = float(wp.position[0]), float(wp.position[1]), float(wp.position[2])
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(partial(self._goal_response_cb, wp=wp))

    def _goal_response_cb(self, future, wp):
        try:
            handle = future.result()
        except Exception as e:
            self.node.get_logger().warning(f'[{self.ns}] nav goal send failed: {e}')
            self._retry_or_give_up()
            return
        if handle is None or not handle.accepted:
            self.node.get_logger().warning(f'[{self.ns}] nav goal rejected')
            self._retry_or_give_up()
            return
        self._goal_handle = handle

        if self._next_arrival_estimator is not None and wp.time is not None:
            now = ros_now_as_datetime(self.node)
            with self._lock:
                idx = self._current_index
            self._next_arrival_estimator(idx, wp.time - now)

        result_future = handle.get_result_async()
        result_future.add_done_callback(partial(self._goal_result_cb, wp=wp))

    def _goal_result_cb(self, future, wp):
        try:
            result = future.result()
        except Exception as e:
            self.node.get_logger().warning(f'[{self.ns}] nav result error: {e}')
            self._retry_or_give_up()
            return
        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().warning(f'[{self.ns}] nav goal failed')
            self._retry_or_give_up()
            return

        self._retries = 0
        with self._lock:
            if not self._active:
                return
            if wp.graph_index is not None and self.updater is not None:
                self.updater.update_current_waypoint(wp.graph_index, wp.position[2])
            self._current_index += 1
            finished = self._current_index >= len(self._waypoints)
            cb = self._path_finished_cb if finished else None
            if finished:
                self._active = False
                self._path_finished_cb = None
                self._next_arrival_estimator = None

        if finished:
            self.node.get_logger().info(f'[{self.ns}] RMF path complete')
            if cb:
                cb()
        else:
            self._send_next_goal()

    def _retry_or_give_up(self):
        with self._lock:
            self._retries += 1
            retries = self._retries
            active = self._active
        if not active:
            return
        if retries > self.cfg.max_retries:
            self._interrupt_rmf(f'giving up after {self.cfg.max_retries} nav goal retries')
            return
        self._send_next_goal()
