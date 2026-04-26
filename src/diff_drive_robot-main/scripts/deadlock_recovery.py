#!/usr/bin/env python3
"""
deadlock_recovery.py — Detects and recovers stuck robots in multi-robot Nav2.

A robot is "stuck" when it has an active navigation goal but hasn't moved more
than `progress_threshold` metres within `stuck_timeout` seconds.

Recovery sequence
─────────────────
  1. Cancel the active navigate_to_pose goal.
  2. Publish a reverse + spin escape manoeuvre directly to cmd_vel.
  3. Re-issue the original goal after `recover_pause` seconds.

The node tracks robot progress via /robotN/odom.  It infers "active goal" by
watching the navigate_to_pose action status topic.

Parameters
──────────
  robot_namespaces     (str)   comma-separated, e.g. "robot1,robot2"
  stuck_timeout        (float) seconds without progress → stuck (default 20.0)
  progress_threshold   (float) metres — minimum movement to reset timer  (default 0.15)
  check_period         (float) seconds between progress checks  (default 2.0)
  reverse_duration     (float) seconds to reverse during recovery  (default 1.5)
  spin_duration        (float) seconds to spin during recovery  (default 2.0)
  recover_pause        (float) seconds to wait before re-issuing goal  (default 1.0)

Usage
─────
  ros2 run diff_drive_robot deadlock_recovery.py \\
      --ros-args -p robot_namespaces:=robot1,robot2
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose


_NAVIGATING_STATUSES = {
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
}


class RobotTracker:
    """Per-robot state tracked by DeadlockRecovery."""

    def __init__(self):
        self.pos          = (0.0, 0.0)
        self.last_progress_pos  = (0.0, 0.0)
        self.last_progress_time = time.monotonic()
        self.active_goal_handle = None
        self.active_goal_pose   = None   # PoseStamped to re-issue
        self.recovering         = False
        self.lock               = threading.Lock()


class DeadlockRecovery(Node):
    def __init__(self):
        super().__init__('deadlock_recovery')

        self.declare_parameter('robot_namespaces',   'robot1,robot2')
        self.declare_parameter('stuck_timeout',      20.0)
        self.declare_parameter('progress_threshold', 0.15)
        self.declare_parameter('check_period',       2.0)
        self.declare_parameter('reverse_duration',   1.5)
        self.declare_parameter('spin_duration',      2.0)
        self.declare_parameter('recover_pause',      1.0)

        ns_param = self.get_parameter('robot_namespaces').value
        self._robots = [r.strip() for r in ns_param.split(',') if r.strip()]

        self._timeout    = self.get_parameter('stuck_timeout').value
        self._thresh     = self.get_parameter('progress_threshold').value
        self._rev_dur    = self.get_parameter('reverse_duration').value
        self._spin_dur   = self.get_parameter('spin_duration').value
        self._pause      = self.get_parameter('recover_pause').value

        self._trackers: dict[str, RobotTracker] = {}
        self._vel_pubs:  dict[str, object]       = {}
        self._nav_clients: dict[str, ActionClient] = {}

        for ns in self._robots:
            t = RobotTracker()
            self._trackers[ns] = t

            self.create_subscription(
                Odometry, f'/{ns}/odom',
                lambda msg, n=ns: self._odom_cb(msg, n), 10)

            self._vel_pubs[ns] = self.create_publisher(
                Twist, f'/{ns}/cmd_vel', 10)

            # Action client to cancel and re-issue navigate_to_pose
            client = ActionClient(self, NavigateToPose, f'/{ns}/navigate_to_pose')
            self._nav_clients[ns] = client

            # Watch goal result to capture goal handles
            # We monkey-patch the client's _goal_response_cb in _send_goal wrapper

        check_period = self.get_parameter('check_period').value
        self.create_timer(check_period, self._check_all)

        self.get_logger().info(
            f'DeadlockRecovery ready  robots={self._robots}  '
            f'timeout={self._timeout}s  threshold={self._thresh}m')

    # ── Odom callback ─────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry, ns: str):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        t = self._trackers[ns]
        with t.lock:
            t.pos = (x, y)

    # ── Progress check ────────────────────────────────────────────────────────

    def _check_all(self):
        for ns, t in self._trackers.items():
            with t.lock:
                if t.recovering:
                    continue
                cx, cy = t.pos
                lx, ly = t.last_progress_pos
                moved = math.hypot(cx - lx, cy - ly)
                if moved >= self._thresh:
                    t.last_progress_pos  = (cx, cy)
                    t.last_progress_time = time.monotonic()
                    continue
                elapsed = time.monotonic() - t.last_progress_time
                if elapsed >= self._timeout:
                    t.recovering = True
                    goal_handle = t.active_goal_handle
                    goal_pose   = t.active_goal_pose

            self.get_logger().warn(
                f'[deadlock] {ns} stuck for {elapsed:.0f}s — triggering recovery')
            threading.Thread(
                target=self._recover, args=(ns, goal_handle, goal_pose),
                daemon=True).start()

    # ── Recovery sequence ─────────────────────────────────────────────────────

    def _recover(self, ns: str, goal_handle, goal_pose):
        pub = self._vel_pubs[ns]
        t   = self._trackers[ns]

        # 1. Cancel active goal
        if goal_handle is not None:
            try:
                future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                self.get_logger().info(f'[recovery] {ns} goal cancelled')
            except Exception as exc:
                self.get_logger().warn(f'[recovery] {ns} cancel error: {exc}')

        # 2. Reverse
        rev = Twist()
        rev.linear.x = -0.15
        deadline = time.monotonic() + self._rev_dur
        while time.monotonic() < deadline:
            pub.publish(rev)
            time.sleep(0.05)

        # 3. Spin in place
        spin = Twist()
        spin.angular.z = 0.8
        deadline = time.monotonic() + self._spin_dur
        while time.monotonic() < deadline:
            pub.publish(spin)
            time.sleep(0.05)

        # 4. Stop
        pub.publish(Twist())
        time.sleep(self._pause)

        # 5. Re-issue original goal if we have one
        if goal_pose is not None:
            self._send_goal(ns, goal_pose)
            self.get_logger().info(f'[recovery] {ns} goal re-issued')
        else:
            self.get_logger().info(f'[recovery] {ns} no goal to re-issue — waiting for Nav2')

        # Reset tracker
        with t.lock:
            t.last_progress_pos  = t.pos
            t.last_progress_time = time.monotonic()
            t.recovering         = False

    # ── Goal sending / tracking ───────────────────────────────────────────────

    def _send_goal(self, ns: str, pose: PoseStamped):
        client = self._nav_clients[ns]
        if not client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(f'[recovery] {ns} navigate_to_pose server unavailable')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        future = client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self._goal_response(f, ns, pose))

    def _goal_response(self, future, ns: str, pose: PoseStamped):
        handle = future.result()
        if not handle or not handle.accepted:
            self.get_logger().warn(f'[recovery] {ns} goal rejected')
            return
        t = self._trackers[ns]
        with t.lock:
            t.active_goal_handle = handle
            t.active_goal_pose   = pose

    def register_goal(self, ns: str, pose: PoseStamped, handle):
        """Call from external code to register a newly accepted goal."""
        t = self._trackers.get(ns)
        if t is None:
            return
        with t.lock:
            t.active_goal_handle = handle
            t.active_goal_pose   = pose
            t.last_progress_pos  = t.pos
            t.last_progress_time = time.monotonic()


def main():
    rclpy.init()
    node = DeadlockRecovery()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
