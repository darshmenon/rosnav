#!/usr/bin/env python3
"""
station_server.py — ROS 2 action/service layer around existing dock/undock.

  DockToStation      Nav2 to staging pose, then aruco_dock.py visual approach
  UndockFromStation  Nav2 BackUp, then Spin to docks.yaml undock_yaw_deg
  GetRobotStatus     latched is_docked + pose + battery

Usage
─────
  ros2 run rosnav_bot station_server.py
  ros2 run rosnav_bot station_server.py --ros-args -p namespace:=robot1

  ros2 service call /get_robot_status rosnav_bot/srv/GetRobotStatus
  ros2 action send_goal /dock_to_station rosnav_bot/action/DockToStation "{station: charging_dock}"
  ros2 action send_goal /undock_from_station rosnav_bot/action/UndockFromStation "{}"
"""

from __future__ import annotations

import math
import os
import signal
import subprocess
import threading
import time
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import BackUp, NavigateToPose, Spin
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

from rosnav_bot.action import DockToStation, UndockFromStation
from rosnav_bot.srv import GetRobotStatus

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False

PKG = 'rosnav_bot'
DEFAULT_DOCK = 'charging_dock'
UNDOCK_DIST = 0.5
UNDOCK_SPEED = 0.05
UNDOCK_TIMEOUT = 15.0
UNDOCK_SPIN_TIMEOUT = 20.0
NAV_TIMEOUT = 120.0
ARUCO_TIMEOUT = 120.0


def _package_share() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(PKG)
    except Exception:
        return os.path.join(os.path.expanduser('~'), 'rosnav', 'src', 'rosnav_bot')


def _resolve_config(path: str) -> str:
    if not path:
        return ''
    if os.path.isabs(path) and os.path.isfile(path):
        return path
    share = _package_share()
    for c in (
        path,
        os.path.join(share, 'config', path),
        os.path.join(share, path),
        os.path.join(os.path.expanduser('~'), 'rosnav', 'src', 'rosnav_bot', 'config', path),
    ):
        if os.path.isfile(c):
            return c
    return path


def _load_yaml(path: str) -> dict:
    if not HAS_YAML or not path or not os.path.isfile(path):
        return {}
    with open(path) as f:
        return yaml.safe_load(f) or {}


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _angle_diff(a: float, b: float) -> float:
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def _topic(ns: str, name: str) -> str:
    ns = (ns or '').strip().strip('/')
    return f'/{ns}/{name}' if ns else f'/{name}'


class StationServer(Node):
    def __init__(self):
        super().__init__('station_server')

        self.declare_parameter('namespace', '')
        self.declare_parameter('docks_file', 'docks.yaml')
        self.declare_parameter('locations_file', 'locations.yaml')
        self.declare_parameter('frame_id', 'map')

        self._ns = str(self.get_parameter('namespace').value or '').strip().strip('/')
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._docks_file = str(self.get_parameter('docks_file').value)
        self._locations_file = str(self.get_parameter('locations_file').value)

        self._cb_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._is_docked = False
        self._dock_name = ''
        self._pose: Optional[tuple[float, float, float]] = None
        self._battery_percent = float('nan')
        self._busy = False
        self._nav_handle = None
        self._backup_handle = None
        self._spin_handle = None
        self._aruco_proc: Optional[subprocess.Popen] = None

        self._nav_client = ActionClient(
            self, NavigateToPose, _topic(self._ns, 'navigate_to_pose'),
            callback_group=self._cb_group)
        self._backup_client = ActionClient(
            self, BackUp, _topic(self._ns, 'backup'),
            callback_group=self._cb_group)
        self._spin_client = ActionClient(
            self, Spin, _topic(self._ns, 'spin'),
            callback_group=self._cb_group)

        self.create_subscription(
            PoseWithCovarianceStamped, _topic(self._ns, 'amcl_pose'),
            self._amcl_cb, 10)
        self.create_subscription(
            BatteryState, _topic(self._ns, 'battery_state'),
            self._battery_cb, 10)
        self._cmd_pub = self.create_publisher(
            Twist, _topic(self._ns, 'cmd_vel'), 10)

        dock_action = _topic(self._ns, 'dock_to_station')
        undock_action = _topic(self._ns, 'undock_from_station')
        status_srv = _topic(self._ns, 'get_robot_status')

        self._dock_server = ActionServer(
            self, DockToStation, dock_action,
            execute_callback=self._execute_dock,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group)
        self._undock_server = ActionServer(
            self, UndockFromStation, undock_action,
            execute_callback=self._execute_undock,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cb_group)
        self.create_service(
            GetRobotStatus, status_srv, self._status_cb,
            callback_group=self._cb_group)

        docks = self._load_docks()
        self.get_logger().info(
            f'station_server ready  ns={self._ns or "(none)"}  '
            f'docks={list(docks.keys()) or "none"}')

    # ── config ────────────────────────────────────────────────────────────────

    def _load_docks(self) -> dict:
        data = _load_yaml(_resolve_config(self._docks_file))
        return dict(data.get('docks', {}) or {})

    def _load_locations(self) -> dict:
        data = _load_yaml(_resolve_config(self._locations_file))
        return dict(data.get('locations', {}) or {})

    def _resolve_staging(self, station: str) -> Optional[tuple[float, float, float]]:
        docks = self._load_docks()
        locations = self._load_locations()
        dock_cfg = dict(docks.get(station, {}))
        staging = dock_cfg.get('staging', station)

        if isinstance(staging, (list, tuple)) and len(staging) >= 2:
            yaw = math.radians(float(staging[2])) if len(staging) > 2 else 0.0
            return float(staging[0]), float(staging[1]), yaw

        name = str(staging)
        if name in locations:
            c = locations[name]
            yaw = math.radians(float(c[2])) if len(c) > 2 else 0.0
            return float(c[0]), float(c[1]), yaw
        if station in locations:
            c = locations[station]
            yaw = math.radians(float(c[2])) if len(c) > 2 else 0.0
            return float(c[0]), float(c[1]), yaw
        return None

    # ── sensors / status ──────────────────────────────────────────────────────

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        self._pose = (p.x, p.y, yaw)

    def _battery_cb(self, msg: BatteryState):
        pct = float(msg.percentage)
        if 0.0 <= pct <= 1.0:
            pct *= 100.0
        self._battery_percent = pct

    def _status_cb(self, request, response):
        with self._lock:
            response.is_docked = self._is_docked
            response.dock_name = self._dock_name
            response.nav_busy = self._busy
        if self._pose:
            response.x, response.y, response.yaw = self._pose
        else:
            response.x = response.y = response.yaw = float('nan')
        response.battery_percent = float(self._battery_percent)
        if math.isnan(response.x):
            response.message = 'no amcl_pose yet'
        elif response.is_docked:
            response.message = f'docked at {response.dock_name or "unknown"}'
        else:
            response.message = 'undocked'
        return response

    def _goal_cb(self, _goal_request):
        with self._lock:
            busy = self._busy
        if busy:
            self.get_logger().warn('rejecting goal — station server busy')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle):
        self._cancel_active()
        return CancelResponse.ACCEPT

    def _set_busy(self, busy: bool):
        with self._lock:
            self._busy = busy

    def _set_docked(self, docked: bool, name: str = ''):
        with self._lock:
            self._is_docked = docked
            self._dock_name = name if docked else ''

    # ── cancel helpers ────────────────────────────────────────────────────────

    def _cancel_active(self):
        nav = self._nav_handle
        backup = self._backup_handle
        spin = self._spin_handle
        proc = self._aruco_proc
        for handle in (nav, backup, spin):
            if handle is not None:
                try:
                    handle.cancel_goal_async()
                except Exception:
                    pass
        if proc is not None and proc.poll() is None:
            try:
                proc.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass

    def _stop_cmd_vel(self):
        self._cmd_pub.publish(Twist())

    # ── action wait ───────────────────────────────────────────────────────────

    def _await_future(self, future, timeout: float):
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not future.done():
            return None
        return future.result()

    def _run_nav2_action(self, client, goal_msg, timeout: float, goal_handle, store_attr: str):
        if not client.wait_for_server(timeout_sec=5.0):
            return False, 'action server unavailable'
        send_future = client.send_goal_async(goal_msg)
        handle = self._await_future(send_future, 10.0)
        if handle is None or not handle.accepted:
            return False, 'goal rejected'
        setattr(self, store_attr, handle)
        result_future = handle.get_result_async()
        deadline = time.time() + timeout
        while not result_future.done():
            if time.time() > deadline:
                handle.cancel_goal_async()
                setattr(self, store_attr, None)
                return False, 'timeout'
            if goal_handle.is_cancel_requested:
                handle.cancel_goal_async()
                setattr(self, store_attr, None)
                return False, 'cancelled'
            time.sleep(0.05)
        setattr(self, store_attr, None)
        result = result_future.result()
        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            status = getattr(result, 'status', '?')
            return False, f'failed (status {status})'
        return True, 'ok'

    def _publish_dock_fb(self, goal_handle, stage: str, message: str):
        fb = DockToStation.Feedback()
        fb.stage = stage
        fb.message = message
        goal_handle.publish_feedback(fb)

    def _publish_undock_fb(self, goal_handle, stage: str, message: str):
        fb = UndockFromStation.Feedback()
        fb.stage = stage
        fb.message = message
        goal_handle.publish_feedback(fb)

    # ── dock ──────────────────────────────────────────────────────────────────

    def _execute_dock(self, goal_handle):
        self._set_busy(True)
        station = (goal_handle.request.station or '').strip() or DEFAULT_DOCK
        result = DockToStation.Result()
        try:
            staging = self._resolve_staging(station)
            if staging is None:
                result.success = False
                result.message = (
                    f'unknown station {station!r}. '
                    f'docks={list(self._load_docks())} '
                    f'locations={list(self._load_locations())}')
                goal_handle.abort()
                return result

            x, y, yaw = staging
            self._publish_dock_fb(
                goal_handle, 'staging',
                f'navigating to staging ({x:.2f}, {y:.2f})')
            self.get_logger().info(
                f'dock({station}) staging ({x:.2f}, {y:.2f}, yaw={math.degrees(yaw):.0f}°)')

            pose = PoseStamped()
            pose.header.frame_id = self._frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            ox, oy, oz, ow = _yaw_to_quat(yaw)
            pose.pose.orientation.x = ox
            pose.pose.orientation.y = oy
            pose.pose.orientation.z = oz
            pose.pose.orientation.w = ow
            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = pose

            ok, msg = self._run_nav2_action(
                self._nav_client, nav_goal, NAV_TIMEOUT, goal_handle, '_nav_handle')
            if not ok:
                result.success = False
                result.message = f'staging failed: {msg}'
                if msg == 'cancelled':
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return result

            self._stop_cmd_vel()
            time.sleep(0.2)

            docks = self._load_docks()
            if station in docks or station == DEFAULT_DOCK:
                self._publish_dock_fb(goal_handle, 'visual', 'ArUco visual approach')
                ok, msg = self._visual_dock(station, goal_handle)
                if not ok:
                    result.success = False
                    result.message = f'visual dock failed: {msg}'
                    if msg == 'cancelled':
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()
                    return result
            else:
                self.get_logger().info(
                    f'no docks.yaml entry for {station} — treating staging arrival as docked')

            self._set_docked(True, station)
            result.success = True
            result.message = f'docked at {station}'
            self._publish_dock_fb(goal_handle, 'done', result.message)
            goal_handle.succeed()
            return result
        finally:
            self._nav_handle = None
            self._aruco_proc = None
            self._set_busy(False)

    def _visual_dock(self, dock_name: str, goal_handle) -> tuple[bool, str]:
        cmd = ['ros2', 'run', PKG, 'aruco_dock.py', '--ros-args']
        if self._ns:
            cmd += ['-p', f'namespace:={self._ns}']
        cmd += [
            '-p', f'dock_name:={dock_name}',
            '-p', f'docks_file:={self._docks_file}',
            '-p', 'prefer_restage:=true',
            '-p', 'max_retries:=3',
        ]
        try:
            proc = subprocess.Popen(cmd)
        except Exception as e:
            return False, str(e)
        self._aruco_proc = proc
        deadline = time.time() + ARUCO_TIMEOUT
        while proc.poll() is None:
            if time.time() > deadline:
                try:
                    proc.send_signal(signal.SIGINT)
                except ProcessLookupError:
                    pass
                return False, 'timeout'
            if goal_handle.is_cancel_requested:
                try:
                    proc.send_signal(signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
                return False, 'cancelled'
            time.sleep(0.1)
        self._aruco_proc = None
        if proc.returncode == 0:
            return True, 'ok'
        return False, f'exit {proc.returncode}'

    # ── undock ────────────────────────────────────────────────────────────────

    def _execute_undock(self, goal_handle):
        self._set_busy(True)
        result = UndockFromStation.Result()
        dist = float(goal_handle.request.dist) or UNDOCK_DIST
        speed = float(goal_handle.request.speed) or UNDOCK_SPEED
        try:
            self._publish_undock_fb(
                goal_handle, 'backup', f'backing up {dist:.2f} m')
            self.get_logger().info(f'undock backup {dist:.2f} m @ {speed:.2f} m/s')

            backup_goal = BackUp.Goal()
            backup_goal.target.x = dist
            backup_goal.speed = speed
            backup_goal.time_allowance.sec = int(UNDOCK_TIMEOUT)

            ok, msg = self._run_nav2_action(
                self._backup_client, backup_goal, UNDOCK_TIMEOUT + 5.0,
                goal_handle, '_backup_handle')
            if not ok:
                result.success = False
                result.message = f'backup failed: {msg}'
                if msg == 'cancelled':
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return result

            dock_name = self._dock_name or DEFAULT_DOCK
            dock_cfg = dict(self._load_docks().get(dock_name, {}))
            undock_yaw_deg = dock_cfg.get('undock_yaw_deg')
            if undock_yaw_deg is not None:
                target_yaw = math.radians(float(undock_yaw_deg))
                current_yaw = self._pose[2] if self._pose else None
                if current_yaw is None:
                    self.get_logger().warn('no amcl_pose for exit spin')
                else:
                    delta = _angle_diff(target_yaw, current_yaw)
                    if abs(delta) >= math.radians(5.0):
                        self._publish_undock_fb(
                            goal_handle, 'spin',
                            f'spinning {math.degrees(delta):+.1f}°')
                        spin_goal = Spin.Goal()
                        spin_goal.target_yaw = float(delta)
                        spin_goal.time_allowance.sec = int(UNDOCK_SPIN_TIMEOUT)
                        ok, msg = self._run_nav2_action(
                            self._spin_client, spin_goal, UNDOCK_SPIN_TIMEOUT + 5.0,
                            goal_handle, '_spin_handle')
                        if not ok and msg == 'cancelled':
                            result.success = False
                            result.message = 'cancelled'
                            goal_handle.canceled()
                            return result
                        if not ok:
                            self.get_logger().warn(f'exit spin incomplete: {msg}')

            self._set_docked(False)
            result.success = True
            result.message = 'undocked'
            self._publish_undock_fb(goal_handle, 'done', result.message)
            goal_handle.succeed()
            return result
        finally:
            self._backup_handle = None
            self._spin_handle = None
            self._set_busy(False)


def main(args=None):
    rclpy.init(args=args)
    node = StationServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._cancel_active()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
