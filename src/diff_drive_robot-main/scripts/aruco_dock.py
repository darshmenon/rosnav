#!/usr/bin/env python3
"""
aruco_dock.py — Final-approach visual docking using an ArUco marker.

Nav2 gets the robot to a staging pose near the dock. This node then takes
over for the last stretch: detect the dock ArUco marker and servo on
lateral offset, marker yaw, and distance.

Also publishes `detected_dock_pose` for Nav2 SimpleChargingDock.

Robustness
──────────
  * Fixed-rate control loop (not tied to camera frame rate)
  * EMA + jump rejection + PnP reprojection gate
  * Soft reverse retries OR Nav2 restage to staging pose
  * Bidirectional search spin
  * Optional BatteryState charge confirmation
  * Per-dock settings from config/docks.yaml

Usage
─────
  ros2 run diff_drive_robot aruco_dock.py --ros-args -p namespace:=robot1
  ros2 run diff_drive_robot aruco_dock.py --ros-args \\
      -p dock_name:=charging_dock -p max_retries:=3

Exit code 0 on successful dock, 1 on timeout/failure.
"""

from __future__ import annotations

import math
import os
import sys
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from sensor_msgs.msg import Image, CameraInfo, BatteryState

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False

PKG = 'diff_drive_robot'
ARUCO_DICT = cv2.aruco.DICT_4X4_50

MAX_LINEAR = 0.15
MAX_REVERSE = 0.08
MAX_ANGULAR = 0.6
KP_LINEAR = 0.6
KP_BEARING = 1.5
KP_YAW = 1.2
KD_BEARING = 0.15
ANGLE_OK = math.radians(3.0)
YAW_OK = math.radians(5.0)
DIST_TOLERANCE = 0.03
STALL_TIMEOUT = 0.4
LOST_TIMEOUT = 5.0
SETTLE_TIME = 0.75
LOG_PERIOD = 1.0
SEARCH_GRACE = 2.0
SEARCH_ANGULAR = 0.3
FILTER_ALPHA = 0.25
FILTER_ALPHA_NEAR = 0.15
NEAR_DIST = 0.7
YAW_BLEND_DIST = 1.0  # yaw-squaring only considered inside this distance — see _servo
BEARING_GATE = math.radians(8.0)  # yaw only blends in once roughly aimed at the marker
JUMP_LATERAL = 0.12
JUMP_DIST = 0.20
JUMP_YAW = math.radians(25.0)
CONTROL_HZ = 20.0
RETRY_REVERSE_DIST = 0.18
RETRY_REVERSE_SPEED = 0.06
RETRY_REVERSE_TIMEOUT = 4.0
REPROJ_MAX_PX = 4.0
DEBUG_IMAGE_PERIOD = 0.5
RESTAGE_TIMEOUT = 60.0
SEARCH_GIVE_UP = 12.0  # no marker during search → retry


def _package_share() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(PKG)
    except Exception:
        return os.path.join(
            os.path.expanduser('~'), 'rosnav', 'src', 'diff_drive_robot-main')


def _resolve_config(path: str) -> str:
    if not path:
        return ''
    if os.path.isabs(path) and os.path.isfile(path):
        return path
    share = _package_share()
    candidates = [
        path,
        os.path.join(share, 'config', path),
        os.path.join(share, path),
        os.path.join(os.path.expanduser('~'), 'rosnav', 'src', 'diff_drive_robot-main', 'config', path),
    ]
    for c in candidates:
        if os.path.isfile(c):
            return c
    return path


def _load_yaml(path: str) -> dict:
    if not HAS_YAML or not path or not os.path.isfile(path):
        return {}
    with open(path) as f:
        return yaml.safe_load(f) or {}


def _rot_to_quat(R: np.ndarray) -> Quaternion:
    q = Quaternion()
    trace = float(R[0, 0] + R[1, 1] + R[2, 2])
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        q.w = 0.25 / s
        q.x = (R[2, 1] - R[1, 2]) * s
        q.y = (R[0, 2] - R[2, 0]) * s
        q.z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q.w = (R[2, 1] - R[1, 2]) / s
        q.x = 0.25 * s
        q.y = (R[0, 1] + R[1, 0]) / s
        q.z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q.w = (R[0, 2] - R[2, 0]) / s
        q.x = (R[0, 1] + R[1, 0]) / s
        q.y = 0.25 * s
        q.z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q.w = (R[1, 0] - R[0, 1]) / s
        q.x = (R[0, 2] + R[2, 0]) / s
        q.y = (R[1, 2] + R[2, 1]) / s
        q.z = 0.25 * s
    return q


class ArucoDock(Node):
    def __init__(self):
        super().__init__('aruco_dock')

        self.declare_parameter('namespace', '')
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_length', 0.24)
        self.declare_parameter('target_distance', 0.40)
        self.declare_parameter('timeout', 90.0)
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('servo', True)
        self.declare_parameter('debug_image_path', '/tmp/aruco_dock_view.jpg')
        self.declare_parameter('dock_name', '')
        self.declare_parameter('docks_file', 'docks.yaml')
        self.declare_parameter('locations_file', 'locations.yaml')
        self.declare_parameter('staging_x', float('nan'))
        self.declare_parameter('staging_y', float('nan'))
        self.declare_parameter('staging_yaw', float('nan'))  # radians
        self.declare_parameter('require_charge', False)
        self.declare_parameter('charge_timeout', 5.0)
        self.declare_parameter('prefer_restage', True)

        ns = self.get_parameter('namespace').value
        self._dock_name = str(self.get_parameter('dock_name').value or '')
        self._servo_enabled = bool(self.get_parameter('servo').value)
        self._debug_image_path = self.get_parameter('debug_image_path').value
        self._prefer_restage = bool(self.get_parameter('prefer_restage').value)
        self._last_debug_write = None

        dock_cfg = self._load_dock_config()
        self._marker_id = int(dock_cfg.get(
            'marker_id', self.get_parameter('marker_id').value))
        self._marker_length = float(dock_cfg.get(
            'marker_length', self.get_parameter('marker_length').value))
        self._target_distance = float(dock_cfg.get(
            'target_distance', self.get_parameter('target_distance').value))
        self._timeout = float(self.get_parameter('timeout').value)
        self._max_retries = int(dock_cfg.get(
            'max_retries', self.get_parameter('max_retries').value))
        self._require_charge = bool(dock_cfg.get(
            'require_charge', self.get_parameter('require_charge').value))
        self._charge_timeout = float(dock_cfg.get(
            'charge_timeout', self.get_parameter('charge_timeout').value))

        self._staging = self._resolve_staging(dock_cfg)
        self._charging = False

        pre = f'/{ns}' if ns else ''
        self._ns = ns
        self._optical_frame = f'{ns}/camera_optical_frame' if ns else 'camera_optical_frame'
        nav_action = f'/{ns}/navigate_to_pose' if ns else '/navigate_to_pose'

        self._cmd_pub = self.create_publisher(Twist, f'{pre}/cmd_vel', 10)
        self._pose_pub = self.create_publisher(PoseStamped, f'{pre}/detected_dock_pose', 10)
        self.create_subscription(Image, f'{pre}/camera/image_raw', self._image_cb, 10)
        self.create_subscription(CameraInfo, f'{pre}/camera/camera_info', self._info_cb, 10)
        self.create_subscription(Odometry, f'{pre}/odom', self._odom_cb, 10)
        self.create_subscription(BatteryState, f'{pre}/battery_state', self._battery_cb, 10)

        self._nav_client = ActionClient(self, NavigateToPose, nav_action)
        self._restage_send_future = None
        self._restage_result_future = None
        self._restage_handle = None
        self._restage_start = None

        # collision_monitor's default stop/slowdown distances are tuned for general
        # nav obstacle avoidance and are typically larger than a dock's target_distance —
        # left alone, it would zero our velocity before we ever reach the marker. Shrink
        # them for the duration of the approach, restoring the original values on exit.
        self._collision_get_client = self.create_client(
            GetParameters, f'{pre}/collision_monitor/get_parameters')
        self._collision_set_client = self.create_client(
            SetParameters, f'{pre}/collision_monitor/set_parameters')
        self._collision_orig = None
        self._collision_restored = False

        self._pose = None
        self._speed = None
        self._last_log = None
        self._exit_code = None

        self._bridge = CvBridge()
        self._dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._detector = cv2.aruco.ArucoDetector(self._dictionary, params)
        self._camera_matrix = None
        self._dist_coeffs = None
        self._rebuild_obj_points()

        self._last_seen = None
        self._settle_start = None
        self._charge_start = None
        self._filt = None
        self._meas = None
        self._prev_bearing = None
        self._start_time = self.get_clock().now()
        self._attempt_start = self._start_time
        self._retries = 0
        self._mode = 'search'
        self._search_dir = 1.0
        self._reverse_start = None
        self._reverse_start_pose = None

        dock_label = f' ({self._dock_name})' if self._dock_name else ''
        mode = 'servo' if self._servo_enabled else 'detect-only'
        staging_str = (
            f' staging=({self._staging[0]:.2f},{self._staging[1]:.2f},'
            f'{math.degrees(self._staging[2]):.0f}°)'
            if self._staging else ' staging=none')
        self.get_logger().info(
            f'Visual docking ({mode}){dock_label} marker={self._marker_id} '
            f'target={self._target_distance:.2f}m retries={self._max_retries}'
            f'{staging_str} charge={self._require_charge}')
        self.create_timer(1.0 / CONTROL_HZ, self._control_tick)
        if self._servo_enabled:
            self._shrink_collision_thresholds()

    def _shrink_collision_thresholds(self):
        if not self._collision_get_client.wait_for_service(timeout_sec=4.0):
            self.get_logger().warning(
                'collision_monitor not found — skipping threshold shrink '
                '(safety layer may block reaching target_distance).')
            return
        req = GetParameters.Request()
        req.names = ['stop_distance', 'slowdown_distance']
        future = self._collision_get_client.call_async(req)
        future.add_done_callback(self._on_collision_params_read)

    def _on_collision_params_read(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warning(f'Could not read collision_monitor params: {e}')
            return
        stop_v, slow_v = result.values
        self._collision_orig = (stop_v.double_value, slow_v.double_value)

        new_stop = max(0.05, self._target_distance - 0.15)
        new_slow = self._target_distance + 0.05
        if new_stop >= self._collision_orig[0] and new_slow >= self._collision_orig[1]:
            return  # existing thresholds are already compatible with this dock

        self._set_collision_thresholds(new_stop, new_slow)
        self.get_logger().info(
            f'Shrunk collision thresholds for docking: stop={new_stop:.2f}m '
            f'slow={new_slow:.2f}m (was {self._collision_orig[0]:.2f}/{self._collision_orig[1]:.2f}m)')

    def _set_collision_thresholds(self, stop_dist: float, slow_dist: float):
        if not self._collision_set_client.service_is_ready():
            return
        req = SetParameters.Request()
        req.parameters = [
            Parameter(name='stop_distance', value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=stop_dist)),
            Parameter(name='slowdown_distance', value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=slow_dist)),
        ]
        self._collision_set_client.call_async(req)

    def _restore_collision_thresholds(self):
        if self._collision_restored or self._collision_orig is None:
            return
        self._collision_restored = True
        self._set_collision_thresholds(*self._collision_orig)

    def _rebuild_obj_points(self):
        half = self._marker_length / 2.0
        self._obj_points = np.array([
            [-half, half, 0],
            [half, half, 0],
            [half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float32)

    def _load_dock_config(self) -> dict:
        docks_path = _resolve_config(str(self.get_parameter('docks_file').value))
        data = _load_yaml(docks_path)
        docks = data.get('docks', {}) or {}
        if self._dock_name and self._dock_name in docks:
            return dict(docks[self._dock_name])
        if len(docks) == 1:
            return dict(next(iter(docks.values())))
        return {}

    def _resolve_staging(self, dock_cfg: dict) -> Optional[Tuple[float, float, float]]:
        sx = float(self.get_parameter('staging_x').value)
        sy = float(self.get_parameter('staging_y').value)
        syaw = float(self.get_parameter('staging_yaw').value)
        if not (math.isnan(sx) or math.isnan(sy) or math.isnan(syaw)):
            return (sx, sy, syaw)

        staging = dock_cfg.get('staging')
        if staging is None:
            return None
        if isinstance(staging, (list, tuple)) and len(staging) >= 2:
            yaw = math.radians(float(staging[2])) if len(staging) > 2 else 0.0
            return (float(staging[0]), float(staging[1]), yaw)

        # Location name → locations.yaml
        loc_path = _resolve_config(str(self.get_parameter('locations_file').value))
        locs = (_load_yaml(loc_path).get('locations') or {})
        name = str(staging)
        if name in locs:
            c = locs[name]
            yaw = math.radians(float(c[2])) if len(c) > 2 else 0.0
            return (float(c[0]), float(c[1]), yaw)
        if self._dock_name and self._dock_name in locs:
            c = locs[self._dock_name]
            yaw = math.radians(float(c[2])) if len(c) > 2 else 0.0
            return (float(c[0]), float(c[1]), yaw)
        return None

    @property
    def exit_code(self):
        return self._exit_code

    def _now(self):
        return self.get_clock().now()

    def _elapsed(self, since) -> float:
        return (self._now() - since).nanoseconds / 1e9

    def _finish(self, code: int, message: str, level: str = 'info'):
        if self._exit_code is not None:
            return
        self._stop()
        self._cancel_restage()
        self._restore_collision_thresholds()
        self._exit_code = code
        log = self.get_logger().info if level == 'info' else self.get_logger().error
        log(message)

    def _info_cb(self, msg: CameraInfo):
        if self._camera_matrix is None:
            self._camera_matrix = np.array(msg.k, dtype=np.float32).reshape(3, 3)
            self._dist_coeffs = np.array(msg.d, dtype=np.float32)

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self._pose = (p.x, p.y)
        self._speed = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def _battery_cb(self, msg: BatteryState):
        self._charging = (
            msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
            or (msg.current is not None and msg.current > 0.05)
        )

    def _clear_attempt_state(self):
        self._filt = None
        self._meas = None
        self._settle_start = None
        self._charge_start = None
        self._prev_bearing = None
        self._last_seen = None
        self._attempt_start = self._now()

    def _begin_retry(self, reason: str):
        if self._retries >= self._max_retries:
            self._finish(
                1,
                f'{reason} — exhausted {self._max_retries} retries.',
                level='error')
            return
        self._retries += 1
        self._stop()
        self._clear_attempt_state()
        # Each attempt sweeps one continuous direction (see _search) — alternate
        # the starting direction per retry so attempts build up coverage instead
        # of repeatedly re-checking the same arc.
        self._search_dir *= -1.0

        use_restage = (
            self._prefer_restage
            and self._staging is not None
            and self._retries % 2 == 1  # 1st, 3rd… retry → restage; even → soft reverse
        )
        if use_restage:
            self.get_logger().warning(
                f'{reason} — restage retry {self._retries}/{self._max_retries}')
            self._start_restage()
        else:
            self.get_logger().warning(
                f'{reason} — soft retry {self._retries}/{self._max_retries} '
                f'(reverse then re-search)')
            self._mode = 'reverse'
            self._reverse_start = self._now()
            self._reverse_start_pose = self._pose

    def _cancel_restage(self):
        handle = self._restage_handle
        self._restage_send_future = None
        self._restage_result_future = None
        self._restage_handle = None
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass

    def _start_restage(self):
        self._mode = 'restage'
        self._restage_start = self._now()
        self._restage_send_future = None
        self._restage_result_future = None
        self._restage_handle = None
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warning('Nav2 unavailable for restage — soft reverse instead')
            self._mode = 'reverse'
            self._reverse_start = self._now()
            self._reverse_start_pose = self._pose
            return

        x, y, yaw = self._staging
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self._restage_send_future = self._nav_client.send_goal_async(goal)
        self.get_logger().info(
            f'Restaging to ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°) …')

    def _run_restage(self):
        if self._restage_start and self._elapsed(self._restage_start) > RESTAGE_TIMEOUT:
            self._cancel_restage()
            self.get_logger().warning('Restage timed out — soft reverse')
            self._mode = 'reverse'
            self._reverse_start = self._now()
            self._reverse_start_pose = self._pose
            return

        if self._restage_send_future is not None and self._restage_send_future.done():
            try:
                handle = self._restage_send_future.result()
            except Exception as e:
                self.get_logger().warning(f'Restage goal failed: {e}')
                self._restage_send_future = None
                self._mode = 'reverse'
                self._reverse_start = self._now()
                self._reverse_start_pose = self._pose
                return
            self._restage_send_future = None
            if handle is None or not handle.accepted:
                self.get_logger().warning('Restage goal rejected — soft reverse')
                self._mode = 'reverse'
                self._reverse_start = self._now()
                self._reverse_start_pose = self._pose
                return
            self._restage_handle = handle
            self._restage_result_future = handle.get_result_async()

        if self._restage_result_future is not None and self._restage_result_future.done():
            try:
                result = self._restage_result_future.result()
                ok = result is not None and result.status == GoalStatus.STATUS_SUCCEEDED
            except Exception:
                ok = False
            self._restage_result_future = None
            self._restage_handle = None
            if ok:
                self.get_logger().info('Restage complete — searching for marker')
            else:
                self.get_logger().warning('Restage did not succeed — searching anyway')
            self._mode = 'search'
            self._attempt_start = self._now()

    def _control_tick(self):
        if self._exit_code is not None or not self._servo_enabled:
            return

        if self._elapsed(self._start_time) > self._timeout:
            self._finish(1, 'Timed out before docking.', level='error')
            return

        if self._mode == 'restage':
            self._run_restage()
            return

        if self._mode == 'reverse':
            self._run_reverse()
            return

        if self._mode == 'charge_wait':
            self._run_charge_wait()
            return

        if self._last_seen is None:
            if self._elapsed(self._attempt_start) > SEARCH_GIVE_UP:
                self._begin_retry('Marker not found during search')
                return
            if self._elapsed(self._attempt_start) > SEARCH_GRACE:
                self._mode = 'search'
                self._search()
            return

        since_seen = self._elapsed(self._last_seen)
        if since_seen > LOST_TIMEOUT:
            self._begin_retry('Marker lost')
            return
        if since_seen > STALL_TIMEOUT:
            self._stop()
            return

        if self._meas is None:
            return

        if self._mode == 'settle':
            # settle handled inside _servo when still aligned; fall through
            pass

        self._mode = 'approach' if self._mode != 'settle' else 'settle'
        lateral, distance, yaw = self._meas
        self._servo(lateral, distance, yaw)

    def _run_charge_wait(self):
        if self._charging:
            self._finish(
                0,
                'Docked and charging confirmed'
                + (f' after {self._retries} retries' if self._retries else '')
                + '.')
            return
        if self._charge_start is None:
            self._charge_start = self._now()
        if self._elapsed(self._charge_start) > self._charge_timeout:
            self._begin_retry('Charge not detected')
            return
        self._stop()
        if self._last_log is None or self._elapsed(self._last_log) >= LOG_PERIOD:
            self._last_log = self._now()
            self.get_logger().info('Waiting for charge confirmation …')

    def _run_reverse(self):
        twist = Twist()
        twist.linear.x = -RETRY_REVERSE_SPEED
        self._cmd_pub.publish(twist)

        moved = 0.0
        if self._pose is not None and self._reverse_start_pose is not None:
            dx = self._pose[0] - self._reverse_start_pose[0]
            dy = self._pose[1] - self._reverse_start_pose[1]
            moved = math.hypot(dx, dy)

        timed_out = (
            self._reverse_start is not None
            and self._elapsed(self._reverse_start) > RETRY_REVERSE_TIMEOUT
        )
        if moved >= RETRY_REVERSE_DIST or timed_out:
            self._stop()
            self._mode = 'search'
            self.get_logger().info('Retry reverse done — searching for marker …')

    def _search(self):
        # One continuous rotation for the whole attempt (up to SEARCH_GIVE_UP),
        # not a back-and-forth wiggle — at SEARCH_ANGULAR that covers ~200°+ of
        # arc per attempt. _begin_retry flips the direction for the next attempt
        # so consecutive attempts build up coverage instead of re-checking the
        # same narrow band forever.
        now = self._now()
        twist = Twist()
        twist.angular.z = SEARCH_ANGULAR * self._search_dir
        self._cmd_pub.publish(twist)

        if self._last_log is not None and self._elapsed(self._last_log) < LOG_PERIOD:
            return
        self._last_log = now
        pos_str = f'({self._pose[0]:.2f}, {self._pose[1]:.2f})' if self._pose else '(?, ?)'
        self.get_logger().info(
            f'pos={pos_str}  searching for marker {self._marker_id} '
            f'(dir={"+/-"[self._search_dir < 0]}) …')

    def _image_cb(self, msg: Image):
        if self._exit_code is not None or self._camera_matrix is None:
            return
        if self._mode in ('reverse', 'restage', 'charge_wait'):
            return

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        corners, ids, _ = self._detector.detectMarkers(frame)
        found = ids is not None and self._marker_id in ids.flatten()

        if not found:
            self._write_debug_image(frame, corners, ids)
            return

        idx = list(ids.flatten()).index(self._marker_id)
        ok, rvec, tvec = cv2.solvePnP(
            self._obj_points, corners[idx][0], self._camera_matrix, self._dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            self._write_debug_image(frame, corners, ids)
            return

        projected, _ = cv2.projectPoints(
            self._obj_points, rvec, tvec, self._camera_matrix, self._dist_coeffs)
        err = float(np.linalg.norm(projected.reshape(-1, 2) - corners[idx][0], axis=1).mean())
        if err > REPROJ_MAX_PX:
            self._write_debug_image(frame, corners, ids)
            return

        lateral = float(tvec[0])
        distance = float(tvec[2])
        yaw = self._marker_yaw(rvec)
        filtered = self._ema(lateral, distance, yaw)
        if filtered is None:
            self._write_debug_image(frame, corners, ids)
            return

        self._meas = filtered
        self._publish_dock_pose(msg.header.stamp, rvec, tvec)
        self._last_seen = self._now()
        if self._mode == 'search':
            self._mode = 'approach'
            self._stop()
            fl0, fd0, fy0 = filtered
            self.get_logger().info(
                f'Marker {self._marker_id} acquired — dist={fd0:.2f}m '
                f'bearing={math.degrees(math.atan2(fl0, fd0)):.1f}° '
                f'yaw={math.degrees(fy0):.1f}° — approaching …')
        fl, fd, _fy = filtered
        self._write_debug_image(
            frame, corners, ids, distance=fd, bearing=math.atan2(fl, fd))

    def _write_debug_image(self, frame, corners, ids, distance=None, bearing=None):
        if not self._debug_image_path:
            return
        now = self._now()
        if self._last_debug_write is not None and \
                self._elapsed(self._last_debug_write) < DEBUG_IMAGE_PERIOD:
            return
        self._last_debug_write = now

        view = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(view, corners, ids)
        label = 'searching …' if distance is None else \
            f'dist={distance:.2f}m bearing={math.degrees(bearing):.1f}deg'
        if self._retries:
            label = f'retry {self._retries}/{self._max_retries} | {label}'
        cv2.putText(view, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imwrite(self._debug_image_path, view)

    def _publish_dock_pose(self, stamp, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self._optical_frame
        pose.pose.position.x = float(tvec[0])
        pose.pose.position.y = float(tvec[1])
        pose.pose.position.z = float(tvec[2])
        pose.pose.orientation = _rot_to_quat(R)
        self._pose_pub.publish(pose)

    @staticmethod
    def _marker_yaw(rvec) -> float:
        R, _ = cv2.Rodrigues(rvec)
        nx, _, nz = R[:, 2]
        return math.atan2(float(nx), float(-nz))

    def _ema(self, lateral: float, distance: float, yaw: float):
        if self._filt is None:
            self._filt = (lateral, distance, yaw)
            return self._filt

        fl, fd, fy = self._filt
        if (
            abs(lateral - fl) > JUMP_LATERAL
            or abs(distance - fd) > JUMP_DIST
            or abs(yaw - fy) > JUMP_YAW
        ):
            return None

        a = FILTER_ALPHA_NEAR if distance < NEAR_DIST else FILTER_ALPHA
        self._filt = (
            a * lateral + (1.0 - a) * fl,
            a * distance + (1.0 - a) * fd,
            a * yaw + (1.0 - a) * fy,
        )
        return self._filt

    def _on_pose_success(self, distance, bearing, yaw):
        if self._require_charge:
            self._mode = 'charge_wait'
            self._charge_start = self._now()
            self.get_logger().info('Pose aligned — waiting for charge …')
            return
        self._finish(
            0,
            f'Docked — {distance:.2f} m, bearing {math.degrees(bearing):.1f}°, '
            f'yaw {math.degrees(yaw):.1f}°'
            + (f' after {self._retries} retries' if self._retries else '')
            + '.')

    def _servo(self, lateral: float, distance: float, yaw: float):
        bearing = math.atan2(lateral, distance)
        dist_error = distance - self._target_distance
        self._log_telemetry(distance, bearing, yaw)

        aligned = (
            abs(bearing) <= ANGLE_OK
            and abs(yaw) <= YAW_OK
            and abs(dist_error) <= DIST_TOLERANCE
        )
        if aligned:
            self._stop()
            self._mode = 'settle'
            now = self._now()
            if self._settle_start is None:
                self._settle_start = now
            elif self._elapsed(self._settle_start) >= SETTLE_TIME:
                self._on_pose_success(distance, bearing, yaw)
            return
        self._settle_start = None
        self._mode = 'approach'

        d_bearing = 0.0
        if self._prev_bearing is not None:
            d_bearing = bearing - self._prev_bearing
        self._prev_bearing = bearing

        # Bearing (aim at marker) and yaw (square to its face) can demand opposite
        # rotation when the robot is off-axis. A continuous blend of the two can
        # still hit a false equilibrium at whatever distance makes the (weighted)
        # terms coincidentally cancel — that happened in practice, twice, at two
        # different distances. Use a hard gate instead of a ramp: yaw only enters
        # the sum once bearing is already small, which bounds its cancelling
        # contribution to at most KP_BEARING*BEARING_GATE — never enough to fully
        # offset a yaw correction that's actually still large.
        yaw_active = distance <= YAW_BLEND_DIST and abs(bearing) <= BEARING_GATE
        yaw_term = KP_YAW * yaw if yaw_active else 0.0

        ang = -KP_BEARING * bearing - yaw_term - KD_BEARING * d_bearing
        twist = Twist()
        twist.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, ang))

        heading_err = abs(bearing) + (abs(yaw) if yaw_active else 0.0)
        slow_down = max(0.0, 1.0 - heading_err / math.radians(25))
        lin = KP_LINEAR * dist_error * slow_down
        twist.linear.x = max(-MAX_REVERSE, min(MAX_LINEAR, lin))
        self._cmd_pub.publish(twist)

    def _stop(self):
        self._cmd_pub.publish(Twist())

    def _log_telemetry(self, distance: float, bearing: float, yaw: float):
        now = self._now()
        if self._last_log is not None and self._elapsed(self._last_log) < LOG_PERIOD:
            return
        self._last_log = now

        pos_str = f'({self._pose[0]:.2f}, {self._pose[1]:.2f})' if self._pose else '(?, ?)'
        lin, ang = self._speed if self._speed else (0.0, 0.0)
        self.get_logger().info(
            f'pos={pos_str}  lin_vel={lin:.2f} m/s  ang_vel={ang:.2f} rad/s  '
            f'marker: dist={distance:.2f} m  bearing={math.degrees(bearing):.1f}°  '
            f'yaw={math.degrees(yaw):.1f}°  attempt={self._retries + 1}/{self._max_retries + 1}')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDock()
    exit_code = 1
    try:
        while rclpy.ok() and node.exit_code is None:
            rclpy.spin_once(node, timeout_sec=0.05)
        exit_code = 0 if node.exit_code is None else node.exit_code
        # Let the restore-thresholds service call (fired from _finish) actually
        # reach collision_monitor before the node — and its client — disappear.
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        node._stop()
        node._cancel_restage()
        node._restore_collision_thresholds()
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
