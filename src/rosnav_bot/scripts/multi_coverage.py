#!/usr/bin/env python3
"""
multi_coverage.py — Multi-robot coverage coordinator.

Splits the map's free-space bounding box into N equal stripes (one per robot)
and dispatches a boustrophedon coverage path to each robot's FollowWaypoints
action server.

Usage
─────
  # Auto-discover robots from /*/cmd_vel topics
  ros2 run rosnav_bot multi_coverage.py

  # Explicit robot list
  ros2 run rosnav_bot multi_coverage.py --ros-args -p robots:=robot1,robot2,robot3

  # Split into vertical stripes instead of horizontal
  ros2 run rosnav_bot multi_coverage.py --ros-args -p axis:=x

Parameters
──────────
  robots          comma-sep namespace list; '' = auto-discover  (default: '')
  sweep_spacing   metres between scan lines                      (default: 0.5)
  robot_radius    clearance from walls in metres                 (default: 0.25)
  map_topic       OccupancyGrid topic                            (default: /map)
  axis            split axis: 'x' = vertical stripes,
                              'y' = horizontal stripes           (default: 'x')
"""

import math
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import FollowWaypoints

try:
    from scipy.ndimage import binary_erosion as _scipy_erosion
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


# ── Helpers ───────────────────────────────────────────────────────────────────

def _make_pose(x: float, y: float, yaw: float, stamp, frame: str) -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = frame
    p.header.stamp    = stamp
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def _erode(free: np.ndarray, r_cells: int) -> np.ndarray:
    """Return eroded free-space mask using scipy or a numpy fallback."""
    struct = np.ones((2 * r_cells + 1, 2 * r_cells + 1), dtype=bool)
    if HAS_SCIPY:
        return _scipy_erosion(free, structure=struct)
    from numpy.lib.stride_tricks import sliding_window_view
    s   = 2 * r_cells + 1
    pad = r_cells
    padded_free = np.pad(free, pad, constant_values=False)
    windows = sliding_window_view(padded_free, (s, s))
    return windows.all(axis=(-2, -1))


def _boustrophedon(
    padded: np.ndarray,
    free: np.ndarray,
    r_min: int, r_max: int,
    c_min: int, c_max: int,
    spacing_cells: int,
    ox: float, oy: float, res: float,
) -> list[tuple[float, float, float]]:
    """Compute boustrophedon waypoints within the given cell bounding box."""
    waypoints: list[tuple[float, float, float]] = []
    row           = r_min
    left_to_right = True

    # Decide which mask to sweep (padded may have been replaced by free on fallback)
    use_padded = padded is not None

    while row <= r_max:
        if use_padded:
            line_cols = [c for c in range(c_min, c_max + 1) if padded[row, c]]
        else:
            line_cols = [c for c in range(c_min, c_max + 1) if free[row, c]]

        if not left_to_right:
            line_cols = list(reversed(line_cols))

        if line_cols:
            prev_c = None
            for c in line_cols:
                if prev_c is None or abs(c - prev_c) >= spacing_cells:
                    wx  = ox + (c + 0.5) * res
                    wy  = oy + (row + 0.5) * res
                    yaw = 0.0 if left_to_right else math.pi
                    waypoints.append((wx, wy, yaw))
                    prev_c = c

        left_to_right = not left_to_right
        row += spacing_cells

    return waypoints


def _compute_region_waypoints(
    msg: OccupancyGrid,
    robot_radius: float,
    sweep_spacing: float,
    x_min: float, x_max: float,
    y_min: float, y_max: float,
) -> list[tuple[float, float, float]]:
    """
    Compute boustrophedon coverage waypoints for a rectangular region
    [x_min, x_max] x [y_min, y_max] inside the given OccupancyGrid.
    """
    info      = msg.info
    res       = info.resolution
    w, h      = info.width, info.height
    ox        = info.origin.position.x
    oy        = info.origin.position.y

    grid = np.array(msg.data, dtype=np.int8).reshape((h, w))
    free = (grid == 0)

    # Erode for robot clearance
    r_cells = max(1, int(math.ceil(robot_radius / res)))
    padded  = _erode(free, r_cells)

    # Full bounding box from navigable cells
    rows, cols = np.where(padded)
    if len(rows) == 0:
        rows, cols = np.where(free)
        padded = None   # signal to use free instead of padded
    if len(rows) == 0:
        return []

    r_min, r_max = int(rows.min()), int(rows.max())
    c_min, c_max = int(cols.min()), int(cols.max())

    # Clip to assigned region
    c_min = max(c_min, int((x_min - ox) / res))
    c_max = min(c_max, int((x_max - ox) / res))
    r_min = max(r_min, int((y_min - oy) / res))
    r_max = min(r_max, int((y_max - oy) / res))

    if c_min >= c_max or r_min >= r_max:
        return []

    spacing_cells = max(1, int(round(sweep_spacing / res)))
    return _boustrophedon(
        padded, free, r_min, r_max, c_min, c_max,
        spacing_cells, ox, oy, res,
    )


# ── Main node ─────────────────────────────────────────────────────────────────

class MultiCoverage(Node):
    def __init__(self):
        super().__init__('multi_coverage')

        self.declare_parameter('robots',        '')
        self.declare_parameter('sweep_spacing', 0.5)
        self.declare_parameter('robot_radius',  0.25)
        self.declare_parameter('map_topic',     '/map')
        self.declare_parameter('axis',          'x')

        robots_param     = self.get_parameter('robots').value
        self._spacing    = self.get_parameter('sweep_spacing').value
        self._radius     = self.get_parameter('robot_radius').value
        map_topic        = self.get_parameter('map_topic').value
        self._axis       = self.get_parameter('axis').value.strip().lower()

        if self._axis not in ('x', 'y'):
            self.get_logger().warn(
                f"axis='{self._axis}' is not 'x' or 'y'; defaulting to 'x'.")
            self._axis = 'x'

        # Robot list: explicit parameter or auto-discovered
        if robots_param.strip():
            self._robots = [r.strip() for r in robots_param.split(',') if r.strip()]
            self.get_logger().info(
                f'Using explicit robot list: {self._robots}')
        else:
            self._robots = self._discover_robots()

        if not self._robots:
            self.get_logger().error(
                'No robots found. Pass robots:=robot1,robot2 or start /*/cmd_vel topics.')
            return

        self.get_logger().info(
            f'MultiCoverage  robots={self._robots}  axis={self._axis}  '
            f'spacing={self._spacing}m  radius={self._radius}m  '
            f'waiting for map on {map_topic} …')

        # Transient-local QoS to catch maps already latched by map_server / SLAM
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._map: OccupancyGrid | None = None
        self.create_subscription(OccupancyGrid, map_topic, self._map_cb, map_qos)

        # Pre-create one action client per robot
        self._clients: dict[str, ActionClient] = {
            ns: ActionClient(self, FollowWaypoints, f'/{ns}/follow_waypoints')
            for ns in self._robots
        }

        # Pending result futures so the node stays alive
        self._pending: set[str] = set()

        # Arm a 30-second map timeout
        self._map_deadline = self.get_clock().now().nanoseconds + 30 * 10**9
        self.create_timer(1.0, self._timeout_check)

    # ── Robot discovery ───────────────────────────────────────────────────────

    def _discover_robots(self) -> list[str]:
        """Find namespaces by scanning /*/cmd_vel topics."""
        # Brief spin to populate topic list
        rclpy.spin_once(self, timeout_sec=2.0)
        topics = self.get_topic_names_and_types()
        robots = sorted(
            t.split('/')[1]
            for t, _ in topics
            if t.count('/') >= 2 and t.endswith('/cmd_vel')
        )
        self.get_logger().info(
            f'Auto-discovered robots: {robots}' if robots
            else 'No /*/cmd_vel topics found yet.')
        return robots

    # ── Map callback ──────────────────────────────────────────────────────────

    def _map_cb(self, msg: OccupancyGrid):
        if self._map is not None:
            return
        self._map = msg
        self.get_logger().info(
            f'Map received: {msg.info.width}×{msg.info.height} '
            f'res={msg.info.resolution:.3f}m')
        self._coordinate()

    def _timeout_check(self):
        if self._map is not None:
            return
        remaining = (self._map_deadline - self.get_clock().now().nanoseconds) / 1e9
        if remaining <= 0:
            self.get_logger().error(
                'Timed out waiting for map (30 s). Shutting down.')
            rclpy.shutdown()
        elif int(remaining) % 5 == 0:
            self.get_logger().info(f'Still waiting for map … ({remaining:.0f}s left)')

    # ── Coordination ──────────────────────────────────────────────────────────

    def _coordinate(self):
        msg   = self._map
        info  = msg.info
        res   = info.resolution
        w, h  = info.width, info.height
        ox    = info.origin.position.x
        oy    = info.origin.position.y

        # Free-space world bounding box
        grid = np.array(msg.data, dtype=np.int8).reshape((h, w))
        free = (grid == 0)
        rows, cols = np.where(free)
        if len(rows) == 0:
            self.get_logger().error('Map has no free cells.')
            return

        world_x_min = ox + int(cols.min()) * res
        world_x_max = ox + (int(cols.max()) + 1) * res
        world_y_min = oy + int(rows.min()) * res
        world_y_max = oy + (int(rows.max()) + 1) * res

        n = len(self._robots)
        self.get_logger().info(
            f'Free-space bbox: x=[{world_x_min:.2f},{world_x_max:.2f}] '
            f'y=[{world_y_min:.2f},{world_y_max:.2f}]  '
            f'splitting into {n} stripe(s) along axis={self._axis}')

        # Compute stripes
        regions = []
        if self._axis == 'x':
            # Vertical stripes: divide along x
            x_step = (world_x_max - world_x_min) / n
            for i in range(n):
                regions.append((
                    world_x_min + i * x_step,
                    world_x_min + (i + 1) * x_step,
                    world_y_min,
                    world_y_max,
                ))
        else:
            # Horizontal stripes: divide along y
            y_step = (world_y_max - world_y_min) / n
            for i in range(n):
                regions.append((
                    world_x_min,
                    world_x_max,
                    world_y_min + i * y_step,
                    world_y_min + (i + 1) * y_step,
                ))

        # Dispatch each robot
        for ns, (rx_min, rx_max, ry_min, ry_max) in zip(self._robots, regions):
            self.get_logger().info(
                f'[{ns}] region x=[{rx_min:.2f},{rx_max:.2f}] '
                f'y=[{ry_min:.2f},{ry_max:.2f}]')

            waypoints = _compute_region_waypoints(
                msg, self._radius, self._spacing,
                rx_min, rx_max, ry_min, ry_max,
            )

            if not waypoints:
                self.get_logger().warn(f'[{ns}] No waypoints in assigned region — skipping.')
                continue

            self.get_logger().info(
                f'[{ns}] {len(waypoints)} waypoints computed. '
                'Waiting for FollowWaypoints server …')

            client = self._clients[ns]
            if not client.wait_for_server(timeout_sec=15.0):
                self.get_logger().error(
                    f'[{ns}] FollowWaypoints server not available — skipping.')
                continue

            now  = self.get_clock().now().to_msg()
            goal = FollowWaypoints.Goal()
            goal.poses = [_make_pose(x, y, yaw, now, 'map') for x, y, yaw in waypoints]

            self.get_logger().info(f'[{ns}] Sending {len(goal.poses)} poses …')
            self._pending.add(ns)

            future = client.send_goal_async(
                goal,
                feedback_callback=lambda fb, robot_ns=ns: self._feedback_cb(fb, robot_ns),
            )
            future.add_done_callback(
                lambda f, robot_ns=ns: self._goal_resp_cb(f, robot_ns))

    # ── Action callbacks ──────────────────────────────────────────────────────

    def _feedback_cb(self, fb, ns: str):
        idx = fb.feedback.current_waypoint
        self.get_logger().info(f'[{ns}] waypoint {idx + 1} reached.')

    def _goal_resp_cb(self, future, ns: str):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error(f'[{ns}] Goal rejected.')
            self._pending.discard(ns)
            self._check_done()
            return
        self.get_logger().info(f'[{ns}] Goal accepted.')
        handle.get_result_async().add_done_callback(
            lambda f, robot_ns=ns: self._result_cb(f, robot_ns))

    def _result_cb(self, future, ns: str):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            missed = list(result.result.missed_waypoints)
            if missed:
                self.get_logger().warn(
                    f'[{ns}] Coverage done. Missed waypoints: {missed}')
            else:
                self.get_logger().info(f'[{ns}] Coverage complete — all waypoints reached.')
        else:
            self.get_logger().error(f'[{ns}] Coverage failed. Status: {result.status}')

        self._pending.discard(ns)
        self._check_done()

    def _check_done(self):
        if not self._pending:
            self.get_logger().info('All robots finished. Shutting down.')
            rclpy.shutdown()


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = MultiCoverage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
