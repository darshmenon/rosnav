#!/usr/bin/env python3
"""
orb_slam3_occupancy_bridge.py — Turn ORB-SLAM3's map->odom TF + the RGB-D
depth image into a real nav_msgs/OccupancyGrid on /map, so slam_algo:=orbslam3
plugs into Nav2/the frontier explorer the same way every other slam_algo does.

Why this exists: ORB-SLAM3 (docker/orb_slam3/, run as a separate sidecar
container — see its README for why it can't join this workspace) is a sparse
feature-based tracker. It publishes map->odom TF and a sparse landmark point
cloud, not an occupancy grid — unlike slam_toolbox/Cartographer/RTAB-Map,
which all build one internally. This node closes that gap the same way
RTAB-Map's Grid/Sensor=1 does for slam_algo:=vslam: project the depth image
through the current map->base_link transform, ray-trace free space, mark
occupied endpoints. It only depends on ORB-SLAM3's TF output (map->odom),
not on the sidecar being in this ROS graph in any other way — so it degrades
gracefully (keeps the last map, logs a warning) if that container isn't
running yet or drops out.

Height-filtered the same way pointcloud_to_scan.py is (obstacle_layer, not
voxel_layer — see concepts.md's "2D Lidar" notes): points between
min_height/max_height above base_link count as obstacles, everything else is
ignored (ceiling, floor noise).

Usage (started automatically by slam_nav.launch.py slam_algo:=orbslam3):
  ros2 run rosnav_bot orb_slam3_occupancy_bridge.py --ros-args -p use_sim_time:=true
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros

UNKNOWN = -1
FREE = 0
OCCUPIED = 100


class LogOddsGrid:
    """Growable log-odds occupancy grid, integrated in bulk with numpy.

    Replaces the old per-point Python Bresenham + hard overwrite: a cell now needs
    repeated hits to become occupied and repeated misses to clear, so one noisy
    depth return neither sticks forever nor blocks free space. 0.0 = never seen.
    """

    def __init__(self, resolution: float, size_m: float,
                 l_hit: float = 0.85, l_miss: float = -0.4,
                 l_min: float = -4.0, l_max: float = 4.0,
                 occ_thresh: float = 1.0, free_thresh: float = -0.3):
        self.res = resolution
        n = int(math.ceil(size_m / resolution))
        self.l = np.zeros((n, n), dtype=np.float32)
        self.ox = None
        self.oy = None
        self.l_hit, self.l_miss = l_hit, l_miss
        self.l_min, self.l_max = l_min, l_max
        self.occ_thresh, self.free_thresh = occ_thresh, free_thresh

    def center_on(self, x: float, y: float) -> None:
        if self.ox is None:
            self.ox = x - self.l.shape[1] * self.res / 2.0
            self.oy = y - self.l.shape[0] * self.res / 2.0

    def ensure_bounds(self, xs: np.ndarray, ys: np.ndarray) -> None:
        if self.ox is None or xs.size == 0:
            return
        col = np.floor((xs - self.ox) / self.res).astype(np.int64)
        row = np.floor((ys - self.oy) / self.res).astype(np.int64)
        pad_l = max(0, -int(col.min()))
        pad_r = max(0, int(col.max()) - (self.l.shape[1] - 1))
        pad_b = max(0, -int(row.min()))
        pad_t = max(0, int(row.max()) - (self.l.shape[0] - 1))
        if pad_l or pad_r or pad_b or pad_t:
            self.l = np.pad(self.l, ((pad_b, pad_t), (pad_l, pad_r)), mode='constant')
            self.ox -= pad_l * self.res
            self.oy -= pad_b * self.res

    def _cells(self, xs, ys):
        col = np.floor((xs - self.ox) / self.res).astype(np.int64)
        row = np.floor((ys - self.oy) / self.res).astype(np.int64)
        return row, col

    def integrate(self, sx: float, sy: float,
                  free_xy: np.ndarray, occ_xy: np.ndarray) -> None:
        """Sensor at (sx, sy). free_xy: Nx2 ray endpoints that only clear space (floor
        hits); occ_xy: Mx2 endpoints that also mark the end cell occupied."""
        ends = np.vstack([free_xy, occ_xy]) if len(free_xy) + len(occ_xy) else None
        if ends is None:
            return
        self.ensure_bounds(np.append(ends[:, 0], sx), np.append(ends[:, 1], sy))
        d = np.hypot(ends[:, 0] - sx, ends[:, 1] - sy)
        n_steps = max(2, int(math.ceil(d.max() / self.res)))
        t = np.linspace(0.0, 1.0, n_steps, endpoint=False, dtype=np.float32)
        # every ray sampled at the same n_steps: cheap, and cells are de-duplicated
        # per frame below so over-sampling short rays does not double count.
        rx = sx + (ends[:, 0:1] - sx) * t[None, :]
        ry = sy + (ends[:, 1:2] - sy) * t[None, :]
        r, c = self._cells(rx.ravel(), ry.ravel())
        h, w = self.l.shape
        ok = (r >= 0) & (r < h) & (c >= 0) & (c < w)
        free_idx = np.unique(r[ok] * w + c[ok])
        if len(occ_xy):
            orr, occ = self._cells(occ_xy[:, 0], occ_xy[:, 1])
            ook = (orr >= 0) & (orr < h) & (occ >= 0) & (occ < w)
            occ_idx = np.unique(orr[ook] * w + occ[ook])
        else:
            occ_idx = np.empty(0, dtype=np.int64)
        free_idx = np.setdiff1d(free_idx, occ_idx, assume_unique=True)
        flat = self.l.reshape(-1)
        flat[free_idx] = np.clip(flat[free_idx] + self.l_miss, self.l_min, self.l_max)
        flat[occ_idx] = np.clip(flat[occ_idx] + self.l_hit, self.l_min, self.l_max)

    def to_occupancy(self) -> np.ndarray:
        out = np.full(self.l.shape, UNKNOWN, dtype=np.int8)
        out[self.l <= self.free_thresh] = FREE
        out[self.l >= self.occ_thresh] = OCCUPIED
        return out


def fit_ground_plane(pts_base: np.ndarray, near: float = 0.4, far: float = 3.0,
                     max_tilt_deg: float = 15.0, bin_m: float = 0.3):
    """Robust ground plane z = a*x + b*y + c in the base frame, or None.

    Takes the lowest point of each bin_m cell in front of the robot, then does a
    few trimmed least-squares passes so walls/obstacles do not drag the plane up.
    Heights measured from it make the floor read ~0 on a slope AND when the robot
    itself pitches on flat ground (a plain base-frame height gets one of the two wrong).
    """
    m = (pts_base[:, 0] > near) & (pts_base[:, 0] < far) & (np.abs(pts_base[:, 1]) < far)
    p = pts_base[m]
    if len(p) < 50:
        return None
    key = (np.floor(p[:, 0] / bin_m).astype(np.int64) * 100003
           + np.floor(p[:, 1] / bin_m).astype(np.int64))
    order = np.lexsort((p[:, 2], key))
    first = np.r_[True, key[order][1:] != key[order][:-1]]
    low = p[order][first]
    if len(low) < 8:
        return None
    sel = np.ones(len(low), bool)
    for _ in range(3):
        A = np.c_[low[sel, 0], low[sel, 1], np.ones(sel.sum())]
        coef, *_ = np.linalg.lstsq(A, low[sel, 2], rcond=None)
        res = low[:, 2] - (low[:, 0] * coef[0] + low[:, 1] * coef[1] + coef[2])
        sel = res < max(0.05, 1.5 * float(np.std(res[sel])))
        if sel.sum() < 6:
            return None
    if math.degrees(math.atan(math.hypot(coef[0], coef[1]))) > max_tilt_deg:
        return None
    return float(coef[0]), float(coef[1]), float(coef[2])


def split_by_height(pts_base: np.ndarray, min_h: float, max_h: float, plane=None):
    """pts_base: Nx3 in the (tilt-aware) base frame. Returns (floor_mask, band_mask).
    Below min_h = floor (clears space only); min_h..max_h = obstacle; above = ignored."""
    h = pts_base[:, 2]
    if plane is not None:
        h = h - (plane[0] * pts_base[:, 0] + plane[1] * pts_base[:, 1] + plane[2])
    return h <= min_h, (h > min_h) & (h < max_h)


class OrbSlam3OccupancyBridge(Node):
    def __init__(self):
        super().__init__('orb_slam3_occupancy_bridge')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('resolution', 0.05)     # matches other backends' Grid/CellSize
        self.declare_parameter('initial_size_m', 20.0)  # grid starts this wide, grows on demand
        self.declare_parameter('min_height', 0.05)      # base_link frame, matches pointcloud_to_scan.py
        self.declare_parameter('max_height', 0.55)
        self.declare_parameter('max_range', 8.0)        # matches rtabmap_vslam_node's Grid/RangeMax
        self.declare_parameter('depth_stride', 6)        # subsample the depth image for speed
        self.declare_parameter('publish_period_sec', 0.5)
        self.declare_parameter('tf_timeout_sec', 0.2)
        self.declare_parameter('max_tf_wait_sec', 1.0)   # drop a depth frame if map->odom never reaches its stamp
        self.declare_parameter('queue_len', 5)
        self.declare_parameter('fit_ground_plane', True)  # heights from a per-frame ground fit

        self._resolution = float(self.get_parameter('resolution').value)
        self._min_h = float(self.get_parameter('min_height').value)
        self._max_h = float(self.get_parameter('max_height').value)
        self._max_wait = float(self.get_parameter('max_tf_wait_sec').value)
        self._queue_len = int(self.get_parameter('queue_len').value)
        self._fit_ground = bool(self.get_parameter('fit_ground_plane').value)
        self._max_range = float(self.get_parameter('max_range').value)
        self._stride = max(1, int(self.get_parameter('depth_stride').value))
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._tf_timeout = Duration(
            seconds=float(self.get_parameter('tf_timeout_sec').value))

        size_m = float(self.get_parameter('initial_size_m').value)
        # grid is centered on wherever the robot is when the first usable TF/depth
        # pair arrives, and grows on demand.
        self._grid = LogOddsGrid(self._resolution, size_m)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._camera_info = None
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._on_camera_info, 5)

        latched_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._map_pub = self.create_publisher(OccupancyGrid, '/map', latched_qos)

        self.create_subscription(
            Image, self.get_parameter('depth_topic').value, self._on_depth, 5)

        period = float(self.get_parameter('publish_period_sec').value)
        self._publish_timer = self.create_timer(period, self._publish_map)
        self._pending = []
        self._dropped = 0
        self._dropped_logged = 0
        self._last_tf_err = ''
        self._tf_seen = False
        self._frames = 0
        self._drain_timer = self.create_timer(0.05, self._drain)

        self.get_logger().info(
            f'orb_slam3_occupancy_bridge: {self.get_parameter("depth_topic").value} '
            f'+ {self._map_frame}->{self._base_frame} TF -> /map '
            f'(res={self._resolution}m, height=[{self._min_h}, {self._max_h}]m). '
            'Waiting for the ORB-SLAM3 sidecar to publish map->odom TF '
            '(docker compose up orb_slam3, or docker/orb_slam3/build_bare_metal.sh) — '
            '/map stays unknown until then.')

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _on_depth(self, msg: Image) -> None:
        """Queue the frame; _drain() integrates it once map->odom has caught up.

        ORB-SLAM3 stamps map->odom with the RGB frame time but publishes it ~100-150 ms
        later (it tracks at 6-9 fps). Looking the transform up the instant the depth
        image arrives therefore lands in the TF buffer's future -> ExtrapolationException
        on nearly every frame -> /map stays empty. Wait for it instead of dropping.
        """
        self._pending.append(msg)

    def _drain(self) -> None:
        keep = []
        now = self.get_clock().now()
        for msg in self._pending:
            res = self._integrate(msg)
            if res == 'wait':
                age = (now - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
                if age < self._max_wait:
                    keep.append(msg)
                else:
                    self._dropped += 1
        self._pending = keep[-self._queue_len:]
        if self._dropped and self._dropped != self._dropped_logged:
            self.get_logger().warn(
                f'{self._dropped} depth frames dropped so far: {self._map_frame}->'
                f'{self._base_frame} TF never reached their stamp within {self._max_wait}s '
                f'({self._frames} integrated). Last error: {self._last_tf_err}',
                throttle_duration_sec=10.0)
            self._dropped_logged = self._dropped

    def _integrate(self, msg: Image) -> str:
        """'ok' integrated, 'wait' TF not there yet (retry), 'skip' unusable."""
        if self._camera_info is None:
            return 'wait'
        stamp = Time.from_msg(msg.header.stamp)
        try:
            tf_map_base = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, stamp)
            tf_map_cam = self._tf_buffer.lookup_transform(
                self._map_frame, msg.header.frame_id, stamp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self._last_tf_err = f'{type(e).__name__}: {str(e)[:120]}'
            if not self._tf_seen:
                self.get_logger().warn(
                    f'{self._map_frame}->{self._base_frame} TF not available yet — is the '
                    'ORB-SLAM3 sidecar running? /map stays unknown until it publishes. '
                    f'({self._last_tf_err})', throttle_duration_sec=10.0)
            return 'wait'
        if not self._tf_seen:
            self._tf_seen = True
            self.get_logger().info(f'{self._map_frame}->{self._base_frame} TF available: '
                                   'integrating depth frames')

        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        fx, fy = self._camera_info.k[0], self._camera_info.k[4]
        cx, cy = self._camera_info.k[2], self._camera_info.k[5]

        vs, us = np.mgrid[0:msg.height:self._stride, 0:msg.width:self._stride]
        z = depth[vs, us]
        valid = np.isfinite(z) & (z > 0.05) & (z < self._max_range)
        if not np.any(valid):
            return 'skip'
        us, vs, z = us[valid].astype(np.float32), vs[valid].astype(np.float32), z[valid]

        # Camera-optical-frame point cloud (Z forward, X right, Y down —
        # matches camera.xacro's camera_optical_joint convention).
        x_cam = (us - cx) * z / fx
        y_cam = (vs - cy) * z / fy
        z_cam = z

        cq = tf_map_cam.transform.rotation
        R = _quat_to_matrix(cq.x, cq.y, cq.z, cq.w)
        t = tf_map_cam.transform.translation
        pts_cam = np.stack([x_cam, y_cam, z_cam], axis=1)
        pts_map = pts_cam @ R.T + np.array([t.x, t.y, t.z])

        # Heights in the *base frame*, not map z minus base z: the band then follows
        # the robot's roll/pitch, so a sloped floor stays floor instead of leaking
        # into the obstacle band.
        bq = tf_map_base.transform.rotation
        bt = tf_map_base.transform.translation
        Rb = _quat_to_matrix(bq.x, bq.y, bq.z, bq.w)
        pts_base = (pts_map - np.array([bt.x, bt.y, bt.z])) @ Rb
        plane = fit_ground_plane(pts_base) if self._fit_ground else None
        floor_m, band_m = split_by_height(pts_base, self._min_h, self._max_h, plane)
        if not (np.any(floor_m) or np.any(band_m)):
            return 'skip'

        self._grid.center_on(t.x, t.y)
        self._grid.integrate(t.x, t.y, pts_map[floor_m, :2], pts_map[band_m, :2])
        self._frames += 1
        if self._frames % 40 == 1:
            self.get_logger().info(
                f'integrated frame {self._frames}: floor={int(floor_m.sum())} '
                f'obstacle={int(band_m.sum())} pts, ground_plane={"fit" if plane else "none"}, grid={self._grid.l.shape}')
        return 'ok'

    def _publish_map(self) -> None:
        if self._grid.ox is None:
            return
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        info = MapMetaData()
        info.resolution = self._resolution
        info.width = self._grid.l.shape[1]
        info.height = self._grid.l.shape[0]
        info.origin.position.x = self._grid.ox
        info.origin.position.y = self._grid.oy
        info.origin.position.z = 0.0
        info.origin.orientation.w = 1.0
        msg.info = info
        msg.data = self._grid.to_occupancy().flatten().tolist()
        self._map_pub.publish(msg)


def _quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    rclpy.init()
    node = OrbSlam3OccupancyBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
