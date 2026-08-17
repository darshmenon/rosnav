#!/usr/bin/env python3
"""
obstacle_tracker.py — Moving obstacle detection + tracking from LaserScans.

Compares range values between the current scan and a scan N frames ago.
Rays that have shortened faster than `min_speed` m/s indicate an approaching
object.  Detected points are clustered, transformed to the map frame via TF,
and fed into a constant-velocity Kalman filter tracker that assigns each
cluster a persistent track ID and estimates its velocity across frames.
Each cluster's point spread is also fit to an ellipse (extended-object
tracking) so tracks carry a size/orientation estimate, not just a centroid.
Tracks are published as RViz markers and a JSON state topic.

Algorithm
─────────
  1. Keep a rolling buffer of the last `history_len` LaserScan messages.
  2. On each new scan, compare current range[i] with range[i] from
     `lookback` frames ago.
  3. A ray is "closing" if  Δrange / Δtime  <  -min_speed  (range shrinking).
  4. Convert closing ray endpoints from robot frame → map frame via TF.
  5. Cluster nearby points within `cluster_radius` metres (single-linkage),
     then fit an ellipse (semi-axes + orientation) to each cluster's point
     covariance to estimate its extent.
  6. Predict all existing tracks forward (constant-velocity Kalman filter),
     greedily associate clusters to tracks by nearest centroid within
     `track_gate_dist`, update matched tracks (position/velocity via Kalman
     filter, extent via exponential smoothing), spawn new tracks for
     unmatched clusters, and age out tracks with too many consecutive
     misses (`track_max_misses`).
  7. Publish tracks with >= `track_min_hits` matched updates:
       /obstacle_tracker/markers  — MarkerArray (RViz extent-sized/oriented
                                    cylinders + velocity arrows + id/speed
                                    labels)
       /obstacle_tracker/state    — std_msgs/String JSON per-track summary
                                    (id, x, y, vx, vy, speed, points,
                                    length, width, orientation)

Parameters
──────────
  robot_ns          namespace prefix           (default: '')
  min_speed         m/s closing threshold       (default: 0.08)
  history_len       scan buffer size            (default: 10)
  lookback          frames to compare           (default: 5)
  cluster_radius    grouping distance m         (default: 0.4)
  marker_lifetime   seconds to show marker      (default: 0.5)
  base_frame        robot frame                 (default: base_link)
  map_frame         world frame                 (default: map)
  track_gate_dist   max association distance m  (default: 0.75)
  track_max_misses  consecutive misses to drop  (default: 4)
  track_min_hits    matches before publishing   (default: 2)
  process_noise     accel noise std (m/s^2)     (default: 0.5)
  measurement_noise centroid position std (m)   (default: 0.2)
  min_extent        min ellipse diameter m      (default: 0.3)
  extent_gain       extent smoothing gain 0-1   (default: 0.3)

Usage
─────
  ros2 run rosnav_bot obstacle_tracker.py
  ros2 run rosnav_bot obstacle_tracker.py --ros-args \\
      -p robot_ns:=robot1 -p min_speed:=0.05
  ros2 topic echo /obstacle_tracker/state
"""

import json
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros
from geometry_msgs.msg import Point, TransformStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class Track:
    """Constant-velocity Kalman filter track: state = [x, y, vx, vy]."""

    _H = np.array([[1.0, 0.0, 0.0, 0.0],
                   [0.0, 1.0, 0.0, 0.0]])

    def __init__(self, track_id: int, x: float, y: float,
                 l1: float = 0.3, l2: float = 0.3, alpha: float = 0.0):
        self.id = track_id
        self.x = np.array([x, y, 0.0, 0.0])
        self.P = np.diag([0.25, 0.25, 1.0, 1.0])
        self.hits = 1
        self.misses = 0
        self.count = 1
        # Extent: ellipse fit to the cluster's point spread (major/minor
        # axis lengths + orientation), smoothed independently of the
        # Kalman-filtered position/velocity state.
        self.l1 = l1
        self.l2 = l2
        self.alpha = alpha

    def update_extent(self, l1: float, l2: float, alpha: float, gain: float):
        # Ellipse orientation is only defined mod pi (the major axis looks
        # the same rotated 180°), so resolve that ambiguity before blending
        # to avoid the smoothed angle snapping back and forth.
        diff = (alpha - self.alpha + math.pi / 2) % math.pi - math.pi / 2
        self.alpha = (self.alpha + gain * diff + math.pi / 2) % math.pi - math.pi / 2
        self.l1 = (1 - gain) * self.l1 + gain * l1
        self.l2 = (1 - gain) * self.l2 + gain * l2

    def predict(self, dt: float, process_noise: float):
        if dt <= 0.0:
            return
        F = np.array([[1.0, 0.0, dt,  0.0],
                      [0.0, 1.0, 0.0, dt ],
                      [0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0]])
        q = process_noise ** 2
        Q = q * np.array([
            [dt**3 / 3, 0.0,       dt**2 / 2, 0.0],
            [0.0,       dt**3 / 3, 0.0,       dt**2 / 2],
            [dt**2 / 2, 0.0,       dt,        0.0],
            [0.0,       dt**2 / 2, 0.0,       dt],
        ])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, zx: float, zy: float, measurement_noise: float, count: int):
        z = np.array([zx, zy])
        R = (measurement_noise ** 2) * np.eye(2)
        y = z - self._H @ self.x
        S = self._H @ self.P @ self._H.T + R
        K = self.P @ self._H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self._H) @ self.P
        self.hits += 1
        self.misses = 0
        self.count = count

    @property
    def pos(self):
        return float(self.x[0]), float(self.x[1])

    @property
    def vel(self):
        return float(self.x[2]), float(self.x[3])


class ObstacleTracker(Node):
    def __init__(self):
        super().__init__('obstacle_tracker')

        self.declare_parameter('robot_ns',           '')
        self.declare_parameter('min_speed',           0.08)
        self.declare_parameter('history_len',         10)
        self.declare_parameter('lookback',            5)
        self.declare_parameter('cluster_radius',      0.4)
        self.declare_parameter('marker_lifetime',     0.5)
        self.declare_parameter('base_frame',          'base_link')
        self.declare_parameter('map_frame',            'map')
        self.declare_parameter('track_gate_dist',      0.75)
        self.declare_parameter('track_max_misses',     4)
        self.declare_parameter('track_min_hits',       2)
        self.declare_parameter('process_noise',        0.5)
        self.declare_parameter('measurement_noise',    0.2)
        self.declare_parameter('min_extent',           0.3)
        self.declare_parameter('extent_gain',          0.3)

        ns                    = self.get_parameter('robot_ns').value
        self._min_speed       = self.get_parameter('min_speed').value
        history_len           = self.get_parameter('history_len').value
        self._lookback        = self.get_parameter('lookback').value
        self._cluster_r       = self.get_parameter('cluster_radius').value
        self._marker_life     = self.get_parameter('marker_lifetime').value
        self._base_frame      = self.get_parameter('base_frame').value
        self._map_frame       = self.get_parameter('map_frame').value
        self._gate_dist        = self.get_parameter('track_gate_dist').value
        self._max_misses       = self.get_parameter('track_max_misses').value
        self._min_hits         = self.get_parameter('track_min_hits').value
        self._process_noise     = self.get_parameter('process_noise').value
        self._measurement_noise = self.get_parameter('measurement_noise').value
        self._min_extent        = self.get_parameter('min_extent').value
        self._extent_gain       = self.get_parameter('extent_gain').value

        if ns:
            self._base_frame = f'{ns}/{self._base_frame}'

        pre = f'/{ns}' if ns else ''

        self._buf: deque = deque(maxlen=history_len)
        self._tracks: list[Track] = []
        self._next_id = 1
        self._last_track_time = None

        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self)

        self._marker_pub = self.create_publisher(
            MarkerArray, f'{pre}/obstacle_tracker/markers', 10)
        self._state_pub  = self.create_publisher(
            String, f'{pre}/obstacle_tracker/state', 10)

        self.create_subscription(LaserScan, f'{pre}/scan', self._scan_cb, 10)

        self.get_logger().info(
            f'ObstacleTracker  ns={ns or "/"}  '
            f'min_speed={self._min_speed} m/s  lookback={self._lookback} frames  '
            f'track_gate={self._gate_dist} m  min_hits={self._min_hits}')

    # ── Scan callback ─────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan):
        self._buf.append(msg)

        if len(self._buf) < self._lookback + 1:
            return

        prev = self._buf[-(self._lookback + 1)]
        curr = msg

        dt = (
            (curr.header.stamp.sec - prev.header.stamp.sec)
            + (curr.header.stamp.nanosec - prev.header.stamp.nanosec) * 1e-9
        )
        if dt <= 0:
            return

        # Collect closing-ray endpoints in robot (base_link) frame
        robot_pts: list[tuple[float, float]] = []
        n = min(len(curr.ranges), len(prev.ranges))

        for i in range(n):
            r_now  = curr.ranges[i]
            r_prev = prev.ranges[i]

            if not (curr.range_min < r_now < curr.range_max):
                continue
            if not (prev.range_min < r_prev < prev.range_max):
                continue

            closing_speed = (r_prev - r_now) / dt   # positive = approaching
            if closing_speed <= self._min_speed:
                continue

            angle = curr.angle_min + i * curr.angle_increment
            robot_pts.append((r_now * math.cos(angle), r_now * math.sin(angle)))

        clusters: list[dict] = []
        if robot_pts:
            try:
                tf: TransformStamped = self._tf_buf.lookup_transform(
                    self._map_frame, self._base_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1))
            except Exception:
                return

            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            q  = tf.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            cos_y, sin_y = math.cos(yaw), math.sin(yaw)

            map_pts = [
                (tx + cos_y * rx - sin_y * ry,
                 ty + sin_y * rx + cos_y * ry)
                for rx, ry in robot_pts
            ]

            clusters = self._cluster(map_pts)

        tracks = self._update_tracks(clusters, curr.header.stamp)
        self._publish(tracks, curr.header.stamp)

    # ── Single-linkage clustering ─────────────────────────────────────────────

    def _cluster(self, pts: list[tuple[float, float]]) -> list[dict]:
        if not pts:
            return []

        assigned = [-1] * len(pts)
        cid      = 0

        for i in range(len(pts)):
            if assigned[i] >= 0:
                continue
            queue = [i]
            assigned[i] = cid
            while queue:
                cur = queue.pop()
                for j in range(len(pts)):
                    if assigned[j] >= 0:
                        continue
                    if math.hypot(pts[cur][0] - pts[j][0], pts[cur][1] - pts[j][1]) < self._cluster_r:
                        assigned[j] = cid
                        queue.append(j)
            cid += 1

        clusters = []
        for c in range(cid):
            members = [pts[i] for i in range(len(pts)) if assigned[i] == c]
            cx = sum(p[0] for p in members) / len(members)
            cy = sum(p[1] for p in members) / len(members)
            l1, l2, alpha = self._fit_extent(members, cx, cy)
            clusters.append({'x': cx, 'y': cy, 'count': len(members),
                              'l1': l1, 'l2': l2, 'alpha': alpha})

        return clusters

    def _fit_extent(self, members: list[tuple[float, float]], cx: float, cy: float):
        """Fit an ellipse (major/minor axis lengths + orientation) to a
        cluster's point spread — extended-object tracking's 'extent'."""
        n = len(members)
        if n < 2:
            return self._min_extent, self._min_extent, 0.0

        cxx = sum((p[0] - cx) ** 2 for p in members) / n
        cyy = sum((p[1] - cy) ** 2 for p in members) / n
        cxy = sum((p[0] - cx) * (p[1] - cy) for p in members) / n

        trace = cxx + cyy
        disc = max(trace * trace / 4.0 - (cxx * cyy - cxy * cxy), 0.0)
        half = math.sqrt(disc)
        lam_major = trace / 2.0 + half
        lam_minor = trace / 2.0 - half

        alpha = 0.5 * math.atan2(2.0 * cxy, cxx - cyy) if (cxx != cyy or cxy != 0.0) else 0.0
        l1 = max(2.0 * math.sqrt(max(lam_major, 0.0)), self._min_extent)
        l2 = max(2.0 * math.sqrt(max(lam_minor, 0.0)), self._min_extent)
        return l1, l2, alpha

    # ── Kalman-filter tracking ───────────────────────────────────────────────

    def _update_tracks(self, clusters: list[dict], stamp) -> list[Track]:
        now = stamp.sec + stamp.nanosec * 1e-9
        dt = 0.0 if self._last_track_time is None else now - self._last_track_time
        self._last_track_time = now

        for t in self._tracks:
            t.predict(dt, self._process_noise)

        # Greedy nearest-neighbor association, gated by track_gate_dist.
        pairs = []
        for ti, t in enumerate(self._tracks):
            tx, ty = t.pos
            for ci, c in enumerate(clusters):
                d = math.hypot(tx - c['x'], ty - c['y'])
                if d <= self._gate_dist:
                    pairs.append((d, ti, ci))
        pairs.sort(key=lambda p: p[0])

        matched_tracks: set = set()
        matched_clusters: set = set()
        for _, ti, ci in pairs:
            if ti in matched_tracks or ci in matched_clusters:
                continue
            matched_tracks.add(ti)
            matched_clusters.add(ci)
            c = clusters[ci]
            self._tracks[ti].update(c['x'], c['y'], self._measurement_noise, c['count'])
            self._tracks[ti].update_extent(c['l1'], c['l2'], c['alpha'], self._extent_gain)

        for ti, t in enumerate(self._tracks):
            if ti not in matched_tracks:
                t.misses += 1

        for ci, c in enumerate(clusters):
            if ci not in matched_clusters:
                nt = Track(self._next_id, c['x'], c['y'], c['l1'], c['l2'], c['alpha'])
                nt.count = c['count']
                self._tracks.append(nt)
                self._next_id += 1

        self._tracks = [t for t in self._tracks if t.misses <= self._max_misses]

        return [t for t in self._tracks if t.hits >= self._min_hits]

    # ── Publish ───────────────────────────────────────────────────────────────

    def _publish(self, tracks: list[Track], stamp):
        markers = MarkerArray()

        # Delete all old markers first
        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        del_marker.header.frame_id = self._map_frame
        del_marker.header.stamp    = stamp
        markers.markers.append(del_marker)

        for t in tracks:
            x, y = t.pos
            vx, vy = t.vel
            speed = math.hypot(vx, vy)

            # Extended-object extent: a cylinder sized/oriented to the
            # tracked ellipse, rather than a fixed-size sphere.
            extent = Marker()
            extent.header.frame_id = self._map_frame
            extent.header.stamp    = stamp
            extent.ns              = 'moving_obstacles'
            extent.id               = t.id
            extent.type             = Marker.CYLINDER
            extent.action           = Marker.ADD
            extent.pose.position.x = x
            extent.pose.position.y = y
            extent.pose.position.z = 0.3
            extent.pose.orientation.z = math.sin(t.alpha / 2.0)
            extent.pose.orientation.w = math.cos(t.alpha / 2.0)
            extent.scale.x = t.l1
            extent.scale.y = t.l2
            extent.scale.z = 0.5
            extent.color.r = 1.0
            extent.color.g = 0.2
            extent.color.b = 0.0
            extent.color.a = 0.85
            extent.lifetime.sec     = int(self._marker_life)
            extent.lifetime.nanosec = int((self._marker_life % 1) * 1e9)
            markers.markers.append(extent)

            if speed > 0.03:
                arrow = Marker()
                arrow.header.frame_id = self._map_frame
                arrow.header.stamp    = stamp
                arrow.ns              = 'obstacle_velocity'
                arrow.id               = t.id
                arrow.type             = Marker.ARROW
                arrow.action           = Marker.ADD
                arrow.points = [
                    Point(x=x, y=y, z=0.3),
                    Point(x=x + vx, y=y + vy, z=0.3),
                ]
                arrow.scale.x = 0.08
                arrow.scale.y = 0.16
                arrow.color.r = 1.0
                arrow.color.g = 0.9
                arrow.color.b = 0.0
                arrow.color.a = 0.9
                arrow.lifetime.sec     = int(self._marker_life)
                arrow.lifetime.nanosec = int((self._marker_life % 1) * 1e9)
                markers.markers.append(arrow)

            label = Marker()
            label.header.frame_id = self._map_frame
            label.header.stamp    = stamp
            label.ns              = 'obstacle_labels'
            label.id               = t.id
            label.type             = Marker.TEXT_VIEW_FACING
            label.action           = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.7
            label.pose.orientation.w = 1.0
            label.scale.z = 0.25
            label.color.r = label.color.g = label.color.b = 1.0
            label.color.a = 0.9
            label.text = f'#{t.id} {speed:.2f}m/s'
            label.lifetime.sec     = int(self._marker_life)
            label.lifetime.nanosec = int((self._marker_life % 1) * 1e9)
            markers.markers.append(label)

        self._marker_pub.publish(markers)

        state_msg = String()
        state_msg.data = json.dumps({
            'moving_obstacles': [
                {
                    'id': t.id,
                    'x': round(t.pos[0], 2),
                    'y': round(t.pos[1], 2),
                    'vx': round(t.vel[0], 2),
                    'vy': round(t.vel[1], 2),
                    'speed': round(math.hypot(*t.vel), 2),
                    'points': t.count,
                    'length': round(t.l1, 2),
                    'width': round(t.l2, 2),
                    'orientation': round(t.alpha, 3),
                }
                for t in tracks
            ]
        })
        self._state_pub.publish(state_msg)

        if tracks:
            self.get_logger().info(
                f'Moving obstacles: {len(tracks)} track(s) — '
                + ', '.join(
                    f'#{t.id}({t.pos[0]:.2f},{t.pos[1]:.2f} @{math.hypot(*t.vel):.2f}m/s)'
                    for t in tracks))


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
