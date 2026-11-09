#!/usr/bin/env python3
"""map_timelapse - turn a running exploration into a GIF without touching the screen.

Subscribes to /map and the robots' odometry, renders a frame every --period seconds
(map, robot trails, explored %, elapsed time) and writes a GIF when it stops.
Pure topic data -> matplotlib/imageio, so it works headless and never captures the desktop.

  ros2 run rosnav_bot map_timelapse.py --robots robot1 robot2 robot3 --duration 300 --out fleet.gif
  ros2 run rosnav_bot map_timelapse.py --duration 200 --out single.gif      # one un-namespaced robot

Robot poses are taken from <ns>/odom; with a SLAM map->odom offset of ~0 (start of a run)
that is the map frame. Trails use /map-frame odom as-is (good enough for a progress view).
"""
import argparse
import time
import types

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

COLORS = ['#e6194b', '#3cb44b', '#4363d8', '#f58231', '#911eb4', '#46f0f0']


def pad_frames(frames, fill=255):
    """Pad every frame (bottom/right) to the largest height/width: cropped frames differ in size."""
    H = max(f.shape[0] for f in frames)
    W = max(f.shape[1] for f in frames)
    out = []
    for f in frames:
        if f.shape[:2] != (H, W):
            canvas = np.full((H, W, 3), fill, dtype=np.uint8)
            canvas[:f.shape[0], :f.shape[1]] = f
            f = canvas
        out.append(f)
    return out


def render(grid, info, trails, elapsed, scale=3):
    """RGB uint8 image of the map with trails; unknown dark grey, free white, occupied black."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    # crop to the known area (+ margin and any robot positions) so early frames are not mostly empty grey
    res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
    known = np.argwhere(grid != -1)
    pts = [p for t in trails.values() for p in t]
    if len(known):
        r0, c0 = known.min(axis=0)
        r1, c1 = known.max(axis=0)
        for x, y in pts:
            r1, c1 = max(r1, int((y - oy) / res)), max(c1, int((x - ox) / res))
            r0, c0 = min(r0, int((y - oy) / res)), min(c0, int((x - ox) / res))
        m = int(1.0 / res)
        r0, c0 = max(0, r0 - m), max(0, c0 - m)
        r1, c1 = min(grid.shape[0] - 1, r1 + m), min(grid.shape[1] - 1, c1 + m)
        grid = grid[r0:r1 + 1, c0:c1 + 1]
        oy, ox = oy + r0 * res, ox + c0 * res
        info = types.SimpleNamespace(resolution=res, origin=types.SimpleNamespace(
            position=types.SimpleNamespace(x=ox, y=oy)))
    h, w = grid.shape
    img = np.full((h, w), 0.35)
    img[grid == 0] = 1.0
    img[grid > 50] = 0.0
    free = int((grid == 0).sum())
    occ = int((grid > 50).sum())
    fig = plt.figure(figsize=(max(w * scale / 100, 3.0), max(h * scale / 100, 2.0) + 0.5), dpi=100)
    ax = fig.add_axes([0, 0.5 / (max(h * scale / 100, 2.0) + 0.5), 1, max(h * scale / 100, 2.0) / (max(h * scale / 100, 2.0) + 0.5)])
    ax.imshow(img, cmap='gray', vmin=0, vmax=1, origin='lower',
              extent=(info.origin.position.x, info.origin.position.x + w * info.resolution,
                      info.origin.position.y, info.origin.position.y + h * info.resolution))
    for i, (ns, pts) in enumerate(sorted(trails.items())):
        if not pts:
            continue
        xs, ys = zip(*pts)
        c = COLORS[i % len(COLORS)]
        ax.plot(xs, ys, '-', color=c, lw=1.2, alpha=0.8)
        ax.plot(xs[-1], ys[-1], 'o', color=c, ms=7, mec='white', label=ns or 'robot')
    if len(trails) > 1 or any(trails):
        ax.legend(loc='lower right', fontsize=8, framealpha=0.7)
    ax.axis('off')
    area = free * info.resolution ** 2
    fig.text(0.01, 0.01, f't={elapsed:5.0f}s   free {area:6.1f} m2   occupied {occ} cells',
             fontsize=11, color='black', va='bottom')
    fig.canvas.draw()
    frame = np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()
    plt.close(fig)
    return frame


class Timelapse(Node):
    def __init__(self, robots, period):
        super().__init__('map_timelapse')
        self.grid = None
        self.info = None
        self.trails = {ns: [] for ns in robots}
        self.create_subscription(
            OccupancyGrid, '/map', self._on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        for ns in robots:
            topic = f'/{ns}/odom' if ns else '/odom'
            self.create_subscription(Odometry, topic, lambda m, ns=ns: self._on_odom(ns, m), qos_profile_sensor_data)

    def _on_map(self, m):
        self.info = m.info
        self.grid = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)

    def _on_odom(self, ns, m):
        p = (m.pose.pose.position.x, m.pose.pose.position.y)
        t = self.trails[ns]
        if not t or (p[0] - t[-1][0]) ** 2 + (p[1] - t[-1][1]) ** 2 > 0.0025:
            t.append(p)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--robots', nargs='*', default=[''], help='robot namespaces (default: one un-namespaced robot)')
    ap.add_argument('--period', type=float, default=4.0, help='seconds between frames')
    ap.add_argument('--duration', type=float, default=120.0, help='total recording seconds')
    ap.add_argument('--out', default='map_timelapse.gif')
    ap.add_argument('--fps', type=float, default=6.0, help='GIF playback frames per second')
    args = ap.parse_args()

    import imageio.v2 as imageio
    rclpy.init()
    node = Timelapse(args.robots, args.period)
    frames, t0, next_t = [], time.time(), 0.0
    while time.time() - t0 < args.duration and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        el = time.time() - t0
        if el >= next_t and node.grid is not None:
            frames.append(render(node.grid, node.info, node.trails, el))
            next_t += args.period
            node.get_logger().info(f'frame {len(frames)} at {el:.0f}s')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if not frames:
        print('no /map received - nothing to write')
        return 1
    # hold the last frame a little longer so the final map is readable
    frames = pad_frames(frames)
    frames += [frames[-1]] * int(args.fps * 2)
    imageio.mimsave(args.out, frames, duration=1.0 / args.fps, loop=0)
    print(f'wrote {args.out} ({len(frames)} frames)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
