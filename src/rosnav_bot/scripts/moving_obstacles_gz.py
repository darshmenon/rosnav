#!/usr/bin/env python3
"""
moving_obstacles_gz.py — move kinematic obstacle models in a running gz-sim world through the /world/<name>/set_pose
service (same idea as the quadruped project's moving_obstacle_gz.py; gz <actor>s are invisible to the lidar, and
this does not depend on the TrajectoryFollower plugin).

The models must exist in the world as non-static, gravity-free bodies (worlds/dynamic_obstacles.world has
dyn_obstacle_1..3). Each path is a closed loop of waypoints driven at constant speed.

  python3 moving_obstacles_gz.py --world dynamic_obstacles            # default: the three dyn_obstacle_* loops
  python3 moving_obstacles_gz.py --world dynamic_obstacles --speed 0.8 --hz 10
Set GZ_PARTITION (and ROS_DOMAIN_ID) the same as the simulation's, e.g. cb100 for run2.sh domain 100.
"""
import argparse
import math
import subprocess
import time

PATHS = {
    'dyn_obstacle_1': [(3.0, -3.0), (3.0, 3.0)],                                  # back and forth
    'dyn_obstacle_2': [(-3.0, 3.0), (3.0, 3.0), (3.0, -3.0), (-3.0, -3.0)],       # square loop
    'dyn_obstacle_3': [(-5.0, 4.0), (5.0, 4.0)],                                  # long line
}


def set_pose(world, model, x, y, z):
    req = f'name: "{model}", position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}, orientation: {{x: 0, y: 0, z: 0, w: 1}}'
    return subprocess.call(['gz', 'service', '-s', f'/world/{world}/set_pose', '--reqtype', 'gz.msgs.Pose',
                            '--reptype', 'gz.msgs.Boolean', '--timeout', '500', '--req', req],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def position(path, dist):
    """Point `dist` metres along the closed loop through the waypoints (the loop returns to its first point)."""
    pts = path + [path[0]]
    seg = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])]
    d = dist % sum(seg)
    for (a, b), L in zip(zip(pts, pts[1:]), seg):
        if d <= L:
            f = d / L if L else 0.0
            return a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])
        d -= L
    return pts[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default='dynamic_obstacles')
    ap.add_argument('--speed', type=float, default=0.5, help='m/s along each path')
    ap.add_argument('--z', type=float, default=0.5)
    ap.add_argument('--hz', type=float, default=10.0)
    ap.add_argument('--seconds', type=float, default=0.0, help='0 = run until killed')
    a = ap.parse_args()
    print(f'moving {list(PATHS)} in world={a.world} at {a.speed} m/s, {a.hz} Hz')
    t0 = time.time(); fails = 0
    while a.seconds <= 0 or time.time() - t0 < a.seconds:
        t = time.time() - t0
        for name, path in PATHS.items():
            x, y = position(path, a.speed * t)
            if set_pose(a.world, name, x, y, a.z) != 0:
                fails += 1
        if fails and fails % 30 == 0:
            print(f'warn: {fails} failed set_pose calls (is the world "{a.world}" running on this GZ_PARTITION?)')
        time.sleep(max(0.0, 1.0 / a.hz - 0.03))


if __name__ == '__main__':
    main()
