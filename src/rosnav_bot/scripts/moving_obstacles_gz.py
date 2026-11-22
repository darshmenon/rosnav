#!/usr/bin/env python3
"""
moving_obstacles_gz.py — move kinematic obstacle models in a running gz-sim world through the /world/<name>/set_pose
service (same idea as the quadruped project's moving_obstacle_gz.py; gz <actor>s are invisible to the lidar, and
this does not depend on the TrajectoryFollower plugin).

The models must exist in the world as non-static, gravity-free bodies.
  worlds/dynamic_obstacles.world  dyn_obstacle_1..3  (cylinders, z=0.5, no yaw)
  worlds/crowd.world              person_1..6        (people, feet at z=0, face the path)

Each path is a closed loop of waypoints driven at constant speed.

  python3 moving_obstacles_gz.py --world dynamic_obstacles
  python3 moving_obstacles_gz.py --world crowd --speed 0.7
Set GZ_PARTITION (and ROS_DOMAIN_ID) the same as the simulation's, e.g. cb100 for run2.sh domain 100.
"""
import argparse
import json
import math
import subprocess
import time

# name -> (z, face_direction, waypoints)
SCENES = {
    'dynamic_obstacles': {
        'dyn_obstacle_1': (0.5, False, [(3.0, -3.0), (3.0, 3.0)]),
        'dyn_obstacle_2': (0.5, False, [(-3.0, 3.0), (3.0, 3.0), (3.0, -3.0), (-3.0, -3.0)]),
        'dyn_obstacle_3': (0.5, False, [(-5.0, 4.0), (5.0, 4.0)]),
    },
    # Aisles left open in worlds/crowd.world. Start phases are staggered in main().
    'crowd': {
        'person_1': (0.0, True, [(-7.0, -5.2), (5.5, -5.2)]),             # south aisle
        'person_2': (0.0, True, [(-8.0, 0.0), (5.5, 0.0)]),                # center aisle
        'person_3': (0.0, True, [(7.2, -5.2), (7.2, 4.5)]),                # east aisle
        'person_4': (0.0, True, [(-8.3, -4.0), (-8.3, 5.0)]),              # west aisle
        'person_5': (0.0, True, [(-7.4, 3.2), (-2.2, 3.2)]),               # between the tables
        'person_6': (0.0, True, [(3.4, 1.7), (5.8, 1.7), (5.8, 4.0), (3.4, 4.0)]),  # around the crate
    },
}


def set_pose(world, model, x, y, z, yaw):
    half = 0.5 * yaw
    req = (f'name: "{model}", position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}, '
           f'orientation: {{x: 0, y: 0, z: {math.sin(half):.5f}, w: {math.cos(half):.5f}}}')
    return subprocess.call(['gz', 'service', '-s', f'/world/{world}/set_pose', '--reqtype', 'gz.msgs.Pose',
                            '--reptype', 'gz.msgs.Boolean', '--timeout', '500', '--req', req],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def set_poses(world, poses):
    """All models in ONE service call (/world/<w>/set_pose_vector) so the whole crowd steps together instead of one
    gz subprocess per person (which updated each person only every few tenths of a second = visible jumping).
    poses: [(name, x, y, z, yaw)]. Returns 0 on success."""
    parts = []
    for name, x, y, z, yaw in poses:
        half = 0.5 * yaw
        parts.append(f'pose {{ name: "{name}", position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}}, '
                     f'orientation: {{x: 0, y: 0, z: {math.sin(half):.5f}, w: {math.cos(half):.5f}}} }}')
    return subprocess.call(['gz', 'service', '-s', f'/world/{world}/set_pose_vector', '--reqtype', 'gz.msgs.Pose_V',
                            '--reptype', 'gz.msgs.Boolean', '--timeout', '500', '--req', ' '.join(parts)],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def ease_yaw(prev, target, dt, rate=1.6):
    """Turn toward `target` at most `rate` rad/s (shortest way round) so people swing round corners, not snap."""
    if prev is None:
        return target
    err = (target - prev + math.pi) % (2 * math.pi) - math.pi
    step = max(-rate * dt, min(rate * dt, err))
    return prev + step


def position(path, dist):
    """Point and heading `dist` metres along the closed loop (the loop returns to its first point)."""
    pts = path + [path[0]]
    seg = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])]
    d = dist % sum(seg)
    for (a, b), L in zip(zip(pts, pts[1:]), seg):
        if d <= L or L == 0.0:
            f = d / L if L else 0.0
            yaw = math.atan2(b[1] - a[1], b[0] - a[0])
            return a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), yaw
        d -= L
    return pts[0][0], pts[0][1], 0.0


def make_gt_publisher():
    """Optional ROS side channel: person ground truth on /social_peds/gt (same JSON as social_force_peds.py), so
    decentral_gz_run.py can score robot-person distance. Returns publish(poses) or None if rclpy is not importable."""
    try:
        import rclpy
        from std_msgs.msg import String
        rclpy.init()
        node = rclpy.create_node('moving_obstacles_gt')
        pub = node.create_publisher(String, '/social_peds/gt', 10)
    except Exception as e:                     # no ROS environment: the mover still works
        print(f'info: /social_peds/gt disabled ({e})')
        return None

    def publish(poses, t):
        msg = String()
        msg.data = json.dumps({'t': t, 'peds': [dict(id=i + 1, x=round(x, 3), y=round(y, 3), vx=0.0, vy=0.0)
                                                for i, (_, x, y, _, _) in enumerate(poses)]})
        pub.publish(msg)
    return publish


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default='dynamic_obstacles')
    ap.add_argument('--speed', type=float, default=0.6, help='m/s along each path')
    ap.add_argument('--hz', type=float, default=10.0)
    ap.add_argument('--seconds', type=float, default=0.0, help='0 = run until killed')
    a = ap.parse_args()
    scene = SCENES.get(a.world)
    if scene is None:
        raise SystemExit(f'no paths for world {a.world!r}; known: {", ".join(SCENES)}')
    print(f'moving {list(scene)} in world={a.world} at {a.speed} m/s, {a.hz} Hz')
    t0 = time.time(); fails = 0
    # Stagger so six people are not at the same phase of their loops.
    phase = {name: i * 1.7 for i, name in enumerate(scene)}
    yaw_now, last, batch_ok = {}, time.time(), True
    publish_gt = make_gt_publisher()
    while a.seconds <= 0 or time.time() - t0 < a.seconds:
        now = time.time(); t = now - t0; dt = now - last; last = now
        poses = []
        for name, (z, face, path) in scene.items():
            x, y, yaw = position(path, a.speed * t + phase[name])
            yaw_now[name] = ease_yaw(yaw_now.get(name), yaw, dt) if face else 0.0
            poses.append((name, x, y, z, yaw_now[name]))
        if publish_gt:
            publish_gt(poses, now)
        if batch_ok and set_poses(a.world, poses) != 0:
            batch_ok = False                      # service missing in this gz version: per-model calls instead
            print('warn: set_pose_vector failed, falling back to one set_pose call per model')
        if not batch_ok:
            for name, x, y, z, yaw in poses:
                if set_pose(a.world, name, x, y, z, yaw) != 0:
                    fails += 1
        if fails and fails % 30 == 0:
            print(f'warn: {fails} failed set_pose calls (is the world "{a.world}" running on this GZ_PARTITION?)')
        time.sleep(max(0.0, 1.0 / a.hz - 0.03))


if __name__ == '__main__':
    main()
