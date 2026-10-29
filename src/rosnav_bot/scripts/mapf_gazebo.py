#!/usr/bin/env python3
"""
mapf_gazebo.py — run a MAPF solution (see mapf_compare.py) on lightweight
diff-drive robots in Gazebo. No Nav2, no SLAM, no sensors: just a generated
world and a lock-step path follower, so 16+ robots stay cheap.

  gen     build world.sdf + plan.json for a random instance, solved by --algo
  follow  ROS 2 node: drive each /robotN along its plan, one grid step at a
          time, all robots released together (barrier) so the discrete
          time-indexed conflict-free guarantee still holds in continuous space

World: wall boxes, one flat tile per robot start and one tile + post per goal
(same colour as the robot). Robots are self-contained SDF models with the
DiffDrive + OdometryPublisher plugins (topics /robotN/cmd_vel, /robotN/ground_truth).

Normally started via:  ros2 launch rosnav_bot mapf_gazebo.launch.py
"""

import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

PALETTE = [  # matplotlib tab20 — same colours as mapf_compare.py plots
    '0.122 0.467 0.706', '0.682 0.780 0.910', '1.000 0.498 0.055', '1.000 0.733 0.471',
    '0.173 0.627 0.173', '0.596 0.875 0.541', '0.839 0.153 0.157', '1.000 0.596 0.588',
    '0.580 0.404 0.741', '0.773 0.690 0.835', '0.549 0.337 0.294', '0.769 0.612 0.580',
    '0.890 0.467 0.761', '0.969 0.714 0.824', '0.498 0.498 0.498', '0.780 0.780 0.780',
    '0.737 0.741 0.133', '0.859 0.859 0.553', '0.090 0.745 0.812', '0.620 0.855 0.898',
]


def log(msg):
    print(f'[mapf_gazebo] {msg}', flush=True)


# ── World generation ─────────────────────────────────────────────────────────

def robot_sdf(name, rgb, x, y, yaw):
    wr, ws = 0.075, 0.40   # wheel radius, wheel separation
    wheel = lambda side, yy: f"""
      <link name='{side}_wheel'>
        <pose>0 {yy} {wr} -1.5707 0 0</pose>
        <inertial><mass>0.5</mass><inertia><ixx>0.002</ixx><iyy>0.002</iyy><izz>0.003</izz></inertia></inertial>
        <collision name='c'><geometry><cylinder><radius>{wr}</radius><length>0.04</length></cylinder></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
        <visual name='v'><geometry><cylinder><radius>{wr}</radius><length>0.04</length></cylinder></geometry>
          <material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse></material></visual>
      </link>
      <joint name='{side}_wheel_joint' type='revolute'>
        <parent>chassis</parent><child>{side}_wheel</child>
        <axis><xyz>0 0 1</xyz></axis>
      </joint>"""
    caster = lambda nm, xx: (
        f"<collision name='{nm}_c'><pose>{xx} 0 {wr - 0.12} 0 0 0</pose>"
        f"<geometry><sphere><radius>{wr}</radius></sphere></geometry>"
        f"<surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface></collision>")
    return f"""
  <model name='{name}'>
    <pose>{x:.3f} {y:.3f} 0.02 0 0 {yaw:.4f}</pose>
    <link name='chassis'>
      <pose>0 0 0.12 0 0 0</pose>
      <inertial><mass>6</mass><inertia><ixx>0.08</ixx><iyy>0.15</iyy><izz>0.2</izz></inertia></inertial>
      <collision name='body_c'><geometry><box><size>0.50 0.30 0.10</size></box></geometry></collision>
      {caster('cf', 0.20)}{caster('cr', -0.20)}
      <visual name='body_v'><geometry><box><size>0.50 0.30 0.10</size></box></geometry>
        <material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material></visual>
      <visual name='nose_v'><pose>0.22 0 0.06 0 0 0</pose><geometry><box><size>0.08 0.12 0.04</size></box></geometry>
        <material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse></material></visual>
    </link>{wheel('left', ws / 2)}{wheel('right', -ws / 2)}
    <plugin filename='gz-sim-diff-drive-system' name='gz::sim::systems::DiffDrive'>
      <left_joint>left_wheel_joint</left_joint><right_joint>right_wheel_joint</right_joint>
      <wheel_separation>{ws}</wheel_separation><wheel_radius>{wr}</wheel_radius>
      <topic>{name}/cmd_vel</topic>
      <odom_topic>{name}/odom</odom_topic><tf_topic>{name}/tf_wheel_raw</tf_topic>
      <frame_id>{name}/odom</frame_id><child_frame_id>{name}/base_link</child_frame_id>
      <max_linear_acceleration>3</max_linear_acceleration>
      <max_angular_acceleration>6</max_angular_acceleration>
      <max_linear_velocity>1.5</max_linear_velocity><max_angular_velocity>3</max_angular_velocity>
    </plugin>
    <plugin filename='gz-sim-odometry-publisher-system' name='gz::sim::systems::OdometryPublisher'>
      <odom_topic>{name}/ground_truth</odom_topic>
      <odom_frame>world</odom_frame><robot_base_frame>{name}/base_link</robot_base_frame>
      <dimensions>2</dimensions><odom_publish_frequency>30</odom_publish_frequency>
      <tf_topic>{name}/tf_gt_raw</tf_topic>
    </plugin>
  </model>"""


def world_sdf(free, starts, goals, cell, to_xy):
    rows, cols = free.shape
    parts = []
    # walls: one static model, one visual+collision per blocked cell + border
    elems = []
    wall_rgb, wh = '0.35 0.37 0.42', 0.6
    for r in range(rows):
        for c in range(cols):
            if not free[r, c]:
                x, y = to_xy((r, c))
                elems.append((f'w{r}_{c}', (cell, cell, wh), f'{x} {y} {wh / 2} 0 0 0'))
    ex, ey = cols * cell, rows * cell
    for nm, sx, sy, px, py in (('bn', ex + 2 * cell, cell, 0, ey / 2 + cell / 2),
                               ('bs', ex + 2 * cell, cell, 0, -ey / 2 - cell / 2),
                               ('be', cell, ey, ex / 2 + cell / 2, 0),
                               ('bw', cell, ey, -ex / 2 - cell / 2, 0)):
        elems.append((nm, (sx, sy, wh), f'{px} {py} {wh / 2} 0 0 0'))
    walls = ''.join(
        f"<collision name='{n}_c'><pose>{p}</pose><geometry><box><size>{s[0]} {s[1]} {s[2]}</size></box></geometry></collision>"
        f"<visual name='{n}_v'><pose>{p}</pose><geometry><box><size>{s[0]} {s[1]} {s[2]}</size></box></geometry>"
        f"<material><ambient>{wall_rgb} 1</ambient><diffuse>{wall_rgb} 1</diffuse></material></visual>"
        for n, s, p in elems)
    parts.append(f"<model name='walls'><static>true</static><link name='link'>{walls}</link></model>")

    # start tiles (flat) + goal tiles with a post, coloured per robot
    markers = []
    t = cell * 0.9
    for i, (s, g) in enumerate(zip(starts, goals)):
        rgb = PALETTE[i % len(PALETTE)]
        sx, sy = to_xy(s)
        gx, gy = to_xy(g)
        markers.append(
            f"<visual name='s{i}'><pose>{sx} {sy} 0.004 0 0 0</pose><geometry><box><size>{t} {t} 0.008</size></box></geometry>"
            f"<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material></visual>"
            f"<visual name='g{i}'><pose>{gx} {gy} 0.004 0 0 0</pose><geometry><box><size>{t} {t} 0.008</size></box></geometry>"
            f"<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material></visual>"
            f"<visual name='gp{i}'><pose>{gx} {gy} 0.4 0 0 0</pose><geometry><cylinder><radius>0.1</radius><length>0.8</length></cylinder></geometry>"
            f"<material><ambient>{rgb} 0.8</ambient><diffuse>{rgb} 0.8</diffuse></material></visual>")
    parts.append(f"<model name='markers'><static>true</static><link name='link'>{''.join(markers)}</link></model>")

    for i, s in enumerate(starts):
        x, y = to_xy(s)
        parts.append(robot_sdf(f'robot{i + 1}', PALETTE[i % len(PALETTE)], x, y, 0.0))

    gsize = max(rows, cols) * cell * 2
    return f"""<?xml version="1.0" ?>
<sdf version='1.8'>
<world name='mapf_grid'>
  <physics name='4ms' type='ignored'><max_step_size>0.004</max_step_size><real_time_factor>1</real_time_factor></physics>
  <plugin filename='gz-sim-physics-system' name='gz::sim::systems::Physics'/>
  <plugin filename='gz-sim-user-commands-system' name='gz::sim::systems::UserCommands'/>
  <plugin filename='gz-sim-scene-broadcaster-system' name='gz::sim::systems::SceneBroadcaster'/>
  <light name='sun' type='directional'><pose>0 0 10 0 0 0</pose><cast_shadows>true</cast_shadows>
    <diffuse>0.8 0.8 0.8 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.4 0.2 -0.9</direction></light>
  <model name='ground_plane'><static>true</static><link name='link'>
    <collision name='c'><geometry><plane><normal>0 0 1</normal><size>{gsize} {gsize}</size></plane></geometry></collision>
    <visual name='v'><geometry><plane><normal>0 0 1</normal><size>{gsize} {gsize}</size></plane></geometry>
      <material><ambient>0.85 0.85 0.85 1</ambient><diffuse>0.85 0.85 0.85 1</diffuse></material></visual>
  </link></model>
  {''.join(parts)}
</world>
</sdf>
"""


def write_map(free, cell, stem, res):
    """Nav2 map_server map (pgm + yaml) matching the world: walls + 1-cell border occupied."""
    import numpy as np
    k = int(round(cell / res))
    big = np.kron(np.pad(free, 1, constant_values=False), np.ones((k, k), dtype=bool))
    img = np.where(big, 254, 0).astype(np.uint8)
    rows, cols = free.shape
    with open(stem + '.pgm', 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (img.shape[1], img.shape[0]))
        f.write(img.tobytes())
    with open(stem + '.yaml', 'w') as f:
        f.write(f"image: {os.path.basename(stem)}.pgm\nmode: trinary\nresolution: {res}\n"
                f"origin: [{-(cols / 2 + 1) * cell}, {-(rows / 2 + 1) * cell}, 0.0]\n"
                "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    log(f'wrote Nav2 map {stem}.yaml ({img.shape[1]}x{img.shape[0]} px @ {res} m)')


def cmd_gen(a):
    import numpy as np
    import mapf_compare as mc
    os.makedirs(a.out, exist_ok=True)
    free, starts, goals = mc.make_instance(a.size, a.agents, a.density, a.seed)
    cells = list(starts) + list(goals)
    assert len(set(cells)) == len(cells), 'a start/goal cell is shared by two robots'
    assert all(free[c] for c in cells), 'a start/goal cell sits on an obstacle'
    hs = [mc.bfs_dist(free, g) for g in goals]
    log(f'instance: {a.agents} robots, {a.size}x{a.size} grid, density {a.density}, seed {a.seed}')
    res = mc.run(a.algo, free, starts, goals, hs, a.timeout, seed=a.seed)
    log(mc.fmt(res))
    if res['paths'] is None or res['conflicts']:
        log(f'ERROR: {a.algo} produced no conflict-free plan')
        return 2
    rows, cols = free.shape
    to_xy = lambda rc: (round((rc[1] - (cols - 1) / 2) * a.cell, 3), round(((rows - 1) / 2 - rc[0]) * a.cell, 3))
    with open(os.path.join(a.out, 'world.sdf'), 'w') as f:
        f.write(world_sdf(free, starts, goals, a.cell, to_xy))
    plan = {'algo': a.algo, 'cell': a.cell, 'soc': res['soc'], 'makespan': res['makespan'],
            'paths': [[to_xy(c) for c in p] for p in res['paths']],
            'goals': [to_xy(g) for g in goals]}
    write_map(free, a.cell, os.path.join(a.out, 'map_mapf_grid'), a.res)
    with open(os.path.join(a.out, 'plan.json'), 'w') as f:
        json.dump(plan, f)
    log(f'wrote {a.out}/world.sdf and plan.json')
    return 0


# ── Follower node ────────────────────────────────────────────────────────────

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def cmd_follow(a):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node

    plan = json.load(open(a.plan))
    paths = plan['paths']
    n = len(paths)
    T = max(len(p) for p in paths) - 1
    pt = lambda i, t: paths[i][min(t, len(paths[i]) - 1)]

    class Follower(Node):
        def __init__(self):
            super().__init__('mapf_follower')
            self.pose = [None] * n
            self.pubs = [self.create_publisher(Twist, f'/robot{i + 1}/cmd_vel', 10) for i in range(n)]
            for i in range(n):
                self.create_subscription(Odometry, f'/robot{i + 1}/ground_truth',
                                         lambda m, i=i: self.on_odom(i, m), 10)
            self.step = 0
            self.step_t0 = None
            self.t_start = None
            self.create_timer(0.05, self.tick)
            self.get_logger().info(f'{n} robots, {T} steps ({plan["algo"]}, SoC {plan["soc"]}), '
                                   f'waiting for odometry on all robots')

        def on_odom(self, i, m):
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.pose[i] = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

        def drive(self, i):
            """Return (Twist, done) for robot i toward its cell target of the current step."""
            x, y, yaw = self.pose[i]
            tx, ty = pt(i, self.step + 1)
            d = math.hypot(tx - x, ty - y)
            cmd = Twist()
            if d < a.tol:
                return cmd, True
            err = wrap(math.atan2(ty - y, tx - x) - yaw)
            if abs(err) > 0.3 and d > 0.15:
                cmd.angular.z = max(-a.wmax, min(a.wmax, 2.5 * err))
                if abs(cmd.angular.z) < 0.35:
                    cmd.angular.z = math.copysign(0.35, err)
            else:
                cmd.linear.x = max(0.12, min(a.vmax, 1.5 * d))
                cmd.angular.z = max(-1.2, min(1.2, 2.5 * err))
            return cmd, False

        def tick(self):
            if any(p is None for p in self.pose):
                return
            now = self.get_clock().now().nanoseconds * 1e-9
            if self.t_start is None:
                self.t_start = self.step_t0 = now
                self.get_logger().info('odometry on all robots — starting')
            if self.step >= T:
                for pub in self.pubs:
                    pub.publish(Twist())
                return
            done = 0
            for i in range(n):
                cmd, ok = self.drive(i)
                self.pubs[i].publish(cmd)
                done += ok
            if done == n:
                self.step += 1
                if self.step % 5 == 0 or self.step == T:
                    self.get_logger().info(f'step {self.step}/{T}  last step {now - self.step_t0:.1f}s  '
                                           f'elapsed {now - self.t_start:.0f}s')
                self.step_t0 = now
                if self.step >= T:
                    errs = [math.hypot(plan['goals'][i][0] - self.pose[i][0], plan['goals'][i][1] - self.pose[i][1])
                            for i in range(n)]
                    self.get_logger().info(f'DONE in {now - self.t_start:.0f}s — goal error max {max(errs):.2f} m, '
                                           f'mean {sum(errs) / n:.2f} m, reached {sum(e < 0.2 for e in errs)}/{n}')

    rclpy.init()
    node = Follower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for pub in node.pubs:
            pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def cmd_tf(a):
    """Ground-truth TF for Nav2 without AMCL: static map->robotN/odom (identity) and
    robotN/odom->robotN/base_link from /robotN/ground_truth. World frame == map frame."""
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    class GtTf(Node):
        def __init__(self):
            super().__init__('mapf_gt_tf')
            self.br = TransformBroadcaster(self)
            self.static = StaticTransformBroadcaster(self)
            statics = []
            for i in range(1, a.robots + 1):
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id, t.child_frame_id = 'map', f'robot{i}/odom'
                t.transform.rotation.w = 1.0
                statics.append(t)
                self.create_subscription(Odometry, f'/robot{i}/ground_truth',
                                         lambda m, i=i: self.on_odom(i, m), 10)
            self.static.sendTransform(statics)
            self.get_logger().info(f'broadcasting ground-truth TF for {a.robots} robot(s)')

        def on_odom(self, i, m):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id, t.child_frame_id = f'robot{i}/odom', f'robot{i}/base_link'
            t.transform.translation.x = m.pose.pose.position.x
            t.transform.translation.y = m.pose.pose.position.y
            t.transform.rotation = m.pose.pose.orientation
            self.br.sendTransform(t)

    rclpy.init()
    node = GtTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    g = sub.add_parser('gen')
    g.add_argument('--out', required=True)
    g.add_argument('--size', type=int, default=24)
    g.add_argument('--agents', type=int, default=16)
    g.add_argument('--density', type=float, default=0.10)
    g.add_argument('--seed', type=int, default=1)
    g.add_argument('--algo', default='ecbs', choices=['pp', 'cbs', 'ecbs'])
    g.add_argument('--cell', type=float, default=1.0)
    g.add_argument('--res', type=float, default=0.1, help='Nav2 map resolution, m/px')
    g.add_argument('--timeout', type=float, default=60.0)
    t = sub.add_parser('tf')
    t.add_argument('--robots', type=int, default=1)
    f = sub.add_parser('follow')
    f.add_argument('--plan', required=True)
    f.add_argument('--vmax', type=float, default=0.6)
    f.add_argument('--wmax', type=float, default=1.8)
    f.add_argument('--tol', type=float, default=0.1)
    a, _ = ap.parse_known_args()   # ros args after --ros-args are ignored
    sys.exit({'gen': cmd_gen, 'follow': cmd_follow, 'tf': cmd_tf}[a.cmd](a))


if __name__ == '__main__':
    main()
