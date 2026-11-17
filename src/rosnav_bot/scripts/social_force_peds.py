#!/usr/bin/env python3
"""
social_force_peds.py — spawn human-looking pedestrians in a running Gazebo world
and drive them with the social-force crowd from social_force_core.py.

  * each person = kinematic model. The visual is the standing MakeHuman mesh
    (models/person_standing) so gpu_lidar hits a person, not a box. A cylinder
    collision stays for contact. gz <actor>s are invisible to this lidar;
  * driven through the model's VelocityControl plugin over a ros_gz_bridge Twist topic
    (one bridge process for the whole crowd), world poses read back from
    /world/<world>/pose/info (truth), so the social forces use real positions;
  * people route over the map with A*, avoid each other, the walls and the robot
    (see social_force_core.py for behaviour: pauses, groups, hesitation, distracted walkers);
  * publishes ground truth on /social_peds/gt (JSON) for metrics (social_metrics.py).

  ros2 run rosnav_bot social_force_peds.py --ros-args -p world:=hospital \
      -p map:=$(ros2 pkg prefix rosnav_bot)/share/rosnav_bot/maps/map_hospital.yaml -p n:=10
"""
import json
import math
import os
import random
import subprocess
import sys
import tempfile
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import social_force_core as sf  # noqa: E402

def _person_mesh_uri():
    """Standing MakeHuman mesh. gpu_lidar hits visuals, so this is what the scan sees."""
    here = os.path.dirname(os.path.realpath(__file__))
    dae = os.path.normpath(os.path.join(
        here, '..', 'models', 'person_standing', 'meshes', 'standing.dae'))
    if not os.path.isfile(dae):
        dae = os.path.normpath(os.path.join(
            here, '..', '..', 'share', 'rosnav_bot', 'models',
            'person_standing', 'meshes', 'standing.dae'))
    return 'file://' + dae if os.path.isfile(dae) else ''


def human_sdf(name, rng):
    h = rng.uniform(1.55, 1.85)
    s = h / 1.7
    mesh = _person_mesh_uri()
    if mesh:
        visual = (
            f'<visual name="body"><pose>0 0 0.02 0.04 0 0</pose>'
            f'<geometry><mesh><uri>{mesh}</uri><scale>{s:.3f} {s:.3f} {s:.3f}</scale></mesh></geometry></visual>'
        )
    else:
        visual = (
            f'<visual name="body"><pose>0 0 {0.9 * s:.3f} 0 0 0</pose><geometry>'
            f'<cylinder><radius>0.22</radius><length>{1.7 * s:.3f}</length></cylinder></geometry></visual>'
        )
    return f'''<?xml version="1.0"?>
<sdf version="1.8"><model name="{name}">
 <link name="link"><gravity>false</gravity>
  <inertial><mass>60</mass><inertia><ixx>10</ixx><iyy>10</iyy><izz>2</izz></inertia></inertial>
  <collision name="col"><pose>0 0 {0.9 * s:.3f} 0 0 0</pose><geometry>
    <cylinder><radius>0.22</radius><length>{1.7 * s:.3f}</length></cylinder></geometry></collision>
  {visual}
 </link>
 <plugin filename="libignition-gazebo-velocity-control-system" name="ignition::gazebo::systems::VelocityControl">
  <link_name>link</link_name></plugin>
</model></sdf>'''


class Peds(Node):
    def __init__(self):
        super().__init__('social_force_peds')
        for k, v in dict(world='hospital', map='', n=8, seed=0, hz=10.0, robot_weight=0.6,
                         spawn_avoid_x=0.0, spawn_avoid_y=0.0, robot_frame='base_link',
                         map_frame='map', spawn=True, offset_x=0.0, offset_y=0.0).items():
            self.declare_parameter(k, v)
        g = lambda k: self.get_parameter(k).value
        self.world_name, self.n = g('world'), int(g('n'))
        self.hz = float(g('hz'))
        # Map frame is spawn-relative; Gazebo poses are world-frame. offset is
        # gazebo = map + offset (the robot's spawn in the world).
        self.off = (float(g('offset_x')), float(g('offset_y')))
        if not g('map'):
            raise SystemExit('pass -p map:=<map yaml> (use the same map as Nav2)')
        self.w = sf.MapWorld(g('map'))
        self.crowd = sf.Crowd(self.w, self.n, seed=int(g('seed')), robot_weight=float(g('robot_weight')),
                              spawn_avoid=(g('spawn_avoid_x'), g('spawn_avoid_y')))
        self.names = [f'ped_{i + 1}' for i in range(self.n)]
        self.procs = []
        self.truth = {}          # name -> (x, y, yaw) from pose/info
        self.robot = None
        self.pubs = [self.create_publisher(Twist, f'/model/{nm}/cmd_vel', 10) for nm in self.names]
        self.gt = self.create_publisher(String, '/social_peds/gt', 10)
        self.marks = self.create_publisher(MarkerArray, '/social_peds/markers', 10)
        self.create_subscription(TFMessage, f'/world/{self.world_name}/pose/info', self._on_pose, 10)
        if g('spawn'):
            self._start_bridge()
            self._spawn_all()
        import tf2_ros
        self.buf = tf2_ros.Buffer()
        self.tl = tf2_ros.TransformListener(self.buf, self)
        self.map_frame, self.base_frame = g('map_frame'), g('robot_frame')
        self.last = time.monotonic()
        self.create_timer(1.0 / self.hz, self._tick)
        self.get_logger().info(
            f'{self.n} pedestrians in world={self.world_name}, map={g("map")}, hz={self.hz}, '
            f'robot_weight={g("robot_weight")}')

    # -- setup
    def _start_bridge(self):
        args = [f'/model/{nm}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' for nm in self.names]
        args.append(f'/world/{self.world_name}/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V')
        self.procs.append(subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge'] + args,
                                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        self.get_logger().info(f'ros_gz_bridge started for {self.n} pedestrians (pid {self.procs[-1].pid})')

    def _spawn_all(self):
        rng = random.Random(int(self.get_parameter('seed').value))
        d = tempfile.mkdtemp(prefix='peds_')
        for nm, (x, y) in zip(self.names, self.crowd.pos):
            f = os.path.join(d, nm + '.sdf')
            open(f, 'w').write(human_sdf(nm, rng))
            gx, gy = x + self.off[0], y + self.off[1]
            r = subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', self.world_name, '-file', f,
                                '-name', nm, '-x', f'{gx:.2f}', '-y', f'{gy:.2f}', '-z', '0.02'],
                               capture_output=True, text=True, timeout=30)
            self.get_logger().info(f'spawned {nm} at ({x:.1f},{y:.1f}) rc={r.returncode}')

    # -- loop
    def _on_pose(self, msg):
        for t in msg.transforms:
            nm = t.child_frame_id
            if nm in self.names:
                q = t.transform.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                self.truth[nm] = (t.transform.translation.x - self.off[0],
                                  t.transform.translation.y - self.off[1], yaw)

    def _robot_xy(self):
        try:
            t = self.buf.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time()).transform
            return (t.translation.x, t.translation.y)
        except Exception:  # noqa: BLE001
            return None

    def _tick(self):
        now = time.monotonic()
        dt = min(0.3, now - self.last)
        self.last = now
        for i, nm in enumerate(self.names):                  # sync the model with the real pose
            if nm in self.truth:
                self.crowd.pos[i] = self.truth[nm][:2]
        self.robot = self._robot_xy()
        vel = self.crowd.step(dt, self.robot)
        out = []
        for i, nm in enumerate(self.names):
            yaw = self.truth.get(nm, (0, 0, 0.0))[2]
            c, s = math.cos(-yaw), math.sin(-yaw)
            m = Twist()
            m.linear.x = float(c * vel[i][0] - s * vel[i][1])  # keep body-frame correct if pushed
            m.linear.y = float(s * vel[i][0] + c * vel[i][1])
            m.angular.z = float(-2.0 * yaw)                   # stay upright / facing +x
            self.pubs[i].publish(m)
            out.append(dict(id=i + 1, x=round(float(self.crowd.pos[i][0]), 3),
                            y=round(float(self.crowd.pos[i][1]), 3),
                            vx=round(float(vel[i][0]), 3), vy=round(float(vel[i][1]), 3)))
        self._publish_markers(out)
        gt = String()
        gt.data = json.dumps({'t': self.get_clock().now().nanoseconds / 1e9, 'peds': out})
        self.gt.publish(gt)

    def _publish_markers(self, peds):
        """RViz: body cylinder + head + ID label + velocity arrow per person (map frame)."""
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for p in peds:
            col = (0.2 + 0.6 * ((p['id'] * 37) % 10) / 10, 0.4 + 0.4 * ((p['id'] * 17) % 10) / 10, 0.9 - 0.5 * ((p['id'] * 23) % 10) / 10)

            def mk(i, typ, z, sx, sy, sz, rgb):
                m = Marker()
                m.header.frame_id, m.header.stamp = 'map', stamp
                m.ns, m.id, m.type = 'ped', p['id'] * 10 + i, typ
                m.pose.position.x, m.pose.position.y, m.pose.position.z = float(p['x']), float(p['y']), float(z)
                m.pose.orientation.w = 1.0
                m.scale.x, m.scale.y, m.scale.z = float(sx), float(sy), float(sz)
                m.color.r, m.color.g, m.color.b, m.color.a = float(rgb[0]), float(rgb[1]), float(rgb[2]), 1.0
                return m
            ma.markers.append(mk(0, Marker.CYLINDER, 0.65, 0.44, 0.44, 1.3, col))
            ma.markers.append(mk(1, Marker.SPHERE, 1.42, 0.24, 0.24, 0.24, (0.95, 0.8, 0.65)))
            lab = mk(2, Marker.TEXT_VIEW_FACING, 1.9, 0.0, 0.0, 0.3, (1, 1, 1))
            lab.text = f'p{p["id"]}'
            ma.markers.append(lab)
            sp = math.hypot(p['vx'], p['vy'])
            if sp > 0.15:
                ar = mk(3, Marker.ARROW, 0.1, 0.05, 0.1, 0.1, (1.0, 0.9, 0.2))
                from geometry_msgs.msg import Point
                ar.pose.position.x = ar.pose.position.y = ar.pose.position.z = 0.0
                ar.points = [Point(x=p['x'], y=p['y'], z=0.1), Point(x=p['x'] + p['vx'], y=p['y'] + p['vy'], z=0.1)]
                ma.markers.append(ar)
        self.marks.publish(ma)

    def shutdown(self):
        for p in self.procs:
            p.terminate()


def main():
    rclpy.init()
    n = Peds()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.shutdown()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
