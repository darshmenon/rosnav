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
from std_msgs.msg import Float64, String
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


def _leg(name, y):
    """A leg link plus a hip joint. The swing is what the lidar sees moving."""
    return f'''
 <link name="{name}"><gravity>false</gravity><pose>0 {y} 0.40 0 0 0</pose>
  <inertial><mass>8</mass><inertia><ixx>0.4</ixx><iyy>0.4</iyy><izz>0.05</izz></inertia></inertial>
  <collision name="c"><pose>0 0 0 0 0 0</pose><geometry><cylinder><radius>0.07</radius><length>0.75</length></cylinder></geometry></collision>
  <visual name="v"><pose>0 0 0 0 0 0</pose><geometry><cylinder><radius>0.07</radius><length>0.75</length></cylinder></geometry>
   <material><ambient>0.15 0.16 0.22 1</ambient><diffuse>0.15 0.16 0.22 1</diffuse></material></visual>
 </link>
 <joint name="hip_{name}" type="revolute">
  <parent>link</parent><child>{name}</child>
  <pose>0 {y} 0.78 0 0 0</pose>
  <axis><xyz>0 1 0</xyz><limit><lower>-0.9</lower><upper>0.9</upper></limit></axis>
 </joint>
 <plugin filename="libgz-sim-joint-position-controller-system.so" name="gz::sim::systems::JointPositionController">
  <joint_name>hip_{name}</joint_name>
  <topic>/model/hip_{name}/cmd_pos</topic>
  <p_gain>8</p_gain><i_gain>0</i_gain><d_gain>0.2</d_gain>
 </plugin>'''


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
            f'<visual name="body"><pose>0 0 {1.15 * s:.3f} 0 0 0</pose><geometry>'
            f'<box><size>0.28 {0.20 * s:.3f} {0.55 * s:.3f}</size></box></geometry></visual>'
        )
    # The topic string is filled with the model name below. The helper uses a
    # placeholder so each person gets their own hip command.
    legs = (_leg('legL', 0.09 * s) + _leg('legR', -0.09 * s)).replace(
        '/model/hip_', f'/model/{name}/hip_')
    return f'''<?xml version="1.0"?>
<sdf version="1.8"><model name="{name}">
 <link name="link"><gravity>false</gravity>
  <inertial><mass>50</mass><inertia><ixx>8</ixx><iyy>8</iyy><izz>1.5</izz></inertia></inertial>
  <collision name="col"><pose>0 0 {1.15 * s:.3f} 0 0 0</pose><geometry>
    <cylinder><radius>0.18</radius><length>{0.7 * s:.3f}</length></cylinder></geometry></collision>
  {visual}
 </link>
 {legs}
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
        self.hips = [(self.create_publisher(Float64, f'/model/{nm}/hip_legL/cmd_pos', 10),
                      self.create_publisher(Float64, f'/model/{nm}/hip_legR/cmd_pos', 10))
                     for nm in self.names]
        self.phase = [0.0] * self.n
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
        for nm in self.names:
            args.append(f'/model/{nm}/hip_legL/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double')
            args.append(f'/model/{nm}/hip_legR/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double')
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
            if nm not in self.truth:
                continue
            x, y, _yaw = self.truth[nm]
            # A pose outside the room (or a NaN) is what blew the ODE hash
            # (aabbBound left the integer range). Keep the last good point.
            if math.isfinite(x) and math.isfinite(y) and abs(x) < 25 and abs(y) < 25:
                self.crowd.pos[i] = (x, y)
        self.robot = self._robot_xy()
        vel = self.crowd.step(dt, self.robot)
        out = []
        cap = 1.6
        for i, nm in enumerate(self.names):
            vx, vy = float(vel[i][0]), float(vel[i][1])
            if not (math.isfinite(vx) and math.isfinite(vy)):
                vx, vy = 0.0, 0.0
            sp = math.hypot(vx, vy)
            if sp > cap:
                vx, vy, sp = vx * cap / sp, vy * cap / sp, cap
            yaw = self.truth.get(nm, (0, 0, 0.0))[2]
            if not math.isfinite(yaw):
                yaw = 0.0
            c, s = math.cos(-yaw), math.sin(-yaw)
            m = Twist()
            m.linear.x = float(c * vx - s * vy)
            m.linear.y = float(s * vx + c * vy)
            heading = math.atan2(vy, vx) if sp > 0.08 else yaw
            err = math.atan2(math.sin(heading - yaw), math.cos(heading - yaw))
            m.angular.z = float(max(-1.5, min(1.5, 2.0 * err)))
            self.pubs[i].publish(m)
            self.phase[i] += sp * dt * 4.0
            swing = 0.55 * math.sin(self.phase[i]) if sp > 0.08 else 0.0
            left, right = Float64(), Float64()
            left.data, right.data = float(swing), float(-swing)
            self.hips[i][0].publish(left)
            self.hips[i][1].publish(right)
            out.append(dict(id=i + 1, x=round(float(self.crowd.pos[i][0]), 3),
                            y=round(float(self.crowd.pos[i][1]), 3),
                            vx=round(vx, 3), vy=round(vy, 3)))
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
        try:
            rclpy.spin(n)
        except Exception:  # noqa: BLE001 - a signal can invalidate the context mid-spin
            if rclpy.ok():
                raise
    except KeyboardInterrupt:
        pass
    finally:
        n.shutdown()
        n.destroy_node()
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
