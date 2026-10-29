#!/usr/bin/env python3
"""
mapf_gazebo.launch.py — N diff-drive robots following a MAPF (CBS/ECBS/PP) plan
on a generated grid world in Gazebo. Lightweight: no Nav2/SLAM/sensors.

  ros2 launch rosnav_bot mapf_gazebo.launch.py                       # 16 robots, ECBS
  ros2 launch rosnav_bot mapf_gazebo.launch.py algo:=cbs agents:=12 seed:=3
  ros2 launch rosnav_bot mapf_gazebo.launch.py gui:=false            # headless

Start = coloured tile under each robot; goal = same-colour tile + post.
See scripts/mapf_compare.py for the algorithm comparison.
"""

import os
import subprocess
import sys
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import _common  # noqa: E402


def _nav2_params(pkg_share, ns, out_dir):
    """Single-robot RPP params -> namespaced, sensorless (static map + inflation only)."""
    import yaml
    with open(os.path.join(pkg_share, 'config', 'nav2_params_rpp.yaml')) as f:
        params = _common.namespace_nav2_params(yaml.safe_load(f), ns)
    drop = {'obstacle_layer', 'voxel_layer', 'denoise_layer',
            'keepout_filter', 'speed_filter', 'binary_filter'}
    glob = params['global_costmap']['global_costmap']['ros__parameters']
    loc = params['local_costmap']['local_costmap']['ros__parameters']
    for cm in (glob, loc):
        cm['plugins'] = [p for p in cm['plugins'] if p not in drop]
    if 'static_layer' not in loc['plugins']:
        loc['plugins'].insert(0, 'static_layer')
        loc['static_layer'] = dict(glob['static_layer'])
    path = os.path.join(out_dir, 'nav2_params.yaml')
    with open(path, 'w') as f:
        f.write(yaml.safe_dump(params).replace('replace_with_pkg_share', pkg_share))
    print(f'[mapf_gazebo.launch] nav2 params -> {path} (planner use_astar='
          f'{params["planner_server"]["ros__parameters"]["GridBased"].get("use_astar")})', flush=True)
    return path


def _setup(context):
    cfg = {k: LaunchConfiguration(k).perform(context)
           for k in ('size', 'agents', 'density', 'seed', 'algo', 'cell', 'gui', 'vmax', 'cam_delay', 'nav')}
    pkg_share = get_package_share_directory('rosnav_bot')
    script = os.path.join(get_package_prefix('rosnav_bot'), 'lib', 'rosnav_bot', 'mapf_gazebo.py')
    out = tempfile.mkdtemp(prefix='mapf_gazebo_')

    print(f'[mapf_gazebo.launch] planning with {cfg["algo"]}: {cfg["agents"]} robots on '
          f'{cfg["size"]}x{cfg["size"]} grid (seed {cfg["seed"]}) -> {out}', flush=True)
    subprocess.run([sys.executable, '-I', script, 'gen', '--out', out,
                    '--size', cfg['size'], '--agents', cfg['agents'], '--density', cfg['density'],
                    '--seed', cfg['seed'], '--algo', cfg['algo'], '--cell', cfg['cell']], check=True)

    world = os.path.join(out, 'world.sdf')
    n = int(cfg['agents'])
    nav = _common.truthy(cfg['nav'])
    bridge_args = []
    for i in range(1, n + 1):
        bridge_args += [f'/robot{i}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                        f'/robot{i}/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry']
        if nav and i == 1:
            bridge_args.append('/robot1/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry')

    actions = [
        LogInfo(msg=f'[mapf_gazebo.launch] world {world}'),
        _common.gazebo_server_action(world, pkg_share),
        Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=bridge_args, output='screen'),
    ]
    if not nav:
        actions.append(Node(package='rosnav_bot', executable='mapf_gazebo.py', name='mapf_follower',
                            output='screen',
                            arguments=['follow', '--plan', os.path.join(out, 'plan.json'), '--vmax', cfg['vmax']]))
    else:
        # Nav2 on robot1 only: ground-truth TF (no AMCL), static map, A* NavFn + RPP, no sensors.
        params = _nav2_params(pkg_share, 'robot1', out)
        actions += [
            LogInfo(msg=f'[mapf_gazebo.launch] NAV mode: Nav2 on robot1, goal = {out}/plan.json goals[0]'),
            Node(package='rosnav_bot', executable='mapf_gazebo.py', name='mapf_gt_tf', output='screen',
                 arguments=['tf', '--robots', '1']),
            Node(package='nav2_map_server', executable='map_server', name='map_server', namespace='robot1',
                 output='screen', parameters=[{'yaml_filename': os.path.join(out, 'map_mapf_grid.yaml'),
                                               'use_sim_time': False}],
                 remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
                 namespace='robot1', output='screen',
                 parameters=[{'autostart': True, 'node_names': ['map_server'], 'use_sim_time': False}]),
            TimerAction(period=6.0, actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'nav2_navigation_global_tf.launch.py')),
                launch_arguments={'use_sim_time': 'false', 'params_file': params, 'namespace': 'robot1'}.items())]),
        ]
    if _common.truthy(cfg['gui']):
        actions.insert(2, _common.gazebo_client_action(pkg_share))
        # fly the GUI camera to a top-down view that frames the whole grid (60° fov)
        z = 0.75 * max(float(cfg['size']), 1.0) * float(cfg['cell'])
        req = ('pose: {position: {x: 0, y: 0, z: %.1f}, '
               'orientation: {x: 0, y: 0.7071068, z: 0, w: 0.7071068}}' % z)
        actions.append(TimerAction(period=float(cfg['cam_delay']), actions=[ExecuteProcess(
            cmd=['gz', 'service', '-s', '/gui/move_to/pose', '--reqtype', 'gz.msgs.GUICamera',
                 '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req],
            output='screen')]))
    return actions


def generate_launch_description():
    def arg(name, default, desc):
        return DeclareLaunchArgument(name, default_value=default, description=desc)
    return LaunchDescription([
        arg('size', '24', 'Grid is size x size cells'),
        arg('agents', '16', 'Number of robots'),
        arg('density', '0.10', 'Obstacle density'),
        arg('seed', '1', 'Instance random seed'),
        arg('algo', 'ecbs', 'MAPF solver: pp | cbs | ecbs'),
        arg('cell', '1.0', 'Cell size, metres'),
        arg('vmax', '0.6', 'Max linear speed, m/s'),
        arg('gui', 'true', 'Open the Gazebo GUI'),
        arg('nav', 'false', 'true = run Nav2 (A* NavFn + RPP) on robot1 instead of the MAPF follower'),
        arg('cam_delay', '12', 'Seconds to wait for the GUI before setting the top-down camera'),
        OpaqueFunction(function=_setup),
    ])
