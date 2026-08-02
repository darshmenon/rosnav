#!/usr/bin/env python3
"""
rmf_fleet.launch.py — Open-RMF full traffic scheduling for this package's
multi-robot fleet.

Starts, in order:
  1. rmf_traffic_schedule   (rmf_traffic_ros2)  — the traffic scheduler that
     negotiates conflict-free itineraries across every registered robot.
  2. rmf_task_dispatcher    (rmf_task_ros2)     — assigns submitted tasks
     (e.g. patrol loops) to whichever registered robot can do them.
  3. rmf_fleet_adapter.py   (this package)      — registers robot1..robotN
     with RMF and bridges RMF path commands to each robot's existing
     /{ns}/navigate_to_pose Nav2 action.

Customize (no code edits needed for most users)
───────────────────────────────────────────────
  config/rmf_fleet.yaml   — lanes, vehicle, docking plugin, battery, tasks
  config/locations.yaml   — named waypoint poses

  ros2 launch diff_drive_robot rmf_fleet.launch.py \\
      robot_count:=2 docking:=noop rmf_config:=/path/to/my_rmf_fleet.yaml

Prerequisite
────────────
multi_robot.launch.py must already be running in *static-map* mode
(explore:=false) with the same robot_count, since RMF's nav graph is
anchored to fixed map-frame coordinates from config/locations.yaml — SLAM/
explore mode has no fixed map for the graph to sit on:

  ros2 launch diff_drive_robot multi_robot.launch.py explore:=false robot_count:=2

Then, in a second terminal:

  ros2 launch diff_drive_robot rmf_fleet.launch.py robot_count:=2

Test it (submits a patrol task; watch rmf_traffic_schedule / Gazebo to see
the fleet negotiate shared corridor space):

  ros2 run diff_drive_robot rmf_submit_task.py patrol room_a room_b --rounds 2
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def _build_all(context, pkg_share: str):
    robot_count = int(LaunchConfiguration('robot_count').perform(context).strip())
    robots_arg = LaunchConfiguration('robots').perform(context).strip()
    fleet_name = LaunchConfiguration('fleet_name').perform(context).strip()
    map_name = LaunchConfiguration('map_name').perform(context).strip()
    adapter_delay = float(LaunchConfiguration('adapter_delay').perform(context).strip())
    rmf_config = LaunchConfiguration('rmf_config').perform(context).strip()
    docking = LaunchConfiguration('docking').perform(context).strip()

    if robots_arg:
        robot_namespaces = [r.strip() for r in robots_arg.split(',') if r.strip()]
    else:
        robot_namespaces = [f'robot{i + 1}' for i in range(robot_count)]
    robots_csv = ','.join(robot_namespaces)

    if not rmf_config:
        rmf_config = os.path.join(pkg_share, 'config', 'rmf_fleet.yaml')

    adapter_args = [
        '--config', rmf_config,
        '--fleet-name', fleet_name,
        '--map-name', map_name,
        '--robots', robots_csv,
    ]
    if docking:
        adapter_args += ['--docking', docking]

    actions = [
        LogInfo(msg=f'[rmf_fleet] fleet_name = {fleet_name}'),
        LogInfo(msg=f'[rmf_fleet] map_name   = {map_name}'),
        LogInfo(msg=f'[rmf_fleet] robots     = {robot_namespaces}'),
        LogInfo(msg=f'[rmf_fleet] config     = {rmf_config}'),
        LogInfo(msg='[rmf_fleet] expecting multi_robot.launch.py explore:=false '
                    'already running with the same robot_count — RMF\'s nav graph '
                    'is anchored to fixed map-frame coordinates.'),

        # 1. Traffic scheduler
        Node(
            package='rmf_traffic_ros2',
            executable='rmf_traffic_schedule',
            name='rmf_traffic_schedule',
            output='screen',
            parameters=[{'use_sim_time': True}]),

        # 2. Task dispatcher
        Node(
            package='rmf_task_ros2',
            executable='rmf_task_dispatcher',
            name='rmf_task_dispatcher',
            output='screen',
            parameters=[{'use_sim_time': True}]),
    ]

    # 3. Fleet adapter — give the schedule/dispatcher nodes a moment to come up
    # first so add_fleet()/task bidding don't race their ROS graph discovery.
    actions.append(LogInfo(
        msg=f'[rmf_fleet] rmf_fleet_adapter.py starting in {adapter_delay:.1f}s for {robots_csv}'))
    actions.append(TimerAction(
        period=adapter_delay,
        actions=[Node(
            package='diff_drive_robot',
            executable='rmf_fleet_adapter.py',
            name='rmf_fleet_adapter',
            output='screen',
            arguments=adapter_args,
            parameters=[{'use_sim_time': True}])]))

    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory('diff_drive_robot')
    return LaunchDescription([
        # Apply to every node in this launch (schedule, dispatcher, adapter).
        SetParameter(name='use_sim_time', value=True),
        DeclareLaunchArgument(
            'robot_count', default_value='2',
            description='Number of robots (robot1..robotN) — match multi_robot.launch.py'),
        DeclareLaunchArgument(
            'robots', default_value='',
            description='Optional explicit comma-separated namespaces, e.g. robot1,robot2 '
                        '— overrides robot_count when set'),
        DeclareLaunchArgument(
            'fleet_name', default_value='rosnav_fleet',
            description='RMF fleet name'),
        DeclareLaunchArgument(
            'map_name', default_value='hospital',
            description='RMF nav graph map label (logical name, not a ROS topic)'),
        DeclareLaunchArgument(
            'rmf_config', default_value='',
            description='Path to rmf_fleet.yaml (default: share/.../config/rmf_fleet.yaml)'),
        DeclareLaunchArgument(
            'docking', default_value='',
            description='Override docking plugin: aruco | noop | module.path:Class'),
        DeclareLaunchArgument(
            'adapter_delay', default_value='3.0',
            description='Seconds to wait after starting schedule/dispatcher before '
                        'starting rmf_fleet_adapter.py'),
        OpaqueFunction(function=_build_all, args=[pkg_share]),
    ])
