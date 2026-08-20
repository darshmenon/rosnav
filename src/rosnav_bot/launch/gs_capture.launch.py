#!/usr/bin/env python3
"""
gs_capture.launch.py — bring up Gazebo + the gs_capture_rig camera for a
Gaussian Splatting capture pass (see scripts/gs_capture.py). No robot, no
nav stack — just the world and one teleportable camera.

Usage:
  ros2 launch rosnav_bot gs_capture.launch.py world_name:=cafe
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common  # noqa: E402


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('rosnav_bot')
    world_name = LaunchConfiguration('world_name').perform(context)
    world_path = os.path.join(pkg_share, 'worlds', f'{world_name}.world')
    gazebo_world_name = _common.resolve_gazebo_world_name(world_path)

    gazebo = _common.gazebo_server_action(world_path, pkg_share)

    spawn_rig = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_gs_capture_rig',
        arguments=[
            '-world', gazebo_world_name,
            '-file', os.path.join(pkg_share, 'models', 'gs_capture_rig', 'model.sdf'),
            '-name', 'gs_capture_rig',
            '-x', '0', '-y', '0', '-z', '1.0',
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gs_capture_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_share, 'config', 'gs_capture_bridge.yaml'),
            'use_sim_time': True,
        }],
        output='screen',
    )

    return [gazebo, spawn_rig, bridge]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='cafe',
                               description='World to capture (must exist in worlds/)'),
        OpaqueFunction(function=_launch_setup),
    ])
