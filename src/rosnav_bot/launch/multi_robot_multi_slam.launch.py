#!/usr/bin/env python3
"""Convenience launch file for multi-robot SLAM exploration.

This does not replace the normal single-SLAM launch path; it only forwards to
multi_robot.launch.py with slam_mode:=multi.
"""

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():
    pkg_share = get_package_share_directory('rosnav_bot')
    multi_robot_launch = os.path.join(pkg_share, 'launch', 'multi_robot.launch.py')

    forwarded_args = {
        'world': LaunchConfiguration('world'),
        'map': LaunchConfiguration('map'),
        'rviz': LaunchConfiguration('rviz'),
        'explore': 'true',
        'headless': LaunchConfiguration('headless'),
        'fleet_mgmt': LaunchConfiguration('fleet_mgmt'),
        'robot_count': LaunchConfiguration('robot_count'),
        'robot_layout': LaunchConfiguration('robot_layout'),
        'spawn_x': LaunchConfiguration('spawn_x'),
        'spawn_y': LaunchConfiguration('spawn_y'),
        'spawn_z': LaunchConfiguration('spawn_z'),
        'spawn_yaw': LaunchConfiguration('spawn_yaw'),
        'spawn_spacing': LaunchConfiguration('spawn_spacing'),
        'validate_spawns': LaunchConfiguration('validate_spawns'),
        'robot_clearance': LaunchConfiguration('robot_clearance'),
        'spawn_search_radius': LaunchConfiguration('spawn_search_radius'),
        'spawn_search_step': LaunchConfiguration('spawn_search_step'),
        'nav2_start_delay': LaunchConfiguration('nav2_start_delay'),
        'amcl_start_delay': LaunchConfiguration('amcl_start_delay'),
        'robot_start_stagger': LaunchConfiguration('robot_start_stagger'),
        'robots_json': LaunchConfiguration('robots_json'),
        'merge_scans': 'false',
        'slam_mode': 'multi',
        'frontier_detector': LaunchConfiguration('frontier_detector'),
        'frontier_scorer': LaunchConfiguration('frontier_scorer'),
        'distance_weight': LaunchConfiguration('distance_weight'),
        'info_gain_weight': LaunchConfiguration('info_gain_weight'),
        'hysteresis_radius': LaunchConfiguration('hysteresis_radius'),
        'hysteresis_gain': LaunchConfiguration('hysteresis_gain'),
        'frontier_clearance_radius': LaunchConfiguration('frontier_clearance_radius'),
        'failed_goal_radius': LaunchConfiguration('failed_goal_radius'),
        'failed_goal_cooldown': LaunchConfiguration('failed_goal_cooldown'),
        'publish_markers': LaunchConfiguration('publish_markers'),
        'nav_wait_warn_sec': LaunchConfiguration('nav_wait_warn_sec'),
        'tf_wait_warn_sec': LaunchConfiguration('tf_wait_warn_sec'),
    }

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='hospital'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('rviz', default_value='True'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('fleet_mgmt', default_value='false'),
        DeclareLaunchArgument('robot_count', default_value='2'),
        DeclareLaunchArgument('robot_layout', default_value='line'),
        DeclareLaunchArgument('spawn_x', default_value='-2.0'),
        DeclareLaunchArgument('spawn_y', default_value='-1.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.3'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('spawn_spacing', default_value='1.2'),
        DeclareLaunchArgument('validate_spawns', default_value='true'),
        DeclareLaunchArgument('robot_clearance', default_value='0.45'),
        DeclareLaunchArgument('spawn_search_radius', default_value='4.0'),
        DeclareLaunchArgument('spawn_search_step', default_value='0.25'),
        DeclareLaunchArgument('nav2_start_delay', default_value='10.0'),
        DeclareLaunchArgument('amcl_start_delay', default_value='13.0'),
        DeclareLaunchArgument('robot_start_stagger', default_value='6.0'),
        DeclareLaunchArgument('robots_json', default_value=''),
        DeclareLaunchArgument('frontier_detector', default_value='wfd'),
        DeclareLaunchArgument('frontier_scorer', default_value='weighted'),
        DeclareLaunchArgument('distance_weight', default_value='1.0'),
        DeclareLaunchArgument('info_gain_weight', default_value='3.0'),
        DeclareLaunchArgument('hysteresis_radius', default_value='2.0'),
        DeclareLaunchArgument('hysteresis_gain', default_value='1.5'),
        DeclareLaunchArgument('frontier_clearance_radius', default_value='0.30'),
        DeclareLaunchArgument('failed_goal_radius', default_value='0.75'),
        DeclareLaunchArgument('failed_goal_cooldown', default_value='45.0'),
        DeclareLaunchArgument('publish_markers', default_value='true'),
        DeclareLaunchArgument('nav_wait_warn_sec', default_value='15.0'),
        DeclareLaunchArgument('tf_wait_warn_sec', default_value='15.0'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(multi_robot_launch),
            launch_arguments=forwarded_args.items(),
        ),
    ])
