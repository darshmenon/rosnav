#!/usr/bin/env python3
"""
_common.py — Shared launch-file helpers for rosnav_bot.

Used by both slam_nav.launch.py (single robot) and multi_robot.launch.py
(fleet) so that world/Gazebo/RSP/laser-filter/nav2-params boilerplate and
the diff/mecanum/ackermann drive-type selection logic exist in exactly one
place. Installed alongside the other launch files (see CMakeLists.txt's
`install(DIRECTORY launch ...)`), so callers reach it with:

    import os, sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import _common
"""

import os
import re
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

ROS_DISTRO = os.environ.get('ROS_DISTRO', 'humble')


def truthy(value: str) -> bool:
    return value.strip().lower() in ('true', '1', 'yes')


def resolve_gazebo_world_name(world_path: str) -> str:
    """Read the <world name="..."> attribute from an SDF file; fall back to the filename stem."""
    fallback = os.path.splitext(os.path.basename(world_path))[0]
    try:
        root = ET.parse(os.path.expanduser(world_path)).getroot()
    except (ET.ParseError, OSError):
        return fallback

    if root.tag == 'world' and root.get('name'):
        return root.get('name')

    world = root.find('world')
    if world is not None and world.get('name'):
        return world.get('name')

    return fallback


def urdf_filename_for(drive_type: str) -> str:
    if drive_type == 'mecanum':
        return 'robot_mecanum.urdf.xacro'
    if drive_type == 'ackermann':
        return 'robot_ackermann.urdf.xacro'
    return 'robot.urdf.xacro'


def nav2_params_filename(controller: str, drive_type: str, ros_distro: str = None) -> str:
    """Pick the single-robot nav2 params file for a drive_type/controller/distro combo."""
    ros_distro = ros_distro or ROS_DISTRO
    if drive_type == 'ackermann':
        return 'nav2_params_ackermann.yaml'
    # Jazzy's params file already defaults to MPPI; the dwb/mppi switch only
    # applies on Humble, which defaults to DWB.
    if ros_distro == 'jazzy':
        return 'nav2_params_mecanum_jazzy.yaml' if drive_type == 'mecanum' else 'nav2_params_jazzy.yaml'
    if drive_type == 'mecanum':
        return 'nav2_params_mecanum.yaml'
    return 'nav2_params_mppi.yaml' if controller == 'mppi' else 'nav2_params.yaml'


def patch_pkg_share_placeholder(raw_params_path: str, pkg_share: str) -> str:
    """Replace the replace_with_pkg_share placeholder with an absolute path;
    returns the path to a temp file containing the patched YAML."""
    with open(raw_params_path) as f:
        patched = re.sub(r'replace_with_pkg_share', pkg_share.replace('\\', '/'), f.read())
    out_path = f'/tmp/diff_drive_nav2_patched_{os.getpid()}_{os.path.basename(raw_params_path)}'
    with open(out_path, 'w') as f:
        f.write(patched)
    return out_path


def gazebo_server_action(world_path: str):
    ros_gz_share = get_package_share_directory('ros_gz_sim')
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r -s -v1 {world_path}',
            'on_exit_shutdown': 'true',
        }.items(),
    )


def gazebo_client_action():
    ros_gz_share = get_package_share_directory('ros_gz_sim')
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g'}.items(),
    )


def rsp_include(pkg_share: str, urdf_path, frame_prefix='', namespace='', lidar_type='2d',
                 lidar3d_height='0.25', lidar3d_vfov_deg='10'):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'rsp.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'urdf': urdf_path,
            'frame_prefix': frame_prefix,
            'namespace': namespace,
            'lidar_type': lidar_type,
            'lidar3d_height': lidar3d_height,
            'lidar3d_vfov_deg': lidar3d_vfov_deg,
        }.items(),
    )


def laser_filter_node(pkg_share: str, namespace: str = ''):
    return Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        namespace=namespace or None,
        parameters=[os.path.join(pkg_share, 'config', 'laser_filters.yaml')],
        remappings=[('scan', 'scan_raw'), ('scan_filtered', 'scan')],
        output='screen',
    )


def spawn_robot_node(gazebo_world_name: str, topic, name, x, y, z, yaw):
    return Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', gazebo_world_name,
            '-topic', topic,
            '-name', name,
            '-x', x, '-y', y, '-z', z, '-Y', yaw,
        ],
        output='screen',
    )


# ──────────────────────────────────────────────────────────────────────────
# Multi-robot nav2 param namespacing
# ──────────────────────────────────────────────────────────────────────────
# nav2_multirobot_params.yaml (diff-drive) is a hand-tuned file that already
# has ROBOT_NS placeholders baked into its text. For drive types that don't
# have such a hand-tuned fleet template (mecanum, ackermann), the same
# namespacing is instead applied programmatically to the single-robot
# nav2_params_*.yaml file selected by nav2_params_filename() above — one
# source of truth per drive type, for both single- and multi-robot launches.
#
# Rule: TF frame-name values get an "<ns>/" prefix (the shared "map" frame is
# left alone, since ROS namespacing can't rewrite parameter string values —
# only topics/services/actions). Absolute topics (leading "/") referring to
# per-robot data get "/<ns>" inserted; relative topics already auto-namespace
# via the node's own ROS namespace and are left untouched.
_FRAME_KEYS = (
    ('amcl', 'base_frame_id'),
    ('amcl', 'odom_frame_id'),
    ('bt_navigator', 'robot_base_frame'),
    ('behavior_server', 'local_frame'),
    ('behavior_server', 'robot_base_frame'),
    ('collision_monitor', 'base_frame_id'),
    ('collision_monitor', 'odom_frame_id'),
)
_ABS_TOPIC_KEYS = (
    ('bt_navigator', 'odom_topic'),
)
# map_server/docking_server/loopback_simulator aren't part of the per-robot
# fleet bringup (map_server runs once, globally; loopback_simulator isn't
# used at all). docking_server IS wired for drive_type:=diff (see
# multi_robot.launch.py's docking_actions, and nav2_multirobot_params.yaml's
# docking_server: section) — but only the diff-drive template carries tuned
# dock config, so it's still stripped here for the mecanum/ackermann
# programmatic-namespacing path, which has no docking_server section at all.
_UNUSED_MULTIROBOT_NODES = ('map_server', 'docking_server', 'loopback_simulator')
# Per-robot initial pose is set dynamically (see multi_robot.launch.py); the
# single-robot file's static default would only be correct for one robot.
_STRIP_AMCL_KEYS = ('set_initial_pose', 'initial_pose_x', 'initial_pose_y', 'initial_pose_z', 'initial_pose_yaw')


def namespace_nav2_params(params: dict, ns: str) -> dict:
    """Rewrite a single-robot nav2 params dict in place for use inside a
    namespaced multi-robot fleet. Mutates and returns `params`."""
    for node, key in _FRAME_KEYS:
        section = params.get(node, {}).get('ros__parameters', {})
        if key in section and section[key] != 'map':
            section[key] = f"{ns}/{section[key]}"

    for node, key in _ABS_TOPIC_KEYS:
        section = params.get(node, {}).get('ros__parameters', {})
        if key in section and str(section[key]).startswith('/'):
            section[key] = f"/{ns}{section[key]}"

    for costmap in ('global_costmap', 'local_costmap'):
        section = params.get(costmap, {}).get(costmap, {}).get('ros__parameters', {})
        if 'robot_base_frame' in section:
            section['robot_base_frame'] = f"{ns}/{section['robot_base_frame']}"
        if costmap == 'local_costmap' and section.get('global_frame') not in (None, 'map'):
            section['global_frame'] = f"{ns}/{section['global_frame']}"
        scan = section.get('obstacle_layer', {}).get('scan', {})
        if str(scan.get('topic', '')).startswith('/'):
            scan['topic'] = f"/{ns}{scan['topic']}"

    amcl = params.get('amcl', {}).get('ros__parameters', {})
    for key in _STRIP_AMCL_KEYS:
        amcl.pop(key, None)

    for node in _UNUSED_MULTIROBOT_NODES:
        params.pop(node, None)

    return params
