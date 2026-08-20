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
from launch.actions import GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

ROS_DISTRO = os.environ.get('ROS_DISTRO', 'humble')


def models_resource_path(pkg_share: str = None) -> str:
    """Absolute path to share/rosnav_bot/models (parent of model://Depot)."""
    pkg_share = pkg_share or get_package_share_directory('rosnav_bot')
    return os.path.join(pkg_share, 'models')


def with_gz_model_path(action, pkg_share: str = None):
    """Ensure GZ_SIM_RESOURCE_PATH includes vendored models/ (offline Depot),
    models/fuel/ (textured SLAM worlds), and share/ (URDF meshes). sdformat
    rewrites a URDF's package://rosnav_bot/... mesh URIs to model://rosnav_bot/...
    during URDF->SDF conversion, which gz sim can only resolve against a
    resource path one level above pkg_share (i.e. .../share, so
    model://rosnav_bot/meshes/... -> .../share/rosnav_bot/meshes/...). Without
    it gz sim silently drops the mesh (logs an error, robot still spawns) —
    confirmed this was already happening for the mir100 chassis skin before
    this fix; RViz was unaffected since it resolves package:// directly via
    resource_retriever, not this path."""
    pkg_share = pkg_share or get_package_share_directory('rosnav_bot')
    models_dir = models_resource_path(pkg_share)
    fuel_dir = os.path.join(models_dir, 'fuel')
    share_dir = os.path.dirname(pkg_share)
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    parts = [p for p in existing.split(':') if p]
    for p in (share_dir, models_dir, fuel_dir):
        if p not in parts:
            parts.insert(0, p)
    return GroupAction([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ':'.join(parts)),
        action,
    ])


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


def urdf_filename_for(drive_type: str, robot_model: str = 'custom') -> str:
    if robot_model in ('mir100', 'husky'):
        if drive_type != 'diff':
            print(f'[rosnav_bot] robot_model={robot_model} only supported for drive_type:=diff '
                  f'(got drive_type:={drive_type}) — ignoring robot_model')
        elif robot_model == 'mir100':
            return 'robot_mir100.urdf.xacro'
        else:
            return 'robot_husky.urdf.xacro'
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


def gazebo_server_action(world_path: str, pkg_share: str = None):
    ros_gz_share = get_package_share_directory('ros_gz_sim')
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r -s -v1 {world_path}',
            'on_exit_shutdown': 'true',
        }.items(),
    )
    return with_gz_model_path(gz, pkg_share)


def gazebo_client_action(pkg_share: str = None):
    ros_gz_share = get_package_share_directory('ros_gz_sim')
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g'}.items(),
    )
    return with_gz_model_path(gz, pkg_share)


def rsp_include(pkg_share: str, urdf_path, frame_prefix='', namespace='', lidar_type='2d',
                 lidar3d_height='0.25', lidar3d_vfov_deg='10', enable_camera='false',
                 enable_rgbd='false'):
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
            'enable_camera': enable_camera,
            'enable_rgbd': enable_rgbd,
        }.items(),
    )


