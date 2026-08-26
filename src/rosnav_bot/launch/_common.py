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
import shutil
import subprocess
import tempfile
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
    # Jazzy's params file already defaults to MPPI; the controller switch only
    # applies on Humble, which defaults to DWB.
    if ros_distro == 'jazzy':
        return 'nav2_params_mecanum_jazzy.yaml' if drive_type == 'mecanum' else 'nav2_params_jazzy.yaml'
    if drive_type == 'mecanum':
        return 'nav2_params_mecanum.yaml'
    if controller == 'mppi':
        return 'nav2_params_mppi.yaml'
    # 'rpp'/'regulated_pure_pursuit': carrot-chasing pursuit controller, the
    # third point of comparison alongside DWB (rollout) and MPPI (optimization).
    if controller in ('rpp', 'regulated_pure_pursuit', 'pure_pursuit'):
        return 'nav2_params_rpp.yaml'
    return 'nav2_params.yaml'


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


def _indent_xml(elem, level=0):
    """Small ElementTree indentation helper for generated runtime SDF files."""
    i = '\n' + level * '  '
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + '  '
        for child in elem:
            _indent_xml(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = i
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = i


def robot_sdf_from_xacro(pkg_share: str, urdf_path: str, *, namespace='',
                         lidar_type='2d', lidar3d_height='0.25',
                         lidar3d_vfov_deg='10', enable_camera='false',
                         enable_rgbd='false', runtime_dir=None) -> ET.Element:
    """Render the selected robot xacro and convert it to an SDF <model>.

    Gazebo Sim has a long-standing issue where sensors on dynamically spawned
    entities can be missed by sensor systems. Baking the robot model into the
    world before gz sim starts avoids that path for single-robot launches.
    """
    runtime_dir = runtime_dir or tempfile.mkdtemp(prefix='rosnav_baked_robot_')
    os.makedirs(runtime_dir, exist_ok=True)
    urdf_out = os.path.join(runtime_dir, 'robot.urdf')

    xacro_cmd = [
        'xacro',
        urdf_path,
        f'namespace:={namespace}',
        f'lidar_type:={lidar_type}',
        f'lidar3d_height:={lidar3d_height}',
        f'lidar3d_vfov_deg:={lidar3d_vfov_deg}',
        f'enable_camera:={enable_camera}',
        f'enable_rgbd:={enable_rgbd}',
    ]
    urdf_text = subprocess.check_output(xacro_cmd, text=True)
    with open(urdf_out, 'w') as f:
        f.write(urdf_text)

    sdf_text = subprocess.check_output(['gz', 'sdf', '-p', urdf_out], text=True)
    sdf_root = ET.fromstring(sdf_text)
    model = sdf_root.find('model') if sdf_root.tag == 'sdf' else None
    if model is None:
        raise RuntimeError(f'gz sdf conversion did not produce an SDF model for {urdf_path}')
    return model


def baked_robot_world(world_path: str, pkg_share: str, robot_name: str, x, y, z, yaw,
                      *, urdf_path: str, namespace='', lidar_type='2d',
                      lidar3d_height='0.25', lidar3d_vfov_deg='10',
                      enable_camera='false', enable_rgbd='false') -> str:
    """Return a temp world path with the robot model inserted before startup."""
    runtime_dir = tempfile.mkdtemp(prefix='rosnav_baked_world_')
    world_src = os.path.expanduser(world_path)
    tree = ET.parse(world_src)
    root = tree.getroot()
    world = root if root.tag == 'world' else root.find('world')
    if world is None:
        raise RuntimeError(f'No <world> element found in {world_src}')

    model = robot_sdf_from_xacro(
        pkg_share,
        urdf_path,
        namespace=namespace,
        lidar_type=lidar_type,
        lidar3d_height=lidar3d_height,
        lidar3d_vfov_deg=lidar3d_vfov_deg,
        enable_camera=enable_camera,
        enable_rgbd=enable_rgbd,
        runtime_dir=runtime_dir,
    )
    model.set('name', robot_name)
    pose = model.find('pose')
    if pose is None:
        pose = ET.Element('pose')
        model.insert(0, pose)
    pose.text = f'{x} {y} {z} 0 0 {yaw}'
    world.append(model)

    # Some world files rely on relative model paths; keep the generated file
    # near a copy of the original as a conservative fallback for those paths.
    shutil.copy2(world_src, os.path.join(runtime_dir, os.path.basename(world_src)))
    out_path = os.path.join(runtime_dir, f'baked_{os.path.basename(world_src)}')
    _indent_xml(root)
    tree.write(out_path, encoding='utf-8', xml_declaration=True)
    return out_path


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


def laser_filter_node(pkg_share: str, namespace: str = '', *,
                      filtered_out: str = 'scan'):
    """scan_raw → cleaned LaserScan on filtered_out (default /scan)."""
    return Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        namespace=namespace or None,
        parameters=[os.path.join(pkg_share, 'config', 'laser_filters.yaml')],
        remappings=[('scan', 'scan_raw'), ('scan_filtered', filtered_out)],
        output='screen',
    )


def scan_quality_gate_node(namespace: str = '', *,
                           mode: str = 'gate',
                           scan_in: str = 'scan_pre',
                           scan_out: str = 'scan'):
    """Drop malformed LaserScans so SLAM / Nav2 never see them.

    Typical chain: scan_raw → laser_filters → scan_pre → gate → scan.
    """
    params = {
        'use_sim_time': True,
        'mode': mode,
        'scan_in': scan_in,
        'scan_out': scan_out,
    }
    if namespace:
        # TF frame ids aren't topics — node namespacing doesn't prefix them,
        # so do it explicitly (map stays global; odom/base_link are per-robot).
        params['odom_frame'] = f'{namespace}/odom'
        params['base_frame'] = f'{namespace}/base_link'
    return Node(
        package='rosnav_bot',
        executable='scan_quality_gate.py',
        name='scan_quality_gate',
        namespace=namespace or None,
        parameters=[params],
        output='screen',
    )


def localization_filter_node(pkg_share: str, namespace: str = '', filter_type: str = 'ekf'):
    """robot_localization EKF or UKF fusing wheel odom + IMU; sole odom->base_link
    TF broadcaster (the gz DiffDrive/Mecanum/Ackermann plugins are routed
    to an unbridged tf_wheel_raw topic instead — see gazebo_control*.xacro).

    ekf_node and ukf_node share the exact same fusion-config schema (odomN_config,
    imuN_config, ...) — ukf.yaml is ekf.yaml's fusion setup plus UKF-only sigma-point
    tuning (alpha/kappa/beta), so swapping filter_type is just executable + params file.
    """
    filter_type = (filter_type or 'ekf').strip().lower()
    if filter_type not in ('ekf', 'ukf'):
        print(f'[rosnav_bot] localization_filter={filter_type!r} not recognized — using ekf')
        filter_type = 'ekf'
    odom_frame = f'{namespace}/odom' if namespace else 'odom'
    params = {
        'use_sim_time': True,
        'odom_frame': odom_frame,
        'base_link_frame': f'{namespace}/base_link' if namespace else 'base_link',
        # robot_localization publishes /tf and the output Odometry's header.frame_id
        # using world_frame, not odom_frame — they must match here (no absolute/map
        # source is fused) or the published edge becomes an unnamespaced "odom" node
        # that collides across robots instead of "{ns}/odom".
        'world_frame': odom_frame,
    }
    return Node(
        package='robot_localization',
        executable=f'{filter_type}_node',
        name=f'{filter_type}_filter_node',
        namespace=namespace or None,
        parameters=[os.path.join(pkg_share, 'config', f'{filter_type}.yaml'), params],
        output='screen',
    )


def gz_bridge_yaml(lidar_type: str = '2d', enable_rgbd: bool = False) -> str:
    """Pick the ros_gz_bridge config for 2D/3D lidar × RGB/RGB-D."""
    is_3d = lidar_type == '3d'
    if is_3d and enable_rgbd:
        return 'gz_bridge_3d_rgbd.yaml'
    if is_3d:
        return 'gz_bridge_3d.yaml'
    if enable_rgbd:
        return 'gz_bridge_rgbd.yaml'
    return 'gz_bridge.yaml'


def pointcloud_to_scan_node(namespace: str = '', *, min_height=0.05, max_height=0.55,
                            range_max=30.0):
    """Project /points (3D lidar) → scan_raw so laser_filters + Nav2 keep /scan.

    Prefers ros-humble-pointcloud-to-laserscan when installed; otherwise uses
    rosnav_bot's pointcloud_to_scan.py.
    """
    remappings = [('cloud_in', 'points'), ('scan', 'scan_raw')]
    params = {
        'use_sim_time': True,
        'min_height': float(min_height),
        'max_height': float(max_height),
        'range_min': 0.30,
        'range_max': float(range_max),
        'angle_min': -3.14159,
        'angle_max': 3.14159,
        'angle_increment': 3.14159 / 180.0,
        'scan_time': 0.1,
        'use_inf': True,
    }
    try:
        get_package_share_directory('pointcloud_to_laserscan')
        return Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_scan',
            namespace=namespace or None,
            output='screen',
            parameters=[params],
            remappings=remappings,
        )
    except Exception:
        return Node(
            package='rosnav_bot',
            executable='pointcloud_to_scan.py',
            name='pointcloud_to_scan',
            namespace=namespace or None,
            output='screen',
            parameters=[params],
            remappings=remappings,
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


def spawn_dynamic_obstacle_node(pkg_share: str, gazebo_world_name: str, name, x, y, z, yaw):
    """Spawn a models/dynamic_obstacle instance from its SDF file directly
    (no robot_description topic to spawn from, unlike spawn_robot_node)."""
    model_path = os.path.join(pkg_share, 'models', 'dynamic_obstacle', 'model.sdf')
    return Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_{name}',
        arguments=[
            '-world', gazebo_world_name,
            '-file', model_path,
            '-name', name,
            '-x', str(x), '-y', str(y), '-z', str(z), '-Y', str(yaw),
        ],
        output='screen',
    )


def dynamic_obstacle_bridge_node(name: str):
    """ROS -> Gazebo Twist bridge feeding the model's VelocityControl plugin."""
    return Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'{name}_cmd_vel_bridge',
        arguments=[f'/model/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        output='screen',
    )


def dynamic_obstacle_driver_node(name: str, axis: str, amplitude, speed):
    return Node(
        package='rosnav_bot',
        executable='dynamic_obstacle_driver.py',
        name=f'{name}_driver',
        output='screen',
        parameters=[{
            'obstacle_name': name,
            'axis': axis,
            'amplitude': float(amplitude),
            'speed': float(speed),
        }],
        remappings=[('cmd_vel', f'/model/{name}/cmd_vel')],
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


def package_available(name: str) -> bool:
    try:
        get_package_share_directory(name)
        return True
    except Exception:
        return False


def prepare_cartographer_config_dir(pkg_share: str) -> str:
    """Merge cartographer_ros lua includes with rosnav_bot's config/cartographer.

    cartographer_node only accepts one -configuration_directory, but our
    rosnav_*.lua files `include "map_builder.lua"` etc. from the upstream
    package. Copy upstream .lua files into a writable runtime dir and overlay
    our basename on top.
    """
    import shutil
    import tempfile

    our_dir = os.path.join(pkg_share, 'config', 'cartographer')
    upstream = os.path.join(
        get_package_share_directory('cartographer_ros'), 'configuration_files')
    runtime = tempfile.mkdtemp(prefix='rosnav_cartographer_')
    for name in os.listdir(upstream):
        if name.endswith('.lua'):
            shutil.copy2(os.path.join(upstream, name), os.path.join(runtime, name))
    for name in os.listdir(our_dir):
        if name.endswith('.lua'):
            shutil.copy2(os.path.join(our_dir, name), os.path.join(runtime, name))
    return runtime


def cartographer_slam_nodes(pkg_share: str, *, use_imu: bool = True,
                            resolution: float = 0.05):
    """Google Cartographer 2D SLAM + occupancy grid publisher (/map)."""
    cfg_dir = prepare_cartographer_config_dir(pkg_share)
    basename = 'rosnav_2d.lua' if use_imu else 'rosnav_2d_no_imu.lua'
    return [
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '-configuration_directory', cfg_dir,
                '-configuration_basename', basename,
            ],
            remappings=[('scan', '/scan'), ('imu', '/imu'), ('odom', '/odom')],
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': True, 'resolution': float(resolution)}],
        ),
    ]


def rtabmap_multisensor_node(*, namespace='', lidar_type='2d', octomap=False,
                             database_path='', wait_for_transform=0.2):
    """RTAB-Map fusing RGB-D + lidar (scan or point cloud) for mapping.

    Wheel odom (fused with IMU by ekf_filter_node) remains the sole
    odom→base_link publisher. ICP registration
    uses the lidar; RGB-D adds visual loop-closure + depth occupancy.
    """
    ns = namespace.strip('/') if namespace else ''
    prefix = f'/{ns}' if ns else ''
    frame_prefix = f'{ns}/' if ns else ''
    database_path = os.path.expanduser(database_path) if database_path else ''
    use_cloud = lidar_type == '3d'

    params = {
        'use_sim_time': True,
        'frame_id': f'{frame_prefix}base_link',
        'odom_frame_id': f'{frame_prefix}odom',
        'map_frame_id': f'{frame_prefix}map',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan': not use_cloud,
        'subscribe_scan_cloud': use_cloud,
        'approx_sync': True,
        'wait_for_transform': float(wait_for_transform),
        'qos_image': 1,
        'qos_camera_info': 1,
        'qos': 1,
        'Rtabmap/TimeThr': '700',
        'Reg/Strategy': '1',       # ICP (lidar); RGB for loop-closure cues
        'Icp/PointToPlane': 'true',
        'Grid/Sensor': '2',        # both laser/cloud and depth
        'Grid/3D': 'true' if octomap else 'false',
        'Grid/CellSize': '0.05',
        'Grid/RangeMax': '12.0',
        'Grid/RayTracing': 'true',
        'Grid/NoiseFilteringRadius': '0.1',
        'Grid/NoiseFilteringMinNeighbors': '5',
        'Mem/IncrementalMemory': 'true',
    }
    if database_path:
        params['database_path'] = database_path

    remappings = [
        ('odom', f'{prefix}/odom' if ns else '/odom'),
        ('rgb/image', f'{prefix}/camera/image_raw' if ns else '/camera/image_raw'),
        ('rgb/camera_info',
         f'{prefix}/camera/camera_info' if ns else '/camera/camera_info'),
        ('depth/image',
         f'{prefix}/camera/depth/image_raw' if ns else '/camera/depth/image_raw'),
    ]
    if use_cloud:
        remappings.append(
            ('scan_cloud', f'{prefix}/points' if ns else '/points'))
    else:
        remappings.append(('scan', f'{prefix}/scan' if ns else '/scan'))
    if ns:
        remappings.extend([
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
            ('map', f'/{ns}/map'),
            ('/map', f'/{ns}/map'),
        ])

    arguments = [] if database_path else ['-d']
    kwargs = {
        'package': 'rtabmap_slam',
        'executable': 'rtabmap',
        'name': 'rtabmap',
        'output': 'screen',
        'parameters': [params],
        'remappings': remappings,
        'arguments': arguments,
    }
    if ns:
        kwargs['namespace'] = ns
    return Node(**kwargs)


_EXPLORER_ALIASES = {
    'builtin': 'builtin',
    'rosnav': 'builtin',
    'wfd': 'builtin',
    'explore_lite': 'explore_lite',
    'm-explore': 'explore_lite',
    'm_explore': 'explore_lite',
    'm-explore-ros2': 'explore_lite',
    'frontier': 'frontier_exploration_ros2',
    'frontier_exploration': 'frontier_exploration_ros2',
    'frontier_exploration_ros2': 'frontier_exploration_ros2',
    'mrtsp': 'frontier_exploration_ros2',
    'rrt': 'rrt_explore',
    'rrt_explore': 'rrt_explore',
    'rrt-explore': 'rrt_explore',
}

_EXPLORER_PACKAGES = {
    'explore_lite': 'explore_lite',
    'frontier_exploration_ros2': 'frontier_exploration_ros2',
    'rrt_explore': 'rrt_explore',
}


def resolve_explorer(explorer: str) -> str:
    """Map explorer:= aliases to a backend; fall back to builtin if the package is missing."""
    backend = _EXPLORER_ALIASES.get((explorer or 'builtin').strip().lower())
    if not backend:
        print(f'[rosnav] unknown explorer={explorer!r} — using builtin')
        return 'builtin'
    pkg = _EXPLORER_PACKAGES.get(backend)
    if pkg and not package_available(pkg):
        print(f'[rosnav] explorer={backend} requested but `{pkg}` is not built — using builtin. '
              f'Link + build: bash src/rosnav_bot/scripts/link_third_party.sh && '
              f'colcon build --symlink-install --packages-up-to {pkg}')
        return 'builtin'
    return backend


def explorer_nodes(backend, pkg_share, *, map_topic='/map', namespaces=None,
                   robot_count=1, builtin_params=None):
    """Nodes for explorer:=builtin|explore_lite|frontier_exploration_ros2|rrt."""
    namespaces = list(namespaces) if namespaces else ['']
    if backend == 'builtin':
        params = dict(builtin_params or {})
        params.setdefault('use_sim_time', True)
        if map_topic:
            params.setdefault('map_topic', map_topic)
        return [Node(
            package='rosnav_bot',
            executable='frontier_explorer.py',
            name='frontier_explorer',
            output='screen',
            parameters=[params],
        )]

    if backend == 'rrt_explore':
        ns0 = namespaces[0] if namespaces and namespaces[0] else ''
        costmap = f'/{ns0}/global_costmap/costmap' if ns0 else '/global_costmap/costmap'
        # rrt_explore's C++ node parses a robot ID out of its own ROS
        # namespace assuming a "robotN" prefix (get_ros_parameters(),
        # rrt.cpp:517) — with no namespace at all (single-robot mode) that
        # parse throws, get_ros_parameters() returns false, and the
        # constructor bails before setting up any subscriptions/timers
        # (rrt.cpp:45): the node stays alive but never explores. Every param
        # below is already absolute-pathed (map_topic/costmap_topic start
        # with '/', robot_base_frame is a TF frame name, not a topic), so
        # giving it a synthetic "robot1" namespace just satisfies that parse
        # without remapping anything else.
        rrt_namespace = ns0 or 'robot1'
        # rrt.cpp used to hardcode its NavigateToPose action client to the
        # plain name "navigate_to_pose", so ROS2 resolved it relative to the
        # node's own namespace — "/robot1/navigate_to_pose" here. That's
        # correct in real multi-robot mode (ns0 set, each robot's own
        # namespaced bt_navigator actually serves that action there). But
        # when ns0 is empty (single-robot; 'robot1' above is purely synthetic
        # to satisfy rrt.cpp's namespace parse), bt_navigator itself is NOT
        # namespaced — it only serves the global "/navigate_to_pose" — so
        # rrt's action client silently talked to a server that doesn't exist
        # and async_send_goal() never completed (confirmed live 2026-08-26:
        # "Goal: x, y" logs, then nothing — no goal ever reaches
        # bt_navigator/controller_server, no motion).
        #
        # A launch-level `remappings=[('navigate_to_pose', '/navigate_to_pose')]`
        # was tried first and does NOT work for this action name — confirmed
        # live: `ros2 action info` still showed this node as a client of the
        # namespaced action even with that remap applied via the process's
        # own `-r navigate_to_pose:=/navigate_to_pose` CLI arg. Fixed
        # properly instead by patching rrt.cpp (vendored at
        # ~/rosnav_sources/rrt-explore, not apt-installed) to read the
        # action name from a new `nav_to_pose_action_name` parameter instead
        # of a hardcoded string, and passing the real absolute name here.
        nav_to_pose_action_name = ns0 and f'/{ns0}/navigate_to_pose' or '/navigate_to_pose'
        return [Node(
            package='rrt_explore',
            executable='rrt',
            name='rrt',
            namespace=rrt_namespace,
            output='screen',
            parameters=[os.path.join(pkg_share, 'config', 'rrt_explore.yaml'), {
                'use_sim_time': True,
                'map_topic': map_topic,
                'costmap_topic': costmap,
                'robot_base_frame': 'base_link',
                'robot_frame_prefix': 'robot',
                'robot_count': int(robot_count),
                'nav_to_pose_action_name': nav_to_pose_action_name,
            }],
        )]

    nodes = []
    for ns in namespaces:
        prefix = f'/{ns}' if ns else ''
        base_frame = f'{ns}/base_link' if ns else 'base_link'
        if backend == 'explore_lite':
            nodes.append(Node(
                package='explore_lite',
                executable='explore',
                name='explore_node',
                namespace=ns,
                output='screen',
                parameters=[os.path.join(pkg_share, 'config', 'explore_lite.yaml'), {
                    'use_sim_time': True,
                    'robot_base_frame': base_frame,
                    'costmap_topic': map_topic,
                }],
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')] if ns else [],
            ))
        elif backend == 'frontier_exploration_ros2':
            nodes.append(Node(
                package='frontier_exploration_ros2',
                executable='frontier_explorer',
                name='frontier_explorer',
                namespace=ns,
                output='screen',
                parameters=[os.path.join(pkg_share, 'config', 'frontier_exploration.yaml'), {
                    'use_sim_time': True,
                    'map_topic': map_topic,
                    'costmap_topic': f'{prefix}/global_costmap/costmap' if ns else '/global_costmap/costmap',
                    'local_costmap_topic': f'{prefix}/local_costmap/costmap' if ns else '/local_costmap/costmap',
                    'robot_base_frame': base_frame,
                    'navigate_to_pose_action_name': 'navigate_to_pose',
                    'autostart': True,
                }],
            ))
    return nodes


def cslam_lidar_nodes(pkg_share, *, robot_id=0, max_nb_robots=1, namespace=''):
    """Swarm-SLAM (C-SLAM) lidar frontend + pose-graph backend."""
    cfg = os.path.join(pkg_share, 'config', 'cslam_lidar.yaml')
    extra = {
        'robot_id': int(robot_id),
        'max_nb_robots': int(max_nb_robots),
        'use_sim_time': True,
    }
    return [
        Node(
            package='cslam',
            executable='loop_closure_detection_node.py',
            name='cslam_loop_closure_detection',
            namespace=namespace,
            output='screen',
            parameters=[cfg, extra]),
        Node(
            package='cslam',
            executable='lidar_handler_node.py',
            name='cslam_map_manager',
            namespace=namespace,
            output='screen',
            parameters=[cfg, extra]),
        Node(
            package='cslam',
            executable='pose_graph_manager',
            name='cslam_pose_graph_manager',
            namespace=namespace,
            output='screen',
            parameters=[cfg, extra]),
    ]


def multi_robot_rgbd_bridge_args(namespace: str):
    """Return (bridge_arguments, remappings) for one namespaced RGB-D camera."""
    ns = namespace.strip('/')
    arguments = [
        f'/{ns}/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        f'/{ns}/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
        f'/{ns}/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        f'/{ns}/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
    ]
    remappings = [
        (f'/{ns}/camera/image', f'/{ns}/camera/image_raw'),
        (f'/{ns}/camera/depth_image', f'/{ns}/camera/depth/image_raw'),
        (f'/{ns}/camera/points', f'/{ns}/camera/depth/points'),
    ]
    return arguments, remappings


def rtabmap_vslam_node(*, namespace='', localization=False, database_path='',
                       wait_for_transform=0.2, octomap=False):
    """RTAB-Map RGB-D visual SLAM / localization (slam_algo:=vslam).

    Uses wheel odom TF (no visual odometry node) so ekf_filter_node (fusing
    wheel odom + IMU) remains the sole odom→base_link publisher — same
    pattern as slam_algo:=3d.
    Occupancy grid comes from the depth camera (Grid/Sensor=1).
    """
    ns = namespace.strip('/') if namespace else ''
    prefix = f'/{ns}' if ns else ''
    frame_prefix = f'{ns}/' if ns else ''
    database_path = os.path.expanduser(database_path) if database_path else ''

    if localization and not database_path:
        raise ValueError('rtabmap_vslam localization requires database_path')

    params = {
        'use_sim_time': True,
        'frame_id': f'{frame_prefix}base_link',
        'odom_frame_id': f'{frame_prefix}odom',
        'map_frame_id': f'{frame_prefix}map',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan_cloud': False,
        'approx_sync': True,
        'wait_for_transform': float(wait_for_transform),
        # ros_gz_bridge publishes camera topics RELIABLE; RTAB defaults to
        # BEST_EFFORT and never matches — force Reliable (1).
        'qos_image': 1,
        'qos_camera_info': 1,
        'qos': 1,
        # Without a time budget, RTAB-Map's per-cycle cost grows unbounded as
        # working memory grows (observed: 0.04s -> 2.2s over ~45 nodes),
        # falling behind real time and smearing the map->odom correction.
        'Rtabmap/TimeThr': '700',
        'Reg/Strategy': '0',       # visual (RGB-D) registration
        'Grid/Sensor': '1',        # occupancy from depth camera
        'Grid/3D': 'true' if octomap else 'false',
        'Grid/CellSize': '0.05',
        'Grid/RangeMax': '8.0',
        'Grid/RayTracing': 'true',
        'Mem/IncrementalMemory': 'false' if localization else 'true',
        'Mem/InitWMWithAllNodes': 'true' if localization else 'false',
    }
    if database_path:
        params['database_path'] = database_path

    remappings = [
        ('odom', f'{prefix}/odom' if ns else '/odom'),
        ('rgb/image', f'{prefix}/camera/image_raw' if ns else '/camera/image_raw'),
        ('rgb/camera_info',
         f'{prefix}/camera/camera_info' if ns else '/camera/camera_info'),
        ('depth/image',
         f'{prefix}/camera/depth/image_raw' if ns else '/camera/depth/image_raw'),
    ]
    if ns:
        remappings.extend([
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
            ('map', f'/{ns}/map'),
            ('/map', f'/{ns}/map'),
        ])

    # Fresh DB each mapping run when no persist path; otherwise open/create
    # database_path. Localization never uses -d (would wipe the map).
    if localization or database_path:
        arguments = []
    else:
        arguments = ['-d']

    kwargs = {
        'package': 'rtabmap_slam',
        'executable': 'rtabmap',
        'name': 'rtabmap',
        'output': 'screen',
        'parameters': [params],
        'remappings': remappings,
        'arguments': arguments,
    }
    if ns:
        kwargs['namespace'] = ns
    return Node(**kwargs)
