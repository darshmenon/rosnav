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
        return [Node(
            package='rrt_explore',
            executable='rrt',
            name='rrt',
            namespace=ns0,
            output='screen',
            parameters=[os.path.join(pkg_share, 'config', 'rrt_explore.yaml'), {
                'use_sim_time': True,
                'map_topic': map_topic,
                'costmap_topic': costmap,
                'robot_base_frame': 'base_link',
                'robot_frame_prefix': 'robot',
                'robot_count': int(robot_count),
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
    ]
    remappings = [
        (f'/{ns}/camera/image', f'/{ns}/camera/image_raw'),
        (f'/{ns}/camera/depth_image', f'/{ns}/camera/depth/image_raw'),
    ]
    return arguments, remappings


def rtabmap_vslam_node(*, namespace='', localization=False, database_path='',
                       wait_for_transform=0.2, octomap=False):
    """RTAB-Map RGB-D visual SLAM / localization (slam_algo:=vslam).

    Uses wheel odom TF (no visual odometry node) so Gazebo DiffDrive remains
    the sole odom→base_link publisher — same pattern as slam_algo:=3d.
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
