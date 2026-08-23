"""
slam_nav.launch.py
==================
One-shot launch: Gazebo + Robot + Nav2 + RViz.

SLAM mode (default):
  ros2 launch rosnav_bot slam_nav.launch.py world_name:=hospital explore:=true

RGB-D visual SLAM (RTAB-Map, textured worlds recommended):
  ros2 launch rosnav_bot slam_nav.launch.py world_name:=cafe \\
    slam_algo:=vslam rtabmap_db:=src/rosnav_bot/maps/rtabmap_cafe.db

VSLAM localization + Nav2 (no AMCL; loads the RTAB .db):
  ros2 launch rosnav_bot slam_nav.launch.py world_name:=cafe \\
    slam:=false slam_algo:=vslam rtabmap_db:=src/rosnav_bot/maps/rtabmap_cafe.db

Nav-on-map mode (pre-built OccupancyGrid, AMCL localization):
  ros2 launch rosnav_bot slam_nav.launch.py world_name:=hospital slam:=false

When slam:=false and slam_algo is not vslam, the launch uses
map_<world_name>.yaml from the maps/ directory with AMCL.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common  # noqa: E402  (shared with multi_robot.launch.py)

ROS_DISTRO = _common.ROS_DISTRO


def _resolve_world_name(raw_name: str, world_path: str) -> str:
    if raw_name:
        return os.path.splitext(os.path.basename(raw_name))[0]
    return os.path.splitext(os.path.basename(world_path))[0]


def _resolve_world_path(world_name_arg: str, world_arg: str, pkg_share: str) -> str:
    world_arg = world_arg.strip()
    if world_arg:
        return os.path.expanduser(world_arg)
    world_name = os.path.splitext(os.path.basename(world_name_arg.strip() or 'cafe'))[0]
    return os.path.join(pkg_share, 'worlds', f'{world_name}.world')


def _resolve_map_prefix(map_prefix_arg: str, world_name: str, pkg_share: str) -> str:
    if map_prefix_arg:
        return os.path.expanduser(map_prefix_arg)
    return os.path.join(pkg_share, 'maps', f'map_{world_name}')


def _resolve_map_yaml(world_name: str, pkg_share: str) -> str:
    candidates = [
        os.path.join(pkg_share, 'maps', f'map_{world_name}.yaml'),
        os.path.join(pkg_share, 'maps', f'{world_name}_map.yaml'),
    ]
    for c in candidates:
        if os.path.exists(c):
            return c
    return candidates[0]


# Per-world spawn points verified live to avoid known SLAM failure modes at
# the generic default (1.5, 1.0): 'outdoor's flat bowl-center default is out
# of lidar range of any terrain (map stays 0x0 forever); 'multi_terrain's
# open flat pad gives RTAB-Map's ICP nothing to lock onto, so drift shows up
# as false obstacles that box the robot in near spawn. Only applied when the
# corresponding spawn_* argument is left at its 'auto' sentinel, so an
# explicit spawn_x/y/z/yaw override always wins.
_WORLD_SPAWN_DEFAULTS = {
    'outdoor': {'x': '40.0', 'y': '0.0', 'z': '9.0', 'yaw': '0.0'},
    'multi_terrain': {'x': '2.2', 'y': '-2.0', 'z': '0.3', 'yaw': '0.0'},
    # Open aisle near west wall of OpenRobotics Depot (~30×15 m)
    'warehouse_depot': {'x': '-11.0', 'y': '0.0', 'z': '0.3', 'yaw': '0.0'},
    # Living-room floor of Lake House (interior ~z=6.90)
    'lake_house': {'x': '1.2', 'y': '0.8', 'z': '7.2', 'yaw': '0.0'},
    # Center aisle of AWS RoboMaker warehouse (away from clutter at y>2)
    'aws_warehouse': {'x': '0.5', 'y': '0.0', 'z': '0.3', 'yaw': '0.0'},
    # Open floor between shelf rows in Tugbot warehouse
    'tugbot_warehouse': {'x': '0.0', 'y': '12.0', 'z': '0.3', 'yaw': '3.14'},
    # Cafe interior, looking down the seating aisle
    'cafe': {'x': '0.0', 'y': '-2.0', 'z': '0.3', 'yaw': '-1.57'},
}
_GENERIC_SPAWN_DEFAULT = {'x': '1.5', 'y': '1.0', 'z': '0.3', 'yaw': '0.0'}


def _resolve_spawn(world_name: str, x_arg: str, y_arg: str, z_arg: str, yaw_arg: str):
    defaults = _WORLD_SPAWN_DEFAULTS.get(world_name, _GENERIC_SPAWN_DEFAULT)
    return (
        defaults['x'] if x_arg == 'auto' else x_arg,
        defaults['y'] if y_arg == 'auto' else y_arg,
        defaults['z'] if z_arg == 'auto' else z_arg,
        defaults['yaw'] if yaw_arg == 'auto' else yaw_arg,
    )


def _build_runtime_actions(context, pkg_share: str):
    world_name_arg = LaunchConfiguration('world_name').perform(context)
    world_arg = LaunchConfiguration('world').perform(context)
    rviz = LaunchConfiguration('rviz')
    headless = LaunchConfiguration('headless')
    explore = LaunchConfiguration('explore')
    slam_arg = LaunchConfiguration('slam').perform(context).lower()
    use_slam = slam_arg in ('true', '1', 'yes')
    robot_name = LaunchConfiguration('robot_name')
    controller = LaunchConfiguration('controller').perform(context).strip().lower()
    drive_type = LaunchConfiguration('drive_type').perform(context).strip().lower()
    robot_model = LaunchConfiguration('robot_model').perform(context).strip().lower()
    lidar_type = LaunchConfiguration('lidar_type').perform(context).strip().lower()
    if lidar_type == '3d' and drive_type != 'diff':
        print(f'[slam_nav] lidar_type=3d only supported for drive_type:=diff '
              f'(got drive_type:={drive_type}) — falling back to 2d')
        lidar_type = '2d'
    lidar3d_height = LaunchConfiguration('lidar3d_height').perform(context).strip()
    lidar3d_vfov_deg = LaunchConfiguration('lidar3d_vfov_deg').perform(context).strip()
    octomap = _common.truthy(LaunchConfiguration('octomap').perform(context))

    slam_algo = LaunchConfiguration('slam_algo').perform(context).strip().lower()
    rtabmap_db = os.path.expanduser(
        LaunchConfiguration('rtabmap_db').perform(context).strip())
    run_cslam = False
    if slam_algo == 'cslam':
        if lidar_type != '3d':
            print('[slam_nav] slam_algo=cslam requires lidar_type:=3d — falling back to slam_algo=2d')
            slam_algo = '2d'
        elif not _common.package_available('cslam'):
            print('[slam_nav] slam_algo=cslam requested but `cslam` is not built '
                  '(bash src/rosnav_bot/scripts/link_third_party.sh --cslam && '
                  'colcon build --packages-up-to cslam) — falling back')
            slam_algo = '3d' if _common.package_available('rtabmap_slam') else '2d'
        else:
            run_cslam = True
            slam_algo = '3d'
    if slam_algo == 'vslam':
        if not _common.package_available('rtabmap_slam'):
            print('[slam_nav] slam_algo=vslam requested but ros-humble-rtabmap-ros is not '
                  'installed (sudo apt install ros-humble-rtabmap-ros) — falling back to slam_algo=2d')
            slam_algo = '2d'
        elif not use_slam and not rtabmap_db:
            print('[slam_nav] slam:=false slam_algo:=vslam needs rtabmap_db:=/path/to.db — '
                  'falling back to AMCL on OccupancyGrid map')
            slam_algo = '2d'
        elif not use_slam and rtabmap_db and not os.path.isfile(rtabmap_db):
            print(f'[slam_nav] rtabmap_db not found: {rtabmap_db} — falling back to AMCL')
            slam_algo = '2d'
        elif use_slam and rtabmap_db:
            os.makedirs(os.path.dirname(os.path.abspath(rtabmap_db)) or '.', exist_ok=True)
    if slam_algo == 'multisensor':
        if not _common.package_available('rtabmap_slam'):
            print('[slam_nav] slam_algo=multisensor needs ros-humble-rtabmap-ros — '
                  'falling back to slam_algo=2d')
            slam_algo = '2d'
        elif use_slam and rtabmap_db:
            os.makedirs(os.path.dirname(os.path.abspath(rtabmap_db)) or '.', exist_ok=True)
    if slam_algo in ('cartographer', 'carto'):
        slam_algo = 'cartographer'
        if not _common.package_available('cartographer_ros'):
            print('[slam_nav] slam_algo=cartographer needs ros-humble-cartographer-ros '
                  '(sudo apt install ros-humble-cartographer-ros) — falling back to slam_algo=2d')
            slam_algo = '2d'
    if slam_algo == '3d' and lidar_type != '3d':
        print('[slam_nav] slam_algo=3d requires lidar_type:=3d — falling back to slam_algo=2d')
        slam_algo = '2d'
    if slam_algo == '3d':
        try:
            get_package_share_directory('rtabmap_slam')
        except Exception:
            print('[slam_nav] slam_algo=3d requested but ros-humble-rtabmap-ros is not '
                  'installed (sudo apt install ros-humble-rtabmap-ros) — falling back to slam_algo=2d')
            slam_algo = '2d'
            run_cslam = False

    world_path = _resolve_world_path(world_name_arg, world_arg, pkg_share)
    world_name = _resolve_world_name(world_name_arg, world_path)
    gazebo_world_name = _common.resolve_gazebo_world_name(world_path)

    spawn_x, spawn_y, spawn_z, spawn_yaw = _resolve_spawn(
        world_name,
        LaunchConfiguration('spawn_x').perform(context).strip(),
        LaunchConfiguration('spawn_y').perform(context).strip(),
        LaunchConfiguration('spawn_z').perform(context).strip(),
        LaunchConfiguration('spawn_yaw').perform(context).strip(),
    )
    map_prefix = _resolve_map_prefix(
        LaunchConfiguration('map_prefix').perform(context).strip(), world_name, pkg_share)
    map_override = LaunchConfiguration('map_override').perform(context).strip()
    # A gs_mask_from_splat.py KeepoutFilter mask is already a plain Nav2 map
    # (trinary 0=occupied/254=free PGM+YAML) — same format map_server/
    # static_layer expect, so it can seed nav-on-map mode directly as a
    # GS-derived floor-plan prior instead of only subtracting space via
    # gs_keepout_mask's filter overlay. See concepts.md §31.
    map_yaml = os.path.expanduser(map_override) if map_override else _resolve_map_yaml(world_name, pkg_share)

    enable_camera_arg = LaunchConfiguration('enable_camera').perform(context).strip().lower()
    enable_rgbd_arg = LaunchConfiguration('enable_rgbd').perform(context).strip().lower()
    enable_yolo_arg = LaunchConfiguration('enable_yolo').perform(context).strip().lower()
    # yolo_detector.py has nothing to detect on without the camera, so
    # enable_yolo:=true pulls it in even if enable_camera wasn't set explicitly.
    # slam_algo=3d also wants it: RTAB-Map's own bag-of-words loop closure
    # needs an RGB image on top of the lidar cloud (lidar ICP alone can
    # alias in geometrically repetitive aisles/corridors).
    # slam_algo=vslam|multisensor forces RGB-D; enable_rgbd:=true does the same for
    # depth-aware costmaps without switching SLAM.
    enable_rgbd = 'true' if (
        slam_algo in ('vslam', 'multisensor') or _common.truthy(enable_rgbd_arg)) else 'false'
    enable_camera = 'true' if (
        _common.truthy(enable_camera_arg) or _common.truthy(enable_yolo_arg)
        or slam_algo in ('3d', 'multisensor') or enable_rgbd == 'true') else 'false'
    urdf_filename = _common.urdf_filename_for(drive_type, robot_model)
    rsp = _common.rsp_include(
        pkg_share, os.path.join(pkg_share, 'urdf', urdf_filename), lidar_type=lidar_type,
        lidar3d_height=lidar3d_height, lidar3d_vfov_deg=lidar3d_vfov_deg,
        enable_camera=enable_camera, enable_rgbd=enable_rgbd)

    gazebo_server = _common.gazebo_server_action(world_path, pkg_share)

    gazebo_client = GroupAction(
        condition=UnlessCondition(headless),
        actions=[_common.gazebo_client_action(pkg_share)],
    )

    spawn_robot = _common.spawn_robot_node(
        gazebo_world_name, 'robot_description', robot_name, spawn_x, spawn_y, spawn_z, spawn_yaw)

    bridge_yaml = _common.gz_bridge_yaml(lidar_type, enable_rgbd == 'true')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={os.path.join(pkg_share, "config", bridge_yaml)}',
        ],
    )

    # Lidar filter chain: raw Gazebo scan (/scan_raw) in, cleaned /scan out.
    # With lidar_type:=3d, pointcloud_to_scan projects /points → /scan_raw first.
    # Optional scan_quality_gate drops malformed frames before SLAM/Nav2 see them.
    scan_gate = _common.truthy(LaunchConfiguration('scan_gate').perform(context))
    laser_filter = _common.laser_filter_node(
        pkg_share, filtered_out=('scan_pre' if scan_gate else 'scan'))
    scan_quality_gate = (
        _common.scan_quality_gate_node() if scan_gate else None)
    # Always on, not a toggle: the gz drive plugin's own TF is routed off
    # /tf (see gazebo_control*.xacro), so ekf_filter_node is the only thing
    # that can publish odom->base_link — without it nothing does.
    ekf_filter = _common.ekf_node(pkg_share)
    points_to_scan = (
        _common.pointcloud_to_scan_node() if lidar_type == '3d' else None)

    # Patch the BT path placeholder before passing params to nav2.
    nav2_params_name = _common.nav2_params_filename(controller, drive_type, ROS_DISTRO)
    _raw_params = os.path.join(pkg_share, 'config', nav2_params_name)
    _params_file = _common.patch_pkg_share_placeholder(_raw_params, pkg_share)

    use_vslam_localize = (not use_slam) and slam_algo == 'vslam'

    if use_slam and slam_algo == 'vslam':
        slam_or_localization = TimerAction(
            period=5.0,
            actions=[_common.rtabmap_vslam_node(
                localization=False,
                database_path=rtabmap_db,
                octomap=octomap,
            )],
        )
    elif use_slam and slam_algo == 'multisensor':
        slam_or_localization = TimerAction(
            period=5.0,
            actions=[_common.rtabmap_multisensor_node(
                lidar_type=lidar_type,
                octomap=octomap,
                database_path=rtabmap_db,
            )],
        )
    elif use_slam and slam_algo == 'cartographer':
        slam_or_localization = TimerAction(
            period=5.0,
            actions=_common.cartographer_slam_nodes(pkg_share, use_imu=True),
        )
    elif use_slam and slam_algo == '3d':
        # RTAB-Map lidar SLAM: consumes the *existing* wheel /odom directly (like
        # slam_toolbox does) instead of running RTAB-Map's own icp_odometry — that
        # keeps this a drop-in swap with exactly one odom->base_link TF publisher
        # (ekf_filter_node, fusing wheel odom + IMU), avoiding a two-parent TF
        # conflict. RTAB-Map only adds the map->odom layer on top, same as
        # slam_toolbox/AMCL today.
        slam_or_localization = TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='rtabmap_slam',
                    executable='rtabmap',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'frame_id': 'base_link',
                        'odom_frame_id': 'odom',
                        'map_frame_id': 'map',
                        'subscribe_depth': False,
                        # RGB (no depth) from the existing docking/YOLO camera: gives
                        # RTAB-Map's bag-of-words detector real image features for loop
                        # closure hypotheses, on top of the lidar cloud. Registration
                        # (the actual transform estimate) still comes from ICP below —
                        # the image is only used to *recognize* a place, not to localize
                        # against it, so this doesn't need camera-lidar extrinsic calib
                        # beyond the URDF's static TF.
                        'subscribe_rgb': True,
                        'subscribe_scan_cloud': True,
                        'approx_sync': True,
                        'wait_for_transform': 0.2,
                        # Match ros_gz_bridge RELIABLE camera pubs (1=Reliable).
                        'qos_image': 1,
                        'qos_camera_info': 1,
                        'qos': 1,
                        # RTAB-Map's internal parameters are strings:
                        'Reg/Strategy': '1',           # ICP registration; RGB is loop-closure-only
                        'Icp/PointToPlane': 'true',
                        'Grid/Sensor': '0',             # occupancy grid from the lidar cloud, not depth
                        # RTAB-Map still projects a 2D nav_msgs/OccupancyGrid onto /map for
                        # Nav2 either way; Grid/3D only switches whether it *also* assembles
                        # the full 3D voxel grid that /octomap_binary and /octomap_full are
                        # built from (empty topics when false — see octomap:=true arg).
                        'Grid/3D': 'true' if octomap else 'false',
                        'Grid/CellSize': '0.05',
                        'Grid/RangeMax': '20.0',
                        # NoiseFilteringRadius defaults to 0 (disabled); the 16-channel 3D
                        # lidar's sparse vertical resolution makes normal-based ground
                        # segmentation (Grid/NormalsSegmentation) noisy, sprinkling spurious
                        # "obstacle" cells across otherwise-flat ground. Filter isolated points
                        # before classification.
                        'Grid/NoiseFilteringRadius': '0.1',
                        'Grid/NoiseFilteringMinNeighbors': '5',
                        # Carve FREE cells along sensor->ground-hit rays instead of only
                        # marking cells with a direct point hit — otherwise the area right
                        # around the robot (rarely hit directly at this lidar's sparse
                        # vertical resolution) stays UNKNOWN/OCCUPIED.
                        'Grid/RayTracing': 'true',
                        # Filter small isolated "obstacle" clusters (ground-plane noise at
                        # this vertical resolution) instead of letting them poison the map.
                        'Grid/ClusterRadius': '0.2',
                        'Grid/MinClusterSize': '20',
                        'Mem/IncrementalMemory': 'true',  # mapping mode (vs. localization)
                    }],
                    remappings=[('odom', '/odom'), ('scan_cloud', '/points'),
                                ('rgb/image', '/camera/image_raw'),
                                ('rgb/camera_info', '/camera/camera_info')],
                    arguments=['-d'],  # fresh database each run — no stale map from a previous session
                )
            ],
        )
    elif use_slam:
        slam_or_localization = TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('slam_toolbox'),
                            'launch',
                            'online_async_launch.py',
                        )
                    ),
                    launch_arguments={
                        'slam_params_file': os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml'),
                        'use_sim_time': 'true',
                    }.items(),
                )
            ],
        )
    elif use_vslam_localize:
        slam_or_localization = TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg=f'[slam_nav] VSLAM localization from {rtabmap_db}'),
                _common.rtabmap_vslam_node(
                    localization=True,
                    database_path=rtabmap_db,
                    octomap=octomap,
                ),
            ],
        )
    else:
        slam_or_localization = LogInfo(msg=f'[slam_nav] SLAM disabled — loading map: {map_yaml}')

    if use_slam or use_vslam_localize:
        # Nav2 navigation only — /map (+ map→odom) comes from SLAM or VSLAM localize.
        nav2 = TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('nav2_bringup'),
                            'launch',
                            'navigation_launch.py',
                        )
                    ),
                    launch_arguments={
                        'use_sim_time': 'true',
                        'params_file': _params_file,
                    }.items(),
                )
            ],
        )
    else:
        nav2 = TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('nav2_bringup'),
                            'launch',
                            'bringup_launch.py',
                        )
                    ),
                    launch_arguments={
                        'use_sim_time': 'true',
                        'map': map_yaml,
                        'params_file': _params_file,
                        'slam': 'False',
                    }.items(),
                )
            ],
        )

    # ── GS keepout costmap filter (optional) ────────────────────────────────
    # See concepts.md §28 and scripts/gs_mask_from_splat.py — nav2_params.yaml
    # always lists the keepout_filter plugin (inert without a live
    # /gs/costmap_filter_info), this just starts the two nodes that publish it.
    gs_keepout_mask = LaunchConfiguration('gs_keepout_mask').perform(context).strip()
    if gs_keepout_mask:
        gs_keepout_filter_params = os.path.join(pkg_share, 'config', 'gs_keepout_filter.yaml')
        gs_keepout_group = TimerAction(
            period=9.0,
            actions=[
                LogInfo(msg=f'[slam_nav] GS keepout filter ENABLED — mask={gs_keepout_mask}'),
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='filter_mask_server',
                    output='screen',
                    parameters=[gs_keepout_filter_params, {
                        'use_sim_time': True,
                        'yaml_filename': gs_keepout_mask,
                    }],
                ),
                Node(
                    package='nav2_map_server',
                    executable='costmap_filter_info_server',
                    name='costmap_filter_info_server',
                    output='screen',
                    parameters=[gs_keepout_filter_params, {'use_sim_time': True}],
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_gs_keepout',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'autostart': True,
                        'node_names': ['filter_mask_server', 'costmap_filter_info_server'],
                    }],
                ),
            ],
        )
    else:
        gs_keepout_group = LogInfo(msg='[slam_nav] GS keepout filter disabled (gs_keepout_mask not set).')

    # ── GS speed costmap filter (optional) ──────────────────────────────────
    # See concepts.md §30 and scripts/gs_speed_mask_from_splat.py — density-
    # graded slow zones instead of gs_keepout_mask's binary no-go. Distinct
    # node names from the keepout group above so both can run together.
    gs_speed_mask = LaunchConfiguration('gs_speed_mask').perform(context).strip()
    if gs_speed_mask:
        gs_speed_filter_params = os.path.join(pkg_share, 'config', 'gs_speed_filter.yaml')
        gs_speed_group = TimerAction(
            period=9.0,
            actions=[
                LogInfo(msg=f'[slam_nav] GS speed filter ENABLED — mask={gs_speed_mask}'),
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='gs_speed_filter_mask_server',
                    output='screen',
                    parameters=[gs_speed_filter_params, {
                        'use_sim_time': True,
                        'yaml_filename': gs_speed_mask,
                    }],
                ),
                Node(
                    package='nav2_map_server',
                    executable='costmap_filter_info_server',
                    name='gs_speed_costmap_filter_info_server',
                    output='screen',
                    parameters=[gs_speed_filter_params, {'use_sim_time': True}],
                ),
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_gs_speed',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'autostart': True,
                        'node_names': ['gs_speed_filter_mask_server', 'gs_speed_costmap_filter_info_server'],
                    }],
                ),
            ],
        )
    else:
        gs_speed_group = LogInfo(msg='[slam_nav] GS speed filter disabled (gs_speed_mask not set).')

    # ── RViz ──────────────────────────────────────────────────────────────
    # Independent of `headless` — that flag controls Gazebo's GPU-heavy 3D
    # client only. RViz has its own renderer and is useful precisely when
    # Gazebo's client is skipped (e.g. watching the map build without
    # paying Gazebo's rendering cost).
    #
    # Per-mode config instead of one do-everything file: rviz2's Ogre1 GL
    # backend segfaults on startup on this box whenever both the
    # SlamToolboxPlugin panel and the Navigation 2 panel are docked at once
    # (confirmed via gdb — crash is in RenderWindowImpl::resize during the
    # initial dock-layout resize storm), so no config may combine them.
    if slam_algo != '2d':
        rviz_config_name = 'vslam.rviz'
    elif use_slam:
        rviz_config_name = 'slam_explore.rviz'
    elif enable_camera == 'true' or enable_rgbd == 'true':
        rviz_config_name = 'cam_nav.rviz'
    else:
        rviz_config_name = 'localization.rviz'
    print(f'[slam_nav] RViz config: {rviz_config_name} '
          f'(slam_algo={slam_algo}, slam={use_slam}, enable_camera={enable_camera}, '
          f'enable_rgbd={enable_rgbd})')
    rviz2 = GroupAction(
        condition=IfCondition(rviz),
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', rviz_config_name)],
            output='screen')])

    # ── Safety Layer: Collision Monitor ───────────────────────────────────
    safety = LaunchConfiguration('safety')
    collision_monitor = GroupAction(
        condition=IfCondition(safety),
        actions=[
            LogInfo(msg='[slam_nav] Collision monitor ENABLED (starting in 15s)…'),
            TimerAction(
                period=15.0,
                actions=[Node(
                    package='rosnav_bot',
                    executable='collision_monitor.py',
                    name='collision_monitor',
                    output='screen',
                    parameters=[{
                        'stop_distance':     0.55,
                        'slowdown_distance': 1.0,
                        'front_angle_deg':   120.0,
                        'watch_all_around':  False,
                        'relay_mode':        True,
                        'use_sim_time':      True,
                    }],
                    remappings=[
                        ('cmd_vel_nav', 'cmd_vel'),
                        ('cmd_vel',     'cmd_vel_safe'),
                    ],
                )]
            ),
        ]
    )

    # ── Mission Layer: Mission Server ──────────────────────────────────────
    mission_server = TimerAction(
        period=15.0,
        actions=[Node(
            package='rosnav_bot',
            executable='mission_server.py',
            name='mission_server',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )]
    )

    # ── Frontier Explorer (SLAM mode only) ───────────────────────────────
    explorer_backend = _common.resolve_explorer(LaunchConfiguration('explorer').perform(context))
    actions = []
    if use_slam and explore.perform(context).lower() in ('true', '1', 'yes'):
        if explorer_backend == 'builtin':
            explore_nodes = _common.explorer_nodes(
                'builtin', pkg_share, map_topic='/map',
                builtin_params={
                    'map_save_path': map_prefix,
                    'use_sim_time': True,
                    'frontier_detector': LaunchConfiguration('frontier_detector'),
                    'frontier_scorer': LaunchConfiguration('frontier_scorer'),
                    'info_gain_weight': LaunchConfiguration('info_gain_weight'),
                    'potential_scale': LaunchConfiguration('potential_scale'),
                    'gain_scale': LaunchConfiguration('gain_scale'),
                    'behavior_tree': LaunchConfiguration('explore_bt'),
                    'validate_on_costmap': LaunchConfiguration('validate_on_costmap'),
                    'costmap_max_cost': LaunchConfiguration('costmap_max_cost'),
                    'goal_pullback': LaunchConfiguration('goal_pullback'),
                    'frontier_clearance_radius': LaunchConfiguration('frontier_clearance_radius'),
                })
        else:
            explore_nodes = _common.explorer_nodes(explorer_backend, pkg_share, map_topic='/map')
        actions = [
            LogInfo(msg=f'[slam_nav] Auto-exploration ENABLED ({explorer_backend}). Starting in 20s...'),
            TimerAction(period=20.0, actions=explore_nodes),
        ]
    frontier_node = GroupAction(actions=actions) if actions else LogInfo(msg='[slam_nav] Frontier explorer disabled.')
    cslam_group = (
        GroupAction(actions=_common.cslam_lidar_nodes(pkg_share))
        if run_cslam else LogInfo(msg='[slam_nav] C-SLAM disabled.'))

    # ── Object Detection: YOLO (optional, off by default) ────────────────
    enable_yolo = LaunchConfiguration('enable_yolo')
    yolo_detector = GroupAction(
        condition=IfCondition(enable_yolo),
        actions=[
            LogInfo(msg='[slam_nav] YOLO object detection ENABLED (starting in 12s)…'),
            TimerAction(
                period=12.0,
                actions=[Node(
                    package='rosnav_bot',
                    executable='yolo_detector.py',
                    name='yolo_detector',
                    output='screen',
                    parameters=[{
                        'model_path': LaunchConfiguration('yolo_model'),
                        'confidence': LaunchConfiguration('yolo_confidence'),
                        'classes': LaunchConfiguration('yolo_classes'),
                        'use_sim_time': True,
                    }],
                )]
            ),
        ]
    )

    # ── GS semantic fusion (optional) ───────────────────────────────────────
    # Lifts yolo_detector.py's 2D detections into real 3D map-frame regions by
    # projecting the splat point cloud through the camera (see concepts.md
    # §29) — needs enable_yolo:=true for detections to fuse against.
    gs_semantic_npz = LaunchConfiguration('gs_semantic_npz').perform(context).strip()
    if gs_semantic_npz and not _common.truthy(enable_yolo_arg):
        print('[slam_nav] gs_semantic_npz set but enable_yolo is not true — '
              'gs_semantic_fusion has no detections to fuse, disabling')
        gs_semantic_npz = ''
    if gs_semantic_npz:
        gs_semantic_fusion = TimerAction(
            period=14.0,
            actions=[
                LogInfo(msg=f'[slam_nav] GS semantic fusion ENABLED — npz={gs_semantic_npz}'),
                Node(
                    package='rosnav_bot',
                    executable='gs_semantic_fusion.py',
                    name='gs_semantic_fusion',
                    output='screen',
                    parameters=[{
                        'npz_path': gs_semantic_npz,
                        'use_sim_time': True,
                    }],
                ),
            ],
        )
    else:
        gs_semantic_fusion = LogInfo(msg='[slam_nav] GS semantic fusion disabled (gs_semantic_npz not set).')

    # ── Dynamic obstacles (optional) ───────────────────────────────────────
    dynamic_obstacles = int(LaunchConfiguration('dynamic_obstacles').perform(context).strip())
    dynamic_obstacle_actions = []
    if dynamic_obstacles > 0:
        do_x = float(LaunchConfiguration('dynamic_obstacle_x').perform(context).strip())
        do_y = float(LaunchConfiguration('dynamic_obstacle_y').perform(context).strip())
        do_axis = LaunchConfiguration('dynamic_obstacle_axis').perform(context).strip()
        do_amplitude = LaunchConfiguration('dynamic_obstacle_amplitude').perform(context).strip()
        do_speed = LaunchConfiguration('dynamic_obstacle_speed').perform(context).strip()
        dynamic_obstacle_actions.append(
            LogInfo(msg=f'[slam_nav] spawning {dynamic_obstacles} dynamic_obstacle(s) '
                        f'patrolling +-{do_amplitude}m along {do_axis} at {do_speed}m/s'))
        for i in range(dynamic_obstacles):
            do_name = f'dynamic_obstacle_{i + 1}'
            dynamic_obstacle_actions += [
                _common.spawn_dynamic_obstacle_node(
                    pkg_share, gazebo_world_name, do_name, do_x + i * 2.0, do_y, 0.3, 0.0),
                _common.dynamic_obstacle_bridge_node(do_name),
                TimerAction(
                    period=5.0,
                    actions=[_common.dynamic_obstacle_driver_node(do_name, do_axis, do_amplitude, do_speed)]),
            ]

    mode = 'SLAM+frontier' if (use_slam and actions) else ('SLAM' if use_slam else f'nav-on-map ({os.path.basename(map_yaml)})')
    if run_cslam:
        mode += '+cslam'
    return [
        LogInfo(msg=f'[slam_nav.launch] mode={mode}, ROS_DISTRO={ROS_DISTRO}, controller={controller}, drive_type={drive_type}'),
        LogInfo(msg=f'[slam_nav.launch] world={world_path}'),
        LogInfo(msg=f'[slam_nav.launch] robot_name={robot_name.perform(context)}'),
        LogInfo(msg=f'[slam_nav.launch] urdf={urdf_filename}, nav2_params={nav2_params_name}'),
        LogInfo(msg=f'[slam_nav.launch] octomap={octomap} (slam_algo=3d only; Grid/3D={"true" if octomap else "false"})'),
        LogInfo(msg=f'[slam_nav.launch] bridge={bridge_yaml}, lidar_type={lidar_type}, '
                    f'enable_rgbd={enable_rgbd}'),
        LogInfo(msg='[slam_nav.launch] odom->base_link TF: ekf_filter_node (wheel odom + IMU fused)'),
        rsp,
        gazebo_server,
        gazebo_client,
        ros_gz_bridge,
        *([points_to_scan] if points_to_scan is not None else []),
        laser_filter,
        *([scan_quality_gate] if scan_quality_gate is not None else []),
        ekf_filter,
        spawn_robot,
        slam_or_localization,
        cslam_group,
        nav2,
        gs_keepout_group,
        gs_speed_group,
        rviz2,
        collision_monitor,
        mission_server,
        frontier_node,
        yolo_detector,
        gs_semantic_fusion,
    ] + dynamic_obstacle_actions


def generate_launch_description():
    pkg_share = get_package_share_directory('rosnav_bot')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value='cafe',
            description='Gazebo world name in package worlds/ (cafe, hospital, '
                        'lake_house, aws_warehouse, tugbot_warehouse, …). Textured '
                        'Fuel worlds need: bash src/rosnav_bot/scripts/download_fuel_worlds.sh',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='',
            description='Optional full world path override (if set, world_name is ignored)',
        ),
        DeclareLaunchArgument('rviz', default_value='True', description='Launch RViz'),
        DeclareLaunchArgument('robot_name', default_value='diff_drive', description='Gazebo robot entity name'),
        DeclareLaunchArgument(
            name='enable_camera', default_value='false',
            description='Include the RGB camera sensor (used by aruco_dock.py, '
                        'yolo_detector.py, and RTAB-Map loop closure). Off by default '
                        'to save render cost — set true when you need docking or YOLO '
                        'object detection. Automatically forced true when slam_algo:=3d '
                        'or slam_algo:=vslam|multisensor / enable_rgbd:=true (RGB-D replaces RGB).'),
        DeclareLaunchArgument(
            name='enable_rgbd', default_value='false',
            description='Use RGB-D camera (depth image + /camera/depth/points for '
                        'Nav2 VoxelLayer). Forced true for slam_algo:=vslam|multisensor.'),
        # Maze default spawn moved away from origin so robot is immediately visible.
        DeclareLaunchArgument(
            name='spawn_x', default_value='auto',
            description="Spawn X. 'auto' picks a per-world verified-safe default (see "
                        "_WORLD_SPAWN_DEFAULTS); an explicit value always overrides it."),
        DeclareLaunchArgument(
            name='spawn_y', default_value='auto',
            description="Spawn Y. 'auto' picks a per-world verified-safe default."),
        DeclareLaunchArgument(
            name='spawn_z', default_value='auto',
            description="Spawn Z. 'auto' picks a per-world verified-safe default."),
        DeclareLaunchArgument(
            name='spawn_yaw', default_value='auto',
            description="Spawn yaw. 'auto' picks a per-world verified-safe default."),
        DeclareLaunchArgument(
            name='map_prefix',
            default_value='',
            description='Output prefix for map_saver_cli. If empty, uses <package_share>/maps/map_<world_name>'),
        DeclareLaunchArgument(
            name='map_override',
            default_value='',
            description='Explicit map yaml for nav-on-map mode (slam:=false), overriding the '
                        'auto-resolved maps/map_<world_name>.yaml. A gs_mask_from_splat.py '
                        'KeepoutFilter mask is already in this exact format, so this can seed '
                        'Nav2 with a GS-derived floor plan before any SLAM. See concepts.md §31.'),
        DeclareLaunchArgument(
            name='slam', default_value='true',
            description='true=live mapping (2d/3d/vslam), false=localize. With '
                        'slam_algo:=vslam and rtabmap_db set, false runs RTAB '
                        'localization + Nav2 (no AMCL). Otherwise AMCL on map yaml.'),
        DeclareLaunchArgument(
            name='rtabmap_db', default_value='',
            description='RTAB-Map SQLite path for slam_algo:=vslam. Mapping: optional '
                        'persist path (created if missing). Localization (slam:=false): '
                        'required existing .db.'),
        DeclareLaunchArgument(
            name='explore', default_value='false',
            description='Auto-start frontier explorer (only valid when slam:=true)'),
        DeclareLaunchArgument(
            name='explorer', default_value='builtin',
            description='Exploration backend when explore:=true: builtin (rosnav WFD/utility), '
                        'explore_lite (m-explore-ros2), frontier (frontier_exploration_ros2 MRTSP), '
                        'or rrt (rrt_explore). Falls back to builtin if the package is not built.'),
        DeclareLaunchArgument(
            name='frontier_detector', default_value='wfd',
            description='Frontier detector: wfd (reachable wavefront), classic, or rrt (sampling-based)'),
        DeclareLaunchArgument(
            name='frontier_scorer', default_value='utility',
            description='Frontier scorer: utility (size/distance tradeoff), weighted, or nearest'),
        DeclareLaunchArgument(
            name='info_gain_weight', default_value='3.0',
            description='Weighted scorer information-gain reward'),
        DeclareLaunchArgument(
            name='potential_scale', default_value='3.0',
            description='Utility scorer distance penalty'),
        DeclareLaunchArgument(
            name='gain_scale', default_value='1.0',
            description='Utility scorer frontier-size reward'),
        DeclareLaunchArgument(
            name='explore_bt', default_value='explore_nav',
            description='BT XML stem or path passed as NavigateToPose.behavior_tree'),
        DeclareLaunchArgument(
            name='validate_on_costmap', default_value='true',
            description='Reject frontier goals in inflated/lethal Nav2 costmap cells'),
        DeclareLaunchArgument(
            name='costmap_max_cost', default_value='1',
            description='Reject costmap OccupancyGrid values >= this (0=free only; '
                        'inflation is 1-98, inscribed 99, lethal 100)'),
        DeclareLaunchArgument(
            name='goal_pullback', default_value='0.55',
            description='Stand back from unknown frontier edge into free space (m). '
                        'Should be >= global inflation_radius (0.5)'),
        DeclareLaunchArgument(
            name='frontier_clearance_radius', default_value='0.55',
            description='Min clearance from occupied cells for frontier goals (m)'),
        DeclareLaunchArgument(
            name='safety', default_value='true',
            description='Launch collision monitor safety layer'),
        DeclareLaunchArgument(
            name='enable_yolo', default_value='false',
            description='Launch the pluggable yolo_detector.py object-detection node on '
                        'the robot camera. Off by default; requires `pip install ultralytics`.'),
        DeclareLaunchArgument(
            name='yolo_model', default_value='yolov8n.pt',
            description='Ultralytics model name/path for yolo_detector.py (enable_yolo:=true only).'),
        DeclareLaunchArgument(
            name='yolo_confidence', default_value='0.5',
            description='YOLO detection confidence threshold (enable_yolo:=true only).'),
        DeclareLaunchArgument(
            name='yolo_classes', default_value='',
            description='Comma-separated class-name allow-list for YOLO (e.g. "person,chair"). '
                        'Empty = all classes (enable_yolo:=true only).'),
        DeclareLaunchArgument(
            name='headless', default_value='false',
            description='Skip Gazebo GUI and RViz (server + nav only)'),
        DeclareLaunchArgument(
            name='controller', default_value='dwb',
            description='Local controller: "dwb" (default) or "mppi" (nav2_mppi_controller). Humble only — ignored on Jazzy, which defaults to MPPI.'),
        DeclareLaunchArgument(
            name='drive_type',
            default_value='diff',
            description='Drive base: "diff" (default), "mecanum" (holonomic, 4 driven wheels), or "ackermann" (car-like front steering). Ackermann always uses MPPI params.'),
        DeclareLaunchArgument(
            name='robot_model',
            default_value='custom',
            description='Chassis visual skin: "custom" (default, this repo\'s own simple box '
                        'chassis), "mir100" (MiR100-shaped mesh, vendored from DFKI-NI/mir_robot), '
                        'or "husky" (Clearpath Husky A200-shaped mesh, vendored from '
                        'husky_description) — all scaled to the same footprint/wheelbase, visual '
                        'only, same physics/nav2 tuning. mir100/husky only supported with '
                        'drive_type:=diff.'),
        DeclareLaunchArgument(
            name='scan_gate',
            default_value='true',
            description='Insert scan_quality_gate between laser_filters and /scan — '
                        'rejects malformed LaserScans (empty frame, broken angles, '
                        'too few valid beams, stamp jumps) so SLAM never sees them. '
                        'Chain: scan_raw → laser_filters → scan_pre → gate → scan.'),
        DeclareLaunchArgument(
            name='lidar_type',
            default_value='2d',
            description='"2d" (default, LaserScan on /scan) or "3d" (16-channel PointCloud2 on '
                        '/points, gpu_lidar). Only supported for drive_type:=diff.'),
        DeclareLaunchArgument(
            name='lidar3d_height', default_value='0.25',
            description='3D lidar mount height (m), lidar_type:=3d only. Needs >=0.10m '
                        'clearance above the chassis top (z=0.15) to avoid self-hits.'),
        DeclareLaunchArgument(
            name='lidar3d_vfov_deg', default_value='10',
            description='3D lidar vertical half-angle in degrees (+/-), lidar_type:=3d only.'),
        DeclareLaunchArgument(
            name='octomap', default_value='false',
            description='slam_algo:=3d|vslam|multisensor. true sets RTAB-Map Grid/3D:=true so '
                        '/octomap_binary and /octomap_full actually populate (a full 3D '
                        'voxel map), on top of the same 2D /map Nav2 always gets.'),
        DeclareLaunchArgument(
            name='slam_algo', default_value='2d',
            description='"2d" (slam_toolbox), "cartographer" (Cartographer 2D+IMU), '
                        '"3d" (RTAB-Map lidar), "vslam" (RTAB-Map RGB-D), '
                        '"multisensor" (RTAB-Map RGB-D+lidar), or "cslam" (Swarm-SLAM). '
                        'cslam/3d need lidar_type:=3d; cartographer needs '
                        'ros-humble-cartographer-ros; cslam needs link_third_party.sh --cslam.'),
        DeclareLaunchArgument(
            'dynamic_obstacles', default_value='0',
            description='Number of patrolling dynamic_obstacle models to spawn '
                        '(see models/dynamic_obstacle) for testing Nav2 dynamic-obstacle '
                        'avoidance and obstacle_tracker.py'),
        DeclareLaunchArgument(
            'dynamic_obstacle_x', default_value='2.0',
            description='Base x position for spawned dynamic obstacles (spaced 2m apart along x)'),
        DeclareLaunchArgument(
            'dynamic_obstacle_y', default_value='0.0',
            description='Base y position for spawned dynamic obstacles'),
        DeclareLaunchArgument(
            'dynamic_obstacle_axis', default_value='y_axis',
            description='Patrol axis for dynamic obstacles: x_axis or y_axis '
                        '(not the bare letters x/y — those YAML-parse as booleans)'),
        DeclareLaunchArgument(
            'dynamic_obstacle_amplitude', default_value='3.0',
            description='Dynamic obstacle patrol half-length in metres'),
        DeclareLaunchArgument(
            'dynamic_obstacle_speed', default_value='0.4',
            description='Dynamic obstacle patrol speed in m/s'),
        DeclareLaunchArgument(
            'gs_semantic_npz', default_value='',
            description='Path to a splat_points.npz (from gs_splat_to_pointcloud.py) for '
                        'gs_semantic_fusion.py to project YOLO detections against for 3D '
                        'semantic markers. Requires enable_yolo:=true. Empty (default) = '
                        'disabled. See concepts.md §29.'),
        DeclareLaunchArgument(
            'gs_keepout_mask', default_value='',
            description='Path to a Nav2 costmap-filter-mask yaml produced by '
                        'scripts/gs_mask_from_splat.py (Gaussian-Splat-derived keepout '
                        'zones). Empty (default) = disabled, no extra nodes started. '
                        'See concepts.md §28.'),
        DeclareLaunchArgument(
            'gs_speed_mask', default_value='',
            description='Path to a Nav2 costmap-filter-mask yaml produced by '
                        'scripts/gs_speed_mask_from_splat.py (Gaussian-Splat density-graded '
                        'slow zones, distinct from the binary gs_keepout_mask). Empty '
                        '(default) = disabled. See concepts.md §30.'),
        OpaqueFunction(function=_build_runtime_actions, args=[pkg_share]),
    ])
