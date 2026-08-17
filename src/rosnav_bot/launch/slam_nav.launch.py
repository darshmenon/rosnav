"""
slam_nav.launch.py
==================
One-shot launch: Gazebo + Robot + Nav2 + RViz.

SLAM mode (default):
  ros2 launch rosnav_bot slam_nav.launch.py world_name:=hospital explore:=true

Nav-on-map mode (pre-built map, AMCL localization):
  ros2 launch rosnav_bot slam_nav.launch.py world_name:=hospital slam:=false

When slam:=false the launch uses map_<world_name>.yaml from the maps/ directory.
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
    world_name = os.path.splitext(os.path.basename(world_name_arg.strip() or 'hospital'))[0]
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

    slam_algo = LaunchConfiguration('slam_algo').perform(context).strip().lower()
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
    map_yaml = _resolve_map_yaml(world_name, pkg_share)

    enable_camera_arg = LaunchConfiguration('enable_camera').perform(context).strip().lower()
    enable_yolo_arg = LaunchConfiguration('enable_yolo').perform(context).strip().lower()
    # yolo_detector.py has nothing to detect on without the camera, so
    # enable_yolo:=true pulls it in even if enable_camera wasn't set explicitly.
    enable_camera = 'true' if (_common.truthy(enable_camera_arg) or _common.truthy(enable_yolo_arg)) else 'false'
    urdf_filename = _common.urdf_filename_for(drive_type, robot_model)
    rsp = _common.rsp_include(
        pkg_share, os.path.join(pkg_share, 'urdf', urdf_filename), lidar_type=lidar_type,
        lidar3d_height=lidar3d_height, lidar3d_vfov_deg=lidar3d_vfov_deg,
        enable_camera=enable_camera)

    gazebo_server = _common.gazebo_server_action(world_path, pkg_share)

    gazebo_client = GroupAction(
        condition=UnlessCondition(headless),
        actions=[_common.gazebo_client_action(pkg_share)],
    )

    spawn_robot = _common.spawn_robot_node(
        gazebo_world_name, 'robot_description', robot_name, spawn_x, spawn_y, spawn_z, spawn_yaw)

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={os.path.join(pkg_share, "config", "gz_bridge.yaml")}',
        ],
    )

    # Lidar filter chain: raw Gazebo scan (/scan_raw) in, cleaned /scan out.
    # SLAM Toolbox, AMCL, and the costmaps below never see the unfiltered feed.
    laser_filter = _common.laser_filter_node(pkg_share)

    # Patch the BT path placeholder before passing params to nav2.
    nav2_params_name = _common.nav2_params_filename(controller, drive_type, ROS_DISTRO)
    _raw_params = os.path.join(pkg_share, 'config', nav2_params_name)
    _params_file = _common.patch_pkg_share_placeholder(_raw_params, pkg_share)

    if use_slam and slam_algo == '3d':
        # RTAB-Map lidar SLAM: consumes the *existing* wheel /odom directly (like
        # slam_toolbox does) instead of running RTAB-Map's own icp_odometry — that
        # keeps this a drop-in swap with exactly one odom->base_link TF publisher
        # (the gz DiffDrive plugin), avoiding a two-parent TF conflict. RTAB-Map
        # only adds the map->odom layer on top, same as slam_toolbox/AMCL today.
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
                        'subscribe_rgb': False,
                        'subscribe_scan_cloud': True,
                        'approx_sync': True,
                        'wait_for_transform': 0.2,
                        # RTAB-Map's internal parameters are strings:
                        'Reg/Strategy': '1',           # ICP — no camera to do Vis registration
                        'Icp/PointToPlane': 'true',
                        'Grid/Sensor': '0',             # occupancy grid from the lidar cloud, not depth
                        'Grid/3D': 'false',              # project to 2D for Nav2's /map
                        'Grid/CellSize': '0.05',
                        'Grid/RangeMax': '20.0',
                        # NoiseFilteringRadius defaults to 0 (disabled); the 16-channel 3D
                        # lidar's sparse vertical resolution makes normal-based ground
                        # segmentation (Grid/NormalsSegmentation) noisy, sprinkling spurious
                        # "obstacle" cells across otherwise-flat ground. Filter isolated points
                        # before classification.
                        'Grid/NoiseFilteringRadius': '0.1',
                        'Grid/NoiseFilteringMinNeighbors': '5',
                        'Mem/IncrementalMemory': 'true',  # mapping mode (vs. localization)
                    }],
                    remappings=[('odom', '/odom'), ('scan_cloud', '/points')],
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

    if use_slam:
        # Same Nav2 bringup regardless of which SLAM backend is producing /map.
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
        slam_or_localization = LogInfo(msg=f'[slam_nav] SLAM disabled — loading map: {map_yaml}')
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

    # ── RViz ──────────────────────────────────────────────────────────────
    # Independent of `headless` — that flag controls Gazebo's GPU-heavy 3D
    # client only. RViz has its own renderer and is useful precisely when
    # Gazebo's client is skipped (e.g. watching the map build without
    # paying Gazebo's rendering cost).
    rviz2 = GroupAction(
        condition=IfCondition(rviz),
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'bot.rviz')],
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
    actions = []
    if use_slam and explore.perform(context).lower() in ('true', '1', 'yes'):
        actions = [
            LogInfo(msg="[slam_nav] Auto-exploration ENABLED. Starting frontier_explorer in 20s..."),
            TimerAction(
                period=20.0,
                actions=[Node(
                    package='rosnav_bot',
                    executable='frontier_explorer.py',
                    name='frontier_explorer',
                    output='screen',
                    parameters=[{
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
                    }]
                )]
            )
        ]
    frontier_node = GroupAction(actions=actions) if actions else LogInfo(msg='[slam_nav] Frontier explorer disabled.')

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

    mode = 'SLAM+frontier' if (use_slam and actions) else ('SLAM' if use_slam else f'nav-on-map ({os.path.basename(map_yaml)})')
    return [
        LogInfo(msg=f'[slam_nav.launch] mode={mode}, ROS_DISTRO={ROS_DISTRO}, controller={controller}, drive_type={drive_type}'),
        LogInfo(msg=f'[slam_nav.launch] world={world_path}'),
        LogInfo(msg=f'[slam_nav.launch] robot_name={robot_name.perform(context)}'),
        LogInfo(msg=f'[slam_nav.launch] urdf={urdf_filename}, nav2_params={nav2_params_name}'),
        rsp,
        gazebo_server,
        gazebo_client,
        ros_gz_bridge,
        laser_filter,
        spawn_robot,
        slam_or_localization,
        nav2,
        rviz2,
        collision_monitor,
        mission_server,
        frontier_node,
        yolo_detector,
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('rosnav_bot')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value='hospital',
            description='Gazebo world name in package worlds/ (example: hospital, corridor, maze, or obstacles)',
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
            description='Include the RGB camera sensor (used by aruco_dock.py and '
                        'yolo_detector.py). Off by default to save render cost — set '
                        'true when you need docking or YOLO object detection.'),
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
            name='slam', default_value='true',
            description='true=SLAM Toolbox (live mapping), false=AMCL on pre-built map'),
        DeclareLaunchArgument(
            name='explore', default_value='false',
            description='Auto-start frontier explorer (only valid when slam:=true)'),
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
            name='slam_algo', default_value='2d',
            description='"2d" (default, slam_toolbox — 2D occupancy grid only) or "3d" '
                        '(RTAB-Map lidar SLAM, real 3D map + projected 2D grid for Nav2). '
                        'Requires lidar_type:=3d and ros-humble-rtabmap-ros installed; '
                        'falls back to 2d with a warning if either is missing.'),
        OpaqueFunction(function=_build_runtime_actions, args=[pkg_share]),
    ])
