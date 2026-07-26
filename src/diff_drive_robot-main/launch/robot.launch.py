import os
import xml.etree.ElementTree as ET
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from nav2_common.launch import RewrittenYaml

# ---------------------------------------------------------------------------
# Distro-agnostic helper: works on Humble (GZ Fortress/Garden) and Jazzy (GZ Harmonic)
# Both distros use the 'ros_gz_sim' / 'ros_gz_bridge' package names; the
# underlying Gazebo version is selected by the environment automatically.
# We just need to note the distro for any args that differ.
ROS_DISTRO = os.environ.get('ROS_DISTRO', 'humble')

# Auto-select nav2 params based on distro:
#   Humble: nav2_params.yaml  (behaviors use /Spin format)
#   Jazzy:  nav2_params_jazzy.yaml  (behaviors use ::Spin format)
# drive_type:=mecanum swaps in the holonomic-tuned variant of either file
# (unlocked vy in DWB for Humble, motion_model:"Omni" in MPPI for Jazzy).
# drive_type:=ackermann always uses MPPI with motion_model:"Ackermann" —
# DWB has no car-like-steering constraint, so there is no Humble/Jazzy split
# for this drive type.
def _nav2_params_filename(drive_type: str) -> str:
    if drive_type == 'ackermann':
        return 'nav2_params_ackermann.yaml'
    if ROS_DISTRO == 'jazzy':
        return 'nav2_params_mecanum_jazzy.yaml' if drive_type == 'mecanum' else 'nav2_params_jazzy.yaml'
    return 'nav2_params_mecanum.yaml' if drive_type == 'mecanum' else 'nav2_params.yaml'


def _resolve_map_file(map_arg: str, world_path: str, home: str, pkg_share: str) -> str:
    if map_arg:
        return map_arg
    world_name = os.path.splitext(os.path.basename(world_path))[0].replace('.world', '')
    maps_dir = os.path.join(pkg_share, 'maps')
    legacy_maps_dir = os.path.join(home, 'rosnav', 'maps')
    candidates = [
        os.path.join(maps_dir, f'{world_name}_map.yaml'),
        os.path.join(maps_dir, f'map_{world_name}.yaml'),
        os.path.join(legacy_maps_dir, f'{world_name}_map.yaml'),
        os.path.join(home, 'rosnav', f'{world_name}_map.yaml'),
        os.path.join(maps_dir, 'my_map.yaml'),
        os.path.join(legacy_maps_dir, 'my_map.yaml'),
        os.path.join(home, 'rosnav', 'my_map.yaml'),
    ]
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    return candidates[0]


def _resolve_gazebo_world_name(world_path: str) -> str:
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


def _build_spawn_action(context, pkg_share: str):
    world_path = LaunchConfiguration('world').perform(context)
    world_name = _resolve_gazebo_world_name(world_path)

    return [
        LogInfo(msg=f'[robot.launch] gazebo world={world_name}'),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', world_name,
                '-topic', 'robot_description',
                '-name', LaunchConfiguration('robot_name'),
                '-x', LaunchConfiguration('spawn_x'),
                '-y', LaunchConfiguration('spawn_y'),
                '-z', LaunchConfiguration('spawn_z'),
                '-Y', LaunchConfiguration('spawn_yaw'),
            ],
            output='screen',
        ),
    ]


def _build_nav2_action(context, pkg_share: str, home: str):
    world_path = LaunchConfiguration('world').perform(context)
    map_arg = LaunchConfiguration('map').perform(context).strip()
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    drive_type = LaunchConfiguration('drive_type').perform(context)
    map_file = _resolve_map_file(map_arg, world_path, home, pkg_share)
    raw_params = os.path.join(pkg_share, 'config', _nav2_params_filename(drive_type))
    bt_xml = os.path.join(pkg_share, 'config', 'bt', 'navigate_w_recovery.xml')
    params_file = RewrittenYaml(
        source_file=raw_params,
        root_key='',
        param_rewrites={'default_nav_to_pose_bt_xml': bt_xml},
        convert_types=True,
    ).perform(context)

    return [
        LogInfo(msg=f'[robot.launch] ROS_DISTRO={ROS_DISTRO}, params={os.path.basename(raw_params)}'),
        LogInfo(msg=f'[robot.launch] using map={map_file}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_file,
                'params_file': params_file,
            }.items(),
        ),
    ]

def generate_launch_description():

    package_name = 'diff_drive_robot'
    pkg_share = get_package_share_directory(package_name)
    home = os.path.expanduser('~')

    # Launch configurations
    world    = LaunchConfiguration('world')
    rviz     = LaunchConfiguration('rviz')
    headless = LaunchConfiguration('headless')
    drive_type = LaunchConfiguration('drive_type')
    robot_name = LaunchConfiguration('robot_name')
    spawn_x    = LaunchConfiguration('spawn_x')
    spawn_y    = LaunchConfiguration('spawn_y')
    spawn_z    = LaunchConfiguration('spawn_z')
    spawn_yaw  = LaunchConfiguration('spawn_yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch Arguments
    declare_world = DeclareLaunchArgument(
        name='world',
        default_value=os.path.join(pkg_share, 'worlds', 'obstacles.world'),
        description='Full path to the Gazebo world file')

    declare_rviz = DeclareLaunchArgument(
        name='rviz',
        default_value='True',
        description='Open RViz if True')

    declare_headless = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Skip Gazebo GUI client (server still runs)')

    declare_drive_type = DeclareLaunchArgument(
        name='drive_type',
        default_value='diff',
        description='Drive base: "diff" (default), "mecanum" (holonomic, 4 driven wheels), or "ackermann" (car-like front steering)')

    declare_map = DeclareLaunchArgument(
        name='map',
        default_value='',
        description='Map yaml path. If empty, auto-use <package_share>/maps/map_<world_name>.yaml (legacy fallbacks still supported)')

    declare_robot_name = DeclareLaunchArgument(
        name='robot_name', default_value='diff_bot',
        description='Name of the robot model in Gazebo')

    declare_spawn_x = DeclareLaunchArgument(
        name='spawn_x', default_value='0.0',
        description='Robot spawn X position')

    declare_spawn_y = DeclareLaunchArgument(
        name='spawn_y', default_value='0.0',
        description='Robot spawn Y position')

    declare_spawn_z = DeclareLaunchArgument(
        name='spawn_z', default_value='0.075',
        description='Robot spawn Z position')

    declare_spawn_yaw = DeclareLaunchArgument(
        name='spawn_yaw', default_value='0.0',
        description='Robot spawn yaw (radians)')

    declare_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value='true',
        description='Use simulation clock')

    # Robot State Publisher — URDF picked at launch time by drive_type
    urdf_filename = PythonExpression([
        "'robot_mecanum.urdf.xacro' if '", drive_type, "' == 'mecanum' "
        "else 'robot_ackermann.urdf.xacro' if '", drive_type, "' == 'ackermann' "
        "else 'robot.urdf.xacro'"
    ])
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'urdf': PathJoinSubstitution([pkg_share, 'urdf', urdf_filename])
        }.items()
    )

    # Gazebo server (headless)
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v1 ', world],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Gazebo client (GUI) — skipped when headless:=true
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g'}.items(),
        condition=UnlessCondition(headless),
    )

    # Spawn robot
    spawn_robot = OpaqueFunction(function=_build_spawn_action, args=[pkg_share])

    # Gazebo <-> ROS bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p',
            f'config_file:={os.path.join(pkg_share, "config", "gz_bridge.yaml")}',
        ]
    )

    # Lidar filter chain: raw Gazebo scan (/scan_raw) in, cleaned /scan out.
    # AMCL and Nav2's costmaps never see the unfiltered feed.
    laser_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        parameters=[os.path.join(pkg_share, 'config', 'laser_filters.yaml')],
        remappings=[('scan', 'scan_raw'), ('scan_filtered', 'scan')],
    )

    # RViz
    rviz2 = GroupAction(
        condition=IfCondition(rviz),
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'bot.rviz')],
            output='screen',
        )]
    )

    # Nav2 full bringup (map_server + AMCL + planner + controller + behaviours)
    nav2_launch = TimerAction(
        period=8.0,
        actions=[OpaqueFunction(function=_build_nav2_action, args=[pkg_share, home])],
    )

    # SLAM Toolbox (uncomment to build or refine a map instead of using a saved one)
    # slam_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
    #     ),
    #     launch_arguments={
    #         'slam_params_file': os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml'),
    #         'use_sim_time': 'true'
    #     }.items(),
    # )

    # Custom navigation / path planning nodes (uncomment when needed)
    # navigation_node = Node(
    #     package='diff_drive_robot',
    #     executable='navigation.py',
    #     name='obstacle_avoidance_navigator',
    #     output='screen'
    # )
    # path_planning_node = Node(
    #     package='diff_drive_robot',
    #     executable='path_planning.py',
    #     name='path_planning',
    #     output='screen'
    # )
    # PID goal controller (no Nav2 required — pure stdlib PID)
    #   tune gains: heading_kp/ki/kd, set goal_x/goal_y
    # pid_controller_node = Node(
    #     package='diff_drive_robot',
    #     executable='pid_controller.py',
    #     name='pid_goal_controller',
    #     output='screen',
    #     parameters=[{'goal_x': 3.0, 'goal_y': 3.0,
    #                  'heading_kp': 2.5, 'heading_ki': 0.01, 'heading_kd': 0.35}]
    # )

    return LaunchDescription([
        # ── Declare ALL arguments first (BUG FIX: these were missing) ──
        declare_world,
        declare_rviz,
        declare_headless,
        declare_drive_type,
        declare_map,
        declare_robot_name,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        declare_spawn_yaw,
        declare_sim_time,
        # ── Nodes ──
        rsp,
        gazebo_server,
        gazebo_client,
        ros_gz_bridge,
        laser_filter,
        spawn_robot,
        rviz2,
        nav2_launch,
        # slam_launch,
        # navigation_node,
        # path_planning_node,
    ])
