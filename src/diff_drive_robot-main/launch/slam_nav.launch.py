"""
slam_nav.launch.py
==================
One-shot launch: Gazebo + Robot + SLAM Toolbox (live mapping) + Nav2 navigation + RViz.

For autonomous frontier exploration add in a 3rd terminal:
  ros2 run diff_drive_robot frontier_explorer.py

To save the map once explored:
  ros2 run nav2_map_server map_saver_cli -f ~/rosnav/my_map

To switch to localization with a saved map, use robot.launch.py instead.
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)

ROS_DISTRO = os.environ.get('ROS_DISTRO', 'humble')

# Auto-select nav2 params based on distro:
#   Humble: nav2_params.yaml  (behaviors use /Spin format)
#   Jazzy:  nav2_params_jazzy.yaml  (behaviors use ::Spin format)
_NAV2_PARAMS = 'nav2_params_jazzy.yaml' if ROS_DISTRO == 'jazzy' else 'nav2_params.yaml'


def _resolve_map_prefix(map_prefix_arg: str, world_path: str, home: str) -> str:
    if map_prefix_arg:
        return os.path.expanduser(map_prefix_arg)
    world_name = os.path.splitext(os.path.basename(world_path))[0]
    return os.path.join(home, 'rosnav', f'{world_name}_map')


def _build_save_hint(context, home: str):
    world_path = LaunchConfiguration('world').perform(context)
    map_prefix_arg = LaunchConfiguration('map_prefix').perform(context).strip()
    map_prefix = _resolve_map_prefix(map_prefix_arg, world_path, home)
    return [
        LogInfo(msg=f'[slam_nav.launch] ROS_DISTRO={ROS_DISTRO}, params={_NAV2_PARAMS}'),
        LogInfo(msg=f'[slam_nav.launch] save map with: ros2 run nav2_map_server map_saver_cli -f {map_prefix}'),
    ]


def generate_launch_description():
    package_name = 'diff_drive_robot'
    pkg_share = get_package_share_directory(package_name)
    home = os.path.expanduser('~')

    # ── Launch configurations ─────────────────────────────────────────────
    world       = LaunchConfiguration('world')
    rviz        = LaunchConfiguration('rviz')
    robot_name  = LaunchConfiguration('robot_name')
    spawn_x     = LaunchConfiguration('spawn_x')
    spawn_y     = LaunchConfiguration('spawn_y')
    spawn_z     = LaunchConfiguration('spawn_z')
    spawn_yaw   = LaunchConfiguration('spawn_yaw')

    # ── Declare arguments ─────────────────────────────────────────────────
    declare_world = DeclareLaunchArgument(
        name='world',
        default_value=os.path.join(pkg_share, 'worlds', 'maze.world'),
        description='Gazebo world file (default: maze.world)')

    declare_rviz = DeclareLaunchArgument(
        name='rviz', default_value='True',
        description='Launch RViz')

    declare_robot_name = DeclareLaunchArgument(
        name='robot_name', default_value='diff_bot')

    declare_spawn_x = DeclareLaunchArgument(name='spawn_x', default_value='0.0')
    declare_spawn_y = DeclareLaunchArgument(name='spawn_y', default_value='0.0')
    declare_spawn_z = DeclareLaunchArgument(name='spawn_z', default_value='0.3')
    declare_spawn_yaw = DeclareLaunchArgument(name='spawn_yaw', default_value='0.0')
    declare_map_prefix = DeclareLaunchArgument(
        name='map_prefix',
        default_value='',
        description='Output prefix for map_saver_cli. If empty, uses ~/rosnav/<world_name>_map')

    # ── Robot State Publisher ─────────────────────────────────────────────
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'urdf': os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
        }.items())

    # ── Gazebo server (headless physics) ─────────────────────────────────
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r -s -v1 ', world],
            'on_exit_shutdown': 'true'
        }.items())

    # ── Gazebo client (GUI) ───────────────────────────────────────────────
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g'}.items())

    # ── Spawn robot ───────────────────────────────────────────────────────
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name',  robot_name,
            '-x', spawn_x, '-y', spawn_y,
            '-z', spawn_z, '-Y', spawn_yaw,
        ],
        output='screen')

    # ── ROS ↔ Gazebo bridge ───────────────────────────────────────────────
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p',
            f'config_file:={os.path.join(pkg_share, "config", "gz_bridge.yaml")}',
        ])

    # ── SLAM Toolbox (online async — builds /map live) ────────────────────
    # Delayed 5 s so Gazebo + RSP are up before SLAM subscribes to /scan
    slam = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('slam_toolbox'),
                    'launch', 'online_async_launch.py')),
            launch_arguments={
                'slam_params_file': os.path.join(
                    pkg_share, 'config', 'mapper_params_online_async.yaml'),
                'use_sim_time': 'true'
            }.items())])

    # ── Nav2 navigation only (NO map_server / NO AMCL — SLAM provides /map) ──
    # Delayed 8 s so SLAM has time to initialise and publish /map
    nav2 = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': os.path.join(pkg_share, 'config', _NAV2_PARAMS),
            }.items())])

    # ── RViz ──────────────────────────────────────────────────────────────
    rviz2 = GroupAction(
        condition=IfCondition(rviz),
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'bot.rviz')],
            output='screen')])

    save_hint = OpaqueFunction(function=_build_save_hint, args=[home])

    return LaunchDescription([
        declare_world, declare_rviz, declare_robot_name,
        declare_spawn_x, declare_spawn_y, declare_spawn_z, declare_spawn_yaw,
        declare_map_prefix,
        save_hint,
        rsp,
        gazebo_server,
        gazebo_client,
        ros_gz_bridge,
        spawn_robot,
        slam,
        nav2,
        rviz2,
    ])
