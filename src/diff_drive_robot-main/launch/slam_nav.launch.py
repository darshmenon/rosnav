"""
slam_nav.launch.py
==================
One-shot launch: Gazebo + Robot + SLAM Toolbox (live mapping) + Nav2 navigation + RViz.

Default workflow (single command):
  ros2 launch diff_drive_robot slam_nav.launch.py world_name:=maze explore:=true

When exploration completes, the map is auto-saved to:
  <package_share>/maps/map_<world_name>
"""

import os

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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROS_DISTRO = os.environ.get('ROS_DISTRO', 'humble')
_NAV2_PARAMS = 'nav2_params_jazzy.yaml' if ROS_DISTRO == 'jazzy' else 'nav2_params.yaml'


def _resolve_world_name(raw_name: str, world_path: str) -> str:
    if raw_name:
        return os.path.splitext(os.path.basename(raw_name))[0]
    return os.path.splitext(os.path.basename(world_path))[0]


def _resolve_world_path(world_name_arg: str, world_arg: str, pkg_share: str) -> str:
    world_arg = world_arg.strip()
    if world_arg:
        return os.path.expanduser(world_arg)
    world_name = os.path.splitext(os.path.basename(world_name_arg.strip() or 'maze'))[0]
    return os.path.join(pkg_share, 'worlds', f'{world_name}.world')


def _resolve_map_prefix(map_prefix_arg: str, world_name: str, pkg_share: str) -> str:
    if map_prefix_arg:
        return os.path.expanduser(map_prefix_arg)
    return os.path.join(pkg_share, 'maps', f'map_{world_name}')


def _build_runtime_actions(context, pkg_share: str):
    world_name_arg = LaunchConfiguration('world_name').perform(context)
    world_arg = LaunchConfiguration('world').perform(context)
    rviz = LaunchConfiguration('rviz')
    explore = LaunchConfiguration('explore')
    robot_name = LaunchConfiguration('robot_name')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')

    world_path = _resolve_world_path(world_name_arg, world_arg, pkg_share)
    world_name = _resolve_world_name(world_name_arg, world_path)
    map_prefix = _resolve_map_prefix(
        LaunchConfiguration('map_prefix').perform(context).strip(), world_name, pkg_share)

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'rsp.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'urdf': os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro'),
        }.items(),
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -s -v1 {world_path}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g'}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', robot_name,
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
        ],
        output='screen',
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={os.path.join(pkg_share, "config", "gz_bridge.yaml")}',
        ],
    )

    slam = TimerAction(
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

    # Patch the BT path placeholder before passing params to nav2.
    # Use a fixed path so repeated launches overwrite rather than accumulate files.
    _raw_params = os.path.join(pkg_share, 'config', _NAV2_PARAMS)
    import re as _re
    with open(_raw_params) as _f:
        _patched = _re.sub(r'replace_with_pkg_share', pkg_share.replace('\\', '/'), _f.read())
    _params_file = f'/tmp/diff_drive_nav2_patched_{os.getpid()}.yaml'
    with open(_params_file, 'w') as _f:
        _f.write(_patched)

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

    # ── Velocity Smoother (starts 2s after nav2) ──────────────────────────
    # Subscribes to /cmd_vel (controller output), publishes /cmd_vel_smoothed.
    # gz_bridge also forwards cmd_vel_smoothed → Gazebo so the robot receives
    # the jerk-limited stream.  cmd_vel is still bridged as fallback.
    velocity_smoother = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                parameters=[_params_file],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_velocity_smoother',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': ['velocity_smoother'],
                }],
            ),
        ],
    )

    # ── RViz ──────────────────────────────────────────────────────────────
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
                    package='diff_drive_robot',
                    executable='collision_monitor.py',
                    name='collision_monitor',
                    output='screen',
                    parameters=[{
                        'stop_distance':     0.30,
                        'slowdown_distance': 0.70,
                        'front_angle_deg':   60.0,
                        'watch_all_around':  False,
                    }],
                )]
            ),
        ]
    )

    # ── Mission Layer: Mission Server ──────────────────────────────────────
    mission_server = TimerAction(
        period=15.0,
        actions=[Node(
            package='diff_drive_robot',
            executable='mission_server.py',
            name='mission_server',
            output='screen',
        )]
    )

    # ── Frontier Explorer (Auto-run) ──────────────────────────────────────
    frontier_node = GroupAction(
        condition=IfCondition(explore),
        actions=[
            LogInfo(msg="[slam_nav] Auto-exploration ENABLED. Starting frontier_explorer in 12s..."),
            TimerAction(
                period=12.0,
                actions=[Node(
                    package='diff_drive_robot',
                    executable='frontier_explorer.py',
                    name='frontier_explorer',
                    output='screen',
                    # Pass the computed map prefix to the explorer so it auto-saves there
                    parameters=[{'map_save_path': map_prefix}]
                )]
            )
        ]
    )

    return [
        LogInfo(msg=f'[slam_nav.launch] ROS_DISTRO={ROS_DISTRO}, params={_NAV2_PARAMS}'),
        LogInfo(msg=f'[slam_nav.launch] world={world_path}'),
        LogInfo(msg=f'[slam_nav.launch] robot_name={robot_name.perform(context)}'),
        LogInfo(msg=f'[slam_nav.launch] save map with: ros2 run nav2_map_server map_saver_cli -f {map_prefix}'),
        LogInfo(msg=f'[slam_nav.launch] explore={explore.perform(context)}'),
        rsp,
        gazebo_server,
        gazebo_client,
        ros_gz_bridge,
        spawn_robot,
        slam,
        nav2,
        velocity_smoother,
        rviz2,
        collision_monitor,
        mission_server,
        frontier_node,
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('diff_drive_robot')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value='maze',
            description='Gazebo world name in package worlds/ (example: maze or obstacles)',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='',
            description='Optional full world path override (if set, world_name is ignored)',
        ),
        DeclareLaunchArgument('rviz', default_value='True', description='Launch RViz'),
        DeclareLaunchArgument('robot_name', default_value='diff_drive', description='Gazebo robot entity name'),
        # Maze default spawn moved away from origin so robot is immediately visible.
        DeclareLaunchArgument(name='spawn_x', default_value='1.5'),
        DeclareLaunchArgument(name='spawn_y', default_value='1.0'),
        DeclareLaunchArgument(name='spawn_z', default_value='0.3'),
        DeclareLaunchArgument(name='spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            name='map_prefix',
            default_value='',
            description='Output prefix for map_saver_cli. If empty, uses <package_share>/maps/map_<world_name>'),
        DeclareLaunchArgument(
            name='explore', default_value='false',
            description='Auto-start frontier explorer and map saving when true'),
        DeclareLaunchArgument(
            name='safety', default_value='true',
            description='Launch collision monitor safety layer'),
        OpaqueFunction(function=_build_runtime_actions, args=[pkg_share]),
    ])
