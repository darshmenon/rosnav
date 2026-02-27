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
)

# ---------------------------------------------------------------------------
# Distro-agnostic helper: works on Humble (GZ Fortress/Garden) and Jazzy (GZ Harmonic)
# Both distros use the 'ros_gz_sim' / 'ros_gz_bridge' package names; the
# underlying Gazebo version is selected by the environment automatically.
# We just need to note the distro for any args that differ.
ROS_DISTRO = os.environ.get('ROS_DISTRO', 'humble')

# Auto-select nav2 params based on distro:
#   Humble: nav2_params.yaml  (behaviors use /Spin format)
#   Jazzy:  nav2_params_jazzy.yaml  (behaviors use ::Spin format)
_NAV2_PARAMS = 'nav2_params_jazzy.yaml' if ROS_DISTRO == 'jazzy' else 'nav2_params.yaml'


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


def _build_nav2_action(context, pkg_share: str, home: str):
    world_path = LaunchConfiguration('world').perform(context)
    map_arg = LaunchConfiguration('map').perform(context).strip()
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    map_file = _resolve_map_file(map_arg, world_path, home, pkg_share)
    params_file = os.path.join(pkg_share, 'config', _NAV2_PARAMS)

    return [
        LogInfo(msg=f'[robot.launch] ROS_DISTRO={ROS_DISTRO}, params={os.path.basename(params_file)}'),
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
        name='spawn_z', default_value='0.3',
        description='Robot spawn Z position')

    declare_spawn_yaw = DeclareLaunchArgument(
        name='spawn_yaw', default_value='0.0',
        description='Robot spawn yaw (radians)')

    declare_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value='true',
        description='Use simulation clock')

    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'urdf': os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
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

    # Gazebo client (GUI)
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g'}.items()
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name',  robot_name,
            '-x',     spawn_x,
            '-y',     spawn_y,
            '-z',     spawn_z,
            '-Y',     spawn_yaw,
        ],
        output='screen'
    )

    # Gazebo <-> ROS bridge
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p',
            f'config_file:={os.path.join(pkg_share, "config", "gz_bridge.yaml")}',
        ]
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
    nav2_launch = OpaqueFunction(function=_build_nav2_action, args=[pkg_share, home])

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

    return LaunchDescription([
        # ── Declare ALL arguments first (BUG FIX: these were missing) ──
        declare_world,
        declare_rviz,
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
        spawn_robot,
        rviz2,
        nav2_launch,
        # slam_launch,
        # navigation_node,
        # path_planning_node,
    ])
