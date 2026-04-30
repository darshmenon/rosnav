#!/usr/bin/env python3
"""
multi_robot.launch.py  —  Scalable multi-robot navigation in Gazebo.

Architecture
────────────
• ROBOTS list is the single source of truth.  Add/remove dicts to change the
  fleet size; the rest of the launch adapts automatically.

• When explore:=true (default):
    - A single SLAM Toolbox instance (driven by robot1) builds the shared /map.
    - robot1 uses SLAM for localisation (no AMCL needed).
    - All other robots localise on that map via their own AMCL node.
    - A single frontier_coordinator assigns unique frontiers to each robot —
      no two robots ever target the same unexplored area.
    - Map is auto-saved when exploration finishes.

• When explore:=false:
    - A static map_server publishes a pre-built map.
    - Every robot runs its own AMCL for localisation.

• Nav2 params use a single template file (nav2_multirobot_params.yaml).
  The placeholder ROBOT_NS is substituted at launch — no per-robot YAML files.

Usage
─────
  # SLAM + frontier exploration in maze (default)
  ros2 launch diff_drive_robot multi_robot.launch.py

  # Pre-built map mode
  ros2 launch diff_drive_robot multi_robot.launch.py explore:=false

  # Different world
  ros2 launch diff_drive_robot multi_robot.launch.py world:=obstacles

  # Send a nav goal to a specific robot
  ros2 action send_goal /robot1/navigate_to_pose nav2_msgs/action/NavigateToPose \\
    "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 1.0}}}}"

  # Add robot3: just append to ROBOTS list below — no other changes needed.
"""

import os
import tempfile
import json
import math
import yaml

from ament_index_python.packages import get_package_share_directory

ROS_DISTRO = os.environ.get('ROS_DISTRO', 'humble')
# Behaviour plugin format differs between distros (/ vs ::)
_MR_PARAMS = (
    'nav2_multirobot_params_jazzy.yaml'
    if ROS_DISTRO == 'jazzy'
    else 'nav2_multirobot_params.yaml'
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription,
    LogInfo, OpaqueFunction, TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ──────────────────────────────────────────────────────────────────────────────
# Fleet configuration — ONLY thing you need to edit to add more robots.
# ──────────────────────────────────────────────────────────────────────────────
ROBOTS = [
    {'name': 'robot1', 'x': '-2.0', 'y': '-1.0', 'z': '0.3', 'yaw': '0.0'},
    {'name': 'robot2', 'x': '-0.8', 'y': '-1.0', 'z': '0.3', 'yaw': '0.0'},
    # Add more robots here — zero other file changes required:
    # {'name': 'robot3', 'x':  '0.5', 'y': '-1.0', 'z': '0.3', 'yaw': '0.0'},
]


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────
def _make_robot_params(
    template_path: str,
    robot_ns: str,
    initial_pose: dict | None = None,
) -> str:
    """Substitute ROBOT_NS in the template YAML and return a temp-file path."""
    with open(template_path) as f:
        content = f.read()
    content = content.replace('ROBOT_NS', robot_ns)
    if initial_pose is not None:
        params = yaml.safe_load(content)
        amcl_params = params.setdefault('amcl', {}).setdefault('ros__parameters', {})
        amcl_params['set_initial_pose'] = True
        amcl_params['initial_pose'] = initial_pose
        content = yaml.safe_dump(params, sort_keys=False)
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix=f'_{robot_ns}.yaml', delete=False, prefix='nav2_mr_')
    tmp.write(content)
    tmp.close()
    return tmp.name


def _resolve_map_file(map_arg: str, world_path: str, pkg_share: str) -> str:
    if map_arg:
        return map_arg
    world_name = os.path.splitext(os.path.basename(world_path))[0]
    home = os.path.expanduser('~')
    candidates = [
        os.path.join(pkg_share, 'maps', f'map_{world_name}.yaml'),
        os.path.join(pkg_share, 'maps', f'{world_name}_map.yaml'),
        os.path.join(home, 'rosnav', 'maps', f'map_{world_name}.yaml'),
        os.path.join(pkg_share, 'maps', 'my_map.yaml'),
    ]
    for c in candidates:
        if os.path.exists(c):
            return c
    return candidates[0]


def _initial_pose_pub(robot_ns: str, x: str, y: str, yaw: str) -> ExecuteProcess:
    yaw_f = float(yaw)
    qz = math.sin(yaw_f / 2.0)
    qw = math.cos(yaw_f / 2.0)
    msg = json.dumps({
        'header': {'frame_id': 'map'},
        'pose': {
            'pose': {
                'position': {'x': float(x), 'y': float(y), 'z': 0.0},
                'orientation': {'z': qz, 'w': qw},
            },
            'covariance': [
                0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942,
            ],
        },
    })
    return ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            f'/{robot_ns}/initialpose',
            'geometry_msgs/msg/PoseWithCovarianceStamped',
            msg,
        ],
        output='screen',
    )


# ──────────────────────────────────────────────────────────────────────────────
# Main launch builder
# ──────────────────────────────────────────────────────────────────────────────
def _build_all(context, pkg_share: str):
    map_arg      = LaunchConfiguration('map').perform(context).strip()
    world_arg    = LaunchConfiguration('world').perform(context).strip()
    explore      = LaunchConfiguration('explore').perform(context).strip().lower() in ('true', '1', 'yes')
    headless     = LaunchConfiguration('headless').perform(context).strip().lower() in ('true', '1', 'yes')
    fleet_mgmt   = LaunchConfiguration('fleet_mgmt').perform(context).strip().lower() in ('true', '1', 'yes')

    slam_pkg     = get_package_share_directory('slam_toolbox')
    ros_gz       = get_package_share_directory('ros_gz_sim')

    # Resolve world file
    if os.path.isabs(world_arg) and os.path.isfile(world_arg):
        world_path = world_arg
    else:
        world_name = world_arg or 'maze'
        # Allow bare name ("maze") or filename ("maze.world")
        world_name = os.path.splitext(os.path.basename(world_name))[0]
        world_path = os.path.join(pkg_share, 'worlds', f'{world_name}.world')

    world_stem   = os.path.splitext(os.path.basename(world_path))[0]
    map_prefix   = os.path.join(pkg_share, 'maps', f'map_{world_stem}')
    template_yaml = os.path.join(pkg_share, 'config', _MR_PARAMS)

    actions = [
        LogInfo(msg=f'[multi_robot] world = {world_path}'),
        LogInfo(msg=f'[multi_robot] fleet = {[r["name"] for r in ROBOTS]}'),
        LogInfo(msg=f'[multi_robot] explore = {explore}'),
    ]

    # ── Gazebo server + GUI ───────────────────────────────────────────────────
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': f'-r -s -v1 {world_path}',
            'on_exit_shutdown': 'true',
        }.items()))
    if not headless:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': '-g'}.items()))

    # Bridge the Gazebo clock exactly once.
    # Multiple /clock bridges can race and produce backward time jumps.
    actions.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'))

    # Bridge Gazebo's global TF exactly once. The Gazebo /tf stream already
    # contains all spawned robots, so duplicating this bridge per-robot causes
    # repeated transforms and cross-namespace contamination.
    actions.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='tf_bridge_global',
        arguments=['/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        output='screen'))

    # ── Shared map source ─────────────────────────────────────────────────────
    if explore:
        # SLAM Toolbox (robot1's 2D lidar → shared /map)
        actions += [
            LogInfo(msg=f'[multi_robot] SLAM mode — map will be auto-saved to {map_prefix}'),
            TimerAction(
                period=6.0,
                actions=[IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(slam_pkg, 'launch', 'online_async_launch.py')),
                    launch_arguments={
                        'slam_params_file': os.path.join(
                            pkg_share, 'config', 'mapper_params_multirobot.yaml'),
                        'use_sim_time': 'true',
                    }.items())]),
        ]
    else:
        # Static map_server
        resolved_map = _resolve_map_file(map_arg, world_path, pkg_share)
        actions += [
            LogInfo(msg=f'[multi_robot] static map = {resolved_map}'),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{'use_sim_time': True, 'yaml_filename': resolved_map}]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_map',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': ['map_server'],
                }]),
        ]

    # ── Per-robot groups ──────────────────────────────────────────────────────
    for idx, robot in enumerate(ROBOTS):
        ns  = robot['name']
        x, y, z, yaw = robot['x'], robot['y'], robot['z'], robot['yaw']
        is_slam_robot = (explore and idx == 0)  # robot1 localises via SLAM

        # Template → per-robot YAML (ROBOT_NS substituted)
        initial_pose = None
        if not is_slam_robot:
            initial_pose = {
                'x': float(x),
                'y': float(y),
                'z': float(z),
                'yaw': float(yaw),
            }
        robot_params = _make_robot_params(template_yaml, ns, initial_pose)

        # Robot State Publisher — frame_prefix + namespace arg makes TF frames unique per robot
        rsp = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'rsp.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'urdf': os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro'),
                'frame_prefix': f'{ns}/',
                'namespace': ns,
            }.items())

        # Spawn in Gazebo
        spawn = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', f'/{ns}/robot_description',
                '-name', ns,
                '-x', x, '-y', y, '-z', z, '-Y', yaw,
            ],
            output='screen')

        # Gazebo ↔ ROS bridge (2D lidar, odom, cmd_vel)
        # TF is handled by dedicated bridge nodes below to avoid the /{ns}/tf empty-topic problem.
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=ns,
            arguments=[
                f'/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                f'/{ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                f'/{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            ],
            output='screen')

        odom_tf = Node(
            package='diff_drive_robot',
            executable='odom_tf_broadcaster.py',
            namespace=ns,
            name='odom_tf_broadcaster',
            parameters=[{'use_sim_time': True}],
            output='screen')

        # Relay map→robot_ns/odom from global /tf into /{ns}/tf so Nav2 nav
        # nodes (which subscribe to the namespaced tf bus) can build the full
        # map→odom→base_link chain.  Critical for robot1 (SLAM) and harmless
        # for robot2+ where AMCL already publishes directly to /{ns}/tf.
        tf_map_relay = Node(
            package='diff_drive_robot',
            executable='tf_map_relay.py',
            namespace=ns,
            name='tf_map_relay',
            parameters=[{'use_sim_time': True}],
            output='screen')

        # AMCL — localises against /map (shared).
        # Skipped for robot1 in SLAM mode (SLAM provides the map→odom TF).
        amcl_node = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace=ns,
            output='screen',
            parameters=[robot_params, {'use_sim_time': True}],
            remappings=[
                ('map', '/map'),
                ('/map', '/map'),
            ])

        amcl_lc = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['amcl'],
            }])

        # Nav2 navigation stack (planner, controller, bt_navigator, …)
        # Pass namespace so RewrittenYaml wraps params under the robot key, making
        # /robot1/controller_server find its parameters. MPPI critics survive yaml.dump.
        nav2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'nav2_navigation_global_tf.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': robot_params,
                'namespace': ns,
            }.items())

        # Assemble per-robot actions
        nav2_group = nav2

        per_robot = [rsp, spawn, bridge, odom_tf, tf_map_relay]

        if is_slam_robot:
            # robot1 in explore mode: SLAM handles localisation
            per_robot += [
                TimerAction(period=10.0, actions=[nav2_group]),
            ]
        elif explore:
            # Other robots in explore mode: need AMCL to localise on SLAM map.
            # SLAM starts at 6s; give it 7s to publish /map before AMCL starts.
            per_robot += [
                TimerAction(period=13.0, actions=[
                    amcl_node,
                    amcl_lc,
                ]),
                TimerAction(period=16.0, actions=[_initial_pose_pub(ns, x, y, yaw)]),
                TimerAction(period=19.0, actions=[nav2_group]),
            ]
        else:
            # Pre-built map mode: all robots use AMCL
            per_robot += [
                TimerAction(period=5.0, actions=[
                    amcl_node,
                    amcl_lc,
                ]),
                TimerAction(period=8.0, actions=[_initial_pose_pub(ns, x, y, yaw)]),
                TimerAction(period=11.0, actions=[nav2_group]),
            ]

        actions.append(GroupAction(per_robot))

    # ── Centralized frontier coordinator (explore mode only) ──────────────────
    robot_ns_list = ','.join(r['name'] for r in ROBOTS)
    if explore:
        # Start after all Nav2 stacks are up (robot1 at 10s, others at 13s)
        actions.append(TimerAction(
            period=20.0,
            actions=[Node(
                package='diff_drive_robot',
                executable='frontier_coordinator.py',
                name='frontier_coordinator',
                output='screen',
                parameters=[{
                    'robot_namespaces': robot_ns_list,
                    'map_save_path': map_prefix,
                }])]))
        actions.append(LogInfo(
            msg=f'[multi_robot] frontier_coordinator will start at t=20s for {robot_ns_list}'))

    # ── Fleet management algorithms (optional) ────────────────────────────────
    if fleet_mgmt:
        actions.append(TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='diff_drive_robot',
                    executable='mission_server.py',
                    name='mission_server',
                    output='screen'),
                Node(
                    package='diff_drive_robot',
                    executable='task_allocator.py',
                    name='task_allocator',
                    output='screen',
                    parameters=[{'robots': robot_ns_list}]),
                Node(
                    package='diff_drive_robot',
                    executable='fleet_health.py',
                    name='fleet_health_monitor',
                    output='screen'),
                Node(
                    package='diff_drive_robot',
                    executable='priority_collision_avoidance.py',
                    name='priority_collision_avoidance',
                    output='screen',
                    parameters=[{'robot_namespaces': robot_ns_list}]),
                Node(
                    package='diff_drive_robot',
                    executable='deadlock_recovery.py',
                    name='deadlock_recovery',
                    output='screen',
                    parameters=[{'robot_namespaces': robot_ns_list}]),
            ]))
        actions.append(LogInfo(
            msg=f'[multi_robot] fleet_mgmt stack will start at t=15s for {robot_ns_list}'))

    # ── RViz ─────────────────────────────────────────────────────────────────
    if not headless:
        actions.append(GroupAction(
            condition=IfCondition(LaunchConfiguration('rviz')),
            actions=[Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', os.path.join(pkg_share, 'rviz', 'bot.rviz')],
                output='screen')]))

    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory('diff_drive_robot')
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='maze',
            description='World name (maze, obstacles) or full path to .world file'),
        DeclareLaunchArgument(
            'map', default_value='',
            description='Pre-built map yaml path. Ignored when explore:=true'),
        DeclareLaunchArgument(
            'rviz', default_value='True', description='Launch RViz'),
        DeclareLaunchArgument(
            'explore', default_value='true',
            description='true = SLAM + frontier exploration (default). '
                        'false = use saved map yaml'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true = Gazebo server only, no GUI or RViz'),
        DeclareLaunchArgument(
            'fleet_mgmt', default_value='false',
            description='true = start priority collision avoidance + deadlock recovery nodes'),
        OpaqueFunction(function=_build_all, args=[pkg_share]),
    ])
