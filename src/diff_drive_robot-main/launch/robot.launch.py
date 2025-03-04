import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction

def generate_launch_description():

    # Package name
    package_name = 'diff_drive_robot'

    # Launch configurations
    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')

    # Path to default world 
    world_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'obstacles.world')

    # Launch Arguments
    declare_world = DeclareLaunchArgument(
        name='world', default_value=world_path,
        description='Full path to the world model file to load')
    
    declare_rviz = DeclareLaunchArgument(
        name='rviz', default_value='True',
        description='Opens rviz if set to True')

    # Launch Robot State Publisher Node
    urdf_path = os.path.join(get_package_share_directory(package_name), 'urdf', 'robot.urdf.xacro')
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ]), launch_arguments={'use_sim_time': 'true', 'urdf': urdf_path}.items()
    )

    # Launch the Gazebo server to initialize the simulation
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]), launch_arguments={'gz_args': ['-r -s -v1 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]), launch_arguments={'gz_args': '-g '}.items()
    )

    # Spawn the robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'diff_bot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.3',
        ],
        output='screen'
    )

    # Launch the Gazebo-ROS bridge
    bridge_params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )
    
    # Launch Rviz with diff bot rviz file
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'bot.rviz')
    rviz2 = GroupAction(
        condition=IfCondition(rviz),
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen',
        )]
    )
    
    navigation_node = Node(
        package='diff_drive_robot',
        executable='navigation.py',
        name='obstacle_avoidance_navigator',  
        output='screen'
    )
    
    path_planning_node = Node(
        package='diff_drive_robot',
        executable='path_planning.py',
        name='path_planning',
        output='screen'
    )

    nav2_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
    ]),
       launch_arguments={
            'use_sim_time': 'true',
            'params_file': os.path.join(
                get_package_share_directory('diff_drive_robot'), 'config', 'nav2_params.yaml'
            )
        }.items()
)


    # nav2_node = Node(
    #     package='diff_drive_robot',  
    #     executable='nav2.py',
    #     output='screen'
    # )
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={
            'slam_params_file': os.path.join(get_package_share_directory(package_name), 'config', 'mapper_params_online_async.yaml'),
            'use_sim_time': 'true'
        }.items(),
    )
    return LaunchDescription([
        declare_rviz,
        declare_world,
        rviz2,
        rsp,
        gazebo_server,
        gazebo_client,
        ros_gz_bridge,
        spawn_robot,
        # navigation_node,
        # nav2_node,    
        nav2_launch
        # slam_launch
    ])
