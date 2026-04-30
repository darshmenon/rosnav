from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command


def generate_launch_description():

    # Package name
    package_name = FindPackageShare("diff_drive_robot")

    # Default robot description if none is specified
    urdf_path = PathJoinSubstitution([package_name, "urdf", "robot.urdf.xacro"])

    # Launch configurations
    urdf = LaunchConfiguration('urdf')
    use_sim_time = LaunchConfiguration('use_sim_time')
    frame_prefix = LaunchConfiguration('frame_prefix')
    namespace = LaunchConfiguration('namespace')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use sim time if true')

    declare_urdf = DeclareLaunchArgument(
            name='urdf', default_value=urdf_path,
            description='Path to the robot description file')

    declare_frame_prefix = DeclareLaunchArgument(
            name='frame_prefix', default_value='',
            description='TF frame prefix for multi-robot setups (e.g. "robot1/")')

    declare_namespace = DeclareLaunchArgument(
            name='namespace', default_value='',
            description='Robot namespace passed to xacro for TF frame IDs (e.g. "robot1")')

    # Create a robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro ', urdf, ' namespace:=', namespace]),
                value_type=str),
            'frame_prefix': frame_prefix,
        }]
    )

    # Launch!
    return LaunchDescription([
        declare_urdf,
        declare_use_sim_time,
        declare_frame_prefix,
        declare_namespace,
        robot_state_publisher
    ])
