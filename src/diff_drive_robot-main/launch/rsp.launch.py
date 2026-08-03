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
    lidar_type = LaunchConfiguration('lidar_type')
    lidar3d_height = LaunchConfiguration('lidar3d_height')
    lidar3d_vfov_deg = LaunchConfiguration('lidar3d_vfov_deg')

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

    declare_lidar_type = DeclareLaunchArgument(
            name='lidar_type', default_value='2d',
            description='"2d" (LaserScan on scan) or "3d" (PointCloud2 on points) — '
                        'only robot.urdf.xacro (drive_type:=diff) supports 3d')

    declare_lidar3d_height = DeclareLaunchArgument(
            name='lidar3d_height', default_value='0.25',
            description='3D lidar mount height (m) above base_link, lidar_type:=3d only. '
                        'Needs >=0.10m clearance above the chassis top (z=0.15) or the '
                        'sensor sees its own chassis at close range — see lidar3d.xacro.')

    declare_lidar3d_vfov_deg = DeclareLaunchArgument(
            name='lidar3d_vfov_deg', default_value='10',
            description='3D lidar vertical half-angle in degrees (+/-), lidar_type:=3d only.')

    # Create a robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro ', urdf, ' namespace:=', namespace,
                         ' lidar_type:=', lidar_type,
                         ' lidar3d_height:=', lidar3d_height,
                         ' lidar3d_vfov_deg:=', lidar3d_vfov_deg]),
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
        declare_lidar_type,
        declare_lidar3d_height,
        declare_lidar3d_vfov_deg,
        robot_state_publisher
    ])
