from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command


def generate_launch_description():

    # Package name
    package_name = FindPackageShare("rosnav_bot")

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
    enable_camera = LaunchConfiguration('enable_camera')
    enable_rgbd = LaunchConfiguration('enable_rgbd')

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

    declare_enable_camera = DeclareLaunchArgument(
            name='enable_camera', default_value='false',
            description='Include the RGB camera sensor (used by aruco_dock.py and '
                        'yolo_detector.py). Off by default to save render cost — set '
                        'true when you need docking or YOLO object detection.')

    declare_enable_rgbd = DeclareLaunchArgument(
            name='enable_rgbd', default_value='false',
            description='Include RGB-D camera (rgbd_camera.xacro) instead of RGB. '
                        'Forced true for slam_algo:=vslam. Publishes depth for RTAB-Map '
                        'and remaps color to /camera/image_raw for YOLO/ArUco.')

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
                         ' lidar3d_vfov_deg:=', lidar3d_vfov_deg,
                         ' enable_camera:=', enable_camera,
                         ' enable_rgbd:=', enable_rgbd]),
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
        declare_enable_camera,
        declare_enable_rgbd,
        robot_state_publisher
    ])
