import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('diff_drive_robot')

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ]),
        launch_arguments=[
            ('slam_params_file', os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')),
            ('use_sim_time', 'true')
        ],
    )

    return LaunchDescription([
        slam_launch
    ])
