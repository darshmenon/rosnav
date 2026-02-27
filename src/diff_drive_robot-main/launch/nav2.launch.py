import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

ROS_DISTRO = os.environ.get('ROS_DISTRO', 'humble')
_NAV2_PARAMS = 'nav2_params_jazzy.yaml' if ROS_DISTRO == 'jazzy' else 'nav2_params.yaml'


def _resolve_map_file(map_arg: str, world_path: str, home: str) -> str:
    if map_arg:
        return map_arg
    world_name = os.path.splitext(os.path.basename(world_path))[0]
    world_map = os.path.join(home, 'rosnav', f'{world_name}_map.yaml')
    default_map = os.path.join(home, 'rosnav', 'my_map.yaml')
    return world_map if os.path.exists(world_map) else default_map


def _build_nav2_action(context, pkg_share: str, home: str):
    world_path = LaunchConfiguration('world').perform(context)
    map_arg = LaunchConfiguration('map').perform(context).strip()
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    map_file = _resolve_map_file(map_arg, world_path, home)
    params_file = os.path.join(pkg_share, 'config', _NAV2_PARAMS)

    return [
        LogInfo(msg=f'[nav2.launch] ROS_DISTRO={ROS_DISTRO}, params={os.path.basename(params_file)}'),
        LogInfo(msg=f'[nav2.launch] using map={map_file}'),
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
    home = os.path.expanduser('~')
    pkg_share = get_package_share_directory('diff_drive_robot')

    declare_map = DeclareLaunchArgument(
        name='map',
        default_value='',
        description='Map yaml path. If empty, auto-use ~/rosnav/<world_name>_map.yaml then fallback to ~/rosnav/my_map.yaml')

    declare_world = DeclareLaunchArgument(
        name='world',
        default_value='obstacles.world',
        description='World name used only for auto map selection when map is empty')

    declare_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation clock if true')

    nav2_bringup = OpaqueFunction(function=_build_nav2_action, args=[pkg_share, home])

    return LaunchDescription([
        declare_map,
        declare_world,
        declare_sim_time,
        nav2_bringup,
    ])
