import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml

ROS_DISTRO = os.environ.get('ROS_DISTRO', 'humble')
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
    raw_params = os.path.join(pkg_share, 'config', _NAV2_PARAMS)
    bt_xml = os.path.join(pkg_share, 'config', 'bt', 'navigate_w_recovery.xml')

    params_file = RewrittenYaml(
        source_file=raw_params,
        root_key='',
        param_rewrites={'default_nav_to_pose_bt_xml': bt_xml},
        convert_types=True,
    ).perform(context)

    return [
        LogInfo(msg=f'[nav2.launch] ROS_DISTRO={ROS_DISTRO}, params={os.path.basename(raw_params)}'),
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
        description='Map yaml path. If empty, auto-use <package_share>/maps/map_<world_name>.yaml (legacy fallbacks still supported)')

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
