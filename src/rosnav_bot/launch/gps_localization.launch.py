"""Dual-EKF + navsat_transform GPS localization (robot_localization).

NOT wired into slam_nav.launch.py yet and NOT run in a sim — static checks only.

Graph (per the robot_localization docs):
  /odom + /imu            -> ekf_filter_node_odom -> /odometry/filtered, odom->base_link
  /gps/fix + /imu + /odometry/filtered
                          -> navsat_transform_node -> /odometry/gps
  /odom + /imu + /odometry/gps
                          -> ekf_filter_node_map  -> /odometry/global, map->odom

Needs ROSNAV_GPS=1 so the robot has the NavSat sensor and the /gps/fix bridge
(urdf/gps.xacro, config/gz_bridge_gps.yaml), and a world with <spherical_coordinates>
(outdoor.world, garden.world). The map-frame EKF owns map->odom, so do not start SLAM's
map->odom publisher alongside it.

Datum: by default navsat_transform takes the first valid fix as the map origin
(wait_for_datum:=false). To pin the map to a fixed lat/lon/yaw (e.g. the world's
spherical origin), pass datum:="[lat, lon, yaw_rad]" — it is then set via the `datum`
parameter and the node waits for it (wait_for_datum:=true).

  ros2 launch rosnav_bot gps_localization.launch.py
  ros2 launch rosnav_bot gps_localization.launch.py datum:="[47.3769, 8.5417, 0.0]"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory('rosnav_bot')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in ('true', '1')
    datum_raw = LaunchConfiguration('datum').perform(context).strip()
    datum = [float(v) for v in datum_raw.strip('[]').split(',') if v.strip()] if datum_raw else []
    if datum and len(datum) != 3:
        raise RuntimeError(f'datum must be "[lat, lon, yaw_rad]", got {datum_raw!r}')

    navsat_params = {
        'use_sim_time': use_sim_time,
        'frequency': 10.0,
        'delay': 3.0,
        'magnetic_declination_radians': 0.0,
        'yaw_offset': 0.0,           # IMU yaw 0 = east (REP-103 ENU), as published by the gz imu
        'zero_altitude': True,
        'broadcast_utm_transform': False,
        'publish_filtered_gps': True,
        'use_odometry_yaw': False,
        'wait_for_datum': bool(datum),
    }
    if datum:
        navsat_params['datum'] = datum

    nodes = [
        LogInfo(msg=f'[gps_localization] datum={datum or "first fix"} use_sim_time={use_sim_time}'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node_odom',
             output='screen',
             parameters=[os.path.join(pkg, 'config', 'ekf_gps_local.yaml'), {'use_sim_time': use_sim_time}],
             remappings=[('odometry/filtered', 'odometry/filtered')]),
        Node(package='robot_localization', executable='navsat_transform_node',
             name='navsat_transform_node', output='screen',
             parameters=[navsat_params],
             remappings=[('imu', 'imu'),
                         ('gps/fix', 'gps/fix'),
                         ('odometry/filtered', 'odometry/filtered'),
                         ('odometry/gps', 'odometry/gps')]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node_map',
             output='screen',
             parameters=[os.path.join(pkg, 'config', 'ekf_gps_global.yaml'), {'use_sim_time': use_sim_time}],
             remappings=[('odometry/filtered', 'odometry/global')]),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'datum', default_value='',
            description='Optional map origin "[lat, lon, yaw_rad]"; empty = first valid GPS fix.'),
        OpaqueFunction(function=_setup),
    ])
