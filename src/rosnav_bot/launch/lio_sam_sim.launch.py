"""
lio_sam_sim.launch.py - LIO-SAM (built outside this repo, e.g. ~/lio_ws) on the sim's 3D lidar + IMU.

Needs: slam_nav.launch.py lidar_type:=3d running, and `source ~/lio_ws/install/setup.bash`.
Starts lidar_ring_time_adapter (adds ring/time to /points -> /points_lio) and the four LIO-SAM nodes.
LIO-SAM's TF goes to /lio_tf (its /tf is remapped) so the robot's TF tree is never touched; its pose is on
/lio_sam/mapping/odometry (frame lio_odom), scored by localization_accuracy_bench.py as `lio_sam`.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('rosnav_bot')
    cfg = os.path.join(pkg, 'config', 'lio_sam_sim.yaml')
    sim = {'use_sim_time': True}
    remap = [('/tf', '/lio_tf'), ('/tf_static', '/lio_tf_static')]
    nodes = [LogInfo(msg='[lio_sam_sim] adapter + LIO-SAM (tf -> /lio_tf, odom -> /lio_sam/mapping/odometry)'),
             Node(package='rosnav_bot', executable='lidar_ring_time_adapter.py', name='lidar_ring_time_adapter',
                  output='screen', parameters=[sim])]
    # lio_base -> laser_frame: the real lidar mount (chassis +0.1 m forward, +0.25 m up), inside the private /lio_tf tree
    nodes.append(Node(package='tf2_ros', executable='static_transform_publisher', name='lio_base_to_lidar',
                      arguments=['0.1', '0', '0.25', '0', '0', '0', 'lio_base', 'laser_frame'],
                      remappings=remap, parameters=[sim]))
    for exe in ('lio_sam_imuPreintegration', 'lio_sam_imageProjection',
                'lio_sam_featureExtraction', 'lio_sam_mapOptimization'):
        nodes.append(Node(package='lio_sam', executable=exe, name=exe, output='screen',
                          parameters=[cfg, sim], remappings=remap))
    return LaunchDescription(nodes)
