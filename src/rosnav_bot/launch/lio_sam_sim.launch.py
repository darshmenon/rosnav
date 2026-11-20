"""
lio_sam_sim.launch.py - LIO-SAM (built outside this repo, e.g. ~/lio_ws) on the sim's 3D lidar + IMU.

Needs: slam_nav.launch.py lidar_type:=3d running, and `source ~/lio_ws/install/setup.bash`.
Starts imu_yaw_relay (/imu -> /imu_rot), lidar_ring_time_adapter (adds ring/time to /points -> /points_lio) and the four LIO-SAM nodes.
LIO-SAM's TF goes to /lio_tf (its /tf is remapped) so the robot's TF tree is never touched; its pose is on
/lio_sam/mapping/odometry (frame lio_odom), scored by localization_accuracy_bench.py as `lio_sam`.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('rosnav_bot')
    cfg = os.path.join(pkg, 'config', 'lio_sam_sim.yaml')
    sim = {'use_sim_time': True}
    remap = [('/tf', '/lio_tf'), ('/tf_static', '/lio_tf_static')]
    nodes = [LogInfo(msg='[lio_sam_sim] adapter + LIO-SAM (tf -> /lio_tf, odom -> /lio_sam/mapping/odometry)'),
             Node(package='rosnav_bot', executable='lidar_ring_time_adapter.py', name='lidar_ring_time_adapter',
                  output='screen', parameters=[sim]),
             # IMU yaw vs odom is measured at startup (auto_cal: -90 deg in the old cafe bag, 0 deg live) and corrected on /imu_rot
             Node(package='rosnav_bot', executable='imu_yaw_relay.py', name='imu_yaw_relay', output='screen',
                  parameters=[sim, {'yaw_deg': 0.0, 'auto_cal': True, 'ref_odom': '/odom'}])]
    # lio_base -> laser_frame: the real lidar mount (chassis +0.1 m forward, +0.25 m up), inside the private /lio_tf tree
    nodes.append(Node(package='tf2_ros', executable='static_transform_publisher', name='lio_base_to_lidar',
                      arguments=['0.1', '0', '0.25', '0', '0', '0', 'lio_base', 'laser_frame'],
                      remappings=remap, parameters=[sim]))
    for exe in ('lio_sam_imuPreintegration', 'lio_sam_imageProjection',
                'lio_sam_featureExtraction', 'lio_sam_mapOptimization'):
        nodes.append(Node(package='lio_sam', executable=exe, name=exe, output='screen',
                          parameters=[cfg, sim], remappings=remap))
    # live health text for RViz (lio_doctor on PATH or PYTHONPATH); harmless no-op if it is not installed
    nodes.append(ExecuteProcess(
        cmd=['python3', '-c',
             'import sys\ntry:\n from lio_doctor.doctor import main\nexcept ImportError:\n print("[lio_sam_sim] lio_doctor not importable, no /lio_health")\n sys.exit(0)\n'
             'sys.exit(main(["--imu","/imu_rot","--cloud","/points_lio","--ref-odom","/odom","--interval","2","--health-topic","/lio_health"]))'],
        name='lio_health', output='screen',
        # source-tree fallback: with --symlink-install this file resolves into src/, next to tools/lio_doctor
        additional_env={'PYTHONPATH': os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'tools', 'lio_doctor')
                        + os.pathsep + os.environ.get('PYTHONPATH', '')}))
    return LaunchDescription(nodes)
