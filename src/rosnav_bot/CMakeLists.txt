cmake_minimum_required(VERSION 3.8)
project(rosnav_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/DockToStation.action"
  "action/UndockFromStation.action"
  "srv/GetRobotStatus.srv"
)
ament_export_dependencies(rosidl_default_runtime)

# rosidl_generator_py already ament_python_install_package()'s ${PROJECT_NAME}
# from the generated tree. Overlay the existing library modules into that package.
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rosnav_bot/rmf
               ${CMAKE_CURRENT_SOURCE_DIR}/rosnav_bot/rl
  DESTINATION ${PYTHON_INSTALL_DIR}/rosnav_bot
)
install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/rosnav_bot/__init__.py
  DESTINATION ${PYTHON_INSTALL_DIR}/rosnav_bot
)

install(
  PROGRAMS
    scripts/navigation.py
    scripts/pid_controller.py
    scripts/llm_nav.py
    scripts/bt_server.py
    scripts/path_planning.py
    scripts/waypoint_nav.py
    scripts/frontier_explorer.py
    scripts/check_odometry.py
    scripts/reset_pose.py
    scripts/odom_tf_broadcaster.py
    scripts/tf_static_relay.py
    scripts/tf_map_relay.py
    scripts/multi_teleop.py
    scripts/fleet_manager.py
    scripts/fleet_gui.py
    scripts/collision_monitor.py
    scripts/mission_server.py
    scripts/coverage_planner.py
    scripts/task_allocator.py
    scripts/obstacle_tracker.py
    scripts/fleet_health.py
    scripts/frontier_coordinator.py
    scripts/map_fusion.py
    scripts/map_merge_known.py
    scripts/collab_loop_closure.py
    scripts/multi_map_merge.py
    scripts/priority_collision_avoidance.py
    scripts/deadlock_recovery.py
    scripts/cbs_planner.py
    scripts/bt_executor.py
    scripts/multi_coverage.py
    scripts/aruco_dock.py
    scripts/station_server.py
    scripts/yolo_detector.py
    scripts/yolo_collect.py
    scripts/yolo_train.py
    scripts/gs_capture.py
    scripts/gs_view_pointcloud.py
    scripts/gs_mask_from_splat.py
    scripts/gs_semantic_fusion.py
    scripts/gs_speed_mask_from_splat.py
    scripts/gs_merge_splats.py
    scripts/gs_suggest_capture_waypoints.py
    scripts/gs_relocalization_check.py
    scripts/gs_train_keepout.sh
    scripts/train_ppo.py
    scripts/rl_policy_node.py
    scripts/rmf_fleet_adapter.py
    scripts/rmf_submit_task.py
    scripts/dynamic_obstacle_driver.py
    scripts/pointcloud_to_scan.py
    scripts/scan_quality_gate.py
    scripts/slam_accuracy_monitor.py
    scripts/benchmark.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch urdf config worlds rviz maps bt_editor models meshes
  DESTINATION share/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
