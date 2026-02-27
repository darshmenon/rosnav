Generated maps are saved here (map_*.yaml / map_*.pgm).

Naming convention: map_<world_name>.yaml  (e.g. map_maze.yaml, map_obstacles.yaml)

To generate a fresh map for any world:
  ros2 launch diff_drive_robot slam_nav.launch.py world_name:=maze explore:=true

To manually save the current SLAM map:
  ros2 run nav2_map_server map_saver_cli -f src/diff_drive_robot-main/maps/map_maze

To use fleet_manager to save:
  ros2 run diff_drive_robot fleet_manager.py savemap src/diff_drive_robot-main/maps/map_maze
