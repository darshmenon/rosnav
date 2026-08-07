Generated maps are saved here (map_*.yaml / map_*.pgm).

Naming convention: map_<world_name>.yaml  (e.g. map_maze.yaml, map_obstacles.yaml)

Available worlds
----------------
  maze        — walled maze, default SLAM test world
  obstacles   — open area with scattered obstacles
  warehouse   — 24×20 m warehouse with shelves and staging zone
  house       — residential floor plan
  corridor    — long straight corridors
  empty       — flat open space
  office      — 22×16 m open-plan office (lobby, meeting rooms, cubicles, server room)
  hospital    — 26×18 m hospital floor (patient rooms, nurse station, 4 m corridor)

Multi-robot launch

Launch Gazebo with the GUI:
  source /opt/ros/humble/setup.bash
  source ~/rosnav/install/setup.bash
  ros2 launch rosnav_bot multi_robot.launch.py

Launch headless:
  source /opt/ros/humble/setup.bash
  source ~/rosnav/install/setup.bash
  ros2 launch rosnav_bot multi_robot.launch.py headless:=true rviz:=false

Before relaunching, stop any older stack first to avoid duplicate /clock and TF publishers:
  pkill -f "ros2 launch rosnav_bot multi_robot.launch.py"
  pkill -f "gz sim"

Notes

- Default mode: `robot1` runs SLAM and builds the shared `/map`; other robots
  localize on that shared map with AMCL.
- Multi-SLAM mode: `ros2 launch rosnav_bot multi_robot.launch.py slam_mode:=multi`
  starts one SLAM Toolbox instance per robot and merges `/robotN/map` into
  `/map_merged`.
- Current launch expects a global `/tf` tree for both robots.

Map generation

To generate a fresh map for any world:
  ros2 launch rosnav_bot multi_robot.launch.py world:=maze

To manually save the current SLAM map:
  ros2 run nav2_map_server map_saver_cli -f src/rosnav_bot/maps/map_maze

To use fleet_manager to save:
  ros2 run rosnav_bot fleet_manager.py savemap src/rosnav_bot/maps/map_maze
