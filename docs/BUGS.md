# Bug log

Found while benchmarking (2026-10-16). Status: FIXED (in working tree, not yet launch-verified) / OPEN.

| # | Bug | Evidence | Status |
|---|---|---|---|
| 1 | TF freeze not fully fixed: controller `map->odom` stuck ("Transform data too old"). The `transform_publish_period` 0.1 fix only reached `mapper_params_online_async.yaml`; 7 other mapper configs still had 0.02. | cafe `slam_algo:=2d`: 2833 errors, map->odom stuck at 291.3 s while clock hit 446 s; cafe `vslam`: 2138 errors | FIXED for slam_toolbox configs (all now 0.1); RTAB-Map's own `map->odom` publisher is untouched, re-run cafe 2d + vslam to confirm |
| 2 | `vslam` run barely moved (0.7 m in 6 min): 68x "Current footprint not available", 111x "Controller patience exceeded" | cafe vslam run | OPEN, probably bug 1 via RTAB's map->odom; re-check after re-run |
| 3 | AMCL initial pose in the wrong frame: `_common.apply_spawn_initial_pose` gave AMCL the world spawn (cafe 0,-2) but saved maps are spawn-relative (SLAM origin = spawn, same as multi_robot's map origin = robot1 spawn). Every `slam:=false` benchmark goal was skipped "INVALID start - AMCL pose is 2.00 m from ground truth". | `/initialpose` (0,0,0) made cafe goal 1 succeed (10 s, 0.217 m) | FIXED (slam_nav passes 0,0,0), launch-verify pending |
| 4 | benchmark.py `loc_tol` skips later goals at 0.67 m AMCL error after goal 1 | cafe goals 2,3 INVALID after correct pose | OPEN: AMCL drift or tolerance too tight, needs AMCL-vs-GT log over the 3 goals |
| 5 | `benchmark_nav_matrix.sh` ran the same combo twice -> second run `rm -f`s the first run's logs (same label) | nm_cafe run | OPEN, minor: add run index to label |
| 6 | Splat training competes with the sim for GPU/CPU; disk is 97% full (14 GB free) | `df -h ~` | OPEN, check output size before training |
| 7 | `/initialpose` published once (`--once`) never reaches AMCL (no discovery yet); needs repeated publishes | pub.log vs only 1 "Setting pose" | OPEN: tooling note, publish with `-t 6 -r 1` |
| 8 | `gps_localization.launch.py` docstring still says "NOT run in a sim"; GPS stack is not wired into slam_nav, no lat/lon goal path | concepts.md §42 | OPEN |
| 9 | Slope layer (`terrain_traversability.py`) untested on a steep slope | concepts.md §42 | OPEN |
