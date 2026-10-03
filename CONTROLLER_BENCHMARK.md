# Controller benchmark: DWB vs MPPI vs RPP

Generated 2026-10-03 20:46. **Status: 16 / 27 trials complete** (sweep still running, partial results).

## Setup
- World: cafe, headless, Nav2 on the pre-built map, AMCL on, single robot (diff drive, Humble).
- Goals: three straight runs from the spawn pose (spawn faces south; map frame starts at the spawn pose, so goals are map (1.55, 0), (2.55, 0), (3.5, 0)).
- Controllers: `nav2_params.yaml` (DWB), `nav2_params_mppi.yaml` (MPPI), `nav2_params_rpp.yaml` (RPP). Same speed caps: 0.15 m/s, 0.4 rad/s.
- One fresh launch per trial, 3 trials per controller per goal, own `ROS_DOMAIN_ID` and `GZ_PARTITION`.
- Goal error = distance from Gazebo ground truth to the goal (`goal_error_frame: gt`). Nav2's goal tolerance is 0.25 m.
- Checks: runs are discarded if the frame is not `gt`, if a success has error above 0.40 m, or if a success travelled well short of the goal.

## Results
| controller | goal (m) | n | success | mean time (s) | mean goal err (m) | mean path (m, gt) | recoveries | invalid |
|---|---|---|---|---|---|---|---|---|
| dwb | 1.55 | 3 | 3/3 | 9.87 | 0.31 | 1.20 | 0 | 0 |
| mppi | 1.55 | 2 | 2/2 | 109.86 | 0.26 | 1.29 | 0 | 0 |
| rpp | 1.55 | 1 | 1/1 | 10.07 | 0.26 | 1.26 | 0 | 1 |
| dwb | 2.55 | 2 | 2/2 | 17.54 | 0.28 | 2.28 | 0 | 0 |
| mppi | 2.55 | 1 | 0/1 | - | - | - | 0 | 0 |
| rpp | 2.55 | 2 | 2/2 | 16.79 | 0.27 | 2.28 | 0 | 0 |
| dwb | 3.5 | 1 | 0/1 | - | - | - | 5 | 0 |
| mppi | 3.5 | 1 | 0/1 | - | - | - | 1 | 0 |
| rpp | 3.5 | 2 | 0/2 | - | - | - | 16 | 0 |

INVALID/FLAGGED:
  rpp 1.55 t1 success but err 1.55m

launch attempts needed: Counter({1: 9, 2: 5, 3: 1, 4: 1})

## Bugs found while benchmarking
1. **Wheel parameters wrong in `urdf/gazebo_control.xacro`** (fixed, uncommitted). The DiffDrive plugin had `wheel_radius 0.1` and `wheel_separation 0.35`, but the URDF wheels have radius 0.075 and sit 0.45 m apart. The robot moved at about 75% of the commanded speed and odometry overstated distance by about 33%. Measured before the fix: 1.31 m odom vs 0.93 m true. After: 1.30 m vs 1.24 m.
2. **`benchmark.py` goal error was meaningless** (fixed, uncommitted). It read `/odom` (origin = spawn) against map-frame goals. Then the `map->base_link` TF turned out to lag the robot by 0.3 to 0.5 m between AMCL updates, so ground truth is now the primary metric. The AMCL value stays in `goal_error_map_tf_m`. The script also did not spin its subscriptions before the first goal, so the start pose was null.
3. **Do not publish `/initialpose` at world coordinates.** The map frame starts at the spawn pose, and AMCL already starts at (0, 0). Publishing (0, -2) pushed AMCL 2 m off and caused pose jumps in earlier runs.
4. **Stale processes broke earlier runs.** Orphaned `parameter_bridge` processes kept publishing old `/clock` and `/odom`. Earlier cafe DWB/RPP results are invalid and were discarded.
5. **MPPI**: `consider_footprint: false` and `model_dt: 0.04` were uncommitted changes made to stop a segfault. Footprint-less collision checking is not comparable with DWB and RPP, and the horizon is only 2.24 s. The root cause of the segfault is not found yet.

## Open issues
- **Nav2 startup is flaky**: about 60% of launches never reach "Managed nodes are active". The container starts but loads no nodes. The runner relaunches automatically. Cleaning stale `/dev/shm/fastrtps_*` files did not fix it. Root cause unknown.
- MPPI is about 10x slower than DWB and RPP on the 1.55 m goal.
- Mecanum, ackermann and Jazzy ignore `controller:=` (see `_common.nav2_params_filename`).
