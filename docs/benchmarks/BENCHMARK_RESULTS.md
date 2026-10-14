# rosnav local-controller benchmark (DWB vs MPPI vs RPP)

Measured 2026-10-03 in the `cafe` world, headless Gazebo, Nav2 on ROS 2 Humble,
diff-drive robot capped at 0.15 m/s, AMCL on a saved map, robot spawned at (0, 0).
Each trial is a single goal from a fresh simulator launch. **One trial per
controller per goal** — these are single runs, not averages.

Goal error is measured against Gazebo ground truth, not `/odom` or `/amcl_pose`
(see "Benchmark fixes"). A trial counts as a success only if Nav2 reports
`SUCCEEDED` **and** the measured ground-truth error is <= 0.5 m.

## Results

| Goal (map frame) | DWB | MPPI | RPP |
|---|---|---|---|
| G1 — 1.55 m straight east (1.55, -0.01) | OK, 9.8 s, 0.131 m/s, err 0.33 m | OK, 14.1 s, 0.092 m/s, err 0.28 m | OK, 10.2 s, 0.130 m/s, err 0.25 m |
| G2 — 2.55 m straight east (2.55, -0.01) | OK, 18.4 s, 0.134 m/s, err 0.23 m | OK, 25.6 s, 0.099 m/s, err 0.08 m | OK, 22.7 s, 0.117 m/s, err 0.21 m |
| G3 — 3.55 m south (0.0, -3.56) | FAIL, 14 recoveries, ended 1.57 m short | FAIL, never moved, 13 recoveries | FAIL, 13 recoveries, ended 2.79 m short |
| G4 — 3.5 m west (-3.45, -0.11) | FAIL, 14 recoveries, ended 3.01 m short | FAIL, 5 recoveries, ended 2.88 m short | FAIL, 0 recoveries, ended 3.97 m short |

Summary: **all three controllers reach goals up to 2.55 m (6/6 successes, zero
recoveries) and none reached either 3.5 m goal (0/6).** DWB and RPP are
indistinguishable on speed (~0.13 m/s); MPPI is ~40-50 % slower (~0.09-0.10 m/s)
but had the smallest error on G2 (0.08 m).

**This is not a ranking of the controllers.** Nothing here separates them on
accuracy or robustness, and the 3.5 m failures are common to all three, which
points at the world / costmap / setup rather than the controllers. I have not
found that cause yet.

## What was fixed to get here

- `scripts/benchmark.py`
  - Goal error used `/odom` against a map-frame goal, so it was meaningless
    (a "successful" goal could show a 7 m error). It now uses Gazebo ground truth
    (frame `gt`), falling back to the `map->base_link` TF, then `/odom` with a warning.
  - A success now also requires measured error <= `success_tol_m` (0.5). Before,
    a goal that Nav2 reported as `SUCCEEDED` after 0.06 s with the robot 3.5 m
    away counted as an "OK". These instant false successes happened in 3 of ~15
    launches; the cause is **not** identified (start localization error was 0.0 m
    in every validated trial, so it was not AMCL mislocalization as I first thought).
  - New per-trial fields: `goal_accepted`, `nav2_reported_success`,
    `start_localization_error_m`, `goal_error_frame`; trials whose start pose is
    >0.5 m from ground truth are marked `invalid` and excluded from the success rate.
- `config/nav2_params_mppi.yaml`
  - Nav2's container segfaulted on every MPPI launch: the CostCritic had
    `consider_footprint: true` while the costmaps were circular (`robot_radius`).
    Both costmaps now use an octagonal footprint with the same 0.34 m circumradius
    DWB/RPP use, so the collision check is footprint-based and the comparison is
    fair. (Confirmed by re-enabling the setting on a circular costmap: segfault returned.)
  - MPPI crawled at 0.012 m/s with the sampling noise at `vx_std: 0.03`,
    `wz_std: 0.08`. Raised to 0.1 / 0.25: G1 went from 114 s to 14 s.
  - `time_steps: 70` with `model_dt: 0.04` keeps the 2.8 s prediction horizon.
- `config/nav2_params.yaml`: DWB `trans_stopped_velocity` 0.25 -> 0.05 (it was above
  the 0.15 m/s speed cap, so the "stopped" check was always true).
- `launch/_common.py`: prints a warning when `controller:=mppi|rpp` is silently
  ignored (ackermann, mecanum, Jazzy).
- `test/test_benchmark_reporting.py`: two stale tests updated to the current 11x9
  `coverage_100` world and its (-1.6, -0.8) spawn.

## Known limits / not done

- One trial per cell. Variation is large (DWB on G4 ended 0.40 m short in one run
  and 3.0 m short in another), so differences of this size are not meaningful.
- Only the `cafe` world and four goals. Nothing on real hardware (simulation only).
- G1/G2 ran before the start-localization check existed; their small errors and
  expected path lengths (1.24-2.65 m travelled) make a bad start unlikely.
- The earlier DWB/RPP results with errors of 3-8 m were produced by the old
  `/odom`-based error code and are discarded.
- Why every controller fails beyond ~3 m is open: all three finish with
  13-14 recoveries on G3, and RPP/DWB wander 24-29 m on a 3.5 m route.
