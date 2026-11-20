# lio_doctor

Is my lidar(-inertial) odometry healthy? Attach to a running KISS-ICP / RKO-LIO / LIO-SAM (or any odometry source) and get coded findings with a one-line fix hint, a 0-100 score, and a **sensor-readiness preflight** that says which of the three estimators can run on your point cloud and IMU.

```bash
lio_doctor --odom /lio_sam/mapping/odometry --cloud /points_lio --imu /imu --ref-odom /gt_odom \
           --frames odom,lio_base,laser_frame --expected-hz 7 --hints
lio_doctor --preflight --cloud /points --imu /imu        # which of KISS-ICP / RKO-LIO / LIO-SAM can run?
lio_doctor --check-cloud cloud.json                      # offline, no ROS: validate a cloud layout (optional `imu_yaw: {imu, odom}` quaternion samples or `imu_yaw_offset_deg` for the heading check)
```

## Checks

| Code | Meaning |
|---|---|
| `ODOM_NONE`, `ODOM_RATE_LOW` | odometry silent, or slower than `--expected-hz` x 0.8 |
| `STAMP_BACKWARDS`, `STAMP_GAP` | non-monotonic stamps; a step over 3x the median step |
| `POSE_JUMP`, `YAW_JUMP` | step larger than `--v-max`/`--w-max` x dt allows |
| `CLOUD_*` | PointCloud2 layout: x/y/z f32, field past `point_step`, unaligned offset, overlap, `point_step` not 4-aligned, data length != `row_step*height`, `ring` not u16, per-point `time` not f32 |
| `STAMP_AT_END` | per-point times all negative = header stamp is the sweep end (estimators expect the start) |
| `IMU_NONE`, `IMU_RATE_LOW`, `IMU_TIME_OFFSET` | IMU missing / under 50 Hz or 5x the lidar rate; gyro-z vs odometry yaw-rate cross-correlation gives the clock offset |
| `IMU_YAW_OFFSET`, `IMU_YAW_NOISY`, `IMU_YAW_SKIPPED` | circular mean/std of IMU yaw minus `--ref-odom` yaw (nearest stamp, within 50 ms): ERROR if \|mean\| > 5 deg (fix: relay `/imu` with the orientation rotated by the opposite yaw, `scripts/imu_yaw_relay.py` `yaw_deg`; `extrinsicRPY` is not the fix, it also rotates acc/gyro), WARN if std > 5 deg, INFO skip if no orientation, no `--ref-odom`, or under 10 matches. Makes LIO-SAM DEGRADED in `--preflight`; KISS-ICP ignores IMU orientation |
| `TF_MISSING` | a `--frames` entry never appears in `/tf` or `/tf_static` |
| `DRIFT_HIGH` | rigid-aligned end error over a `--drift-window` against `--ref-odom` above 5 % of the path |

The cloud checks encode the bug class found bringing up LIO-SAM on the sim (`concepts.md` sec 44): a writer that emitted 22-byte records while declaring `point_step` 20 with an unaligned `time` field. `test/test_core.py` locks it in.

## Preflight (the unique bit)

| Estimator | Needs | Without it |
|---|---|---|
| KISS-ICP | valid x/y/z | per-point time missing: DEGRADED (no deskew) |
| RKO-LIO | valid x/y/z, IMU | no per-point time: DEGRADED, needs `deskew:=false` |
| LIO-SAM | x/y/z, `ring` u16, `time` f32, stamp at sweep **start**, IMU with orientation | BLOCKED |

A malformed layout blocks all three. Output ends with `can run: ...`.

## Options

`--once` (report after `--duration`, exit 1 on ERROR), `--json`, `--hints`, `--interval`. Sensor QoS (best-effort) is used on every subscription.

## Limits

Not yet tried on a live stack: the node and CLI are covered by unit tests on synthetic data only. The stamp heuristic needs a per-point time field. Time-offset estimation assumes planar rotation (gyro-z) and enough turning in the window. Drift compares against a reference you trust; it cannot tell which side is wrong.

```bash
colcon build --packages-select lio_doctor && python3 -m pytest test   # tests need no ROS
```

## Fault-injection check

`scenarios/lio_faults.yaml` pauses each stage of the sim LIO-SAM chain with `nav2_fault_inject` and asserts the doctor notices (and recovers). Command is in the file header.
