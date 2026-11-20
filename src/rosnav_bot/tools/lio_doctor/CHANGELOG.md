# Changelog

## Unreleased
- `CLOUD_STALE` / `IMU_STALE` (ERROR): the cloud or IMU stopped for more than 2 s (a dead adapter or relay). Before, only a silent odometry was caught.
- `--health-topic /lio_health [--health-frame base_link]`: publishes an RViz text Marker (score, rates, ERROR/WARN codes; green/amber/red). `lio_sam_sim.launch.py` starts it and `rviz/lidar3d.rviz` shows it.
- `scenarios/lio_faults.yaml`: `nav2_fault_inject` run that pauses the adapter, relay and mapOptimization and asserts `CLOUD_STALE`, `IMU_STALE`, `ODOM_NONE` plus recovery (3/3 live, detected in 1.5-2.2 s).
- `--tf-topics` (e.g. `/lio_tf,/lio_tf_static` for a LIO stack with a private tree). Static topics are now read TRANSIENT_LOCAL; the old VOLATILE subscriber never saw latched transforms (false `TF_MISSING`). `IMU_YAW_OFFSET` now prints the exact relay fix (`imu_yaw_relay yaw_deg:=+X`). Run `--preflight --cloud /points_lio` to check the adapted cloud LIO-SAM actually reads.
- IMU heading vs odometry check (`check_imu_yaw`): circular mean/std of IMU yaw minus reference yaw after nearest-stamp matching (50 ms). `IMU_YAW_OFFSET` (ERROR, |mean| > 5 deg), `IMU_YAW_NOISY` (WARN, std > 5 deg), `IMU_YAW_SKIPPED` (INFO). Live node uses `--ref-odom`; `--check-cloud` accepts `imu_yaw` samples or `imu_yaw_offset_deg`; `readiness` marks LIO-SAM DEGRADED on an offset. Motivated by the sim gz IMU -90 deg yaw offset.

## 0.1.0
- First release: ROS-free `core.py` (odometry rate, stamp monotonicity and gaps, pose jumps, PointCloud2 field validity, cloud stamp start/end heuristic, IMU rate and IMU-vs-odometry time offset, TF frame presence, windowed drift against a reference odometry) and a thin ROS 2 node / CLI.
- `--preflight`: checks the cloud fields and stamps against what KISS-ICP, RKO-LIO and LIO-SAM each need and prints which of the three can run (READY / DEGRADED / BLOCKED, with reasons and fixes).
- `--check-cloud spec.json`: validate a cloud layout offline, no ROS.
- Unit tests with synthetic data, including the malformed 22-byte-record cloud that broke LIO-SAM. Not yet tried on a live LIO stack.
