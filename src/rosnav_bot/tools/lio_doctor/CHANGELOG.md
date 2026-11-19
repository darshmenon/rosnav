# Changelog

## 0.1.0
- First release: ROS-free `core.py` (odometry rate, stamp monotonicity and gaps, pose jumps, PointCloud2 field validity, cloud stamp start/end heuristic, IMU rate and IMU-vs-odometry time offset, TF frame presence, windowed drift against a reference odometry) and a thin ROS 2 node / CLI.
- `--preflight`: checks the cloud fields and stamps against what KISS-ICP, RKO-LIO and LIO-SAM each need and prints which of the three can run (READY / DEGRADED / BLOCKED, with reasons and fixes).
- `--check-cloud spec.json`: validate a cloud layout offline, no ROS.
- Unit tests with synthetic data, including the malformed 22-byte-record cloud that broke LIO-SAM. Not yet tried on a live LIO stack.
