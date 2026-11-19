# lio_doctor Quality Declaration

This declaration follows [REP-2004](https://reps.openrobotics.org/rep-2004/). **lio_doctor claims Quality Level 5**, the lowest level: it is a developer tool that attaches to a running lidar(-inertial) odometry stack, it is not a runtime dependency, and it does not meet the higher-level requirements below.

| # | Section | Status |
|---|---|---|
| 1 | Version policy | Semantic Versioning (`package.xml`, `CHANGELOG.md`). Currently 0.1.0 (< 1.0.0: the API may change between minor releases). |
| 2 | Change control | Lives in the rosnav repository; every change lands through a commit and runs CI. No required review (single maintainer). |
| 3 | Documentation | `README.md` with usage and limits, `CHANGELOG.md`. No generated API docs. |
| 4 | Testing | pytest unit tests in `test/` on synthetic data (no ROS needed), run in CI. No coverage measurement. The ROS node itself is not yet tested against a live stack. |
| 5 | Dependencies | ROS 2 packages only (`rclpy`, `nav_msgs`, `sensor_msgs`, `tf2_msgs`); the core uses only the Python standard library. |
| 6 | Platform support | Developed on Ubuntu 22.04 / ROS 2 Humble. Other platforms untested. |
| 7 | Security | Read-only: subscribes to topics, publishes nothing, opens no ports beyond ROS 2 DDS. |

## Known gaps to reach a higher level

- No coverage measurement or target.
- flake8 is configured in `setup.cfg`; `ament_lint` copyright/pep257 are not wired in.
- No integration test against a live node.
- No generated API documentation; not released to the buildfarm.
