# rosnav_check Quality Declaration

Follows [REP-2004](https://reps.openrobotics.org/rep-2004/). **rosnav_check claims Quality Level 5**, the lowest: a developer/CI tool, not a runtime dependency.

| # | Section | Status |
|---|---|---|
| 1 | Version policy | SemVer (`package.xml`, `CHANGELOG.md`); 0.1.0, API may change. |
| 2 | Change control | Lives in the rosnav repository; single maintainer. |
| 3 | Documentation | `README.md`, `CHANGELOG.md`. |
| 4 | Testing | pytest on synthetic findings JSON, no ROS. `--live` mode and `eval` are not tested against real stacks. |
| 5 | Dependencies | Python standard library only; optional sibling tools and `~/fusion_tools`. |
| 6 | Platform support | Ubuntu 22.04 / Humble developed; others untested. |
| 7 | Security | Reads JSON files; shells only to the sibling doctors and eval_traj.sh with fixed argv (no shell string). |
