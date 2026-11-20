# rosnav_check

One report from all rosnav doctors. ROS-free core, thin CLI.

```bash
rosnav_check preflight --cloud-spec spec.json          # offline: lio_doctor cloud/readiness checks
rosnav_check preflight --live 8                        # also run lio/slam/nav2 doctors live (needs ROS + stack)
rosnav_check merge lio_out.json slam_out.json nav2=n.json --json
rosnav_check map-guard map_cafe.yaml --ref-git HEAD     # rotated / half-size / wrong-spawn map? (ROS-free)
rosnav_check eval ref.tum est1.tum est2.tum            # shells to ~/fusion_tools/eval_traj.sh (read-only)
```

Common schema: `tool, code, severity (ERROR/WARN/INFO/OK), msg, hint`. Sorted worst first. Exit code: 0 clean, 1 worst WARN, 2 worst ERROR (CI-friendly). Missing tools become a `TOOL_UNAVAILABLE` WARN, never a crash.

`merge` input: any doctor `--json` object (`{"findings": [...]}`) or a bare list. Tool name comes from a `tool` key, then the file name (lio/slam/nav2), or force it with `tool=path`.

## Cross-tool root-cause rules (real codes only)

| Rule | Fires when | Says |
|---|---|---|
| CLOUD_UPSTREAM | lio: STAMP_AT_END/STAMP_UNKNOWN/CLOUD_* AND slam: DRIFT_HIGH/NO_MAP_ODOM/BIG_JUMP/JUMP_STORM/MAP_STALL | cloud stamp/field problem upstream of SLAM |
| CLOUD_UPSTREAM_LIO | lio cloud code AND lio POSE_JUMP/YAW_JUMP/DRIFT_HIGH/ODOM_NONE/STAMP_GAP | fix the cloud first |
| IMU_CLOCK | lio IMU_TIME_OFFSET AND POSE_JUMP/YAW_JUMP/DRIFT_HIGH | fix the clock offset first |
| SLAM_TF_ROOT | nav2 TF_FREEZE/TF_EDGE_STALE/NO_MAP_ODOM AND slam NO_MAP_ODOM/TRACKING_LOST | Nav2 TF trouble is a symptom of SLAM |
| TF_FREEZE_FILTERS | nav2 TF_FREEZE AND FILTER_NO_INFO/FILTER_NO_MASK | inert costmap filters |
| MAP_SOURCE_STALL | slam MAP_STALL/MAP_NONE AND nav2 MAP_STALL/COSTMAP_STALE | SLAM map starves Nav2 |
| MAP_JUMPS_HURT_NAV | slam BIG_JUMP/JUMP_STORM AND nav2 NO_PROGRESS/GOAL_FLAPPING/RECOVERY_LOOP | map corrections destabilise planning |
| CLOCK_MIX | lio STAMP_BACKWARDS AND nav2 CLOCK_SKEW | sim/wall time mix |

Rules match on (tool, code): lio's own DRIFT_HIGH never satisfies the slam side.

## Limits
slam_doctor and nav2_doctor have no offline mode, so offline preflight only covers lio_doctor `--check-cloud`; their findings arrive via `--live` or `merge`. `--live` is untested against a running stack. Tool lookup: sibling `tools/*` dirs, then `~/nav2_doctor`.
