# Changelog

## Unreleased
- `map-guard <map.yaml> [--ref ref.yaml | --ref-git REV] [--initial-pose X Y]`: ROS-free check of a saved map against the AMCL start (outside / occupied / unknown cell), a reference map (`MAP_ROTATED` long axis flipped, `MAP_SMALL` < 60% area) and its known-cell fraction. Motivated by the cafe remap built from a -90 deg spawn yaw that came out rotated and half-size.

## 0.1.0
- First release: `preflight` (offline lio cloud check, optional `--live` doctors), `merge`, `eval` wrapper, common finding schema, severity-based exit code, 8 cross-tool root-cause rules. Pure-Python tests; live mode not tried against a running stack.
