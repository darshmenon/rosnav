# Vendored Gazebo models

## Depot (OpenRobotics)

PBR warehouse used by `worlds/warehouse_depot.world`.

Fetched with **Gazebo Fuel only** (no git):

```bash
bash src/rosnav_bot/scripts/download_depot_model.sh
colcon build --packages-select rosnav_bot
source install/setup.bash
```

Launch files prepend `share/rosnav_bot/models` to `GZ_SIM_RESOURCE_PATH` so `model://Depot` works offline after the download.
