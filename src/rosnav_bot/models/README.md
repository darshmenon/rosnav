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

## Textured Fuel worlds (camera / RTAB-Map)

PBR lake house, AWS RoboMaker warehouse, Tugbot warehouse, and cafe — the
most realistic option, but the meshes are 10s-100s of MB each and stay out
of git. Fetch with:

```bash
bash src/rosnav_bot/scripts/download_fuel_worlds.sh
```

Models land in `models/fuel/` (gitignored). See `models/fuel/README.md`.

## `textures/` — baked PBR floor/wall textures (committed)

Small (~300 KB total) procedural PNGs applied to `house.world` and
`hospital.world`'s floors/walls as `<pbr><metal><albedo_map>`, so RTAB-Map /
visual SLAM has real texture to track on the two primitive-box worlds
without needing the (gitignored, much larger) Fuel worlds above. Small
enough to commit directly. Regenerate with:

```bash
python3 src/rosnav_bot/scripts/gen_world_textures.py
```
