# Fuel models for camera / RTAB-Map 3D SLAM

Textured buildings and furniture downloaded from [Gazebo Fuel](https://app.gazebosim.org/fuel/models).
Not stored in git (large meshes — the Lake House alone is ~240 MB). Fetch with:

```bash
bash src/rosnav_bot/scripts/download_fuel_worlds.sh
colcon build --packages-select rosnav_bot
source install/setup.bash
```

| World | Why it helps visual SLAM |
|---|---|
| `lake_house` | PBR Lake House + furniture (Gazebo Harmonic demo) |
| `aws_warehouse` | AWS RoboMaker warehouse shelves / clutter / walls |
| `tugbot_warehouse` | OpenRobotics Warehouse + Mov.AI shelves |
| `cafe` | Classic textured cafe interior + tables |

Launch (camera + 3D lidar + RTAB-Map):

```bash
ros2 launch rosnav_bot slam_nav.launch.py world_name:=aws_warehouse \
  enable_camera:=true lidar_type:=3d slam_algo:=3d
```

For a much smaller, push-able alternative see `../textures/` — baked PBR
floor/wall textures applied directly to `house.world` and `hospital.world`.
