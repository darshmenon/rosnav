#!/usr/bin/env python3
"""
multi_robot.launch.py  —  Scalable multi-robot navigation in Gazebo.

Architecture
────────────
• Fleet size and spawn layout are launch parameters. Use robot_count/layout
  for generated fleets, or robots_json for exact custom poses.

• When explore:=true (default, slam_mode:=single):
    - A single SLAM Toolbox instance (driven by robot1) builds the shared /map.
    - robot1 uses SLAM for localisation (no AMCL needed).
    - All other robots localise on that map via their own AMCL node.
    - A single frontier_coordinator assigns unique frontiers to each robot —
      no two robots ever target the same unexplored area.
    - Map is auto-saved when exploration finishes.

• When explore:=true slam_mode:=multi:
    - Every robot runs its own slam_toolbox with a private <ns>/map frame
      (avoids map→odom TF fights).
    - Static map→<ns>/map TFs use known spawn offsets vs robot1.
    - map_merge_known stitches /robotN/map grids into /map_merged in world
      coordinates (occupied wins; survives grid growth/re-origin).
    - Nav2 + frontier_coordinator consume /map_merged.

• When explore:=false:
    - A static map_server publishes a pre-built map.
    - Every robot runs its own AMCL for localisation.

• Nav2 params use a single template file (nav2_multirobot_params.yaml).
  The placeholder ROBOT_NS is substituted at launch — no per-robot YAML files.

Usage
─────
  # SLAM + frontier exploration in hospital (default)
  ros2 launch rosnav_bot multi_robot.launch.py

  # Pre-built map mode
  ros2 launch rosnav_bot multi_robot.launch.py explore:=false

  # Different world
  ros2 launch rosnav_bot multi_robot.launch.py world:=obstacles

  # Fuse non-SLAM robots' scans into /map_fused (opt-in; SLAM/AMCL untouched)
  ros2 launch rosnav_bot multi_robot.launch.py merge_scans:=true

  # Experimental: every robot runs SLAM and maps are merged
  ros2 launch rosnav_bot multi_robot.launch.py slam_mode:=multi

  # Swap frontier plugins without editing code
  ros2 launch rosnav_bot multi_robot.launch.py frontier_detector:=wfd frontier_scorer:=utility

  # Spawn more robots without editing this file
  ros2 launch rosnav_bot multi_robot.launch.py robot_count:=4 robot_layout:=grid

  # Fleet of mecanum (holonomic) robots instead of diff-drive
  ros2 launch rosnav_bot multi_robot.launch.py drive_type:=mecanum

  # Same drive_type/controller options as slam_nav.launch.py (single robot),
  # applied fleet-wide — diff keeps its hand-tuned fleet nav2 params; mecanum/
  # ackermann reuse and namespace the single-robot nav2_params_*.yaml.
  ros2 launch rosnav_bot multi_robot.launch.py drive_type:=ackermann

  # Send a nav goal to a specific robot
  ros2 action send_goal /robot1/navigate_to_pose nav2_msgs/action/NavigateToPose \\
    "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 1.0}}}}"
"""

import os
import sys
import tempfile
import json
import math
import yaml
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common  # noqa: E402  (shared with slam_nav.launch.py)

ROS_DISTRO = _common.ROS_DISTRO
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription,
    LogInfo, OpaqueFunction, TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────
_truthy = _common.truthy


def _resolve_robots(
    robot_count: int,
    robot_layout: str,
    spawn_x: float,
    spawn_y: float,
    spawn_z: float,
    spawn_yaw: float,
    spawn_spacing: float,
    robots_json: str,
) -> list[dict]:
    if robots_json:
        robots = json.loads(robots_json)
        if not isinstance(robots, list) or not robots:
            raise ValueError('robots_json must be a non-empty JSON list')
        return [_normalise_robot(r, i) for i, r in enumerate(robots)]

    if robot_count < 1:
        raise ValueError('robot_count must be >= 1')

    layout = robot_layout.strip().lower()
    if layout not in ('line', 'grid', 'circle'):
        raise ValueError('robot_layout must be one of: line, grid, circle')

    robots = []
    cols = max(1, math.ceil(math.sqrt(robot_count)))
    radius = max(spawn_spacing, robot_count * spawn_spacing / (2.0 * math.pi))

    for i in range(robot_count):
        if layout == 'line':
            x = spawn_x + i * spawn_spacing
            y = spawn_y
        elif layout == 'grid':
            row, col = divmod(i, cols)
            x = spawn_x + col * spawn_spacing
            y = spawn_y + row * spawn_spacing
        else:
            angle = 2.0 * math.pi * i / robot_count
            x = spawn_x + radius * math.cos(angle)
            y = spawn_y + radius * math.sin(angle)
        robots.append({
            'name': f'robot{i + 1}',
            'x': f'{x:.3f}',
            'y': f'{y:.3f}',
            'z': f'{spawn_z:.3f}',
            'yaw': f'{spawn_yaw:.3f}',
        })
    return robots


def _normalise_robot(robot: dict, idx: int) -> dict:
    required = ('x', 'y')
    for key in required:
        if key not in robot:
            raise ValueError(f'robots_json[{idx}] missing {key!r}')
    return {
        'name': str(robot.get('name', f'robot{idx + 1}')),
        'x': str(robot['x']),
        'y': str(robot['y']),
        'z': str(robot.get('z', 0.3)),
        'yaw': str(robot.get('yaw', 0.0)),
    }


def _world_obstacles(world_path: str) -> list[dict]:
    try:
        root = ET.parse(os.path.expanduser(world_path)).getroot()
    except (ET.ParseError, OSError):
        return []

    obstacles = []
    for model in root.findall('.//model'):
        model_pose = _parse_pose(model.findtext('pose'))
        model_x, model_y, _, _, _, model_yaw = model_pose
        for collision in model.findall('.//collision'):
            geom = collision.find('geometry')
            if geom is None:
                continue
            col_pose = _parse_pose(collision.findtext('pose'))
            x = model_x + col_pose[0]
            y = model_y + col_pose[1]
            yaw = model_yaw + col_pose[5]
            box = geom.find('box')
            cylinder = geom.find('cylinder')
            if box is not None:
                size = _parse_floats(box.findtext('size'), 3)
                if size[2] < 0.1:
                    continue
                obstacles.append({
                    'kind': 'box',
                    'x': x,
                    'y': y,
                    'yaw': yaw,
                    'sx': size[0],
                    'sy': size[1],
                })
            elif cylinder is not None:
                radius = float(cylinder.findtext('radius', '0'))
                length = float(cylinder.findtext('length', '0'))
                if length < 0.1:
                    continue
                obstacles.append({
                    'kind': 'cylinder',
                    'x': x,
                    'y': y,
                    'radius': radius,
                })
    return obstacles


def _world_spawn_areas(world_path: str) -> list[dict]:
    try:
        root = ET.parse(os.path.expanduser(world_path)).getroot()
    except (ET.ParseError, OSError):
        return []

    areas = []
    for model in root.findall('.//model'):
        name = model.get('name', '')
        model_pose = _parse_pose(model.findtext('pose'))
        model_x, model_y, _, _, _, model_yaw = model_pose
        for collision in model.findall('.//collision'):
            geom = collision.find('geometry')
            box = geom.find('box') if geom is not None else None
            if box is None:
                continue
            size = _parse_floats(box.findtext('size'), 3)
            if size[2] >= 0.1 or max(size[0], size[1]) >= 99.0:
                continue
            if not name.startswith('zone_'):
                continue
            col_pose = _parse_pose(collision.findtext('pose'))
            areas.append({
                'kind': 'box',
                'x': model_x + col_pose[0],
                'y': model_y + col_pose[1],
                'yaw': model_yaw + col_pose[5],
                'sx': size[0],
                'sy': size[1],
            })
    return areas


def _parse_pose(text: str | None) -> list[float]:
    values = _parse_floats(text, 6)
    return values


def _parse_floats(text: str | None, count: int) -> list[float]:
    values = [float(v) for v in (text or '').split()]
    values += [0.0] * (count - len(values))
    return values[:count]


def _safe_spawns(
    robots: list[dict],
    world_path: str,
    validate_spawns: bool,
    robot_clearance: float,
    search_radius: float,
    search_step: float,
) -> tuple[list[dict], list[str]]:
    if not validate_spawns:
        return robots, []

    obstacles = _world_obstacles(world_path)
    spawn_areas = _world_spawn_areas(world_path)
    if not obstacles:
        return robots, ['spawn validation skipped: no parseable SDF obstacles found']

    placed: list[tuple[float, float]] = []
    adjusted = []
    out = []
    for robot in robots:
        x = float(robot['x'])
        y = float(robot['y'])
        nx, ny = _nearest_free_spawn(
            x, y, obstacles, spawn_areas, placed, robot_clearance,
            search_radius, search_step)
        if (round(nx, 4), round(ny, 4)) != (round(x, 4), round(y, 4)):
            adjusted.append(
                f"{robot['name']}: ({x:.2f}, {y:.2f}) -> ({nx:.2f}, {ny:.2f})")
        placed.append((nx, ny))
        moved = dict(robot)
        moved['x'] = f'{nx:.3f}'
        moved['y'] = f'{ny:.3f}'
        out.append(moved)
    return out, adjusted


def _nearest_free_spawn(
    x: float,
    y: float,
    obstacles: list[dict],
    spawn_areas: list[dict],
    placed: list[tuple[float, float]],
    clearance: float,
    search_radius: float,
    search_step: float,
) -> tuple[float, float]:
    if _is_spawn_free(x, y, obstacles, spawn_areas, placed, clearance):
        return x, y

    step = max(0.05, search_step)
    rings = max(1, math.ceil(search_radius / step))
    for ring in range(1, rings + 1):
        radius = ring * step
        samples = max(16, ring * 12)
        for i in range(samples):
            angle = 2.0 * math.pi * i / samples
            cx = x + radius * math.cos(angle)
            cy = y + radius * math.sin(angle)
            if _is_spawn_free(cx, cy, obstacles, spawn_areas, placed, clearance):
                return cx, cy
    raise RuntimeError(
        f'No free spawn found near ({x:.2f}, {y:.2f}) within {search_radius:.2f} m')


def _is_spawn_free(
    x: float,
    y: float,
    obstacles: list[dict],
    spawn_areas: list[dict],
    placed: list[tuple[float, float]],
    clearance: float,
) -> bool:
    if spawn_areas and not any(_point_in_box(x, y, area, -clearance) for area in spawn_areas):
        return False
    for px, py in placed:
        if math.hypot(x - px, y - py) < clearance * 2.0:
            return False
    for obs in obstacles:
        if obs['kind'] == 'cylinder':
            if math.hypot(x - obs['x'], y - obs['y']) <= obs['radius'] + clearance:
                return False
        else:
            if _point_in_box(x, y, obs, clearance):
                return False
    return True


def _point_in_box(x: float, y: float, box: dict, margin: float) -> bool:
    dx = x - box['x']
    dy = y - box['y']
    c = math.cos(-box['yaw'])
    s = math.sin(-box['yaw'])
    lx = c * dx - s * dy
    ly = s * dx + c * dy
    return (abs(lx) <= box['sx'] / 2.0 + margin and
            abs(ly) <= box['sy'] / 2.0 + margin)


def _multirobot_template_yaml(pkg_share: str, controller: str, drive_type: str) -> str:
    """Base nav2 params file for the fleet, per drive_type.

    diff (the default) keeps using the existing hand-tuned fleet templates —
    no behaviour change for existing fleets. mecanum/ackermann have no such
    hand-tuned template, so the matching single-robot nav2_params_*.yaml (also
    used by slam_nav.launch.py) is namespaced programmatically at launch time
    instead — see _make_robot_params / _common.namespace_nav2_params.
    """
    if drive_type == 'diff':
        fname = 'nav2_multirobot_params_jazzy.yaml' if ROS_DISTRO == 'jazzy' else 'nav2_multirobot_params.yaml'
    else:
        fname = _common.nav2_params_filename(controller, drive_type, ROS_DISTRO)
    return os.path.join(pkg_share, 'config', fname)


def _make_robot_params(
    template_path: str,
    robot_ns: str,
    pkg_share: str,
    initial_pose: dict | None = None,
    nav_map_topic: str = '/map',
) -> str:
    """Return a namespaced-for-this-robot copy of a nav2 params file as a temp-file path.

    Two kinds of source files are supported:
      - Hand-tuned fleet templates (nav2_multirobot_params*.yaml) already contain
        a literal "ROBOT_NS" placeholder in their text; substituted directly.
      - Plain single-robot nav2_params_*.yaml files (used for drive types that
        don't have a hand-tuned fleet template, e.g. mecanum/ackermann) are
        namespaced programmatically via _common.namespace_nav2_params — one
        source of truth per drive type, shared with slam_nav.launch.py.
    """
    with open(template_path) as f:
        content = f.read()
    if 'ROBOT_NS' in content:
        content = content.replace('ROBOT_NS', robot_ns)
        params = yaml.safe_load(content)
    else:
        params = _common.namespace_nav2_params(yaml.safe_load(content), robot_ns)

    bt_params = params.get('bt_navigator', {}).get('ros__parameters', {})
    if 'default_nav_to_pose_bt_xml' in bt_params:
        bt_params['default_nav_to_pose_bt_xml'] = bt_params['default_nav_to_pose_bt_xml'].replace(
            'replace_with_pkg_share', pkg_share.replace('\\', '/'))
    if initial_pose is not None:
        amcl_params = params.setdefault('amcl', {}).setdefault('ros__parameters', {})
        amcl_params['set_initial_pose'] = True
        amcl_params['initial_pose'] = initial_pose
    static_layer = (
        params.setdefault('global_costmap', {})
        .setdefault('global_costmap', {})
        .setdefault('ros__parameters', {})
        .setdefault('static_layer', {})
    )
    static_layer['map_topic'] = nav_map_topic
    content = yaml.safe_dump(params, sort_keys=False)
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix=f'_{robot_ns}.yaml', delete=False, prefix='nav2_mr_')
    tmp.write(content)
    tmp.close()
    return tmp.name


def _make_slam_params(template_path: str, robot_ns: str) -> dict:
    """Per-robot slam_toolbox params with a unique local map frame.

    Using map_frame=<ns>/map avoids N slam nodes fighting over map→odom on /tf.
    A static map→<ns>/map TF (spawn offset vs robot1) ties locals into one tree.
    """
    with open(template_path) as f:
        content = f.read().replace('ROBOT_NS', robot_ns)
        params = yaml.safe_load(content) or {}
    slam_params = dict((params.get('slam_toolbox') or {}).get('ros__parameters', {}))
    slam_params.update({
        'odom_frame': f'{robot_ns}/odom',
        'base_frame': f'{robot_ns}/base_link',
        'map_frame': f'{robot_ns}/map',
        'scan_topic': f'/{robot_ns}/scan',
        'map_start_at_dock': True,
        'map_start_pose': [0.0, 0.0, 0.0],
        'use_map_saver': False,
        'enable_interactive_mode': False,
    })
    return slam_params


def _init_poses_for_merge(robots: list[dict]) -> list[dict]:
    """Local-map origins in the global map frame (origin = robot1 spawn)."""
    ox = float(robots[0]['x'])
    oy = float(robots[0]['y'])
    # Gazebo odom/map axes stay world-aligned; spawn yaw lives in base_link.
    return [
        {'x': float(r['x']) - ox, 'y': float(r['y']) - oy, 'yaw': 0.0}
        for r in robots
    ]


def _resolve_map_file(map_arg: str, world_path: str, pkg_share: str) -> str:
    if map_arg:
        return map_arg
    world_name = os.path.splitext(os.path.basename(world_path))[0]
    home = os.path.expanduser('~')
    candidates = [
        os.path.join(pkg_share, 'maps', f'map_{world_name}.yaml'),
        os.path.join(pkg_share, 'maps', f'{world_name}_map.yaml'),
        os.path.join(home, 'rosnav', 'maps', f'map_{world_name}.yaml'),
        os.path.join(pkg_share, 'maps', 'my_map.yaml'),
    ]
    for c in candidates:
        if os.path.exists(c):
            return c
    return candidates[0]


_resolve_gazebo_world_name = _common.resolve_gazebo_world_name


def _load_node_params(params_path: str, node_name: str) -> dict:
    """Return ros__parameters for a single node from a generated YAML file."""
    with open(params_path) as f:
        content = yaml.safe_load(f) or {}
    return (content.get(node_name) or {}).get('ros__parameters', {})


def _initial_pose_pub(robot_ns: str, x: str, y: str, yaw: str) -> ExecuteProcess:
    yaw_f = float(yaw)
    qz = math.sin(yaw_f / 2.0)
    qw = math.cos(yaw_f / 2.0)
    msg = json.dumps({
        'header': {'stamp': 'now', 'frame_id': 'map'},
        'pose': {
            'pose': {
                'position': {'x': float(x), 'y': float(y), 'z': 0.0},
                'orientation': {'z': qz, 'w': qw},
            },
            'covariance': [
                0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942,
            ],
        },
    })
    return ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub',
            '--use-sim-time',
            '--qos-reliability', 'best_effort',
            '--wait-matching-subscriptions', '1',
            '--rate', '1',
            '--times', '20',
            '--keep-alive', '1.0',
            f'/{robot_ns}/initialpose',
            'geometry_msgs/msg/PoseWithCovarianceStamped',
            msg,
        ],
        output='screen',
    )


# ──────────────────────────────────────────────────────────────────────────────
# Main launch builder
# ──────────────────────────────────────────────────────────────────────────────
def _build_all(context, pkg_share: str):
    map_arg      = LaunchConfiguration('map').perform(context).strip()
    world_arg    = LaunchConfiguration('world').perform(context).strip()
    explore      = _truthy(LaunchConfiguration('explore').perform(context))
    headless     = _truthy(LaunchConfiguration('headless').perform(context))
    fleet_mgmt   = _truthy(LaunchConfiguration('fleet_mgmt').perform(context))
    merge_scans  = _truthy(LaunchConfiguration('merge_scans').perform(context))
    slam_mode = LaunchConfiguration('slam_mode').perform(context).strip().lower()
    frontier_detector = LaunchConfiguration('frontier_detector').perform(context).strip()
    frontier_scorer = LaunchConfiguration('frontier_scorer').perform(context).strip()
    explore_bt = LaunchConfiguration('explore_bt').perform(context).strip()
    distance_weight = float(LaunchConfiguration('distance_weight').perform(context).strip())
    info_gain_weight = float(LaunchConfiguration('info_gain_weight').perform(context).strip())
    potential_scale = float(LaunchConfiguration('potential_scale').perform(context).strip())
    gain_scale = float(LaunchConfiguration('gain_scale').perform(context).strip())
    validate_on_costmap = _truthy(LaunchConfiguration('validate_on_costmap').perform(context))
    costmap_max_cost = int(float(LaunchConfiguration('costmap_max_cost').perform(context).strip()))
    goal_pullback = float(LaunchConfiguration('goal_pullback').perform(context).strip())
    hysteresis_radius = float(LaunchConfiguration('hysteresis_radius').perform(context).strip())
    hysteresis_gain = float(LaunchConfiguration('hysteresis_gain').perform(context).strip())
    frontier_clearance_radius = float(LaunchConfiguration('frontier_clearance_radius').perform(context).strip())
    failed_goal_radius = float(LaunchConfiguration('failed_goal_radius').perform(context).strip())
    failed_goal_cooldown = float(LaunchConfiguration('failed_goal_cooldown').perform(context).strip())
    publish_markers = _truthy(LaunchConfiguration('publish_markers').perform(context))
    nav_wait_warn_sec = float(LaunchConfiguration('nav_wait_warn_sec').perform(context).strip())
    tf_wait_warn_sec = float(LaunchConfiguration('tf_wait_warn_sec').perform(context).strip())
    robot_count = int(LaunchConfiguration('robot_count').perform(context).strip())
    robot_layout = LaunchConfiguration('robot_layout').perform(context).strip()
    spawn_x = float(LaunchConfiguration('spawn_x').perform(context).strip())
    spawn_y = float(LaunchConfiguration('spawn_y').perform(context).strip())
    spawn_z = float(LaunchConfiguration('spawn_z').perform(context).strip())
    spawn_yaw = float(LaunchConfiguration('spawn_yaw').perform(context).strip())
    spawn_spacing = float(LaunchConfiguration('spawn_spacing').perform(context).strip())
    validate_spawns = _truthy(LaunchConfiguration('validate_spawns').perform(context))
    robot_clearance = float(LaunchConfiguration('robot_clearance').perform(context).strip())
    spawn_search_radius = float(LaunchConfiguration('spawn_search_radius').perform(context).strip())
    spawn_search_step = float(LaunchConfiguration('spawn_search_step').perform(context).strip())
    nav2_start_delay = float(LaunchConfiguration('nav2_start_delay').perform(context).strip())
    robot_start_stagger = float(LaunchConfiguration('robot_start_stagger').perform(context).strip())
    amcl_start_delay = float(LaunchConfiguration('amcl_start_delay').perform(context).strip())
    robots_json = LaunchConfiguration('robots_json').perform(context).strip()
    drive_type = LaunchConfiguration('drive_type').perform(context).strip().lower()
    robot_model = LaunchConfiguration('robot_model').perform(context).strip().lower()
    controller = LaunchConfiguration('controller').perform(context).strip().lower()
    lidar_type = LaunchConfiguration('lidar_type').perform(context).strip().lower()
    if lidar_type == '3d' and drive_type != 'diff':
        print(f'[multi_robot] lidar_type=3d only supported for drive_type:=diff '
              f'(got drive_type:={drive_type}) — falling back to 2d')
        lidar_type = '2d'
    lidar3d_height = LaunchConfiguration('lidar3d_height').perform(context).strip()
    lidar3d_vfov_deg = LaunchConfiguration('lidar3d_vfov_deg').perform(context).strip()
    if slam_mode not in ('single', 'multi'):
        raise ValueError('slam_mode must be one of: single, multi')

    slam_pkg     = get_package_share_directory('slam_toolbox')
    urdf_filename = _common.urdf_filename_for(drive_type, robot_model)

    # Resolve world file
    if os.path.isabs(world_arg) and os.path.isfile(world_arg):
        world_path = world_arg
    else:
        world_name = world_arg or 'hospital'
        # Allow bare name ("maze") or filename ("maze.world")
        world_name = os.path.splitext(os.path.basename(world_name))[0]
        world_path = os.path.join(pkg_share, 'worlds', f'{world_name}.world')

    world_stem   = os.path.splitext(os.path.basename(world_path))[0]
    gazebo_world_name = _resolve_gazebo_world_name(world_path)
    map_prefix   = os.path.join(pkg_share, 'maps', f'map_{world_stem}')
    template_yaml = _multirobot_template_yaml(pkg_share, controller, drive_type)
    slam_template_yaml = os.path.join(
        pkg_share, 'config',
        'mapper_params_per_robot.yaml' if slam_mode == 'multi'
        else 'mapper_params_multirobot.yaml')
    robots = _resolve_robots(
        robot_count, robot_layout, spawn_x, spawn_y, spawn_z, spawn_yaw,
        spawn_spacing, robots_json)
    robots, spawn_adjustments = _safe_spawns(
        robots, world_path, validate_spawns, robot_clearance,
        spawn_search_radius, spawn_search_step)

    actions = [
        LogInfo(msg=f'[multi_robot] world = {world_path}'),
        LogInfo(msg=f'[multi_robot] fleet = {[r["name"] for r in robots]}'),
        LogInfo(msg=f'[multi_robot] spawn poses = {[(r["name"], r["x"], r["y"]) for r in robots]}'),
        LogInfo(msg=f'[multi_robot] explore = {explore}'),
        LogInfo(msg=f'[multi_robot] slam_mode = {slam_mode}'),
        LogInfo(msg=f'[multi_robot] drive_type = {drive_type}, controller = {controller}, urdf = {urdf_filename}'),
        LogInfo(msg=f'[multi_robot] nav2 template = {os.path.basename(template_yaml)} '
                    f'({"hand-tuned fleet template" if "ROBOT_NS" in open(template_yaml).read() else "single-robot params, namespaced at launch"})'),
    ]
    for msg in spawn_adjustments:
        actions.append(LogInfo(msg=f'[multi_robot] adjusted spawn: {msg}'))

    # ── Gazebo server + GUI ───────────────────────────────────────────────────
    actions.append(_common.gazebo_server_action(world_path, pkg_share))
    if not headless:
        actions.append(_common.gazebo_client_action(pkg_share))

    # Bridge the Gazebo clock exactly once.
    # Multiple /clock bridges can race and produce backward time jumps.
    actions.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'))

    # Bridge Gazebo's global TF exactly once. The Gazebo /tf stream already
    # contains all spawned robots, so duplicating this bridge per-robot causes
    # repeated transforms and cross-namespace contamination.
    actions.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='tf_bridge_global',
        arguments=['/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        output='screen'))

    # ── Shared map source ─────────────────────────────────────────────────────
    if explore and slam_mode == 'single':
        # SLAM Toolbox (robot1's 2D lidar → shared /map)
        actions += [
            LogInfo(msg=f'[multi_robot] SLAM mode — map will be auto-saved to {map_prefix}'),
            TimerAction(
                period=6.0,
                actions=[IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(slam_pkg, 'launch', 'online_async_launch.py')),
                    launch_arguments={
                        'slam_params_file': os.path.join(
                            pkg_share, 'config', 'mapper_params_multirobot.yaml'),
                        'use_sim_time': 'true',
                    }.items())]),
        ]
    elif explore:
        init_poses = _init_poses_for_merge(robots)
        # Static map → robotN/map so Nav2 can resolve map→base_link while each
        # slam_toolbox owns a private robotN/map→robotN/odom transform.
        for robot, pose in zip(robots, init_poses):
            ns = robot['name']
            actions.append(Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'{ns}_global_to_local_map',
                arguments=[
                    str(pose['x']), str(pose['y']), '0',
                    str(pose['yaw']), '0', '0',
                    'map', f'{ns}/map',
                ],
                output='screen'))
        actions += [
            LogInfo(msg='[multi_robot] multi-SLAM mode — per-robot /robotN/map + known-pose merge → /map_merged'),
            TimerAction(
                period=12.0,
                actions=[Node(
                    package='rosnav_bot',
                    executable='map_merge_known.py',
                    name='map_merge_known',
                    output='screen',
                    parameters=[{
                        'robot_namespaces': ','.join(r['name'] for r in robots),
                        'init_poses_json': json.dumps(init_poses),
                        'output_topic': '/map_merged',
                        'world_frame': 'map',
                        'publish_rate': 2.0,
                        'use_sim_time': True,
                    }])]),
        ]
    else:
        # Static map_server
        resolved_map = _resolve_map_file(map_arg, world_path, pkg_share)
        actions += [
            LogInfo(msg=f'[multi_robot] static map = {resolved_map}'),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{'use_sim_time': True, 'yaml_filename': resolved_map}]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_map',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': ['map_server'],
                }]),
        ]

    # ── Per-robot groups ──────────────────────────────────────────────────────
    # SLAM map origin = robot1's odom origin = robot1's Gazebo world start position.
    # All AMCL initial poses must be in map frame, so subtract robot1's world coords.
    slam_x_f = float(robots[0]['x'])
    slam_y_f = float(robots[0]['y'])

    for idx, robot in enumerate(robots):
        ns  = robot['name']
        x, y, z, yaw = robot['x'], robot['y'], robot['z'], robot['yaw']
        is_slam_robot = explore and (slam_mode == 'multi' or idx == 0)

        # Convert Gazebo world coords → map frame coords for AMCL.
        # Map origin = robot1's starting world position, so offset by robot1's spawn.
        map_x = str(float(x) - slam_x_f)
        map_y = str(float(y) - slam_y_f)

        # Template → per-robot YAML (ROBOT_NS substituted)
        initial_pose = None
        if not is_slam_robot:
            initial_pose = {
                'x': float(x) - slam_x_f,
                'y': float(y) - slam_y_f,
                'z': 0.0,
                'yaw': float(yaw),
            }
        nav_map_topic = '/map_merged' if explore and slam_mode == 'multi' else '/map'
        robot_params = _make_robot_params(template_yaml, ns, pkg_share, initial_pose, nav_map_topic)
        actions.append(LogInfo(msg=f'[multi_robot] {ns}: nav2 params -> {robot_params} (map_topic={nav_map_topic})'))

        # Robot State Publisher — frame_prefix + namespace arg makes TF frames unique per robot
        rsp = _common.rsp_include(
            pkg_share, os.path.join(pkg_share, 'urdf', urdf_filename),
            frame_prefix=f'{ns}/', namespace=ns, lidar_type=lidar_type,
            lidar3d_height=lidar3d_height, lidar3d_vfov_deg=lidar3d_vfov_deg)

        # Spawn in Gazebo
        spawn = _common.spawn_robot_node(
            gazebo_world_name, f'/{ns}/robot_description', ns, x, y, z, yaw)

        # Gazebo ↔ ROS bridge (lidar, odom, cmd_vel)
        # TF is handled by dedicated bridge nodes below to avoid the /{ns}/tf empty-topic problem.
        # The gz-side lidar topic is fixed to {ns}/scan by lidar.xacro, so the bridge
        # argument below must keep that name — the remapping is what renames the ROS-side
        # output to scan_raw, freeing "scan" for laser_filter's cleaned republish.
        bridge_args = [
            f'/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            f'/{ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ]
        bridge_remaps = [(f'/{ns}/scan', f'/{ns}/scan_raw')]
        if lidar_type == '3d':
            # gz-sim's gpu_lidar publishes gz.msgs.PointCloudPacked on a nested
            # "<topic>/points" (not the sensor's own <topic>, which carries
            # gz.msgs.LaserScan) — bridge that nested topic, then remap down
            # to a clean /{ns}/points on the ROS side.
            bridge_args.append(
                f'/{ns}/points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked')
            bridge_remaps.append((f'/{ns}/points/points', f'/{ns}/points'))
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            namespace=ns,
            arguments=bridge_args,
            remappings=bridge_remaps,
            output='screen')

        laser_filter = _common.laser_filter_node(pkg_share, namespace=ns)

        # AMCL — localises against /map (shared).
        # Skipped for robot1 in SLAM mode (SLAM provides the map→odom TF).
        amcl_node = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace=ns,
            output='screen',
            parameters=[_load_node_params(robot_params, 'amcl'), {'use_sim_time': True}],
            remappings=[
                ('tf', '/tf'),
                ('tf_static', '/tf_static'),
                ('map', '/map'),
                ('/map', '/map'),
            ])

        amcl_lc = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['amcl'],
            }])

        # opennav_docking — only the diff-drive template carries a tuned
        # docking_server: section (see nav2_multirobot_params.yaml); mecanum/
        # ackermann get no docking_server section, so skip it entirely rather
        # than starting a node with no dock config. Runs alongside nav2_group
        # via its own small lifecycle manager, independent of the shared
        # nav2_navigation_global_tf.launch.py bringup used by every drive_type.
        docking_actions = []
        if drive_type == 'diff':
            docking_node = Node(
                package='opennav_docking',
                executable='opennav_docking',
                name='docking_server',
                namespace=ns,
                output='screen',
                parameters=[_load_node_params(robot_params, 'docking_server'), {'use_sim_time': True}],
            )
            docking_lc = Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_docking',
                namespace=ns,
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': ['docking_server'],
                }])
            docking_actions = [docking_node, docking_lc]

        # Nav2 navigation stack (planner, controller, bt_navigator, …)
        # Pass namespace so RewrittenYaml wraps params under the robot key, making
        # /robot1/controller_server find its parameters. MPPI critics survive yaml.dump.
        nav2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'nav2_navigation_global_tf.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': robot_params,
                'namespace': ns,
            }.items())

        slam_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=ns,
            output='screen',
            parameters=[_make_slam_params(slam_template_yaml, ns), {'use_sim_time': True}],
            remappings=[
                ('tf', '/tf'),
                ('tf_static', '/tf_static'),
                ('map', f'/{ns}/map'),
                ('/map', f'/{ns}/map'),
            ])

        # Assemble per-robot actions
        nav2_group = nav2
        robot_stagger = idx * robot_start_stagger
        robot_nav2_delay = nav2_start_delay + robot_stagger
        robot_amcl_delay = amcl_start_delay + robot_stagger
        initial_pose_delay = robot_amcl_delay + 3.0
        amcl_nav2_delay = robot_amcl_delay + 6.0
        static_amcl_delay = 5.0 + robot_stagger
        static_initial_pose_delay = static_amcl_delay + 3.0
        static_nav2_delay = static_amcl_delay + 6.0

        per_robot = [rsp, spawn, bridge, laser_filter]

        if is_slam_robot:
            # SLAM handles localisation for this robot.
            slam_actions = []
            if slam_mode == 'multi':
                slam_actions.append(TimerAction(period=6.0 + robot_stagger, actions=[slam_node]))
            actions.append(LogInfo(
                msg=f'[multi_robot] {ns}: SLAM-localized, nav2 t={robot_nav2_delay:.1f}s'))
            per_robot += [
                *slam_actions,
                TimerAction(period=robot_nav2_delay, actions=[nav2_group, *docking_actions]),
            ]
        elif explore:
            # Other robots in explore mode: need AMCL to localise on SLAM map.
            # Stagger each robot so Nav2 lifecycle activation does not stampede.
            actions.append(LogInfo(
                msg=f'[multi_robot] {ns}: AMCL t={robot_amcl_delay:.1f}s, '
                    f'initialpose t={initial_pose_delay:.1f}s, nav2 t={amcl_nav2_delay:.1f}s'))
            per_robot += [
                TimerAction(period=robot_amcl_delay, actions=[
                    amcl_node,
                    amcl_lc,
                ]),
                TimerAction(period=initial_pose_delay, actions=[_initial_pose_pub(ns, map_x, map_y, yaw)]),
                TimerAction(period=amcl_nav2_delay, actions=[nav2_group, *docking_actions]),
            ]
        else:
            # Pre-built map mode: all robots use AMCL
            actions.append(LogInfo(
                msg=f'[multi_robot] {ns}: AMCL t={static_amcl_delay:.1f}s, '
                    f'initialpose t={static_initial_pose_delay:.1f}s, nav2 t={static_nav2_delay:.1f}s'))
            per_robot += [
                TimerAction(period=static_amcl_delay, actions=[
                    amcl_node,
                    amcl_lc,
                ]),
                TimerAction(period=static_initial_pose_delay, actions=[_initial_pose_pub(ns, map_x, map_y, yaw)]),
                TimerAction(period=static_nav2_delay, actions=[nav2_group, *docking_actions]),
            ]

        actions.append(GroupAction(per_robot))

    # ── Auxiliary-scan map fusion (opt-in, merge_scans:=true) ──────────────────
    # robot1 drives slam_toolbox exclusively (see mapper_params_multirobot.yaml);
    # every other robot only localises via AMCL, so its scans never reach /map.
    # This node layers those robots' scans into cells slam_toolbox still marks
    # unknown, publishing /map_fused for consumers that opt in — it never
    # touches slam_toolbox, AMCL, or /map itself. Default (merge_scans:=false)
    # leaves the existing SLAM/localisation pipeline completely untouched.
    frontier_map_topic = '/map_merged' if explore and slam_mode == 'multi' else '/map'
    if explore and merge_scans and slam_mode == 'single':
        aux_robots = ','.join(r['name'] for r in robots[1:])
        frontier_map_topic = '/map_fused'
        if aux_robots:
            actions.append(TimerAction(
                period=20.0,
                actions=[Node(
                    package='rosnav_bot',
                    executable='map_fusion.py',
                    name='map_fusion',
                    output='screen',
                    parameters=[{
                        'robot_namespaces': aux_robots,
                        'base_map_topic': '/map',
                        'output_topic': frontier_map_topic,
                    }])]))
            actions.append(LogInfo(
                msg=f'[multi_robot] map_fusion will start at t=20s fusing [{aux_robots}] -> {frontier_map_topic}'))
    elif explore and merge_scans and slam_mode == 'multi':
        actions.append(LogInfo(
            msg='[multi_robot] merge_scans ignored in slam_mode:=multi; using /map_merged from map_merge_known'))

    # ── Centralized frontier coordinator (explore mode only) ──────────────────
    robot_ns_list = ','.join(r['name'] for r in robots)
    if explore:
        # Start after all Nav2 stacks are up (robot1 at 10s, others at 13s)
        actions.append(TimerAction(
            period=22.0 if slam_mode == 'multi' else (21.0 if merge_scans else 20.0),
            actions=[Node(
                package='rosnav_bot',
                executable='frontier_coordinator.py',
                name='frontier_coordinator',
                output='screen',
                parameters=[{
                    'robot_namespaces': robot_ns_list,
                    'map_save_path': map_prefix,
                    'map_topic': frontier_map_topic,
                    'frontier_detector': frontier_detector,
                    'frontier_scorer': frontier_scorer,
                    'distance_weight': distance_weight,
                    'info_gain_weight': info_gain_weight,
                    'potential_scale': potential_scale,
                    'gain_scale': gain_scale,
                    'validate_on_costmap': validate_on_costmap,
                    'costmap_max_cost': costmap_max_cost,
                    'goal_pullback': goal_pullback,
                    'hysteresis_radius': hysteresis_radius,
                    'hysteresis_gain': hysteresis_gain,
                    'frontier_clearance_radius': frontier_clearance_radius,
                    'failed_goal_radius': failed_goal_radius,
                    'failed_goal_cooldown': failed_goal_cooldown,
                    'publish_markers': publish_markers,
                    'nav_wait_warn_sec': nav_wait_warn_sec,
                    'tf_wait_warn_sec': tf_wait_warn_sec,
                    'behavior_tree': explore_bt,
                }])]))
        actions.append(LogInfo(
            msg=f'[multi_robot] frontier_coordinator will start at t={22.0 if slam_mode == "multi" else (21.0 if merge_scans else 20.0)}s '
                f'for {robot_ns_list} (map_topic={frontier_map_topic}, '
                f'detector={frontier_detector}, scorer={frontier_scorer}, bt={explore_bt})'))

    # ── Fleet management algorithms (optional) ────────────────────────────────
    if fleet_mgmt:
        actions.append(TimerAction(
            period=15.0,
            actions=[
                Node(
                    package='rosnav_bot',
                    executable='mission_server.py',
                    name='mission_server',
                    output='screen'),
                Node(
                    package='rosnav_bot',
                    executable='task_allocator.py',
                    name='task_allocator',
                    output='screen',
                    parameters=[{'robots': robot_ns_list}]),
                Node(
                    package='rosnav_bot',
                    executable='fleet_health.py',
                    name='fleet_health_monitor',
                    output='screen'),
                Node(
                    package='rosnav_bot',
                    executable='priority_collision_avoidance.py',
                    name='priority_collision_avoidance',
                    output='screen',
                    parameters=[{'robot_namespaces': robot_ns_list}]),
                Node(
                    package='rosnav_bot',
                    executable='deadlock_recovery.py',
                    name='deadlock_recovery',
                    output='screen',
                    parameters=[{'robot_namespaces': robot_ns_list}]),
            ]))
        actions.append(LogInfo(
            msg=f'[multi_robot] fleet_mgmt stack will start at t=15s for {robot_ns_list}'))

    # ── RViz ─────────────────────────────────────────────────────────────────
    if not headless:
        rviz_config = LaunchConfiguration('rviz_config').perform(context).strip()
        if not rviz_config:
            rviz_config = 'multi_robot_merged.rviz' if slam_mode == 'multi' else 'multi_robot.rviz'
        actions.append(GroupAction(
            condition=IfCondition(LaunchConfiguration('rviz')),
            actions=[Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', os.path.join(pkg_share, 'rviz', rviz_config)],
                output='screen')]))

    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory('rosnav_bot')
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='hospital',
            description='World name (hospital, corridor, maze, obstacles) or full path to .world file'),
        DeclareLaunchArgument(
            'map', default_value='',
            description='Pre-built map yaml path. Ignored when explore:=true'),
        DeclareLaunchArgument(
            'rviz', default_value='True', description='Launch RViz'),
        DeclareLaunchArgument(
            'rviz_config', default_value='',
            description='RViz config filename under rviz/. Default auto-picks '
                        'multi_robot_merged.rviz (slam_mode:=multi) or multi_robot.rviz (slam_mode:=single).'),
        DeclareLaunchArgument(
            'explore', default_value='true',
            description='true = SLAM + frontier exploration (default). '
                        'false = use saved map yaml'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true = Gazebo server only, no GUI or RViz'),
        DeclareLaunchArgument(
            'fleet_mgmt', default_value='false',
            description='true = start priority collision avoidance + deadlock recovery nodes'),
        DeclareLaunchArgument(
            'drive_type', default_value='diff',
            description='Fleet-wide drive base: "diff" (default, hand-tuned fleet nav2 params), '
                        '"mecanum" (holonomic) or "ackermann" (car-like steering). '
                        'mecanum/ackermann reuse the single-robot nav2_params_*.yaml, namespaced '
                        'per-robot at launch time — see slam_nav.launch.py for the same drive types.'),
        DeclareLaunchArgument(
            'robot_model', default_value='custom',
            description='Fleet-wide chassis visual skin: "custom" (default, this repo\'s own '
                        'simple box chassis), "mir100" (MiR100-shaped mesh), or "husky" '
                        '(Clearpath Husky A200-shaped mesh) — all scaled to the same footprint, '
                        'visual only. Only supported with drive_type:=diff.'),
        DeclareLaunchArgument(
            'lidar_type', default_value='2d',
            description='Fleet-wide lidar: "2d" (default, LaserScan on /{ns}/scan) or "3d" '
                        '(PointCloud2 on /{ns}/points, gpu_lidar). Only drive_type:=diff.'),
        DeclareLaunchArgument(
            'lidar3d_height', default_value='0.25',
            description='3D lidar mount height (m), lidar_type:=3d only. Needs >=0.10m '
                        'clearance above the chassis top (z=0.15) to avoid self-hits.'),
        DeclareLaunchArgument(
            'lidar3d_vfov_deg', default_value='10',
            description='3D lidar vertical half-angle in degrees (+/-), lidar_type:=3d only.'),
        DeclareLaunchArgument(
            'controller', default_value='dwb',
            description='Local controller for drive_type:=mecanum: "dwb" (default) or "mppi". '
                        'Humble only — ignored on Jazzy. Ignored for drive_type:=diff (fleet always '
                        'uses its hand-tuned MPPI template) and drive_type:=ackermann (always MPPI).'),
        DeclareLaunchArgument(
            'robot_count', default_value='2',
            description='Number of generated robots when robots_json is empty'),
        DeclareLaunchArgument(
            'robot_layout', default_value='line',
            description='Generated spawn layout: line, grid, or circle'),
        DeclareLaunchArgument(
            'spawn_x', default_value='-2.0',
            description='Base x position for generated robot spawns'),
        DeclareLaunchArgument(
            'spawn_y', default_value='-1.0',
            description='Base y position for generated robot spawns'),
        DeclareLaunchArgument(
            'spawn_z', default_value='0.3',
            description='Spawn z position for generated robot spawns'),
        DeclareLaunchArgument(
            'spawn_yaw', default_value='0.0',
            description='Spawn yaw for generated robot spawns'),
        DeclareLaunchArgument(
            'spawn_spacing', default_value='1.2',
            description='Spacing between generated robot spawns'),
        DeclareLaunchArgument(
            'validate_spawns', default_value='true',
            description='true = parse SDF collisions and relocate spawns out of walls/obstacles'),
        DeclareLaunchArgument(
            'robot_clearance', default_value='0.45',
            description='Minimum spawn clearance from walls/obstacles and other robots'),
        DeclareLaunchArgument(
            'spawn_search_radius', default_value='4.0',
            description='Maximum radius to search when relocating blocked spawns'),
        DeclareLaunchArgument(
            'spawn_search_step', default_value='0.25',
            description='Radial step used when searching for free spawn positions'),
        DeclareLaunchArgument(
            'nav2_start_delay', default_value='10.0',
            description='Base delay before starting robot1 Nav2 in explore mode'),
        DeclareLaunchArgument(
            'amcl_start_delay', default_value='13.0',
            description='Base delay before starting AMCL for non-SLAM robots in explore mode'),
        DeclareLaunchArgument(
            'robot_start_stagger', default_value='6.0',
            description='Additional seconds of AMCL/Nav2 startup delay per robot index'),
        DeclareLaunchArgument(
            'robots_json', default_value='',
            description='Optional exact robot list JSON, e.g. '
                        '\'[{"name":"robot1","x":-2,"y":-1},{"name":"robot2","x":-0.8,"y":-1}]\''),
        DeclareLaunchArgument(
            'merge_scans', default_value='false',
            description='true = fuse non-SLAM robots\' scans into unknown /map cells '
                        '(published as /map_fused) so the fleet maps faster than '
                        'robot1 alone. Opt-in; does not touch SLAM or AMCL.'),
        DeclareLaunchArgument(
            'slam_mode', default_value='single',
            description='single = robot1 SLAM + AMCL for other robots; '
                        'multi = each robot runs namespaced SLAM and maps merge to /map_merged'),
        DeclareLaunchArgument(
            'frontier_detector', default_value='wfd',
            description='Frontier detector plugin: wfd = reachable wavefront frontiers; '
                        'classic = all free/unknown boundary cells'),
        DeclareLaunchArgument(
            'frontier_scorer', default_value='utility',
            description='Frontier scorer plugin: utility = size/distance tradeoff; '
                        'weighted = info gain minus distance; nearest = closest valid frontier'),
        DeclareLaunchArgument(
            'explore_bt', default_value='explore_nav',
            description='BT XML stem or path passed as NavigateToPose.behavior_tree '
                        'for frontier goals (default explore_nav)'),
        DeclareLaunchArgument(
            'distance_weight', default_value='1.0',
            description='Weighted scorer distance penalty'),
        DeclareLaunchArgument(
            'info_gain_weight', default_value='3.0',
            description='Weighted scorer information gain reward'),
        DeclareLaunchArgument(
            'potential_scale', default_value='3.0',
            description='Utility scorer distance penalty'),
        DeclareLaunchArgument(
            'gain_scale', default_value='1.0',
            description='Utility scorer frontier-size reward'),
        DeclareLaunchArgument(
            'validate_on_costmap', default_value='true',
            description='Reject frontier goals in inflated/lethal Nav2 costmap cells'),
        DeclareLaunchArgument(
            'costmap_max_cost', default_value='1',
            description='Reject costmap OccupancyGrid values >= this (0=free; '
                        'inflation 1-98 — use 1 to stay out of inflation layer)'),
        DeclareLaunchArgument(
            'goal_pullback', default_value='0.55',
            description='Stand back from unknown frontier into free space (m)'),
        DeclareLaunchArgument(
            'hysteresis_radius', default_value='2.0',
            description='Radius for keeping a robot near its current exploration region'),
        DeclareLaunchArgument(
            'hysteresis_gain', default_value='1.5',
            description='Weighted scorer bonus for current-region continuity'),
        DeclareLaunchArgument(
            'frontier_clearance_radius', default_value='0.55',
            description='Minimum OccupancyGrid clearance around selected frontier goals'),
        DeclareLaunchArgument(
            'failed_goal_radius', default_value='0.75',
            description='Radius for matching recently failed frontier goals'),
        DeclareLaunchArgument(
            'failed_goal_cooldown', default_value='45.0',
            description='Seconds to avoid a frontier area after Nav2 reports failure'),
        DeclareLaunchArgument(
            'publish_markers', default_value='true',
            description='Publish RViz debug markers on /exploration/frontiers'),
        DeclareLaunchArgument(
            'nav_wait_warn_sec', default_value='15.0',
            description='Seconds between frontier coordinator warnings for missing Nav2 action servers'),
        DeclareLaunchArgument(
            'tf_wait_warn_sec', default_value='15.0',
            description='Seconds between frontier coordinator warnings for missing map->base_link TF'),
        OpaqueFunction(function=_build_all, args=[pkg_share]),
    ])
