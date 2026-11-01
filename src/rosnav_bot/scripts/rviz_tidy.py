#!/usr/bin/env python3
"""
rviz_tidy.py — give the per-mode RViz configs one consistent, uncluttered structure (idempotent).

  python3 scripts/rviz_tidy.py rviz/slam_explore.rviz [more.rviz ...]

What it does to each config:
  * groups the flat display list into folders: Robot & TF / Maps / Navigation / SLAM & 3D / Sensors / Cameras
  * gives displays consistent names (Map, Global Costmap, Local Costmap, ...) instead of two displays called "Map"
  * drops displays on topics that no longer exist (/camera/depth/points is not bridged any more)
  * quiets TF (no names/arrows, smaller axes) and softens the costmap overlays
  * adds the MPC controller's horizon (/mpc/prediction) and obstacle points (/mpc/obstacles) to Navigation
Displays keep all their own settings; only structure, names and a few defaults change.
"""
import sys

import yaml

GROUPS = ['Robot & TF', 'Maps', 'Navigation', 'SLAM & 3D', 'Sensors', 'Cameras']
DEAD_TOPICS = {'/camera/depth/points', '/exploration_goal_flags'}   # flags clutter the map
RENAME_BY_TOPIC = {
    '/map': 'Map',
    '/global_costmap/costmap': 'Global Costmap',
    '/local_costmap/costmap': 'Local Costmap',
    '/plan': 'Global Plan',
    '/local_plan': 'Local Plan',
    '/scan': 'Lidar Scan',
    '/points': 'Lidar 3D Cloud',
    '/cloud_map': 'Map Cloud',
    '/octomap_occupied_space': 'Octomap',
    '/odom': 'Wheel Odom',
    '/odometry/filtered': 'Fused Odom',
    '/camera/image_raw': 'Camera',
    '/camera/depth/image_raw': 'Depth',
    '/world_monitor/image_raw': 'World Monitor',
    '/mapPath': 'Pose-Graph Path',
    '/plan_smoothed': 'Smoothed Plan',
    '/obstacle_markers': 'Dynamic Obstacles',
    '/exploration_goal_flags': 'Exploration Goals',
    '/scan_odom_cloud': 'Costmap Scan Cloud (odom)',
    '/points_traversability': 'Traversability',
}


def topic_of(d):
    t = d.get('Topic')
    return t.get('Value', '') if isinstance(t, dict) else (t or '')


def group_of(d):
    cls, topic, name = d['Class'].split('/')[-1], topic_of(d), d.get('Name', '')
    if cls in ('TF', 'RobotModel', 'Grid'):
        return 'Robot & TF'
    if cls == 'Map' and 'costmap' not in topic:
        return 'Maps'
    if cls == 'Map':
        return 'Maps'
    if cls in ('Path', 'Polygon') or topic in ('/explore/frontiers', '/exploration_goal_flags', '/trajectories', '/obstacle_markers') \
            or topic.startswith('/mpc/'):
        return 'Navigation'
    if topic == '/scan_odom_cloud':
        return 'Sensors'
    if cls in ('MapCloud', 'MapGraph', 'PointCloud2') or 'graph' in topic.lower() or 'octomap' in topic:
        return 'SLAM & 3D'
    if cls == 'MarkerArray':
        return 'SLAM & 3D' if 'slam' in topic or 'graph' in name.lower() else 'Navigation'
    if cls in ('LaserScan', 'Odometry'):
        return 'Sensors'
    if cls == 'Image':
        return 'Cameras'
    return 'Sensors'


def mpc_displays():
    qos = {'Depth': 5, 'Durability Policy': 'Volatile', 'Filter size': 10,
           'History Policy': 'Keep Last', 'Reliability Policy': 'Reliable'}
    return [
        {'Alpha': 1, 'Buffer Length': 1, 'Class': 'rviz_default_plugins/Path', 'Color': '0; 220; 255',
         'Enabled': True, 'Head Diameter': 0.3, 'Head Length': 0.2, 'Length': 0.3, 'Line Style': 'Lines',
         'Line Width': 0.03, 'Name': 'MPC Horizon', 'Offset': {'X': 0, 'Y': 0, 'Z': 0.05},
         'Pose Color': '0; 220; 255', 'Pose Style': 'None', 'Radius': 0.03, 'Shaft Diameter': 0.1,
         'Shaft Length': 0.1, 'Topic': dict(qos, Value='/mpc/prediction'), 'Value': True},
        {'Class': 'rviz_default_plugins/MarkerArray', 'Enabled': True, 'Name': 'MPC Obstacles',
         'Namespaces': {}, 'Topic': dict(qos, Value='/mpc/obstacles'), 'Value': True},
    ]


def extra_displays():
    """Other published topics worth showing, added when a config lacks them (off by default if noisy)."""
    q = {'Depth': 5, 'Durability Policy': 'Volatile', 'Filter size': 10, 'History Policy': 'Keep Last',
         'Reliability Policy': 'Reliable'}
    path = lambda name, topic, col, on=True: {
        'Alpha': 1, 'Buffer Length': 1, 'Class': 'rviz_default_plugins/Path', 'Color': col, 'Enabled': on,
        'Head Diameter': 0.3, 'Head Length': 0.2, 'Length': 0.3, 'Line Style': 'Lines', 'Line Width': 0.03,
        'Name': name, 'Offset': {'X': 0, 'Y': 0, 'Z': 0.03}, 'Pose Color': col, 'Pose Style': 'None',
        'Radius': 0.03, 'Shaft Diameter': 0.1, 'Shaft Length': 0.1, 'Topic': dict(q, Value=topic), 'Value': on}
    marks = lambda name, topic, on=True: {
        'Class': 'rviz_default_plugins/MarkerArray', 'Enabled': on, 'Name': name, 'Namespaces': {},
        'Topic': dict(q, Value=topic), 'Value': on}
    return [
        path('Smoothed Plan', '/plan_smoothed', '255; 170; 0'),
        {'Class': 'rviz_default_plugins/Marker', 'Enabled': True, 'Name': 'Dynamic Obstacles', 'Namespaces': {},
         'Topic': dict(q, Value='/obstacle_markers'), 'Value': True},
        {'Alpha': 1, 'Autocompute Intensity Bounds': True, 'Autocompute Value Bounds': {'Max Value': 10, 'Min Value': 0, 'Value': True},
         'Axis': 'Z', 'Channel Name': 'intensity', 'Class': 'rviz_default_plugins/PointCloud2', 'Color': '255; 85; 255',
         'Color Transformer': 'FlatColor', 'Decay Time': 0, 'Enabled': False, 'Invert Rainbow': False,
         'Max Color': '255; 255; 255', 'Max Intensity': 4096, 'Min Color': '0; 0; 0', 'Min Intensity': 0,
         'Name': 'Costmap Scan Cloud (odom)', 'Position Transformer': 'XYZ', 'Selectable': True, 'Size (Pixels)': 3,
         'Size (m)': 0.05, 'Style': 'Flat Squares', 'Topic': dict(q, Value='/scan_odom_cloud'), 'Use Fixed Frame': True,
         'Use rainbow': True, 'Value': False},
    ]


def tidy(path):
    with open(path) as f:
        cfg = yaml.safe_load(f)
    vm = cfg['Visualization Manager']
    flat = []
    for d in vm['Displays']:                      # flatten earlier groups so re-runs are idempotent
        if d['Class'] == 'rviz_common/Group':
            flat.extend(d.get('Displays', []))
        else:
            flat.append(d)
    kept, seen = [], set()
    for d in flat:
        topic = topic_of(d)
        if topic in DEAD_TOPICS:
            continue
        if topic in RENAME_BY_TOPIC:
            d['Name'] = RENAME_BY_TOPIC[topic]
        cls = d['Class'].split('/')[-1]
        if d['Name'] == 'Depth' and cls == 'Image':      # depth image is black in sim, wastes dock space
            d['Enabled'] = d['Value'] = False
        if cls == 'TF':
            d['Show Names'], d['Show Arrows'], d['Marker Scale'] = False, False, 0.4
        if cls == 'Map' and 'costmap' in topic:
            d['Alpha'] = 0.35 if d['Name'].startswith('Global') else 0.5
            d['Draw Behind'] = True
        if d['Name'] in seen:                     # still duplicated: disambiguate
            d['Name'] = f"{d['Name']} ({topic or cls})"
        seen.add(d['Name'])
        kept.append(d)
    names = {d['Name'] for d in kept}
    if 'multi' not in path and 'gs_' not in path and not any(topic_of(d).startswith('/mpc/') for d in kept):
        kept.extend(x for x in mpc_displays() if x['Name'] not in names)
    if 'multi' not in path and 'gs_' not in path:
        have = {topic_of(d) for d in kept}
        kept.extend(x for x in extra_displays() if topic_of(x) not in have)
    buckets = {g: [] for g in GROUPS}
    for d in kept:
        buckets[group_of(d)].append(d)
    vm['Displays'] = [{'Class': 'rviz_common/Group', 'Displays': buckets[g], 'Enabled': True, 'Name': g}
                      for g in GROUPS if buckets[g]]
    for p in cfg.get('Panels', []):               # open the groups in the Displays panel
        if p.get('Class') == 'rviz_common/Displays':
            p['Expanded'] = ['/Global Options1', '/Status1'] + [f'/{g}1' for g in GROUPS]
    with open(path, 'w') as f:
        yaml.safe_dump(cfg, f, sort_keys=False, default_flow_style=False, width=120)
    print(f'{path}: ' + ', '.join(f'{g}={len(buckets[g])}' for g in GROUPS if buckets[g]))


if __name__ == '__main__':
    for p in sys.argv[1:]:
        tidy(p)
