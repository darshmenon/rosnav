

def test_global_drop_scan_patch_removes_only_the_scan_source():
    import importlib.util, os
    sp = importlib.util.spec_from_file_location('c', os.path.join(os.path.dirname(__file__), '..', 'launch', '_common.py'))
    c = importlib.util.module_from_spec(sp); sp.loader.exec_module(c)
    d = {'global_costmap': {'global_costmap': {'ros__parameters': {'obstacle_layer': {'observation_sources': 'scan points'}}}}}
    assert c.global_drop_scan_patch(d) is True
    assert d['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['observation_sources'] == 'points'
    assert c.global_drop_scan_patch(d) is False                    # nothing left to drop
    assert c.global_drop_scan_patch({}) is False                   # no global costmap section


def test_global_drop_scan_patch_after_namespacing():
    import importlib.util, os
    sp = importlib.util.spec_from_file_location('c', os.path.join(os.path.dirname(__file__), '..', 'launch', '_common.py'))
    c = importlib.util.module_from_spec(sp); sp.loader.exec_module(c)
    d = {'global_costmap': {'global_costmap': {'ros__parameters': {'obstacle_layer': {
        'observation_sources': 'scan points', 'scan': {'topic': '/robot2/scan'}, 'points': {'topic': '/robot2/points'}}}}}}
    assert c.global_drop_scan_patch(d) is True
    assert d['global_costmap']['global_costmap']['ros__parameters']['obstacle_layer']['observation_sources'] == 'points'


def test_fleet_rviz_config_any_robot_count():
    import importlib.util, os
    sp = importlib.util.spec_from_file_location('c', os.path.join(os.path.dirname(__file__), '..', 'launch', '_common.py'))
    _common = importlib.util.module_from_spec(sp); sp.loader.exec_module(_common)
    base = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'rviz', 'multi_robot.rviz')
    for names in (['robot1'], ['robot1', 'robot2', 'robot3'], ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i']):
        cfg = _common.fleet_rviz_config(names, base)
        disp = cfg['Visualization Manager']['Displays']
        group = next(d for d in disp if d['Name'] == 'Robot & TF')
        models = [d for d in group['Displays'] if d['Name'].endswith(' Model')]
        assert [m['TF Prefix'] for m in models] == names
        topics = {d['Topic']['Value'] for d in disp if 'Topic' in d}
        for n in names:
            assert f'/{n}/plan' in topics and f'/{n}/scan' in topics
        assert not any(d.get('Name') == 'Sensors' for d in disp)


def test_docking_lifecycle_manager_bond_disabled():
    """Nav2 Humble opennav_docking cannot be re-configured (navigator_bt_xml is
    re-declared), so the docking lifecycle manager must not reset it on a missed bond."""
    import os, re
    p = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'launch', 'multi_robot.launch.py')
    src = open(p).read()
    blk = src[src.index("name='lifecycle_manager_docking'"):]
    blk = blk[:blk.index('docking_actions = [docking_node')]
    assert re.search(r"'bond_timeout':\s*0\.0", blk)


def _load_common():
    import importlib.util, os
    sp = importlib.util.spec_from_file_location('c', os.path.join(os.path.dirname(__file__), '..', 'launch', '_common.py'))
    c = importlib.util.module_from_spec(sp); sp.loader.exec_module(c)
    return c


def _walk(displays):
    for d in displays:
        yield d
        yield from _walk(d.get('Displays', []))


def test_fleet_rviz_config_map_display_cloud_replaces_primary_map_only():
    import os
    c = _load_common()
    base = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'rviz', 'multi_robot.rviz')
    cfg = c.fleet_rviz_config(['robot1', 'robot2'], base)          # default = new behaviour
    ds = list(_walk(cfg['Visualization Manager']['Displays']))
    assert not any(d['Name'] == 'Map' for d in ds)
    clouds = {d['Topic']['Value']: d for d in ds if d['Class'] == 'rviz_default_plugins/PointCloud2'}
    assert set(clouds) == {'/map_viz/free', '/map_viz/occupied'}
    assert all(d['Color Transformer'] == 'FlatColor' for d in clouds.values())
    # costmap displays stay Map displays
    assert any(d['Class'] == 'rviz_default_plugins/Map' and d['Name'] == 'Global Costmap' for d in ds)


def test_fleet_rviz_config_map_display_map_keeps_original_and_rejects_bad_value():
    import os
    import pytest
    c = _load_common()
    base = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'rviz', 'multi_robot.rviz')
    cfg = c.fleet_rviz_config(['robot1'], base, map_display='map')
    ds = list(_walk(cfg['Visualization Manager']['Displays']))
    m = next(d for d in ds if d['Name'] == 'Map')
    assert m['Class'] == 'rviz_default_plugins/Map' and m['Color Scheme'] == 'map'
    assert not any(d['Class'] == 'rviz_default_plugins/PointCloud2' for d in ds)
    with pytest.raises(ValueError):
        c.fleet_rviz_config(['robot1'], base, map_display='bogus')


def test_occupancy_cloud_grid_to_points():
    import importlib.util, os
    sp = importlib.util.spec_from_file_location('oc', os.path.join(os.path.dirname(__file__), '..', 'scripts', 'occupancy_cloud.py'))
    oc = importlib.util.module_from_spec(sp); sp.loader.exec_module(oc)
    # 3x2 grid: row0 = [free, occupied, unknown], row1 = [free, 50 (mid), 100]
    occ, free = oc.grid_to_points([0, 100, -1, 0, 50, 100], 3, 2, 0.5, 1.0, 2.0)
    assert sorted(map(tuple, occ[:, :2].tolist())) == [(1.75, 2.25), (2.25, 2.75)]
    assert sorted(map(tuple, free[:, :2].tolist())) == [(1.25, 2.25), (1.25, 2.75)]
    assert (occ[:, 2] > free[:, 2]).all()
    e_occ, e_free = oc.grid_to_points([-1] * 4, 2, 2, 0.1, 0, 0)
    assert len(e_occ) == 0 and len(e_free) == 0
