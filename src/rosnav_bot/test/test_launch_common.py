

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
