import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from scripts.nav2_doctor import Doctor


def codes(d, now):
    return {f.code for f in d.diagnose(now)}


def healthy(now=0.0):
    d = Doctor()
    d.tf('map', 'odom', now); d.tf('odom', 'base_link', now)
    d.seen('odom', now); d.seen('scan', now)
    for n in ('controller_server', 'planner_server'):
        d.lifecycle[n] = 'active'
    return d


def test_healthy():
    assert codes(healthy(), 1.0) == {'HEALTHY'}


def test_no_tf():
    assert 'TF_NONE' in codes(Doctor(), 1.0)


def test_tf_freeze():
    assert 'TF_FREEZE' in codes(healthy(), 10.0)


def test_single_edge_stale():
    d = healthy()
    d.tf('odom', 'base_link', 9.5)   # odom->base alive, map->odom died
    assert 'TF_EDGE_STALE' in codes(d, 10.0)


def test_inactive_node():
    d = healthy(100.0); d.lifecycle['controller_server'] = 'inactive'
    d.seen('odom', 100.0); d.started = 0.0
    assert 'NODE_INACTIVE' in codes(d, 100.5)


def test_inactive_during_startup_is_warn():
    d = healthy(); d.lifecycle['controller_server'] = 'unconfigured'
    c = codes(d, 1.0)
    assert 'NODE_STARTING' in c and 'NODE_INACTIVE' not in c


def test_flapping():
    d = healthy()
    for t in (0.1, 0.2, 0.3, 0.4):
        d.goal(t)
    assert 'GOAL_FLAPPING' in codes(d, 1.0)


def test_cmd_zero_with_active_goal():
    d = healthy(); d.goal_active = True; d.cmd(0, 0, 0.9)
    assert 'CMD_ZERO' in codes(d, 1.0)


def test_moving_ok():
    d = healthy(); d.goal_active = True; d.cmd(0.2, 0, 0.9)
    assert 'CMD_ZERO' not in codes(d, 1.0)



# ---- edge cases -------------------------------------------------------
from scripts.nav2_doctor import fmt, STALE_S, FLAP_WINDOW_S


def test_leading_slash_frames_normalised():
    d = Doctor(); d.tf('/map', '/odom', 0.0); d.tf('/odom', '/base_link', 0.0)
    assert ('map', 'odom') in d.tf_edges and 'NO_MAP_ODOM' not in codes(d, 0.5)


def test_namespaced_frames_count_as_map_odom():
    d = Doctor(); d.tf('robot1/map', 'robot1/odom', 0.0)
    assert 'NO_MAP_ODOM' not in codes(d, 0.5)


def test_missing_map_odom_warns():
    d = Doctor(); d.tf('odom', 'base_link', 0.0)
    assert 'NO_MAP_ODOM' in codes(d, 0.5)


def test_stale_boundary():
    d = healthy()
    assert 'TF_FREEZE' not in codes(d, STALE_S - 0.01)
    assert 'TF_FREEZE' in codes(d, STALE_S + 0.5)


def test_flapping_window_expires():
    d = healthy()
    for t in (0.0, 0.1, 0.2, 0.3):
        d.goal(t)
    d.goal(FLAP_WINDOW_S + 5)           # old goals fall out of window
    assert 'GOAL_FLAPPING' not in codes(d, FLAP_WINDOW_S + 5.5)


def test_three_goals_not_flapping():
    d = healthy()
    for t in (0.1, 0.2, 0.3):
        d.goal(t)
    assert 'GOAL_FLAPPING' not in codes(d, 1.0)


def test_no_goal_zero_cmd_is_fine():
    d = healthy(); d.cmd(0, 0, 0.9)
    assert codes(d, 1.0) == {'HEALTHY'}


def test_goal_active_without_cmd_vel():
    d = healthy(); d.goal_active = True
    assert 'NO_CMD_VEL' in codes(d, 1.0)


def test_cmd_vel_stops_while_goal_active():
    d = healthy(); d.goal_active = True
    d.cmd(0.3, 0, 0.0); d.cmd(0, 0, 5.0)
    d.tf('map', 'odom', 5.0); d.tf('odom', 'base_link', 5.0); d.seen('odom', 5.0); d.seen('scan', 5.0)
    assert 'CMD_ZERO' in codes(d, 5.5)


def test_angular_only_counts_as_moving():
    d = healthy(); d.goal_active = True; d.cmd(0.0, 0.5, 0.9)
    assert 'CMD_ZERO' not in codes(d, 1.0)


def test_unreachable_node_warns_not_errors():
    d = healthy(); d.lifecycle['bt_navigator'] = None
    c = {f.code: f.level for f in d.diagnose(1.0)}
    assert c['NODE_MISSING'] == 'WARN' and 'HEALTHY' in c


def test_active_nodes_are_fine():
    d = healthy(); d.lifecycle['bt_navigator'] = 'active'
    assert codes(d, 1.0) == {'HEALTHY'}


def test_costmap_stale_only_when_previously_seen():
    d = healthy()
    assert 'COSTMAP_STALE' not in codes(d, 1.0)
    d.seen('local_costmap', 0.0); d.tf('map', 'odom', 5.0); d.tf('odom', 'base_link', 5.0)
    d.seen('odom', 5.0); d.seen('scan', 5.0)
    assert 'COSTMAP_STALE' in codes(d, 5.5)


def test_scan_and_odom_none_warn():
    d = Doctor(); d.tf('map', 'odom', 0.0)
    c = codes(d, 0.5)
    assert {'ODOM_NONE', 'SCAN_NONE'} <= c


def test_healthy_has_no_errors_or_warns():
    assert [f.level for f in healthy().diagnose(0.5)] == ['OK']


def test_fmt_renders_every_finding():
    d = Doctor()
    out = fmt(d.diagnose(1.0))
    assert 'TF_NONE' in out and out.count('\n') == len(d.diagnose(1.0)) - 1


def test_recovery_clears_errors():
    d = healthy()
    assert 'TF_FREEZE' in codes(d, 10.0)
    d.tf('map', 'odom', 10.0); d.tf('odom', 'base_link', 10.0); d.seen('odom', 10.0); d.seen('scan', 10.0)
    assert codes(d, 10.2) == {'HEALTHY'}


def test_custom_startup_grace():
    d = healthy(); d.startup_s = 0.0; d.started = 0.0
    d.lifecycle['controller_server'] = 'inactive'
    assert 'NODE_INACTIVE' in codes(d, 1.0)
