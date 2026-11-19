import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import decentral_nav_node as n  # noqa: E402

PLAN = np.stack([np.linspace(0, 10, 41), np.zeros(41)], axis=1)   # straight along +x


def test_follows_plan_direction():
    for ctrl in ('vo', 'mppi_prior'):
        vx, vy, info = n.compute_velocity(ctrl, 0, [[1.0, 0.0]], [[0.0, 0.0]], PLAN, vmax=0.5)
        assert vx > 0.2 and abs(vy) < 0.25, (ctrl, vx, vy)
        assert info['reason'] == 'ok'


def test_at_goal_zero():
    vx, vy, info = n.compute_velocity('vo', 0, [[9.9, 0.0]], [[0.0, 0.0]], PLAN)
    assert (vx, vy) == (0.0, 0.0) and info['reason'] == 'at_goal'


def test_head_on_neighbour_deflects_and_speed_bounded():
    pos, vel = [[2.0, 0.0], [4.0, 0.0]], [[0.4, 0.0], [-0.4, 0.0]]
    vx, vy, _ = n.compute_velocity('vo', 0, pos, vel, PLAN, vmax=0.5)
    assert abs(vy) > 0.05                      # sidesteps instead of driving straight into it
    assert math.hypot(vx, vy) <= 0.5 + 1e-6


def test_scan_obstacle_off_plan_blocks_cells_but_plan_stays_clear():
    wall = np.array([[5.0, 2.0], [5.0, 3.0], [5.0, 0.1]])   # last point sits on the plan -> dropped
    w = n.PlanWorld(PLAN, wall, [[1.0, 0.0]])
    assert w.n_blocked == 2
    vx, _, info = n.compute_velocity('vo', 0, [[1.0, 0.0]], [[0.0, 0.0]], PLAN, wall, vmax=0.5)
    assert vx > 0.2 and info['blocked_cells'] == 2


def test_plan_world_pref_dir_and_dist_field():
    w = n.PlanWorld(PLAN, None, [[1.0, 1.0]])
    p = np.array([1.0, 1.0]) - w.origin
    d = w.pref_dir(p, w.plan[-1])
    assert d[0] > 0.5                            # heads back toward the plan then along it
    df = w.dist_field(w.plan[-1])
    ix, iy = int(round(p[0])), int(round(p[1]))
    assert df[iy, ix] < df[iy, max(ix - 2, 0)]   # closer to goal = smaller


def test_unicycle_tracker():
    v, w = n.track_unicycle((0.5, 0.0), 0.0, v_max=0.5)
    assert v > 0 and abs(w) < 1e-9
    v, w = n.track_unicycle((0.0, 0.5), 0.0, v_max=0.5, w_prev=0.0)
    assert w > 0 and v < 0.05                    # 90 deg off: rotate in place
    v, w = n.track_unicycle((-0.5, 0.0), 0.0)
    assert v == 0.0
    v, _ = n.track_unicycle((0.5, 0.0), 0.0, v_prev=0.0, a_max=1.0, dt=0.1)
    assert v <= 0.1 + 1e-9                       # slew limited
    assert n.track_unicycle((0.0, 0.0), 1.0) == (0.0, 0.0)


def _wall_points(x0, x1, half, step=0.05):
    xs = np.arange(x0, x1 + step, step)
    return np.vstack([np.stack([xs, np.full_like(xs, half)], 1), np.stack([xs, np.full_like(xs, -half)], 1)])


def test_two_robots_pass_each_other_in_a_1p5m_corridor():
    """Head-on swap in a 1.5 m corridor (walls at +-0.75 m, robot radius 0.3 m): exact-point collision checks leave
    room to pass; the old 1 m wall squares did not."""
    scan = _wall_points(-6.0, 6.0, 0.75)
    pos = np.array([[-5.0, 0.0], [5.0, 0.0]])
    goals = [np.array([5.0, 0.0]), np.array([-5.0, 0.0])]
    vel = np.zeros((2, 2))
    min_d = 9.0
    for _ in range(900):
        new = []
        for i in range(2):
            plan = np.stack([np.linspace(pos[i, 0], goals[i][0], 40), np.linspace(pos[i, 1], goals[i][1], 40)], 1)
            vx, vy, _ = n.compute_velocity('vo', i, pos, vel, plan, scan, vmax=0.5)
            new.append(np.array([vx, vy]))
        vel = np.array(new)
        pos = pos + vel * 0.1
        min_d = min(min_d, float(np.linalg.norm(pos[0] - pos[1])))
        assert (np.abs(pos[:, 1]) < 0.75 - 0.25).all(), 'a robot hit the wall'
        if all(np.linalg.norm(pos[i] - goals[i]) < 0.4 for i in range(2)):
            break
    assert all(np.linalg.norm(pos[i] - goals[i]) < 0.4 for i in range(2))
    assert min_d > 0.55


def test_goal_rotation_only_near_goal_and_off_heading():
    assert n.goal_rotation(1.0, 0.0, 1.5) is None             # far from the goal: keep driving
    assert n.goal_rotation(0.05, 1.5, 1.5) is None            # at the goal, heading already right
    w = n.goal_rotation(0.05, 0.0, 1.5)
    assert 0 < w <= 0.6                                       # turn toward +yaw, capped
    assert n.goal_rotation(0.05, 0.0, -1.5) < 0               # and the short way round
    assert n.goal_rotation(0.05, 3.0, -3.0) > 0               # wraps across +-pi


def test_creeps_through_the_last_25cm_then_stops():
    plan = np.array([[x, 0.0] for x in np.linspace(0, 5, 21)])
    vx, vy, info = n.compute_velocity('vo', 0, np.array([[4.8, 0.0]]), np.zeros((1, 2)), plan)
    assert info['reason'] == 'creep' and 0.0 < vx <= 0.2 and abs(vy) < 1e-9
    vx, vy, info = n.compute_velocity('vo', 0, np.array([[4.95, 0.0]]), np.zeros((1, 2)), plan)
    assert info['reason'] == 'at_goal' and (vx, vy) == (0.0, 0.0)
