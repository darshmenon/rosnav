import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import decentral_nav_node as n  # noqa: E402

PLAN = np.stack([np.linspace(0, 10, 41), np.zeros(41)], axis=1)   # straight along +x


def test_follows_plan_direction():
    for ctrl in ('vo', 'mppi_prior', 'orca'):
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


# ── near-goal dither fixes (2026-11-20) ──────────────────────────────────────
def test_handoff_follows_plan_straight_in_near_goal():
    for ctrl in ('vo', 'mppi_prior', 'orca'):
        vx, vy, info = n.compute_velocity(ctrl, 0, [[9.2, 0.0]], [[0.0, 0.0]], PLAN, vmax=0.5)
        assert info['reason'] == 'handoff' and vx > 0.2 and abs(vy) < 1e-6, (ctrl, vx, vy)
        assert info['why'].startswith('handoff')


def test_handoff_defers_to_avoidance_when_neighbour_on_last_stretch():
    pos, vel = [[9.2, 0.0], [9.6, 0.0]], [[0.3, 0.0], [-0.3, 0.0]]   # moving neighbour dead ahead of the goal line
    vx, vy, info = n.compute_velocity('vo', 0, pos, vel, PLAN, vmax=0.5)
    assert info['reason'] == 'ok' and info['why'].startswith('avoid') and abs(vy) > 0.05


def test_handoff_ignores_neighbour_parked_off_the_path():
    pos, vel = [[9.2, 0.0], [9.2, 1.5]], [[0.0, 0.0], [0.0, 0.0]]
    _, _, info = n.compute_velocity('vo', 0, pos, vel, PLAN, vmax=0.5)
    assert info['reason'] == 'handoff'


def test_fine_cells_do_not_wall_in_the_goal_or_plan():
    # points beside the plan whose 1 m cell would reach onto it, and points around the goal: no coarse cells
    pts = np.array([[9.0, 0.6], [9.5, -0.6], [5.0, 0.45], [5.0, 3.0]])
    w = n.PlanWorld(PLAN, pts, [[9.0, 0.0]])
    assert w.n_blocked == 1                      # goal-zone points and a point whose cell centre is on the plan are exact-point-only; only [5,3] is a cell
    assert w.pts_tree is not None                # ...but collision checks still see all of them


def test_parked_neighbour_does_not_trigger_mppi_prior_yield():
    """Both slow, parked neighbour (lower id) 1.5 m off the route: the id-priority sidestep must not fire; the robot
    keeps driving along its plan."""
    me = np.array([[3.0, 1.5], [3.0, 0.0]])          # row 0 parked, row 1 = me (i > j: would be the yielder)
    vel = np.zeros((2, 2))
    vx, vy, _ = n.compute_velocity('mppi_prior', 1, me, vel, PLAN, vmax=0.5, arrived=[0])
    assert vx > 0.15 and abs(vy) < 0.35


def test_heading_hysteresis_holds_small_error_and_rejects_transient_flip():
    h = n.HeadingHysteresis(dead=0.12, flip=1.2, persist=3)
    assert h.update(0.4, 0.0)[:2] == (0.4, 0.0)
    vx, vy, why = h.update(0.4 * math.cos(0.05), 0.4 * math.sin(0.05))   # jitter < dead band: unchanged heading
    assert abs(vy) < 1e-9 and why is None
    vx, vy, why = h.update(-0.4, 0.05)                                   # 180 deg flip, 1st tick: held at half speed
    assert vx > 0 and math.hypot(vx, vy) < 0.25 and 'hysteresis' in why
    h.update(0.4, 0.0)                                                   # flip vanished -> counter resets
    for _ in range(2):
        h.update(-0.4, 0.05)
    vx, vy, why = h.update(-0.4, 0.05)                                   # persisted 3 ticks -> accepted
    assert vx < 0 and 'accepted' in why


def test_dithering_goal_approach_converges():
    """Offline replay of the live stall: wall points 0.7 m either side, noisy VO output near the goal. With handoff +
    hysteresis + unicycle the robot reaches the goal tolerance (it used to hover ~0.8-0.35 m out)."""
    plan = np.stack([np.linspace(0, 6, 25), np.zeros(25)], 1)
    scan = _wall_points(-1.0, 7.0, 0.7)
    pos, yaw, v, w = np.array([3.5, 0.0]), 0.0, 0.0, 0.0
    h = n.HeadingHysteresis()
    rng = np.random.default_rng(1)
    for _ in range(400):
        vx, vy, info = n.compute_velocity('mppi_prior', 0, pos[None], np.zeros((1, 2)), plan, scan, rng, 0.5)
        vx, vy, _ = h.update(vx, vy)
        v, w = n.track_unicycle((vx, vy), yaw, v, w)
        yaw += w * 0.1
        pos = pos + 0.1 * v * np.array([math.cos(yaw), math.sin(yaw)])
        if np.linalg.norm(pos - plan[-1]) < 0.15:
            break
    assert np.linalg.norm(pos - plan[-1]) < 0.15, pos


# ── orca controller (2026-11-20) ─────────────────────────────────────────────
def test_orca_is_a_valid_ctrl():
    assert 'orca' in n.CTRLS and set(n.CTRLS) <= set(n.sim.CTRLS)


def test_orca_head_on_deflects_and_bounded():
    pos, vel = [[2.0, 0.0], [4.0, 0.0]], [[0.4, 0.0], [-0.4, 0.0]]
    vx, vy, info = n.compute_velocity('orca', 0, pos, vel, PLAN, vmax=0.5)
    assert info['reason'] == 'ok' and math.hypot(vx, vy) <= 0.5 + 1e-6
    assert abs(vy) > 0.02 or vx < 0.4            # ORCA yields its share of the avoidance


def test_orca_handoff_and_creep_same_as_vo():
    for me, reason in (([[9.2, 0.0]], 'handoff'), ([[9.85, 0.0]], 'creep')):
        _, _, io = n.compute_velocity('orca', 0, me, [[0.0, 0.0]], PLAN, vmax=0.5)
        _, _, iv = n.compute_velocity('vo', 0, me, [[0.0, 0.0]], PLAN, vmax=0.5)
        assert io['reason'] == iv['reason'] == reason
    pos, vel = [[9.2, 0.0], [9.6, 0.0]], [[0.3, 0.0], [-0.3, 0.0]]
    _, _, info = n.compute_velocity('orca', 0, pos, vel, PLAN, vmax=0.5)
    assert info['why'].startswith('avoid')


def test_orca_parked_neighbour_is_a_static_obstacle_not_a_yield_partner():
    me = np.array([[3.0, 1.5], [3.0, 0.0]])
    vx, vy, info = n.compute_velocity('orca', 1, me, np.zeros((2, 2)), PLAN, vmax=0.5, arrived=[0])
    assert vx > 0.15 and 'parked' in info['why']


def test_orca_rejects_unknown_ctrl_in_node_list():
    assert 'nope' not in n.CTRLS


def test_orca_roundabout_bias_only_with_a_moving_neighbour_nearby():
    solo = n.compute_velocity('orca', 0, [[1.0, 0.0]], [[0.0, 0.0]], PLAN, vmax=0.5)
    assert abs(solo[1]) < 0.05                                   # alone: straight along the plan
    far = n.compute_velocity('orca', 0, [[1.0, 0.0], [1.0, 6.0]], [[0.0, 0.0], [0.0, 0.0]], PLAN, vmax=0.5)
    assert abs(far[1]) < 0.05                                    # neighbour 6 m away: no bias
    near = n.compute_velocity('orca', 0, [[1.0, 0.0], [1.0, 2.5]], [[0.0, 0.0], [0.0, 0.0]], PLAN, vmax=0.5)
    assert near[1] > 0.03                                        # neighbour within 3 m: CCW bias
    parked = n.compute_velocity('orca', 0, [[1.0, 0.0], [1.0, 2.5]], [[0.0, 0.0], [0.0, 0.0]], PLAN, vmax=0.5, arrived=[1])
    assert abs(parked[1]) < 0.05                                 # arrived neighbours do not trigger it


def test_orca_stall_breaker_higher_id_sidesteps_when_mutually_stopped():
    pos, vel = [[3.0, 0.0], [3.8, 0.0]], np.zeros((2, 2))
    lo = n.compute_velocity('orca', 0, pos, vel, PLAN, vmax=0.5)
    hi = n.compute_velocity('orca', 1, pos, vel, PLAN, vmax=0.5)
    assert abs(hi[1]) > 0.1 or abs(hi[0]) < 0.1 and abs(hi[1]) > 0.05, hi    # id 1 moves sideways
    assert math.hypot(lo[0], lo[1]) <= 0.5 + 1e-6


# ── orca in a 1.5 m corridor (live corridor_4 deadlock, 2026-11-20) ──────────
def _corridor_walls(width=1.5, length=10.0):
    xs = np.arange(-0.5, length + 0.5, 0.05)
    return np.vstack([np.stack([xs, np.full_like(xs, s * width / 2)], 1) for s in (1, -1)])


def test_orca_nose_to_nose_in_corridor_has_a_sidestep():
    """Walls at +-0.75 round to +-0.5 in the 1 m cost-field cells; ORCA must see the exact returns, else the +-0.15 m
    lane left no sidestep and both robots parked."""
    walls = _corridor_walls()
    pos = np.array([[5.5, -0.15], [4.85, 0.15]])
    plan = np.stack([np.linspace(5.5, -4.0, 40), np.linspace(-0.15, 0.3, 40)], 1)
    vx, vy, info = n.compute_velocity('orca', 0, pos, np.zeros((2, 2)), plan, walls, vmax=0.5, ids=[1, 0])
    assert math.hypot(vx, vy) > 0.1, (vx, vy, info)
    assert info['why'].startswith('stall-break'), info['why']


def test_orca_stall_break_uses_global_ids_not_row_index():
    """The node is always row 0, so `i > j` on rows never fired: the higher GLOBAL id must be the one that yields."""
    pos, vel = np.array([[3.0, 0.0], [3.8, 0.0]]), np.zeros((2, 2))
    hi = n.compute_velocity('orca', 0, pos, vel, PLAN, vmax=0.5, ids=[5, 2])
    lo = n.compute_velocity('orca', 0, pos, vel, PLAN, vmax=0.5, ids=[2, 5])
    assert hi[2]['why'].startswith('stall-break') and abs(hi[1]) > 0.05
    assert not lo[2]['why'].startswith('stall-break')


def test_orca_ignores_neighbour_bodies_in_the_scan_as_walls():
    w = n.PlanWorld(PLAN, np.array([[5.0, 1.0], [5.2, 1.0]]), [[1.0, 0.0], [5.1, 1.0]])
    assert len(n._ExactWalls(w, np.array([[1.0, 0.0], [5.1, 1.0]]) - w.origin, 0, near=9.0).slabs) == 0


def _run_corridor_swap(ctrl, T=90.0, dt=0.1):
    """4 node-logic instances (2 per side) head-on through a 1.5 m corridor: scan = wall returns + neighbour bodies,
    straight plans, odom-derived velocities, heading hysteresis + unicycle tracker, exactly the node's tick path."""
    starts = [(-3.0, -0.25, 0.0), (-4.5, 0.25, 0.0), (13.0, 0.25, math.pi), (14.5, -0.25, math.pi)]
    goals = np.array([(14.0, -0.3), (15.5, 0.3), (-4.0, 0.3), (-5.5, -0.3)])
    pos = np.array([[s[0], s[1]] for s in starts])
    yaw = np.array([s[2] for s in starts])
    v, w = np.zeros(4), np.zeros(4)
    hy = [n.HeadingHysteresis() for _ in range(4)]
    walls = _corridor_walls()
    for k in range(int(T / dt)):
        vel = np.stack([v * np.cos(yaw), v * np.sin(yaw)], 1)
        parked = [np.linalg.norm(goals[j] - pos[j]) < 0.5 and abs(v[j]) < 0.05 for j in range(4)]
        nv, nw = v.copy(), w.copy()
        for i in range(4):
            order = [i] + [j for j in range(4) if j != i]
            plan = np.stack([np.linspace(pos[i][0], goals[i][0], 60), np.linspace(pos[i][1], goals[i][1], 60)], 1)
            sc = [walls[np.linalg.norm(walls - pos[i], axis=1) < 4.0]]
            a = np.linspace(0, 2 * math.pi, 12, endpoint=False)
            for j in order[1:]:
                if np.linalg.norm(pos[j] - pos[i]) < 4:
                    sc.append(pos[j] + 0.3 * np.stack([np.cos(a), np.sin(a)], 1))
            arr = [kk for kk, j in enumerate(order) if kk and parked[j]]
            vx, vy, _ = n.compute_velocity(ctrl, 0, pos[order], vel[order], plan, np.vstack(sc), None, 0.5, 1.0, arr,
                                           ids=order)
            vx, vy, _ = hy[i].update(vx, vy)
            if np.linalg.norm(goals[i] - pos[i]) < 0.15:
                nv[i] = nw[i] = 0.0
            else:
                nv[i], nw[i] = n.track_unicycle((vx, vy), yaw[i], v[i], w[i], 0.5, 1.0, dt=dt)
        v, w = nv, nw
        yaw = yaw + w * dt
        pos = pos + dt * np.stack([v * np.cos(yaw), v * np.sin(yaw)], 1)
        if min(np.linalg.norm(pos[a_] - pos[b_]) for a_ in range(4) for b_ in range(a_)) < 0.6:
            return False, k * dt
        if all(np.linalg.norm(goals[j] - pos[j]) < 0.3 for j in range(4)):
            return True, k * dt
    return False, T


def test_orca_four_robots_head_on_in_1p5m_corridor_all_arrive():
    ok, t = _run_corridor_swap('orca')
    assert ok, f'corridor deadlock/collision at t={t}'


def test_people_tracker_finite_difference_and_radius():
    import json
    pt = n.PeopleTracker(radius=3.0, max_age=1.0)
    pt.update(json.dumps({'peds': [{'id': 1, 'x': 0.0, 'y': 0.0, 'vx': 0.0, 'vy': 0.0},
                                   {'id': 2, 'x': 9.0, 'y': 0.0}]}), 0.0)
    pt.update(json.dumps({'peds': [{'id': 1, 'x': 0.4, 'y': 0.0, 'vx': 0.0, 'vy': 0.0},
                                   {'id': 2, 'x': 9.0, 'y': 0.0}]}), 1.0)
    near = pt.near((0.0, 0.0), 1.0)
    assert [n[0] for n in near] == [1]                    # person 2 is outside the radius
    assert abs(near[0][2][0] - 0.2) < 1e-9                # 0.4 m/s raw, alpha 0.5 smoothing from 0
    assert pt.near((0.0, 0.0), 5.0) == []                 # stale feed drops out


def test_people_tracker_uses_feed_velocity_and_survives_bad_json():
    import json
    pt = n.PeopleTracker()
    assert pt.update('not json', 0.0) == 0
    pt.update(json.dumps({'peds': [{'id': 7, 'x': 1.0, 'y': 1.0, 'vx': 0.3, 'vy': -0.1}]}), 0.0)
    assert pt.near((1.0, 1.0), 0.0)[0][2] == (0.3, -0.1)
