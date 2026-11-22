import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import decentral_nav_sim as d  # noqa: E402

KEYS = {'success', 'makespan', 'delay', 'collisions', 'coll_ticks', 'n'}
HAVE_MAP = os.path.exists(os.path.join(d.MAP_DIR, 'den312d.map'))
needs_map = pytest.mark.skipif(not HAVE_MAP, reason='MovingAI map not available')


def test_world_ok_and_dist_field():
    w = d.open_world(12)
    assert w.ok((6.0, 6.0))
    assert not w.ok((0.0, 0.0))        # border wall
    assert not w.ok((-1.0, 5.0))       # outside
    f = w.dist_field((6, 6))
    assert f[6, 6] == 0
    assert f[6, 9] == 3
    assert np.isinf(f[0, 0])           # wall cell unreachable


def _check_scenario(kind, n):
    world, starts, goals, _ = d.make_scenario(kind, n, 0)
    assert len(starts) == len(goals) == n
    assert len({tuple(np.round(s, 3)) for s in starts}) == n
    for s, g in zip(starts, goals):
        assert world.ok(s) and world.ok(g)
        assert np.isfinite(world.dist_field(g)[int(round(s[1])), int(round(s[0]))])
    return starts, goals


def test_swap_scenario():
    starts, goals = _check_scenario('swap', 4)
    assert len({tuple(np.round(g, 3)) for g in goals}) == 4


@needs_map
def test_maze_scenario():
    starts, goals = _check_scenario('maze', 4)
    assert len({tuple(np.round(g, 3)) for g in goals}) == 4


@needs_map
def test_convoy_scenario():
    _, _, _, leader = d.make_scenario('convoy', 4, 0)
    assert isinstance(leader, dict) and leader['spacing'] > 0 and leader['trail'] is not None
    _check_scenario('convoy', 4)


@pytest.mark.parametrize('ctrl', d.CTRLS)
def test_simulate_returns_metrics(ctrl):
    r = d.simulate('swap', ctrl, n=3, seed=0, max_t=6.0)
    assert KEYS <= set(r)
    assert r['n'] == 3
    assert r['collisions'] >= 0 and r['coll_ticks'] >= 0
    assert np.isfinite(r['makespan'])


def test_pref_collides_mppi_prior_does_not_in_swap():
    pref = [d.simulate('swap', 'pref', n=6, seed=k, max_t=40.0)['collisions'] for k in (0, 1)]
    prior = [d.simulate('swap', 'mppi_prior', n=6, seed=k, max_t=40.0)['collisions'] for k in (0, 1)]
    assert sum(pref) > 0
    assert sum(prior) == 0


def test_corridor_and_crowd_scenarios():
    world, starts, goals, _ = d.make_scenario('corridor', 6, 0)
    assert len(starts) == len(goals) == 6
    assert world.ok((11.0, 3.5)) and not world.ok((11.0, 2.4))     # 1.5 m wide inside
    _check_scenario('corridor', 6)
    _, _, _, extra = d.make_scenario('crowd', 5, 0)
    assert len(extra['peds']['pos']) == 5
    _check_scenario('crowd', 5)


def test_realism_options_and_ped_metrics():
    base = d.simulate('swap', 'vo', n=3, seed=0, max_t=5.0)
    same = d.simulate('swap', 'vo', n=3, seed=0, max_t=5.0, obs_noise=0.0, delay_steps=0, act_noise=0.0)
    assert base['makespan'] == same['makespan'] and base['coll_ticks'] == same['coll_ticks']
    noisy = d.simulate('swap', 'vo', n=3, seed=0, max_t=5.0, record=True, obs_noise=0.2, delay_steps=3, act_noise=0.1)
    clean = d.simulate('swap', 'vo', n=3, seed=0, max_t=5.0, record=True)
    assert not np.allclose(noisy['trail'], clean['trail'])
    r = d.simulate('crowd', 'vo', n=3, seed=0, max_t=8.0)
    assert r['ped_contacts'] >= 0 and r['ped_intrusion_s'] >= 0 and np.isfinite(r['min_ped_dist'])


# ── ORCA ─────────────────────────────────────────────────────────────────────
def _run_orca(pos, goals, steps=400, world=None):
    """Drive robots with orca_velocity only; returns (trail, min pair distance)."""
    world = world or d.open_world(30)
    pos = np.array(pos, float)
    goals = np.array(goals, float)
    vel = np.zeros_like(pos)
    mind = np.inf
    for _ in range(steps):
        cmds = np.array([d.command('orca', i, pos, vel, goals, world, None)
                         for i in range(len(pos))])
        new = pos + cmds * d.DT
        vel, pos = (new - pos) / d.DT, new
        dm = np.linalg.norm(pos[:, None] - pos[None], axis=2) + np.eye(len(pos)) * 99
        mind = min(mind, float(dm.min()))
        if np.all(np.linalg.norm(goals - pos, axis=1) < 0.3):
            break
    return pos, mind


def test_orca_single_robot_reaches_goal():
    pos, _ = _run_orca([[5, 15]], [[22, 15]])
    assert np.linalg.norm(pos[0] - [22, 15]) < 0.3


def test_orca_head_on_no_collision():
    pos, mind = _run_orca([[5, 15], [20, 15]], [[20, 15], [5, 15]], steps=600)
    assert mind > 2 * d.R - 0.02
    assert np.linalg.norm(pos - [[20, 15], [5, 15]], axis=1).max() < 0.35


def _circle_swap(n, jitter, steps=900):
    c = np.array([15.0, 15.0])
    ang = np.arange(n) * 2 * math.pi / n
    st = c + 6 * np.stack([np.cos(ang), np.sin(ang)], 1)
    st_j = st + np.random.default_rng(1).normal(0, jitter, st.shape)
    pos, mind = _run_orca(st_j, 2 * c - st, steps=steps)
    return np.linalg.norm(pos - (2 * c - st), axis=1).max(), mind


def test_orca_symmetric_swap_six():
    err, mind = _circle_swap(6, 0.0)              # exactly symmetric start
    assert mind > 2 * d.R - 0.02 and err < 0.35


def test_orca_swap_four_jittered():
    # exactly symmetric 4/8-robot swaps can jam in the centre (known ORCA symmetry
    # deadlock, no tie-break); 5 cm of start jitter is enough to resolve it
    err, mind = _circle_swap(4, 0.05)
    assert mind > 2 * d.R - 0.02 and err < 0.35


def test_orca_lp_fallback_when_infeasible():
    # robot boxed in by three static neighbours whose half-planes cannot all hold:
    # the LP must still return a finite velocity within the speed limit
    world = d.open_world(30)
    pos = np.array([[15.0, 15.0], [15.45, 15.0], [14.55, 15.0], [15.0, 15.45]])
    vel = np.zeros_like(pos)
    pref = np.array([1.0, 0.0])
    v = d.orca_velocity(0, pos, vel, np.array([25.0, 15.0]), world, pref)
    assert np.all(np.isfinite(v)) and np.linalg.norm(v) <= d.VMAX + 1e-9
    lines = [d._orca_line(pos[j] - pos[0], -vel[j], 2 * d.R + 0.05, 3.0) for j in (1, 2, 3)]
    lines = [(u, dr) for u, dr in lines]
    _, fail = d._lp2(lines, d.VMAX, pref, False)
    assert fail < len(lines)                      # genuinely infeasible, fallback used


def test_orca_lp_feasible_matches_pref():
    v, fail = d._lp2([], d.VMAX, np.array([0.4, 0.3]), False)
    assert fail == 0 and np.allclose(v, [0.4, 0.3])
    v, _ = d._lp2([], d.VMAX, np.array([3.0, 4.0]), False)
    assert np.allclose(np.linalg.norm(v), d.VMAX)


def test_orca_registered():
    assert 'orca' in d.CTRLS and 'orca' in d.CTRL_NAMES
