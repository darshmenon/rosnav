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
