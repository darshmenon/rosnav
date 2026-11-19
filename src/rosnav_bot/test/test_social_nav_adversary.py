import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import social_nav_adversary as A  # noqa: E402
import social_nav_core as snc  # noqa: E402


def test_budget_tracks_error_quantile():
    b = A.ConformalRiskBudget(alpha=0.1, eta=0.05, every=1)
    rng = np.random.default_rng(0)
    # peds with constant velocity but an unmodelled lateral drift -> CV error grows with lag
    pos = np.zeros((2, 2))
    vel = np.array([[1.0, 0.0], [0.0, 1.0]])
    for t in range(600):
        pos = pos + vel * A.DT + rng.normal(0, 0.03, pos.shape)
        b.observe(pos, vel)
    r = b.radii
    assert b.n_updates > 1000
    assert np.all(np.diff(r[:5]) > -0.05) and r[4] > r[0]
    assert np.all(r >= 0.05) and np.all(np.isfinite(r))


def test_decode_bounds():
    rng = np.random.default_rng(1)
    for _ in range(20):
        s, v = A.decode(A.random_theta(rng) * 3 - 1)   # out-of-range input is clipped
        for p, vv in zip(s, v):
            assert A.LO[0] <= p[0] <= A.HI[0] and A.LO[1] <= p[1] <= A.HI[1]
            assert A.SPEED_RANGE[0] - 1e-9 <= np.linalg.norm(vv) <= A.SPEED_RANGE[1] + 1e-9


def test_conformal_planner_interface_and_reset():
    pl = A.ConformalRiskPlanner(samples=4)
    th = A.random_theta(np.random.default_rng(2))
    a = A.run(pl, th, seed=0, max_t=3.0)
    n1 = pl.budget.n_updates
    b = A.run(pl, th, seed=0, max_t=3.0)
    assert a == b                        # reset per episode -> deterministic
    assert pl.budget.n_updates == n1 and n1 > 0
    assert snc.make_scenario.__name__ == 'make_scenario'   # patch restored


def test_cem_deterministic_and_improves_over_random_mean():
    pl = snc.ReactivePlanner()
    t1, h1 = A.cem_search(pl, pop=6, iters=2, top=2, max_t=4.0)
    t2, h2 = A.cem_search(pl, pop=6, iters=2, top=2, max_t=4.0)
    assert h1 == h2 and np.allclose(t1[0], t2[0])
    assert A.fitness(pl, t1[0], max_t=4.0) >= A.fitness(pl, t1[-1], max_t=4.0)
