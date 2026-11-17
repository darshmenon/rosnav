import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import social_nav_core as s  # noqa: E402


def test_predict_cv_shape_and_end():
    p = s.predict_cv(np.array([0., 0.]), np.array([1., 0.]), horizon_s=2.0)
    assert p.shape == (20, 2)
    assert np.allclose(p[-1], [2.0, 0.0])


def test_safety_filter_caps_and_brakes():
    v = s.safety_filter(np.array([5., 0.]), np.array([0., 0.]), [])
    assert np.linalg.norm(v) <= s.V_MAX + 1e-9
    near = [np.array([0.7, 0.0])]  # inside brake zone
    assert np.linalg.norm(s.safety_filter(np.array([.8, 0.]), np.zeros(2), near)) < 0.4


def test_predictive_never_worse_on_collisions():
    r = s.evaluate([s.ReactivePlanner(), s.PredictivePlanner()], seeds=5)
    col = {(x['planner'], x['scenario']): x['collision_rate'] for x in r}
    for k in ('crossing', 'head_on', 'crowd'):
        assert col[('predictive', k)] <= col[('reactive', k)]


def test_risk_planner_respects_walls():
    # everything at x >= 0 is "inside a wall" (clearance 0.2 m); goal is straight through it
    clear = lambda P: np.where(P[..., 0] < 0.0, 1.5, 0.2)
    pl = s.RiskPlanner(clear_fn=clear)
    v = pl.plan(np.array([-0.2, 0.0]), np.array([5.0, 0.0]), [], [])
    assert v[0] <= 0.35                      # never commits to a rollout that ends in the wall
    v2 = pl.plan(np.array([-0.2, 0.0]), np.array([5.0, 0.0]), [np.array([3.0, 3.0])], [np.zeros(2)])
    assert v2[0] <= 0.35
