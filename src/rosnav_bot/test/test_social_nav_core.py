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


def test_futures_replace_the_gaussian_samples():
    """A person standing on the goal line must push the robot off +x when those futures are
    passed in. The same person placed 5 m to the side must not."""
    T = int(3.0 / s.DT)
    on_path = np.zeros((4, 1, T, 2))
    on_path[:, 0, :, 0] = 2.0
    aside = on_path.copy()
    aside[:, 0, :, 1] = 5.0
    ped, vel = [np.array([2.0, 0.0])], [np.zeros(2)]
    blocked = s.RiskPlanner(w_risk=40, w_goal=0.2, w_commit=0, samples=4).plan(
        np.zeros(2), np.array([6.0, 0.0]), ped, vel, futures=on_path)
    clear = s.RiskPlanner(w_risk=40, w_goal=0.2, w_commit=0, samples=4).plan(
        np.zeros(2), np.array([6.0, 0.0]), ped, vel, futures=aside)
    assert abs(blocked[1]) > abs(clear[1]) or blocked[0] < clear[0]


def _on_path_futures(when):
    """One person on the +x goal line at a single horizon step. `when` is 'early' or 'late'."""
    T = int(3.0 / s.DT)
    fut = np.zeros((4, 1, T, 2))
    fut[:, :, :, 1] = 8.0
    step = 0 if when == 'early' else T - 1
    fut[:, 0, step, 0] = 0.4 if when == 'early' else 2.4
    fut[:, 0, step, 1] = 0.0
    return fut


def test_horizon_discount_counts_a_late_hit_less_than_an_early_one():
    ped, vel = [np.array([2.4, 0.0])], [np.zeros(2)]
    goal = np.array([6.0, 0.0])
    late = s.RiskPlanner(horizon_discount=0.0, samples=4, w_commit=0, seed=0)
    early = s.RiskPlanner(horizon_discount=0.0, samples=4, w_commit=0, seed=0)
    late.plan(np.zeros(2), goal, ped, vel, futures=_on_path_futures('late'))
    early.plan(np.zeros(2), goal, ped, vel, futures=_on_path_futures('early'))
    straight = int(np.argmax(late.cands[:, 0]))
    assert early.last['risk'][straight] > late.last['risk'][straight]


def test_plan_extra_stays_inside_the_fixed_disc_and_grows_when_closing():
    cands = np.array([[0.8, 0.0], [-0.8, 0.0]])
    extra = s._plan_extra(cands, np.zeros(2), [np.array([3.0, 0.0])], [np.zeros(2)])
    assert extra.max() <= 0.15 + 1e-9
    assert extra[0] > extra[1]


def test_adaptive_risk_weight_matches_zero_risk_when_every_move_is_blocked():
    ped, vel = [np.array([1.2, 0.0])], [np.zeros(2)]
    goal = np.array([6.0, 0.0])
    cut = s.RiskPlanner(w_risk=50, w_commit=0, samples=4, seed=1, adapt_when=0.0, adapt_scale=0.0)
    free = s.RiskPlanner(w_risk=0, w_commit=0, samples=4, seed=1)
    assert np.allclose(cut.plan(np.zeros(2), goal, ped, vel), free.plan(np.zeros(2), goal, ped, vel))


def test_sigma_scale_scales_the_default_linear_sigma():
    ped, vel = [np.array([2.0, 0.0])], [np.zeros(2)]
    spread = {}
    for scale in (0.0, 1.0):
        pl = s.RiskPlanner(samples=64, sigma_scale=scale, seed=0)
        seen = {}
        orig = s._collision_risk
        s._collision_risk = lambda tr, S, *a, _o=orig, _d=seen: (_d.setdefault('S', S), _o(tr, S, *a))[1]
        try:
            pl.plan(np.zeros(2), np.array([6.0, 0.0]), ped, vel)
        finally:
            s._collision_risk = orig
        spread[scale] = float(seen['S'].std(axis=0).max())
    assert spread[0.0] == 0.0 and spread[1.0] > 0.05
