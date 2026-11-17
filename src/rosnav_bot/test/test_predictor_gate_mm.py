import os
import sys

import numpy as np
import pytest

torch = pytest.importorskip('torch')
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
import predictor_gate as pg  # noqa: E402
import predictor_multimodal as mm  # noqa: E402
import social_nav_core as s  # noqa: E402
import train_ped_predictor as tp  # noqa: E402


def test_blend_extremes():
    cv, lr = torch.zeros(2, 3, 12, 2), torch.ones(2, 3, 12, 2)
    assert torch.equal(pg.blend(cv, lr, torch.zeros(2, 3)), cv)
    assert torch.equal(pg.blend(cv, lr, torch.ones(2, 3)), lr)
    assert torch.allclose(pg.blend(cv, lr, torch.full((2, 3), 0.25)), torch.full_like(cv, 0.25))


def test_online_gate_is_causal():
    rng = np.random.default_rng(0)
    el, ec = rng.random(80), rng.random(80)
    g = pg.online_g(el, ec, kappa=20.0, decay=0.97)
    el2, ec2 = el.copy(), ec.copy()
    el2[50:] += 5.0                                   # change the future: g[:62] must not move (lag = 12)
    ec2[50:] -= 5.0
    g2 = pg.online_g(el2, ec2, kappa=20.0, decay=0.97)
    assert np.allclose(g[:62], g2[:62])
    assert not np.allclose(g[62:], g2[62:])


def test_online_gate_follows_the_better_predictor():
    n = 200
    worse = pg.online_g(np.full(n, 0.5), np.full(n, 0.2), kappa=20.0, decay=0.97)     # learned worse -> CV
    better = pg.online_g(np.full(n, 0.2), np.full(n, 0.5), kappa=20.0, decay=0.97)
    assert worse[-1] < 0.1 and better[-1] > 0.9
    assert worse[0] == 1.0 and worse[5] == 1.0        # prior until the first outcomes are known


def test_mc_gate_distrusts_uncertain_agents():
    gate = pg.Gate('mc', tau=1.0)
    f = torch.zeros(3, 4)
    f[:, 0] = torch.tensor([0.0, 0.5, 2.0])
    g = gate.g(f)
    assert g[0] == 1.0 and g[1] == 0.5 and g[2] == 0.0


def test_multimodal_shapes_and_from_single(tmp_path):
    single = tp.SocialTransformer(dropout=0.2)
    p = tmp_path / 'single.pt'
    torch.save(single.state_dict(), p)
    net = mm.from_single(str(p), K=4)
    x = torch.randn(2, tp.NMAX, tp.OBS, 2)
    m = torch.zeros(2, tp.NMAX, dtype=torch.bool)
    m[:, :5] = True
    net.eval()
    out = net(x, m)
    assert out.shape == (2, tp.NMAX, 4, tp.PRED, 2)
    assert torch.equal(net.emb[0].weight, single.emb[0].weight)               # encoder copied
    assert (out[:, :5, 0] - out[:, :5, 1]).abs().max() > 1e-3                 # heads start distinct


def test_min_at_k_and_spray_baseline():
    x = torch.randn(3, 6, tp.OBS, 2).cumsum(2) * 0.3
    y = torch.randn(3, 6, tp.PRED, 2).cumsum(2) * 0.3 + x[:, :, -1:]
    m = torch.ones(3, 6, dtype=torch.bool)
    sp = mm.cv_spray(x, 10, 0.3)
    assert sp.shape == (3, 6, 10, tp.PRED, 2)
    assert torch.allclose(sp[:, :, 0], tp.cv_pred(x), atol=1e-5)               # head 0 is plain CV
    a1, f1 = mm.min_at_k(sp, y, m, 1)
    a10, f10 = mm.min_at_k(sp, y, m, 10)
    assert a10 <= a1 and f10 <= f1


def test_multimodal_predictor_futures_shape():
    pr = object.__new__(mm.MultimodalPredictor)
    pr.net, pr.K = mm.SocialTransformerMM(5), 5
    pr.net.eval()
    hist = np.cumsum(np.random.default_rng(0).normal(size=(3, tp.OBS, 2)) * 0.2, axis=1)
    f = pr.futures(hist, horizon_s=3.0, k=7)
    assert f.shape == (7, 3, 30, 2) and np.isfinite(f).all()


def test_risk_planner_uses_sampled_futures():
    pl = s.RiskPlanner(seed=0)
    pos, goal = np.array([0.0, 0.0]), np.array([6.0, 0.0])
    T = 30
    t = np.arange(1, T + 1)[:, None] * s.DT
    blocking = np.broadcast_to(np.array([1.2, 0.0]) + 0 * t, (8, 1, T, 2)).copy()       # stationary person on the path
    far = blocking + np.array([0.0, 8.0])
    v_block = pl.plan(pos, goal, [np.array([1.2, 0.0])], [np.zeros(2)], futures=blocking)
    risk_block = pl.last['risk'].max()
    pl2 = s.RiskPlanner(seed=0)
    v_far = pl2.plan(pos, goal, [np.array([1.2, 8.0])], [np.zeros(2)], futures=far)
    assert risk_block > 0.5 and pl2.last['risk'].max() == 0.0
    assert v_far[0] > 0.7 and abs(v_far[1]) < 0.2                              # free path: ~full speed toward the goal
    assert v_block[0] < v_far[0] or abs(v_block[1]) > 0.05                    # blocked: slows or steers around
