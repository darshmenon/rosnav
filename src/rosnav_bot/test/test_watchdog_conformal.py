import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import online_conformal as oc  # noqa: E402
import prediction_watchdog as pw  # noqa: E402

GOOD = {1: np.cumsum(np.full((12, 2), 0.3), axis=0)}
CV = {1: np.cumsum(np.full((12, 2), 0.3), axis=0)}


def run(w, n, now0=0.0, **kw):
    out = None
    for i in range(n):
        out = w.check(now0 + i * 0.25, now0 + i * 0.25, **kw)
    return out


def test_ok_when_healthy():
    w = pw.PredictionWatchdog()
    mode, reasons, tr = run(w, 5, paths=GOOD, cv_paths=CV, last_infer_ms=3.0)
    assert mode == pw.OK and not reasons and tr is None


def test_stale_tracks_fall_back():
    w = pw.PredictionWatchdog(max_track_age_s=1.0)
    mode, reasons, tr = w.check(10.0, 5.0, GOOD, CV, 3.0)
    assert mode == pw.FALLBACK and 'stale_tracks' in reasons and tr == (pw.OK, pw.FALLBACK)
    assert pw.sanitize_paths(GOOD, CV, mode) == {}


def test_slow_or_failed_inference_falls_back():
    assert pw.PredictionWatchdog().check(1, 1, GOOD, CV, 400.0)[0] == pw.FALLBACK
    assert pw.PredictionWatchdog().check(1, 1, GOOD, CV, None, infer_error=True)[0] == pw.FALLBACK


def test_teleport_nan_and_ood_degrade_to_cv():
    tele = {1: GOOD[1].copy()}; tele[1][5] += 50.0
    nan = {1: GOOD[1].copy()}; nan[1][3, 0] = np.nan
    far = {1: GOOD[1] + np.array([9.0, 0.0])}
    for bad, reason in ((tele, 'teleport'), (nan, 'non_finite'), (far, 'far_from_cv')):
        mode, reasons, _ = pw.PredictionWatchdog().check(1, 1, bad, CV, 3.0)
        assert mode == pw.DEGRADED and reason in reasons
        assert pw.sanitize_paths(bad, CV, mode) == CV


def test_hysteresis_recovery():
    w = pw.PredictionWatchdog(recover_ticks=4)
    w.check(10.0, 0.0, GOOD, CV, 3.0)                       # stale -> FALLBACK
    modes = [w.check(10.0 + i, 10.0 + i, GOOD, CV, 3.0)[0] for i in range(1, 6)]
    assert modes[:3] == [pw.FALLBACK] * 3 and modes[3] == pw.OK     # needs 4 clean ticks, not 1


def test_online_conformal_recovers_coverage_after_shift():
    rng = np.random.default_rng(0)
    c = oc.OnlineConformal([1.0], alpha=0.1, eta=0.05)
    errs = np.abs(rng.normal(0, 0.4, 600)) * 1.5                 # calibrated-ish noise (std 0.4 -> r90 ~ 0.99)
    for e in errs[:200]:
        c.update(0, e)
    pre = c.radii[0]
    for e in np.abs(rng.normal(0, 0.8, 1500)) * 1.5:             # crowd got twice as unpredictable
        c.update(0, e)
    assert c.radii[0] > 1.5 * pre                                 # radius grew
    assert 0.8 < c.cov[0] < 0.97                                  # coverage back near 90%


def test_ledger_matures_predictions():
    c = oc.OnlineConformal([0.5] * 12)
    led = oc.PredictionLedger(c, dt=0.4)
    path = np.stack([np.arange(1, 13) * 0.4, np.zeros(12)], 1)   # walking at 1 m/s along x
    led.add(7, 0.0, path)
    n = sum(led.observe(7, t, (t * 1.0, 0.0)) for t in np.arange(0, 5.5, 0.1))
    assert n >= 11 and np.all(c.n[:11] > 0)


def test_online_gate_trusts_better_model_and_drops_worse_one():
    rng = np.random.default_rng(1)
    for learned_noise, cv_noise, want in ((0.05, 0.6, 'high'), (0.6, 0.05, 'low')):
        gate = oc.OnlineGate(min_n=30)
        for i in range(80):
            t0 = i * 0.4
            truth = np.stack([t0 + np.arange(1, 13) * 0.4, np.zeros(12)], 1)      # walking at 1 m/s along x
            gate.add(1, t0, truth + rng.normal(0, learned_noise, truth.shape), truth + rng.normal(0, cv_noise, truth.shape))
            for tt in np.arange(t0, t0 + 0.4, 0.1):
                gate.observe(1, tt + 0.0, (tt * 1.0, 0.0))
        assert (gate.g > 0.8) if want == 'high' else (gate.g < 0.2)


def test_online_gate_prior_is_learned_until_enough_data():
    assert oc.OnlineGate().g == 1.0
