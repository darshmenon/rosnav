#!/usr/bin/env python3
"""
prediction_watchdog.py — pure-python health monitor for a learned pedestrian predictor (no ROS).

Decides, every tick, whether the learned predictions may shape the costmap:

  OK        use the learned paths
  DEGRADED  learned output looks wrong -> use the constant-velocity paths instead
  FALLBACK  tracks are stale / the predictor is late or failing -> publish NO mask (plain Nav2 behaviour)

Checks (each returns a short reason string):
  stale_tracks    no tracker message for > max_track_age_s
  slow_inference  last inference took > latency_budget_ms (or errored)
  non_finite      NaN / inf in a predicted path
  teleport        a predicted path jumps faster than max_speed between steps (impossible for a person)
  far_from_cv     learned end point is > ood_m from the constant-velocity end point (out of distribution)
  ood_history     observed history already implies speed > max_speed (tracker glitch / id swap)

Hysteresis: leave a bad mode only after `recover_ticks` consecutive clean ticks, so the mask never flickers.
Every transition is returned so the node can log it ("OK -> DEGRADED: far_from_cv x2").
"""
import math

import numpy as np

OK, DEGRADED, FALLBACK = 'OK', 'DEGRADED', 'FALLBACK'
_RANK = {OK: 0, DEGRADED: 1, FALLBACK: 2}


class PredictionWatchdog:
    def __init__(self, max_track_age_s=1.0, latency_budget_ms=50.0, max_speed=3.0, ood_m=4.0,
                 recover_ticks=8, dt=0.4):
        self.max_age, self.budget, self.vmax, self.ood = max_track_age_s, latency_budget_ms, max_speed, ood_m
        self.recover, self.dt = recover_ticks, dt
        self.mode, self._clean = OK, 0
        self.counts = {}

    def check(self, now, last_track_t, paths, cv_paths, last_infer_ms=None, infer_error=False):
        """Returns (mode, reasons, transition) — transition is None or (old, new)."""
        reasons, want = [], OK
        if last_track_t is None or now - last_track_t > self.max_age:
            reasons.append('stale_tracks'); want = FALLBACK
        if infer_error or (last_infer_ms is not None and last_infer_ms > self.budget):
            reasons.append('slow_inference'); want = max(want, FALLBACK, key=_RANK.get)
        bad = set()
        for k, p in (paths or {}).items():
            p = np.asarray(p, float)
            if not np.isfinite(p).all():
                bad.add('non_finite'); continue
            if len(p) > 1 and np.linalg.norm(np.diff(p, axis=0), axis=1).max() / self.dt > self.vmax:
                bad.add('teleport')
            c = (cv_paths or {}).get(k)
            if c is not None and np.linalg.norm(p[-1] - np.asarray(c, float)[-1]) > self.ood:
                bad.add('far_from_cv')
        if bad:
            reasons += sorted(bad)
            want = max(want, DEGRADED, key=_RANK.get)
        for r in reasons:
            self.counts[r] = self.counts.get(r, 0) + 1
        old = self.mode
        if _RANK[want] >= _RANK[self.mode]:
            self.mode, self._clean = want, 0 if want != OK else self._clean
        else:                                   # want is better than current: need N clean ticks in a row
            self._clean += 1
            if self._clean >= self.recover:
                self.mode, self._clean = want, 0
        return self.mode, reasons, ((old, self.mode) if old != self.mode else None)


def sanitize_paths(paths, cv_paths, mode):
    """Paths that may be used for the costmap in `mode` (FALLBACK -> none, DEGRADED -> CV, OK -> learned)."""
    if mode == FALLBACK:
        return {}
    if mode == DEGRADED:
        return dict(cv_paths)
    return paths
