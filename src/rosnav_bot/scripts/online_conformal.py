#!/usr/bin/env python3
"""
online_conformal.py — online (adaptive) conformal radii for pedestrian prediction (no ROS).

The offline conformal radii (conformal_calibrate.py) are fixed from one calibration set. Crowds change:
denser, faster, new scene. This tracks the (1-alpha) quantile of the REALISED prediction error per horizon step
online, with the pinball-loss gradient step (Angelopoulos et al., "Online conformal prediction"):

    r_k  <-  max(0, r_k + eta * (1[err_k > r_k] - alpha))

Coverage drifts back to 1-alpha after a shift in a few dozen updates. eta is scaled by the running error scale so one
setting works from 0.1 m to 4 m radii. A PredictionLedger remembers predicted paths and, as ground truth arrives later,
turns them into per-step errors automatically.
"""
import numpy as np


class OnlineConformal:
    def __init__(self, radii0, alpha=0.1, eta=0.05, floor=0.05, ceil=8.0):
        self.r = np.asarray(radii0, float).copy()
        self.alpha, self.eta, self.floor, self.ceil = alpha, eta, floor, ceil
        self.scale = np.maximum(self.r, 0.1)
        self.n = np.zeros(len(self.r), int)
        self.hit = np.zeros(len(self.r))          # running coverage per step (EMA)
        self.cov = np.full(len(self.r), 1 - alpha)

    def update(self, k, err):
        """err = realised distance (m) between predicted and true position at horizon step k."""
        self.scale[k] = 0.98 * self.scale[k] + 0.02 * max(err, 0.05)
        miss = float(err > self.r[k])
        self.r[k] = float(np.clip(self.r[k] + self.eta * self.scale[k] * (miss - self.alpha) * 4, self.floor, self.ceil))
        self.cov[k] = 0.98 * self.cov[k] + 0.02 * (1 - miss)
        self.n[k] += 1

    @property
    def radii(self):
        return self.r.copy()


class PredictionLedger:
    """Stores predictions; when the future arrives, feeds the errors to an OnlineConformal.

    add(id, t0, path)        path[k] is the predicted position at t0 + (k+1)*dt
    observe(id, t, xy)       call with every new track position; matures any step whose time has come (within tol)
    """
    def __init__(self, conf, dt=0.4, tol=0.2, keep_s=8.0):
        self.conf, self.dt, self.tol, self.keep = conf, dt, tol, keep_s
        self.items = []                          # [id, t0, path, done_mask]

    def add(self, tid, t0, path):
        path = np.asarray(path, float)
        self.items.append([tid, t0, path, np.zeros(len(path), bool)])

    def observe(self, tid, t, xy):
        n_upd = 0
        for it in self.items:
            if it[0] != tid:
                continue
            for k in range(len(it[2])):
                if not it[3][k] and abs(t - (it[1] + (k + 1) * self.dt)) <= self.tol:
                    it[3][k] = True
                    self.conf.update(min(k, len(self.conf.r) - 1), float(np.linalg.norm(it[2][k] - np.asarray(xy))))
                    n_upd += 1
        self.items = [it for it in self.items if t - it[1] < self.keep]
        return n_upd


class ErrorTracker:
    """Duck-types OnlineConformal for PredictionLedger: keeps an EMA of realised error (all horizon steps pooled)."""
    def __init__(self, n_steps=12, decay=0.98):
        self.r = np.zeros(n_steps)
        self.decay, self.mean, self.n = decay, 0.0, 0

    def update(self, k, err):
        self.mean = err if self.n == 0 else self.decay * self.mean + (1 - self.decay) * err
        self.n += 1


class OnlineGate:
    """Causal blend weight g in [0,1] between a learned and a constant-velocity predictor, from REALISED errors.

    paths = cv + g * (learned - cv). g = 1 until `min_n` matured samples exist (prior: trust the learned model).
    `min_n` is 12, one prediction horizon, so a bad model is not trusted for the 40-sample warm-up that
    hurt the small eth scene;
    afterwards g moves toward 0 when CV has been closer to the truth than the learned model (hotel-like scenes) and
    toward 1 when the learned model is better. `kappa` sets the sensitivity (relative error gap -> g).
    """
    def __init__(self, n_steps=12, dt=0.4, kappa=4.0, min_n=12, decay=0.98):
        self.tl, self.tc = ErrorTracker(n_steps, decay), ErrorTracker(n_steps, decay)
        self.ledger_l = PredictionLedger(self.tl, dt=dt)
        self.ledger_c = PredictionLedger(self.tc, dt=dt)
        self.kappa, self.min_n = kappa, min_n

    def add(self, tid, t0, learned, cv):
        self.ledger_l.add(tid, t0, learned)
        self.ledger_c.add(tid, t0, cv)

    def observe(self, tid, t, xy):
        self.ledger_l.observe(tid, t, xy)
        self.ledger_c.observe(tid, t, xy)

    @property
    def g(self):
        if min(self.tl.n, self.tc.n) < self.min_n:
            return 1.0
        el, ec = self.tl.mean, self.tc.mean
        gap = (ec - el) / max(ec, el, 0.05)              # >0: learned better
        return float(np.clip(0.5 + 0.5 * self.kappa * gap, 0.0, 1.0))

    def blend(self, learned, cv):
        g = self.g
        return {k: np.asarray(cv[k]) + g * (np.asarray(learned[k]) - np.asarray(cv[k])) if k in cv else learned[k]
                for k in learned}
