"""Exploration metrics: coverage, map accuracy, time-to-percent. Pure numpy, no ROS."""
import numpy as np

from .mapio import Grid

TIME_TO = (25, 50, 75, 90)


def resample(live: Grid, truth: Grid, dx=0.0, dy=0.0, yaw=0.0):
    """Nearest-neighbour live -> truth grid. A live point p lands at R(yaw) p + (dx, dy) in truth
    coordinates (R rotates about the world origin). Outside live = unknown."""
    h, w = truth.shape
    ys = truth.oy + (np.arange(h) + 0.5) * truth.resolution - dy
    xs = truth.ox + (np.arange(w) + 0.5) * truth.resolution - dx
    if yaw == 0.0:
        j = np.floor((xs - live.ox) / live.resolution).astype(int)
        i = np.floor((ys - live.oy) / live.resolution).astype(int)
        out = np.full((h, w), -1, dtype=np.int8)
        ok_i = (i >= 0) & (i < live.shape[0]); ok_j = (j >= 0) & (j < live.shape[1])
        out[np.ix_(ok_i, ok_j)] = live.data[np.ix_(i[ok_i], j[ok_j])]
        return out
    X, Y = np.meshgrid(xs, ys)
    c, s = np.cos(-yaw), np.sin(-yaw)                  # inverse rotation: truth -> live frame
    px, py = c * X - s * Y, s * X + c * Y
    j = np.floor((px - live.ox) / live.resolution).astype(int)
    i = np.floor((py - live.oy) / live.resolution).astype(int)
    ok = (i >= 0) & (i < live.shape[0]) & (j >= 0) & (j < live.shape[1])
    out = np.full((h, w), -1, dtype=np.int8)
    out[ok] = live.data[i[ok], j[ok]]
    return out


def _dilate(m, r=1):
    out = m.copy()
    for di in range(-r, r + 1):
        for dj in range(-r, r + 1):
            out |= np.roll(np.roll(m, di, 0), dj, 1)
    return out


def _align_xy(a, b, res, max_shift_m):
    """Best integer-cell shift of wall mask `a` onto `b` by FFT cross-correlation -> (value, di, dj)."""
    h, w = b.shape
    F = np.fft.rfft2(b.astype(float), s=(2 * h, 2 * w))
    G = np.fft.rfft2(a.astype(float), s=(2 * h, 2 * w))
    c = np.fft.irfft2(F * np.conj(G), s=(2 * h, 2 * w))     # c[s] = sum b[x+s] * a[x]
    ms = int(max_shift_m / res)
    idx_i = np.arange(-ms, ms + 1) % (2 * h)
    idx_j = np.arange(-ms, ms + 1) % (2 * w)
    win = c[np.ix_(idx_i, idx_j)]
    k = np.unravel_index(int(np.argmax(win)), win.shape)
    return float(win[k]), int(k[0]) - ms, int(k[1]) - ms


def align(live: Grid, truth: Grid, max_shift_m=3.0):
    """Best (dx, dy) in metres that overlays live walls on truth walls (translation only)."""
    a = resample(live, truth) == 100
    b = truth.data == 100
    if a.sum() < 10 or b.sum() < 10:
        return 0.0, 0.0
    _, si, sj = _align_xy(a, b, truth.resolution, max_shift_m)
    return sj * truth.resolution, si * truth.resolution


def align_yaw(live: Grid, truth: Grid, max_shift_m=3.0, max_yaw_deg=15.0, step_deg=1.0):
    """Best (dx, dy, yaw_rad): rotation about the world origin searched on a grid of angles, translation by
    FFT for each. Returns (dx, dy, 0.0) when the yaw search is switched off (max_yaw_deg == 0)."""
    b = truth.data == 100
    if b.sum() < 10:
        return 0.0, 0.0, 0.0
    best = (-1.0, 0.0, 0, 0)
    n = int(round(max_yaw_deg / step_deg)) if max_yaw_deg > 0 else 0
    cands = sorted(range(-n, n + 1), key=abs)                       # prefer small rotations on ties
    for k in cands:
        yaw = np.deg2rad(k * step_deg)
        a = resample(live, truth, 0.0, 0.0, yaw) == 100
        if a.sum() < 10:
            continue
        v, si, sj = _align_xy(a, b, truth.resolution, max_shift_m)
        if v > best[0] * (1 + 1e-9):
            best = (v, yaw, si, sj)
    if best[0] < 0:
        return 0.0, 0.0, 0.0
    _, yaw, si, sj = best
    return sj * truth.resolution, si * truth.resolution, float(yaw)


def score(live: Grid, truth: Grid, dx=0.0, dy=0.0, tol_m=0.10, yaw=0.0):
    """Coverage + accuracy of `live` vs ground-truth `truth` after moving live by (dx, dy, yaw).
    A wall cell matches a wall within `tol_m` metres."""
    return _score_r(resample(live, truth, dx, dy, yaw), truth, dx, dy, tol_m, yaw)


def _score_r(r, truth: Grid, dx=0.0, dy=0.0, tol_m=0.10, yaw=0.0):
    tol_cells = max(0, int(round(tol_m / truth.resolution)))
    t_free, t_occ = truth.data == 0, truth.data == 100
    l_free, l_occ = r == 0, r == 100
    coverage = (l_free & t_free).sum() / max(t_free.sum(), 1) * 100
    wall_cov = (_dilate(l_occ, tol_cells) & t_occ).sum() / max(t_occ.sum(), 1) * 100   # recall
    prec = (l_occ & _dilate(t_occ, tol_cells)).sum() / max(l_occ.sum(), 1) * 100
    f1 = 2 * prec * wall_cov / max(prec + wall_cov, 1e-9)
    leak = (l_free & t_occ & ~_dilate(l_occ, tol_cells)).sum() / max(t_occ.sum(), 1) * 100
    return {'coverage_pct': float(coverage), 'wall_recall_pct': float(wall_cov),
            'wall_precision_pct': float(prec), 'wall_f1_pct': float(f1),
            'false_free_pct': float(leak), 'dx_m': float(dx), 'dy_m': float(dy),
            'yaw_deg': float(np.rad2deg(yaw)),
            'covered_area_m2': float((l_free & t_free).sum() * truth.resolution ** 2)}


def time_to_percent(series, key='coverage_pct', targets=TIME_TO):
    """series = [(t, metrics_dict)] ascending in t. Linear-interpolated first crossing, else None."""
    out = {}
    for p in targets:
        out[str(p)] = None
        prev = None
        for t, m in series:
            v = m[key]
            if v >= p:
                if prev is None or v == prev[1]:
                    out[str(p)] = float(t)
                else:
                    out[str(p)] = float(prev[0] + (t - prev[0]) * (p - prev[1]) / (v - prev[1]))
                break
            prev = (t, v)
    return out


def path_length(points):
    p = np.asarray(points, dtype=float)
    return float(np.hypot(*np.diff(p, axis=0).T).sum()) if len(p) > 1 else 0.0


def efficiency(final, path_length_m):
    """Extra run-level metrics: explored area per metre driven (m2/m)."""
    area = final['covered_area_m2']
    return {'covered_area_m2': float(area),
            'efficiency_m2_per_m': float(area / path_length_m) if path_length_m > 0 else None}


def multi_score(lives, truth: Grid, offsets=None, tol_m=0.10):
    """Several robots' maps against one truth. Returns (merged_metrics, per_robot) where merged is the
    union map (free if any robot saw it free, wall if any saw a wall) and per_robot lists each robot's
    own coverage, the area only it covered (unique), and its share of the overlap."""
    offsets = offsets or [(0.0, 0.0, 0.0)] * len(lives)
    rs = [resample(g, truth, *o) for g, o in zip(lives, offsets)]
    merged = np.full(truth.shape, -1, dtype=np.int8)
    for r in rs:
        merged[(r == 0) & (merged == -1)] = 0
    for r in rs:
        merged[r == 100] = 100
    t_free = truth.data == 0
    total = max(int(t_free.sum()), 1)
    cov = [(r == 0) & t_free for r in rs]
    count = np.sum(cov, axis=0) if cov else np.zeros(truth.shape)
    union = int((count >= 1).sum())
    per = []
    for k, c in enumerate(cov):
        others = np.zeros_like(c)
        for q, o in enumerate(cov):
            if q != k:
                others |= o
        per.append({'robot': k, 'coverage_pct': float(c.sum() / total * 100),
                    'unique_pct': float((c & ~others).sum() / total * 100),
                    'overlap_pct': float((c & others).sum() / max(int(c.sum()), 1) * 100)})
    m = _score_r(merged, truth, tol_m=tol_m)
    m['overlap_pct'] = float((count >= 2).sum() / max(union, 1) * 100)     # of the covered area, seen by 2+
    m['robots'] = len(lives)
    return m, per


def _umeyama_2d(est, gt):
    """Rigid (rotation + translation) transform taking est points onto gt points, least squares."""
    ce, cg = est.mean(axis=0), gt.mean(axis=0)
    h = (est - ce).T @ (gt - cg)
    u, _, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(vt.T @ u.T)) or 1.0
    r = vt.T @ np.diag([1.0, d]) @ u.T
    return r, cg - r @ ce


def localisation_error(est, gt, max_dt=0.25, align=True):
    """Absolute trajectory error of `est` against ground truth `gt`, both [(t, x, y)].
    Each est sample is paired with the nearest-in-time gt sample (within `max_dt` s). With `align` the
    est trajectory is first rigidly fitted to gt (standard ATE), so only drift counts, not the frame.
    Returns {ate_rmse_m, ate_max_m, final_error_m, drift_pct, pairs} or None if nothing pairs."""
    if len(est) < 2 or len(gt) < 2:
        return None
    g = np.asarray(sorted(gt), dtype=float)
    e = np.asarray(sorted(est), dtype=float)
    k = np.clip(np.searchsorted(g[:, 0], e[:, 0]), 1, len(g) - 1)
    k -= (np.abs(g[k - 1, 0] - e[:, 0]) < np.abs(g[k, 0] - e[:, 0]))      # nearest of the two neighbours
    ok = np.abs(g[k, 0] - e[:, 0]) <= max_dt
    if ok.sum() < 2:
        return None
    pe, pg = e[ok, 1:3], g[k[ok], 1:3]
    if align:
        r, t = _umeyama_2d(pe, pg)
        pe = pe @ r.T + t
    err = np.hypot(*(pe - pg).T)
    length = path_length(pg)
    return {'ate_rmse_m': float(np.sqrt((err ** 2).mean())), 'ate_max_m': float(err.max()),
            'final_error_m': float(err[-1]),
            'drift_pct': float(err[-1] / length * 100) if length > 0 else None, 'pairs': int(ok.sum())}
