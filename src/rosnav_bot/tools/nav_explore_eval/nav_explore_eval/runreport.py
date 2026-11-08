"""Shared run plumbing: scoring one grid (optionally aligned) and building / validating run reports."""
from .metrics import TIME_TO, align, align_yaw, efficiency, multi_score, path_length, score, time_to_percent

SCHEMA_VERSION = 2
REQUIRED = ('label', 'final', 'time_to_pct_s', 'path_length_m', 'series')
FINAL_KEYS = ('coverage_pct', 'wall_f1_pct', 'false_free_pct')


class Scorer:
    """Aligns (translation, optionally yaw) and scores live maps against one truth map."""

    def __init__(self, truth, do_align=False, rotate=False, max_shift=3.0, max_yaw=15.0, tol=0.10):
        self.truth, self.do_align, self.rotate = truth, do_align, rotate
        self.max_shift, self.max_yaw, self.tol = max_shift, max_yaw, tol

    def pose(self, grid):
        if not self.do_align:
            return 0.0, 0.0, 0.0
        if self.rotate:
            return align_yaw(grid, self.truth, self.max_shift, self.max_yaw)
        dx, dy = align(grid, self.truth, self.max_shift)
        return dx, dy, 0.0

    def __call__(self, grid):
        dx, dy, yaw = self.pose(grid)
        return score(grid, self.truth, dx, dy, self.tol, yaw)

    def multi(self, grids):
        poses = [self.pose(g) for g in grids]
        return multi_score(grids, self.truth, poses, self.tol)


def build_report(label, truth_path, series, pts, extra=None):
    """series = [(t, metrics)], pts = odom xy list. Returns the JSON-able run report."""
    final = series[-1][1]
    length = path_length(pts)
    rep = {'schema': SCHEMA_VERSION, 'label': label, 'truth': truth_path, 'duration_s': series[-1][0],
           'path_length_m': length, 'final': final, 'time_to_pct_s': time_to_percent(series),
           'efficiency': efficiency(final, length), 'series': [dict(t=t, **m) for t, m in series]}
    rep.update(extra or {})
    return rep


def validate(rep):
    """List of problems in a run report (empty = valid)."""
    if not isinstance(rep, dict):
        return ['not a JSON object']
    errs = [f'missing key: {k}' for k in REQUIRED if k not in rep]
    if errs:
        return errs
    errs += [f'final.{k} missing' for k in FINAL_KEYS if k not in rep['final']]
    errs += [f'time_to_pct_s[{p}] missing' for p in TIME_TO if str(p) not in rep['time_to_pct_s']]
    if not isinstance(rep['label'], str) or not rep['label'].strip():
        errs.append('label must be a non-empty string')
    if not rep['series']:
        errs.append('series is empty')
    for k in FINAL_KEYS:
        v = rep['final'].get(k)
        if isinstance(v, (int, float)) and not -1e-6 <= v <= 100 + 1e-6:
            errs.append(f'final.{k}={v} outside 0..100')
    return errs
