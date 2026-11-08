"""Repeat statistics, leaderboard and HTML report from run JSONs. No ROS needed."""
import base64
import glob
import html
import io
import json
import os
import statistics

from .metrics import TIME_TO
from .runreport import validate

METRICS = [('coverage_pct', 'cov %'), ('wall_f1_pct', 'F1 %'), ('false_free_pct', 'leak %')]


def load_reports(paths):
    """Expand dirs/globs, load JSON, return [(path, report)]. Skips nothing: bad files raise ValueError."""
    files = []
    for p in paths:
        files += sorted(glob.glob(os.path.join(p, '*.json'))) if os.path.isdir(p) else (sorted(glob.glob(p)) or [p])
    out = []
    for f in files:
        try:
            out.append((f, json.load(open(f))))
        except (OSError, json.JSONDecodeError) as e:
            raise ValueError(f'{f}: {e}')
    return out


def _ms(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return None, None, 0
    return statistics.fmean(vals), (statistics.stdev(vals) if len(vals) > 1 else 0.0), len(vals)


def aggregate(reports):
    """Group by label -> {label: {'n', metric: (mean, std, n), 'time_to': {pct: (mean, std, n_reached)}, ...}}."""
    groups = {}
    for r in reports:
        groups.setdefault(r['label'], []).append(r)
    out = {}
    for label, rs in groups.items():
        g = {'n': len(rs)}
        for k, _ in METRICS:
            g[k] = _ms([r['final'][k] for r in rs])
        g['path_length_m'] = _ms([r['path_length_m'] for r in rs])
        g['efficiency'] = _ms([(r.get('efficiency') or {}).get('efficiency_m2_per_m') for r in rs])
        g['ate'] = _ms([(r.get('localisation') or {}).get('ate_rmse_m') for r in rs])
        g['time_to'] = {p: _ms([r['time_to_pct_s'].get(str(p)) for r in rs]) for p in TIME_TO}
        g['reached'] = {p: sum(1 for r in rs if r['time_to_pct_s'].get(str(p)) is not None) for p in TIME_TO}
        out[label] = g
    return out


def _fmt(ms, nd=1):
    if ms is None or ms[0] is None:
        return '-'
    return f'{ms[0]:.{nd}f}' + (f' ± {ms[1]:.{nd}f}' if ms[2] > 1 else '')


def _fmt_time(g, p):
    ms = g['time_to'][p]
    if ms[0] is None:
        return '-'
    s = _fmt(ms, 0)
    return s if g['reached'][p] == g['n'] else f"{s} ({g['reached'][p]}/{g['n']})"


def stats_rows(agg):
    head = ['label', 'n', 'cov%', 'f1%', 'leak%', 'path_m', 'm2/m'] + [f't{p}%(s)' for p in TIME_TO]
    with_ate = any(g['ate'][0] is not None for g in agg.values())
    rows = [head + (['ate_m'] if with_ate else [])]
    for label, g in sorted(agg.items(), key=lambda kv: -(kv[1]['coverage_pct'][0] or 0)):
        rows.append([label, str(g['n']), _fmt(g['coverage_pct']), _fmt(g['wall_f1_pct']), _fmt(g['false_free_pct']),
                     _fmt(g['path_length_m']), _fmt(g['efficiency'], 2)] + [_fmt_time(g, p) for p in TIME_TO]
                    + ([_fmt(g['ate'], 3)] if with_ate else []))
    return rows


def text_table(rows):
    w = [max(len(r[i]) for r in rows) for i in range(len(rows[0]))]
    return '\n'.join('  '.join(c.rjust(w[i]) for i, c in enumerate(r)) for r in rows)


def markdown_table(rows):
    lines = ['| ' + ' | '.join(rows[0]) + ' |', '|' + '---|' * len(rows[0])]
    return '\n'.join(lines + ['| ' + ' | '.join(r) + ' |' for r in rows[1:]])


def leaderboard(directory):
    """(markdown, problems). Every results/*.json must validate; ranking = mean final coverage."""
    found, problems = load_reports([directory]), []
    good = []
    for f, r in found:
        errs = validate(r)
        if errs:
            problems += [f'{f}: {e}' for e in errs]
        else:
            good.append(r)
    if not good:
        return '', problems or [f'no result files in {directory}']
    truths = sorted({os.path.basename(r.get('truth', '') or '?') for r in good})
    md = ['# Leaderboard', '',
          f'Ranked by mean final coverage. {len(good)} run(s), ground truth: {", ".join(truths)}.', '',
          markdown_table(stats_rows(aggregate(good))), '',
          'Add a run: `nav_explore_eval record ... --out results/<name>.json` and open a pull request.', '']
    return '\n'.join(md), problems


def _chart_png(reports):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7, 3.6))
    for r in reports:
        ax.plot([s['t'] for s in r['series']], [s['coverage_pct'] for s in r['series']], label=r['label'])
    ax.set_xlabel('time (s)'); ax.set_ylabel('coverage (%)'); ax.grid(alpha=.3)
    ax.legend(fontsize=8)
    return _png_b64(fig)


def _png_b64(fig):
    import matplotlib.pyplot as plt
    buf = io.BytesIO()
    fig.tight_layout(); fig.savefig(buf, format='png', dpi=120); plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode()


def overlay_png(live, truth, scorer):
    """Truth walls grey, live walls blue, missed truth walls red (leaks: live free over a wall) -> base64 PNG."""
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from .metrics import _dilate, resample
    dx, dy, yaw = scorer.pose(live)
    r = resample(live, truth, dx, dy, yaw)
    tol = max(0, int(round(scorer.tol / truth.resolution)))
    t_occ, t_free = truth.data == 100, truth.data == 0
    l_occ, l_free = r == 100, r == 0
    leak = l_free & t_occ & ~_dilate(l_occ, tol)
    img = np.full(truth.shape + (3,), 255, dtype=np.uint8)
    img[t_free] = (235, 245, 235)
    img[l_free & t_free] = (150, 205, 150)                      # explored
    img[t_occ] = (120, 120, 120)
    img[l_occ & _dilate(t_occ, tol)] = (40, 90, 200)            # matched wall
    img[l_occ & ~_dilate(t_occ, tol)] = (230, 140, 0)           # spurious wall
    img[leak] = (210, 30, 30)                                   # dangerous: wall mapped as free
    h, w = truth.shape
    fig, ax = plt.subplots(figsize=(6, 6 * h / max(w, 1)))
    ax.imshow(img, origin='lower', extent=(truth.ox, truth.ox + w * truth.resolution,
                                            truth.oy, truth.oy + h * truth.resolution))
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    return _png_b64(fig)


def html_report(reports, overlay=None, title='Exploration report'):
    e = html.escape
    agg = aggregate(reports)
    rows = stats_rows(agg)
    th = ''.join(f'<th>{e(c)}</th>' for c in rows[0])
    body = ''.join('<tr>' + ''.join(f'<td>{e(c)}</td>' for c in r) + '</tr>' for r in rows[1:])
    parts = [f'<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">'
             f'<title>{e(title)}</title><style>body{{font:15px system-ui;margin:2rem auto;max-width:900px;padding:0 1rem;'
             f'color:#222}}table{{border-collapse:collapse;width:100%}}td,th{{border-bottom:1px solid #ddd;padding:.35rem .5rem;'
             f'text-align:right}}th:first-child,td:first-child{{text-align:left}}img{{max-width:100%}}'
             f'small{{color:#666}}</style></head><body><h1>{e(title)}</h1>',
             f'<h2>Summary</h2><table><tr>{th}</tr>{body}</table>',
             f'<h2>Coverage over time</h2><img alt="coverage over time" src="data:image/png;base64,{_chart_png(reports)}">']
    if overlay:
        parts.append('<h2>Map against ground truth</h2>'
                     f'<img alt="map overlay" src="data:image/png;base64,{overlay}">'
                     '<p><small>green = explored free space, blue = matched wall, orange = spurious wall, '
                     'red = wall mapped as free (leak), grey = unmatched truth wall</small></p>')
    parts.append('</body></html>')
    return ''.join(parts)
