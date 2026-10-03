#!/usr/bin/env python3
"""Generate matplotlib comparison charts for SLAM and exploration sweeps.

Reads benchmark.py JSON files and batch sweep CSVs, then writes:
  - summary.csv
  - coverage_bars.png
  - elapsed_bars.png
  - drift_bars.png
  - yaw_drift_bars.png
  - coverage_timeline.png
  - speed_vs_coverage.png

Example:
  python3 src/rosnav_bot/scripts/gen_slam_explorer_compare.py \
      --inputs 'sweep_logs/*.json' 'sweep_logs/*.csv' --out-dir images/compare
"""

from __future__ import annotations

import argparse
import csv
import gc
import glob
import json
import math
import os
import warnings
from pathlib import Path
from typing import Any

os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib')

import matplotlib

warnings.filterwarnings('ignore', message='Unable to import Axes3D.*')
matplotlib.use('Agg')
import matplotlib.pyplot as plt


COLORS = {
    'builtin': '#2a78d6',
    'explore_lite': '#eb6834',
    'frontier': '#1baf7a',
    'rrt': '#eda100',
    '2d': '#6f7bd9',
    'online_async': '#6f7bd9',
    'online_sync': '#9854c5',
    'cartographer': '#1baf7a',
    '3d': '#d55252',
    'vslam': '#3d9aa4',
}
GRID = '#eef1f5'
TEXT = '#10161f'
MUTED = '#697386'


def setup_style() -> None:
    plt.rcParams['font.family'] = 'DejaVu Sans'
    plt.rcParams['axes.edgecolor'] = '#c7ccd4'
    plt.rcParams['axes.linewidth'] = 0.8
    plt.rcParams['text.color'] = TEXT
    plt.rcParams['axes.labelcolor'] = '#4a5568'
    plt.rcParams['xtick.color'] = '#4a5568'
    plt.rcParams['ytick.color'] = TEXT
    plt.rcParams['figure.facecolor'] = 'white'
    plt.rcParams['axes.facecolor'] = 'white'
    plt.rcParams['savefig.facecolor'] = 'white'


def expand_inputs(patterns: list[str]) -> list[Path]:
    paths: list[Path] = []
    for pattern in patterns:
        matches = glob.glob(os.path.expanduser(pattern))
        paths.extend(Path(m) for m in matches) if matches else paths.append(Path(pattern))
    return sorted({p.expanduser().resolve() for p in paths if p.expanduser().exists()})


def to_float(value: Any) -> float | None:
    if value in (None, ''):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def infer_parts(label: str) -> dict[str, str]:
    tokens = label.replace('-', '_').split('_')
    if label.startswith('explore_lite'):
        explorer = 'explore_lite'
        rest = tokens[2:]
    else:
        explorer = tokens[0] if tokens else 'unknown'
        rest = tokens[1:]

    world = 'unknown'
    slam_algo = '2d'
    skip = {'refix', 'v2fix', 'v3fix', 'fixed', 'slam', 'accuracy', 'nav', 'batch'}
    slam_tokens = {'2d', '3d', 'vslam', 'cartographer', 'online', 'async', 'sync'}
    for token in rest:
        if token in slam_tokens:
            slam_algo = token if slam_algo == '2d' else f'{slam_algo}_{token}'
        elif token not in skip and world == 'unknown':
            world = token
    return {'explorer': explorer or 'unknown', 'world': world, 'slam_algo': slam_algo}


def series_key(row: dict[str, Any]) -> str:
    return f"{row.get('world', 'unknown')} / {row.get('explorer', 'unknown')} / {row.get('slam_algo', '2d')}"


def read_json(path: Path) -> list[dict[str, Any]]:
    data = json.loads(path.read_text())
    if not isinstance(data, dict):
        return []
    label = str(data.get('label') or path.stem)
    parts = infer_parts(label)
    return [{
        'source': str(path),
        'label': label,
        'mode': data.get('mode') or path.stem.rsplit('_', 1)[-1],
        'explorer': data.get('explorer') or parts['explorer'],
        'slam_algo': data.get('slam_algo') or parts['slam_algo'],
        'world': data.get('world') or parts['world'],
        'coverage_pct': to_float(data.get('final_coverage_pct')),
        'time_to_converge_sec': to_float(data.get('time_to_converge_sec')),
        'elapsed_sec': to_float(data.get('duration_sec')),
        'final_drift_m': to_float(data.get('final_drift_m')),
        'max_drift_m': to_float(data.get('max_drift_m')),
        'yaw_drift_deg': to_float(data.get('final_drift_yaw_deg')),
        'timeline': data.get('coverage_timeline') or [],
    }]


def read_csv(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open(newline='') as f:
        for raw in csv.DictReader(f):
            explorer = raw.get('explorer') or raw.get('backend') or 'unknown'
            slam_algo = raw.get('slam_algo') or raw.get('slam') or '2d'
            world = raw.get('world') or 'unknown'
            if world == 'unknown' or world.replace('.', '', 1).isdigit():
                continue
            if explorer == 'unknown':
                continue
            coverage_pct = to_float(raw.get('final_pct') or raw.get('coverage_pct'))
            elapsed_sec = to_float(raw.get('elapsed_sec') or raw.get('duration_sec'))
            offgrid_errors = to_float(raw.get('offgrid_errors'))
            if coverage_pct is None and elapsed_sec is None and offgrid_errors is None:
                continue
            rows.append({
                'source': str(path),
                'label': raw.get('label') or f'{explorer}_{slam_algo}_{world}',
                'mode': 'batch',
                'explorer': explorer,
                'slam_algo': slam_algo,
                'world': world,
                'coverage_pct': coverage_pct,
                'elapsed_sec': elapsed_sec,
                'offgrid_errors': offgrid_errors,
                'timeline': [],
            })
    return rows


def load_rows(paths: list[Path]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for path in paths:
        try:
            if path.suffix == '.json':
                rows.extend(read_json(path))
            elif path.suffix == '.csv':
                rows.extend(read_csv(path))
        except Exception as exc:
            print(f'warning: skipped {path}: {exc}')
    return rows


def color_for(row: dict[str, Any]) -> str:
    return COLORS.get(str(row.get('explorer')), COLORS.get(str(row.get('slam_algo')), MUTED))


def save(fig, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=180, bbox_inches='tight')
    plt.close(fig)
    gc.collect()
    print(f'wrote {path}')


def bar_chart(rows: list[dict[str, Any]], metric: str, title: str, xlabel: str, out: Path) -> None:
    data = [r for r in rows if to_float(r.get(metric)) is not None]
    if not data:
        return
    # A single run often has separate accuracy/slam-mode JSON records that
    # both carry a run-level metric like elapsed_sec — same label, same
    # value, would otherwise plot as an exact duplicate bar. Keep the first.
    seen: set[tuple[Any, float]] = set()
    deduped = []
    for r in data:
        key = (r.get('label'), float(r[metric]))
        if key in seen:
            continue
        seen.add(key)
        deduped.append(r)
    data = deduped
    data = sorted(data, key=lambda r: (series_key(r), str(r.get('mode'))))
    values = [float(r[metric]) for r in data]
    labels = [series_key(r) for r in data]
    # Multiple runs of the same world/explorer/slam_algo (e.g. a fix
    # iterated on the same world, or a batch sweep re-run) render as
    # identical, indistinguishable bars otherwise — disambiguate only the
    # ones that actually collide, using each row's own (unique) label.
    dupes = {lbl for lbl in labels if labels.count(lbl) > 1}
    labels = [f"{lbl} [{r.get('label', '?')}]" if lbl in dupes else lbl
              for lbl, r in zip(labels, data)]

    fig, ax = plt.subplots(figsize=(11, max(4.2, min(12.0, 0.38 * len(data) + 1.5))))
    y = range(len(data))
    bars = ax.barh(y, values, color=[color_for(r) for r in data], height=0.62, zorder=3)
    ax.set_yticks(list(y))
    ax.set_yticklabels(labels, fontsize=8.5)
    ax.invert_yaxis()
    ax.set_xlabel(xlabel)
    ax.set_title(title, loc='left', fontsize=13, fontweight='bold')
    ax.grid(axis='x', color=GRID, linewidth=1, zorder=0)
    ax.set_axisbelow(True)
    for spine in ('top', 'right', 'left'):
        ax.spines[spine].set_visible(False)
    ax.tick_params(left=False)
    xmax = max(values) or 1.0
    for bar, value in zip(bars, values):
        ax.text(bar.get_width() + xmax * 0.015, bar.get_y() + bar.get_height() / 2,
                f'{value:.3g}', va='center', ha='left', fontsize=8.5, fontweight='bold')
    save(fig, out)


def coverage_timeline(rows: list[dict[str, Any]], out: Path) -> None:
    data = [r for r in rows if r.get('timeline')]
    if not data:
        return
    fig, ax = plt.subplots(figsize=(10.5, 5.2))
    for row in sorted(data, key=series_key):
        pts = [p for p in row['timeline'] if to_float(p.get('t')) is not None]
        if not pts:
            continue
        ax.plot([float(p['t']) for p in pts],
                [float(p['coverage_pct']) for p in pts],
                label=series_key(row), linewidth=2.0, color=color_for(row))
    ax.set_title('Coverage Over Time', loc='left', fontsize=13, fontweight='bold')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('coverage (%)')
    ax.grid(color=GRID, linewidth=1)
    ax.legend(fontsize=8, frameon=False)
    save(fig, out)


def speed_vs_coverage(rows: list[dict[str, Any]], out: Path) -> None:
    data = [r for r in rows if to_float(r.get('coverage_pct')) is not None and to_float(r.get('elapsed_sec')) is not None]
    if not data:
        return
    fig, ax = plt.subplots(figsize=(8.8, 5.4))
    for row in data:
        elapsed = float(row['elapsed_sec'])
        coverage = float(row['coverage_pct'])
        ax.scatter(elapsed, coverage, s=75, color=color_for(row), edgecolor='white', linewidth=0.8, zorder=3)
        ax.text(elapsed, coverage + 1.0, series_key(row), fontsize=7.2, color=MUTED)
    ax.set_title('Speed vs Coverage', loc='left', fontsize=13, fontweight='bold')
    ax.set_xlabel('elapsed time (s)')
    ax.set_ylabel('final coverage (%)')
    ax.grid(color=GRID, linewidth=1)
    save(fig, out)


def write_summary(rows: list[dict[str, Any]], out: Path) -> None:
    keys = ['coverage_pct', 'elapsed_sec', 'time_to_converge_sec', 'final_drift_m',
            'max_drift_m', 'yaw_drift_deg', 'offgrid_errors']
    with out.open('w') as f:
        f.write('label,world,explorer,slam_algo,mode,' + ','.join(keys) + ',source\n')
        for row in sorted(rows, key=series_key):
            vals = []
            for key in keys:
                value = to_float(row.get(key))
                vals.append('' if value is None else f'{value:.6g}')
            f.write(f"{row.get('label','')},{row.get('world','')},{row.get('explorer','')},"
                    f"{row.get('slam_algo','')},{row.get('mode','')},"
                    + ','.join(vals) + f",{row.get('source','')}\n")
    print(f'wrote {out}')


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--inputs', nargs='+', default=['sweep_logs/*.json', 'sweep_logs/*.csv'])
    parser.add_argument('--out-dir', default='images/compare')
    args = parser.parse_args()

    setup_style()
    rows = load_rows(expand_inputs(args.inputs))
    if not rows:
        raise SystemExit('no benchmark rows found')

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    write_summary(rows, out_dir / 'summary.csv')
    bar_chart(rows, 'coverage_pct', 'Final Coverage', 'coverage (%)', out_dir / 'coverage_bars.png')
    bar_chart(rows, 'elapsed_sec', 'Elapsed Time', 'seconds', out_dir / 'elapsed_bars.png')
    bar_chart(rows, 'final_drift_m', 'Final SLAM Drift', 'meters', out_dir / 'drift_bars.png')
    bar_chart(rows, 'yaw_drift_deg', 'Final Yaw Drift', 'degrees', out_dir / 'yaw_drift_bars.png')
    coverage_timeline(rows, out_dir / 'coverage_timeline.png')
    speed_vs_coverage(rows, out_dir / 'speed_vs_coverage.png')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
