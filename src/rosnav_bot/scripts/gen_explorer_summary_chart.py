#!/usr/bin/env python3
"""Regenerate images/explorer_summary_stats.png — aggregate mean/min/max
coverage per explorer backend across every successful run in
sweep_logs/batch_results_combined.csv (2026-08-26 multi-world sweep).

Not a ROS node — a doc-generation tool, run manually:
    python3 gen_explorer_summary_chart.py ../../../images
"""
import csv
import statistics
import sys
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

plt.rcParams['font.family'] = 'DejaVu Sans'
plt.rcParams['axes.edgecolor'] = '#c7ccd4'
plt.rcParams['axes.linewidth'] = 0.8
plt.rcParams['text.color'] = '#10161f'
plt.rcParams['axes.labelcolor'] = '#4a5568'
plt.rcParams['xtick.color'] = '#4a5568'
plt.rcParams['ytick.color'] = '#10161f'
plt.rcParams['figure.facecolor'] = 'white'
plt.rcParams['axes.facecolor'] = 'white'
plt.rcParams['savefig.facecolor'] = 'white'

COLORS = {'builtin': '#2a78d6', 'explore_lite': '#eb6834', 'frontier': '#1baf7a', 'rrt': '#eda100'}
GRID = '#eef1f5'
MUTED = '#8892a0'


def load(csv_path):
    """Returns {explorer: (nonzero_values, zero_count)}. True zeros are
    excluded from the stats themselves — every 0% in this dataset traces to
    a robot-never-moved infrastructure bug (explore_lite issuing zero nav
    goals), not genuinely poor exploration, so averaging them in would
    conflate "bad at exploring" with "failed to function." Reported
    separately as a per-backend failure count instead."""
    by_explorer = defaultdict(list)
    zeros = defaultdict(int)
    with open(csv_path) as f:
        for r in csv.DictReader(f):
            pct = r.get('final_pct')
            if pct in (None, ''):
                continue
            try:
                val = float(pct)
            except ValueError:
                continue
            if val == 0:
                zeros[r['explorer']] += 1
            else:
                by_explorer[r['explorer']].append(val)
    return by_explorer, zeros


def make_fig(csv_path, out_path):
    by_explorer, zeros = load(csv_path)
    rows = sorted(by_explorer.items(), key=lambda kv: -statistics.median(kv[1]))

    fig, ax = plt.subplots(figsize=(9.8, 4.8))
    names = [f'{name}\n(n={len(v)}{f", {zeros[name]} failed" if zeros[name] else ""})'
             for name, v in rows][::-1]
    medians = [statistics.median(v) for _, v in rows][::-1]
    mins = [min(v) for _, v in rows][::-1]
    maxs = [max(v) for _, v in rows][::-1]
    colors = [COLORS.get(name, MUTED) for name, _ in rows][::-1]

    y = range(len(rows))
    err_low = [m - lo for m, lo in zip(medians, mins)]
    err_high = [hi - m for m, hi in zip(medians, maxs)]
    bars = ax.barh(y, medians, color=colors, height=0.5, zorder=3,
                    xerr=[err_low, err_high], capsize=4,
                    error_kw={'ecolor': '#4a5568', 'linewidth': 1.2})
    ax.set_yticks(list(y))
    ax.set_yticklabels(names, fontsize=10.5)
    ax.set_xlim(0, 100)
    ax.set_xlabel('coverage % — bar = median, whiskers = min/max (zero-coverage failures excluded, shown as n)',
                  fontsize=9.5)
    ax.set_title('Explorer backend comparison — all worlds, this session',
                  fontsize=13, fontweight='bold', pad=14, loc='left', color='#10161f')
    ax.grid(axis='x', color=GRID, linewidth=1, zorder=0)
    ax.set_axisbelow(True)
    for spine in ('top', 'right', 'left'):
        ax.spines[spine].set_visible(False)
    ax.tick_params(left=False)

    for bar, m, hi in zip(bars, medians, maxs):
        ax.text(hi + 2.5, bar.get_y() + bar.get_height() / 2, f'{m:.1f}%',
                va='center', ha='left', fontsize=10, fontweight='bold', color='#10161f')

    fig.text(0.01, -0.05,
              'Zero-coverage runs (robot never moved — an explore_lite infrastructure bug, not '
              'exploration quality) excluded from stats, counted separately as "failed" per '
              'backend. Small-n backends (frontier, rrt) not yet statistically conclusive '
              'against builtin/explore_lite\'s larger samples. '
              'Source: sweep_logs/batch_results_combined.csv.',
              fontsize=7.6, color=MUTED, ha='left', wrap=True)
    fig.tight_layout(rect=[0, 0.04, 1, 1])
    fig.savefig(out_path, dpi=200, bbox_inches='tight')
    plt.close(fig)


if __name__ == '__main__':
    out_dir = sys.argv[1] if len(sys.argv) > 1 else '.'
    csv_path = sys.argv[2] if len(sys.argv) > 2 else 'sweep_logs/batch_results_combined.csv'
    out_path = f'{out_dir}/explorer_summary_stats.png'
    make_fig(csv_path, out_path)
    print('wrote', out_path)
