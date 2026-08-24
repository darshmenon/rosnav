#!/usr/bin/env python3
"""Regenerate images/explorer_backend_{coverage,accuracy}.png for the README
from the recorded 2026-08-23 headless test session (concepts.md §9).

Not a ROS node — a doc-generation tool, run manually:
    python3 gen_explorer_backend_charts.py ../../../images

The figures below hardcode that session's numbers; update them by hand if a
fresh explorer-backend comparison run supersedes this one.
"""
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

S1_BUILTIN = '#2a78d6'
S2_EXPLORE_LITE = '#eb6834'
S3_FRONTIER = '#1baf7a'
S4_RRT = '#eda100'
GRID = '#eef1f5'
MUTED = '#8892a0'


def hbar_panel(ax, rows, xmax, xlabel, title):
    """rows: list of (name, value, color, outcome_label)"""
    names = [r[0] for r in rows][::-1]
    values = [r[1] for r in rows][::-1]
    colors = [r[2] for r in rows][::-1]
    outcomes = [r[3] for r in rows][::-1]

    y = range(len(rows))
    bars = ax.barh(y, values, color=colors, height=0.58, zorder=3)
    ax.set_yticks(list(y))
    ax.set_yticklabels(names, fontsize=11, fontweight='medium')
    ax.set_xlim(0, xmax)
    ax.set_xlabel(xlabel, fontsize=10)
    ax.set_title(title, fontsize=12.5, fontweight='bold', pad=14, loc='left', color='#10161f')
    ax.grid(axis='x', color=GRID, linewidth=1, zorder=0)
    ax.set_axisbelow(True)
    for spine in ('top', 'right', 'left'):
        ax.spines[spine].set_visible(False)
    ax.tick_params(left=False)

    for bar, val, outcome in zip(bars, values, outcomes):
        w = bar.get_width()
        label = f'{val:g}%'
        ax.text(w + xmax * 0.015, bar.get_y() + bar.get_height() / 2, label,
                va='center', ha='left', fontsize=10.5, fontweight='bold', color='#10161f')
        ax.text(xmax * 0.015, bar.get_y() - 0.02, outcome,
                va='top', ha='left', fontsize=8.3, color=MUTED, style='italic',
                transform=ax.transData)


def make_coverage_fig(out_path):
    # Backends/runs that never produced real coverage (rrt: doesn't run at all
    # in single-robot mode; explore_lite on warehouse: 0% goal-tolerance stall
    # loop) are left out of the bar chart — a 0% bar reads as "ran and covered
    # nothing" rather than "never moved", which is confusing. builtin's
    # pre-fix warehouse number (46.3%) is also left out: it's superseded by
    # the local_costmap fix already in the current code, so showing it next
    # to the post-fix number (37.2%, lower) reads as a regression rather than
    # what it is — two different bugs, same backend. Excluded runs are
    # called out in the footnote instead of plotted.
    fig, ax = plt.subplots(figsize=(9, 4.3))

    rows = [
        ('builtin — house', 58, S1_BUILTIN, 'wedged — manual save needed'),
        ('explore_lite — house', 22, S2_EXPLORE_LITE, 'finished cleanly'),
        ('frontier (MRTSP) — house', 29, S3_FRONTIER, 'stuck retrying frontier'),
        ('builtin — warehouse', 37.2, S1_BUILTIN, 'post-fix; new stall, partial save'),
    ]
    hbar_panel(ax, rows, 70, 'coverage %', 'Coverage across recorded runs')

    fig.suptitle('rosnav_bot exploration backend comparison — headless, 2026-08-23',
                 fontsize=13.5, fontweight='bold', x=0.02, ha='left', y=1.03, color='#10161f')
    fig.text(0.02, -0.03,
              'Excluded: rrt (never runs in single-robot mode) and explore_lite/warehouse (0%, goal-tolerance '
              'stall loop) — neither produced real coverage. builtin/warehouse pre-fix (46.3%) also excluded — '
              "superseded by the local_costmap fix already in the current code. Source: concepts.md § 9.",
              fontsize=8.2, color=MUTED, ha='left', wrap=True)
    fig.tight_layout(rect=[0, 0.06, 1, 0.93])
    fig.savefig(out_path, dpi=200, bbox_inches='tight')
    plt.close(fig)


def vbar_metric(ax, label, unit, builtin_val, explore_val, builtin_disp=None, explore_disp=None):
    vals = [builtin_val, explore_val]
    colors = [S1_BUILTIN, S2_EXPLORE_LITE]
    names = ['builtin', 'explore_lite']
    bars = ax.bar(names, vals, color=colors, width=0.55, zorder=3)
    ax.set_title(label, fontsize=11.5, fontweight='bold', pad=10, color='#10161f')
    ax.grid(axis='y', color=GRID, linewidth=1, zorder=0)
    ax.set_axisbelow(True)
    for spine in ('top', 'right'):
        ax.spines[spine].set_visible(False)
    ax.tick_params(bottom=False, labelsize=10)
    ax.set_ylim(0, max(vals) * 1.35)
    disp = [builtin_disp or f'{builtin_val:g}{unit}', explore_disp or f'{explore_val:g}{unit}']
    for bar, d in zip(bars, disp):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + max(vals) * 0.04, d,
                ha='center', va='bottom', fontsize=9.7, fontweight='bold', color='#10161f')


def make_accuracy_fig(out_path):
    fig, axes = plt.subplots(1, 3, figsize=(10.5, 4.0))

    vbar_metric(axes[0], 'Final position drift', 'm', 1.098, 0.198,
                explore_disp='0.198m\n(peak 0.522m)')
    vbar_metric(axes[1], 'Yaw drift (magnitude)', '°', 133.8, 75.6)
    vbar_metric(axes[2], 'Time to converge', 's', 9.8, 6.3)

    fig.suptitle('SLAM accuracy under load — builtin vs explore_lite (120s runs, house.world, 2026-08-23)',
                 fontsize=12.5, fontweight='bold', x=0.02, ha='left', y=1.06, color='#10161f')
    fig.text(0.02, -0.03,
              "builtin's in-place spinning while wedged directly corrupts localization — 5.5× worse "
              'final drift — for only a modest coverage edge (57.7% vs 49.9% this run). Source: concepts.md § 9.',
              fontsize=8.5, color=MUTED, ha='left')
    fig.tight_layout(rect=[0, 0.03, 1, 0.90])
    fig.savefig(out_path, dpi=200, bbox_inches='tight')
    plt.close(fig)


if __name__ == '__main__':
    import sys
    out_dir = sys.argv[1] if len(sys.argv) > 1 else '.'
    make_coverage_fig(f'{out_dir}/explorer_backend_coverage.png')
    make_accuracy_fig(f'{out_dir}/explorer_backend_accuracy.png')
    print('wrote', f'{out_dir}/explorer_backend_coverage.png')
    print('wrote', f'{out_dir}/explorer_backend_accuracy.png')
