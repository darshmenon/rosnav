#!/usr/bin/env python3
"""Regenerate images/explore_lite_fix_progress.png for the README from the
2026-08-25/26 explore_lite warehouse-stall fix session (concepts.md / see
git log "Fix explore stall and SLAM drift").

Not a ROS node — a doc-generation tool, run manually:
    python3 gen_explore_lite_fix_chart.py ../../../images

The figure hardcodes that session's live-run numbers; update them by hand if
a fresh warehouse.world explore_lite run supersedes this one.
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

BAD = '#c0392b'
OK = '#eb6834'
GOOD = '#1baf7a'
GRID = '#eef1f5'
MUTED = '#8892a0'

WAREHOUSE_ROWS = [
    # name, coverage %, color, outcome note
    ('Original\n(before fix)', 0.0, BAD, 'permanent 0% — goal inside xy_goal_tolerance,\n2288 instant "reached" cycles, zero motion'),
    ('v1 patch\n(pick farther frontier)', 0.0, BAD, 'still 0% — only one frontier region existed,\nnothing farther to fall back to'),
    ('v2 patch\n(retarget within frontier)', 42.3, OK, 'fixed — real exploration, but hit a\nseparate SLAM map_odom drift stall later'),
    ('v2 + stable SLAM profile', 45.3, GOOD, 'best result — drift contained,\nstill exploring when stopped manually'),
]

# world, time-to-signal (min), coverage % at that point, color, note
WORLD_SPEED_ROWS = [
    ('warehouse.world\n(~150m²)', 10.5, 42.3, OK, 'multi-minute runs; slow to\niterate on a fix'),
    ('bench_room_small.world\n(~25m²)', 2.5, 39.7, GOOD, 'same-class stall reproduced\n4x faster — built for iteration'),
]


def make_coverage_panel(ax):
    names = [r[0] for r in WAREHOUSE_ROWS][::-1]
    values = [r[1] for r in WAREHOUSE_ROWS][::-1]
    colors = [r[2] for r in WAREHOUSE_ROWS][::-1]
    notes = [r[3] for r in WAREHOUSE_ROWS][::-1]

    y = range(len(WAREHOUSE_ROWS))
    bars = ax.barh(y, values, color=colors, height=0.6, zorder=3)
    ax.set_yticks(list(y))
    ax.set_yticklabels(names, fontsize=10, fontweight='medium')
    ax.set_xlim(0, 55)
    ax.set_xlabel('warehouse.world map coverage (%)', fontsize=9.5)
    ax.set_title('Before/after fix — warehouse.world', fontsize=11.5, fontweight='bold',
                 pad=12, loc='left', color='#10161f')
    ax.grid(axis='x', color=GRID, linewidth=1, zorder=0)
    ax.set_axisbelow(True)
    for spine in ('top', 'right', 'left'):
        ax.spines[spine].set_visible(False)
    ax.tick_params(left=False)

    for bar, val, note in zip(bars, values, notes):
        w = bar.get_width()
        label = f'{val:g}%'
        ax.text(w + 1.0, bar.get_y() + bar.get_height() / 2, label,
                va='center', ha='left', fontsize=10, fontweight='bold', color='#10161f')
        ax.text(1.0, bar.get_y() - 0.06, note,
                va='top', ha='left', fontsize=7.6, color=MUTED, style='italic')


def make_speed_panel(ax):
    names = [r[0] for r in WORLD_SPEED_ROWS][::-1]
    values = [r[1] for r in WORLD_SPEED_ROWS][::-1]
    covs = [r[2] for r in WORLD_SPEED_ROWS][::-1]
    colors = [r[3] for r in WORLD_SPEED_ROWS][::-1]
    notes = [r[4] for r in WORLD_SPEED_ROWS][::-1]

    y = range(len(WORLD_SPEED_ROWS))
    bars = ax.barh(y, values, color=colors, height=0.5, zorder=3)
    ax.set_yticks(list(y))
    ax.set_yticklabels(names, fontsize=10, fontweight='medium')
    ax.set_ylim(-0.65, len(WORLD_SPEED_ROWS) - 0.5)
    ax.set_xlim(0, 13)
    ax.set_xlabel('time to reach a comparable coverage signal (min)', fontsize=9.5)
    ax.set_title('Benchmark world choice — speed of iteration', fontsize=11.5, fontweight='bold',
                 pad=12, loc='left', color='#10161f')
    ax.grid(axis='x', color=GRID, linewidth=1, zorder=0)
    ax.set_axisbelow(True)
    for spine in ('top', 'right', 'left'):
        ax.spines[spine].set_visible(False)
    ax.tick_params(left=False)

    for bar, val, note, cov in zip(bars, values, notes, covs):
        w = bar.get_width()
        label = f'{val:g} min'
        ax.text(w + 0.25, bar.get_y() + bar.get_height() / 2, label,
                va='center', ha='left', fontsize=10, fontweight='bold', color='#10161f')
        ax.text(0.15, bar.get_y() - 0.10, note,
                va='top', ha='left', fontsize=7.6, color=MUTED, style='italic')


def make_fig(out_path):
    fig, axes = plt.subplots(1, 2, figsize=(13.5, 4.6))
    make_coverage_panel(axes[0])
    make_speed_panel(axes[1])

    fig.suptitle('explore_lite warehouse stall — fix + faster benchmark world (headless, 2026-08-25/26)',
                 fontsize=13, fontweight='bold', x=0.01, ha='left', y=1.04, color='#10161f')
    fig.text(0.01, -0.05,
              'Root cause: frontier centroid landed inside Nav2 xy_goal_tolerance (0.25m), so goals '
              '"succeeded" with zero real motion. Fix retargets to the farthest point on the same '
              "frontier's own known boundary. bench_room_small.world (5x5m room) was added so future "
              'stall repros/fixes take minutes, not 10+. Source: EXPLORATION_TESTING_NOTES.md (repo-local, not pushed).',
              fontsize=8.2, color=MUTED, ha='left')
    fig.tight_layout(rect=[0, 0.04, 1, 0.93])
    fig.savefig(out_path, dpi=200, bbox_inches='tight')
    plt.close(fig)


if __name__ == '__main__':
    import sys
    out_dir = sys.argv[1] if len(sys.argv) > 1 else '.'
    out_path = f'{out_dir}/explore_lite_fix_progress.png'
    make_fig(out_path)
    print('wrote', out_path)
