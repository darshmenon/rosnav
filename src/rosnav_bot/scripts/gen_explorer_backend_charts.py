#!/usr/bin/env python3
"""Regenerate images/explorer_backend_{coverage,accuracy}.png for the README.

Coverage panel: recorded 2026-08-23 headless test session (concepts.md §9),
with warehouse.world explore_lite's old excluded 0% now replaced by the real
post-fix number from the 2026-08-25/26 session (see git log "Fix explore
stall and SLAM drift" and EXPLORATION_TESTING_NOTES.md, repo-local).

Accuracy panel: re-run 2026-08-26 against the CURRENT (post-fix) code —
builtin's local-minimum watchdog + explore_lite's frontier-retarget fix v2
(nearest frontier point that clears min_goal_distance_, not the farthest —
v1 of that fix regressed house.world, caught by this same benchmark and
fixed same-session) — same protocol as 2026-08-23 (benchmark.py
mode:=accuracy + mode:=slam, 120s, house.world).

Not a ROS node — a doc-generation tool, run manually:
    python3 gen_explorer_backend_charts.py ../../../images

The figures below hardcode session numbers; update them by hand if a fresh
explorer-backend comparison run supersedes this one.
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
    # rrt is still left out — it never runs at all in single-robot mode, so a
    # 0% bar would misleadingly read as "ran and covered nothing" rather than
    # "never moved". builtin's original pre-fix warehouse number (46.3%) is
    # also left out for the same reason as before: superseded by the
    # local_costmap fix already in the current code.
    fig, ax = plt.subplots(figsize=(9, 4.6))

    rows = [
        ('builtin — house', 58, S1_BUILTIN, 'wedged — manual save needed (2026-08-23)'),
        ('explore_lite — house', 22, S2_EXPLORE_LITE, 'finished cleanly (2026-08-23)'),
        ('frontier (MRTSP) — house', 29, S3_FRONTIER, 'stuck retrying frontier (2026-08-23)'),
        ('builtin — warehouse', 37.2, S1_BUILTIN, 'post local_costmap-fix; new stall (2026-08-23)'),
        ('explore_lite — warehouse', 45.3, S2_EXPLORE_LITE, 'FIXED 2026-08-25/26 — was permanent 0%'),
    ]
    hbar_panel(ax, rows, 70, 'coverage %', 'Coverage across recorded runs')

    fig.suptitle('rosnav_bot exploration backend comparison — headless, 2026-08-23 + 2026-08-25/26',
                 fontsize=13.5, fontweight='bold', x=0.02, ha='left', y=1.03, color='#10161f')
    fig.text(0.02, -0.05,
              'Excluded: rrt (never runs in single-robot mode). builtin/warehouse pre-fix (46.3%) also excluded — '
              'superseded by the local_costmap fix already in the current code. explore_lite/warehouse was 0% '
              "(permanent goal-tolerance stall loop) until the 2026-08-25/26 fix shown here — see git log "
              '"Fix explore stall and SLAM drift" + EXPLORATION_TESTING_NOTES.md (repo-local, not pushed).',
              fontsize=8.2, color=MUTED, ha='left', wrap=True)
    fig.tight_layout(rect=[0, 0.08, 1, 0.93])
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

    # 2026-08-26 re-run against the CURRENT (post-fix, v2) code, same
    # protocol as 2026-08-23: benchmark.py mode:=accuracy + mode:=slam in
    # parallel, 120s, house.world.
    # Raw JSONs: sweep_logs/builtin_house_refix_*.json (builtin, unchanged
    # since the v1->v2 explore_lite change doesn't touch builtin) and
    # sweep_logs/explore_lite_house_v2fix_*.json (explore_lite, v2 fix).
    vbar_metric(axes[0], 'Final position drift', 'm', 0.335, 0.216)
    vbar_metric(axes[1], 'Yaw drift (magnitude)', '°', 98.2, 21.2)
    vbar_metric(axes[2], 'Final coverage', '%', 48.5, 26.1)

    fig.suptitle('SLAM accuracy under load — builtin vs explore_lite (120s runs, house.world, 2026-08-26 re-run)',
                 fontsize=12.5, fontweight='bold', x=0.02, ha='left', y=1.06, color='#10161f')
    fig.text(0.02, -0.05,
              "explore_lite's frontier-retarget fix (added to stop a warehouse.world zero-motion loop, "
              'centroid-inside-goal-tolerance) initially picked the FARTHEST point on a frontier, which briefly '
              'regressed this same house.world benchmark (drift 0.712m, yaw 115.6°, caught by this benchmark and '
              'fixed same-session — see EXPLORATION_TESTING_NOTES.md). Picking the NEAREST point that still '
              'clears the distance floor instead (shown here) keeps explore_lite the lower-drift backend, though '
              "coverage is down from 2026-08-23 (49.9%) for both — both backends hit a local minimum by ~t=30s.",
              fontsize=8.0, color=MUTED, ha='left', wrap=True)
    fig.tight_layout(rect=[0, 0.11, 1, 0.90])
    fig.savefig(out_path, dpi=200, bbox_inches='tight')
    plt.close(fig)


if __name__ == '__main__':
    import sys
    out_dir = sys.argv[1] if len(sys.argv) > 1 else '.'
    make_coverage_fig(f'{out_dir}/explorer_backend_coverage.png')
    make_accuracy_fig(f'{out_dir}/explorer_backend_accuracy.png')
    print('wrote', f'{out_dir}/explorer_backend_coverage.png')
    print('wrote', f'{out_dir}/explorer_backend_accuracy.png')
