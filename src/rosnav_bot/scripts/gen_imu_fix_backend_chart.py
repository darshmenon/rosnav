#!/usr/bin/env python3
"""Regenerate images/imu_fix_backend_coverage.png for the README from the
2026-08-26 post-IMU-fix backend comparison session (see
EXPLORATION_TESTING_NOTES.md "MAJOR FINDING: /imu never actually publishes",
repo-local, not pushed; git log "Fix rrt_explore and corridor world, harden
cleanup script").

Not a ROS node — a doc-generation tool, run manually:
    python3 gen_imu_fix_backend_chart.py ../../../images

The figure hardcodes that session's live-run numbers; update them by hand if
a fresh empty_room.world backend comparison run supersedes this one.
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

GOOD = '#1baf7a'
BAD = '#c0392b'
GRID = '#eef1f5'
MUTED = '#8892a0'

# name, coverage %, color, outcome note
ROWS = [
    ('explore_lite', 87.6, GOOD, 'still climbing when stopped —\nmap_odom yaw stable 0.0° to -0.2°\nfor the whole run'),
    ('frontier (MRTSP)', 86.8, GOOD, 'still climbing when stopped,\nsame stability'),
    ('builtin', 3.3, BAD, 'declared "exploration complete" in ~15s —\nall 23 frontier candidates marked\n"failed" immediately (separate,\nnot-yet-diagnosed bug)'),
]

BEFORE_AFTER = [
    ('Before IMU fix\n(warehouse/maze/corridor, best runs)', 46.7, BAD, 'map_odom yaw oscillated\n26° to 100°+ continuously'),
    ('After IMU fix\n(empty_room, explore_lite)', 87.6, GOOD, 'map_odom yaw stable\n0.0° to -0.2°'),
]


def hbar_panel(ax, rows, xmax, xlabel, title):
    names = [r[0] for r in rows][::-1]
    values = [r[1] for r in rows][::-1]
    colors = [r[2] for r in rows][::-1]
    notes = [r[3] for r in rows][::-1]

    y = range(len(rows))
    bars = ax.barh(y, values, color=colors, height=0.55, zorder=3)
    ax.set_yticks(list(y))
    ax.set_yticklabels(names, fontsize=10.5, fontweight='medium')
    ax.set_ylim(-0.75, len(rows) - 0.5)
    ax.set_xlim(0, xmax)
    ax.set_xlabel(xlabel, fontsize=9.5)
    ax.set_title(title, fontsize=12, fontweight='bold', pad=12, loc='left', color='#10161f')
    ax.grid(axis='x', color=GRID, linewidth=1, zorder=0)
    ax.set_axisbelow(True)
    for spine in ('top', 'right', 'left'):
        ax.spines[spine].set_visible(False)
    ax.tick_params(left=False)

    for bar, val, note in zip(bars, values, notes):
        w = bar.get_width()
        ax.text(w + xmax * 0.015, bar.get_y() + bar.get_height() / 2, f'{val:g}%',
                va='center', ha='left', fontsize=10.5, fontweight='bold', color='#10161f')
        ax.text(xmax * 0.015, bar.get_y() - 0.12, note,
                va='top', ha='left', fontsize=7.6, color=MUTED, style='italic')


def make_fig(out_path):
    fig, axes = plt.subplots(1, 2, figsize=(13.5, 5.0))

    hbar_panel(axes[0], ROWS, 100, 'empty_room.world coverage (%)',
              'Backend comparison — after the IMU fix')
    hbar_panel(axes[1], BEFORE_AFTER, 100, 'coverage (%)',
              'Before vs. after the IMU fix')

    fig.suptitle('IMU-never-publishes fix — the real bottleneck behind this session\'s ~45-49% coverage ceiling',
                 fontsize=13, fontweight='bold', x=0.01, ha='left', y=1.04, color='#10161f')
    fig.text(0.01, -0.08,
              'Root cause: /imu registered its topic but never emitted data on any world tested — a known, still-open '
              'upstream Gazebo bug (gazebosim/gz-sim#687) triggered by spawning robots dynamically via a ROS 2 launch '
              'file rather than baking them into the world SDF. ekf_filter_node had been running on wheel-odometry-only '
              'yaw the whole session, feeding slam_toolbox a noisy prior that forced large map_odom correction jumps. '
              'Fixed by declaring the ignition-gazebo-imu-system plugin explicitly in the world SDF. '
              'Source: EXPLORATION_TESTING_NOTES.md (repo-local, not pushed).',
              fontsize=8.0, color=MUTED, ha='left', wrap=True)
    fig.tight_layout(rect=[0, 0.08, 1, 0.92])
    fig.savefig(out_path, dpi=200, bbox_inches='tight')
    plt.close(fig)


if __name__ == '__main__':
    import sys
    out_dir = sys.argv[1] if len(sys.argv) > 1 else '.'
    out_path = f'{out_dir}/imu_fix_backend_coverage.png'
    make_fig(out_path)
    print('wrote', out_path)
