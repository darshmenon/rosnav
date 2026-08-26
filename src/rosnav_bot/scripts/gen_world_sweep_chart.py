#!/usr/bin/env python3
"""Regenerate images/world_sweep_coverage.png for the README from the
2026-08-26 full-repo mapping sweep (sweep_logs/batch_map_all_worlds.sh,
explorer:=explore_lite, post kill_sim/launch-detection fixes — see
EXPLORATION_TESTING_NOTES.md, repo-local, not committed).

Not a ROS node — a doc-generation tool, run manually:
    python3 gen_world_sweep_chart.py ../../../images

Hardcodes that sweep's numbers; update by hand (or extend ROWS) once the
8 worlds that failed to launch (bt_navigator lifecycle hang, suspected
cumulative session resource exhaustion — not yet root-caused) are retried.
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
WARN = '#eda100'
BAD = '#c0392b'
GRID = '#eef1f5'
MUTED = '#8892a0'

# name, coverage %, color, note
ROWS = [
    ('bench_room_small', 69.4, GOOD, 'clean run, not stuck'),
    ('bench_room_cluttered', 53.0, GOOD, 'clean run, not stuck'),
    ('corridor', 15.8, WARN, 'reproducible low ceiling —\nnot yet root-caused'),
    ('coverage_100', 0.0, BAD, 'robot never moved —\nexplore_lite issued zero goals'),
    ('simple_rooms', 0.0, BAD, 'same symptom as coverage_100'),
]

PENDING = [
    'house', 'office', 'warehouse', 'aws_warehouse', 'tugbot_warehouse',
    'warehouse_depot', 'lake_house', 'multi_terrain',
]


def make_fig(out_path):
    fig, ax = plt.subplots(figsize=(9.5, 5.2))

    names = [r[0] for r in ROWS][::-1]
    values = [r[1] for r in ROWS][::-1]
    colors = [r[2] for r in ROWS][::-1]
    notes = [r[3] for r in ROWS][::-1]

    # 0%-coverage rows render as an invisible zero-width bar otherwise — give
    # them a small visible stub so the label isn't floating unanchored; the
    # printed value still shows the real (unrounded) number.
    plot_values = [max(v, 1.2) for v in values]

    y = range(len(ROWS))
    bars = ax.barh(y, plot_values, color=colors, height=0.55, zorder=3)
    ax.set_yticks(list(y))
    ax.set_yticklabels(names, fontsize=10.5, fontweight='medium')
    ax.set_ylim(-0.75, len(ROWS) - 0.25)
    ax.set_xlim(0, 100)
    ax.set_xlabel('map coverage (%)', fontsize=9.5)
    ax.set_title('Multi-world mapping sweep — explore_lite, one pass per world',
                  fontsize=13, fontweight='bold', pad=14, loc='left', color='#10161f')
    ax.grid(axis='x', color=GRID, linewidth=1, zorder=0)
    ax.set_axisbelow(True)
    for spine in ('top', 'right', 'left'):
        ax.spines[spine].set_visible(False)
    ax.tick_params(left=False)

    for bar, val, note in zip(bars, values, notes):
        w = bar.get_width()
        ax.text(w + 1.5, bar.get_y() + bar.get_height() / 2, f'{val:g}%',
                va='center', ha='left', fontsize=10.5, fontweight='bold', color='#10161f')
        ax.text(1.5, bar.get_y() - 0.14, note,
                va='top', ha='left', fontsize=7.6, color=MUTED, style='italic')

    fig.text(0.01, -0.06,
              f'{len(PENDING)} more worlds ({", ".join(PENDING)}) failed to launch this pass '
              '(bt_navigator lifecycle hang after ~10+ hrs of repeated launch/teardown cycles in '
              'the same session — suspected cumulative resource exhaustion, not yet confirmed '
              'world-specific) and are pending a fresh retry. corridor/coverage_100/simple_rooms '
              'findings are real, reproducible anomalies under active investigation, not sweep '
              'artifacts. Source: EXPLORATION_TESTING_NOTES.md (repo-local, not committed).',
              fontsize=7.6, color=MUTED, ha='left', wrap=True)

    fig.tight_layout(rect=[0, 0.09, 1, 1])
    fig.savefig(out_path, dpi=200, bbox_inches='tight')
    plt.close(fig)


if __name__ == '__main__':
    import sys
    out_dir = sys.argv[1] if len(sys.argv) > 1 else '.'
    out_path = f'{out_dir}/world_sweep_coverage.png'
    make_fig(out_path)
    print('wrote', out_path)
