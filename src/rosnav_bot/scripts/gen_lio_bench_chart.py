#!/usr/bin/env python3
"""Chart the LIO benchmark (benchmark_kiss_icp.sh PATTERN=weave) from its result.json -> PNG (RMSE, log scale)."""
import json
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

src, out = sys.argv[1], sys.argv[2]
d = json.load(open(src))
names = [('kiss_icp', 'KISS-ICP'), ('rko_lio', 'RKO-LIO'), ('lio_sam', 'LIO-SAM'), ('ekf', 'EKF'), ('wheel_odom', 'wheel odom')]
vals = [d['estimators'][k]['rmse_m'] for k, _ in names]
fig, ax = plt.subplots(figsize=(6.4, 3.4), dpi=150)
bars = ax.bar([n for _, n in names], vals, color=['#2a9d8f', '#2a9d8f', '#e76f51', '#8d99ae', '#8d99ae'])
ax.set_yscale('log')
ax.set_ylabel('position RMSE vs ground truth (m, log)')
ax.set_title(f"Cafe weave, {d['duration_s']:.0f} s, {d['ground_truth_path_m']:.1f} m true path", fontsize=10)
for b, v in zip(bars, vals):
    ax.text(b.get_x() + b.get_width() / 2, v * 1.08, f'{v:.2f}', ha='center', fontsize=8)
ax.spines[['top', 'right']].set_visible(False)
fig.tight_layout()
fig.savefig(out)
