# Decentralized vs centralized MAPF

24x24 grid, obstacle density 0.12, 8 seeds per n, 1 cell = 1 m, 1 m/s. Centralized (PP, ECBS w=1.5) plans are conflict-free and executed lock-step (time = plan length; planning time excluded, 20s budget). Decentralized (vo, mppi_prior) run in the continuous simulator (r=0.3 m, goal tol 0.35 m, 90s limit) from the same obstacles/starts/goals. Makespan/SoC averaged over successful runs only; decentral SoC is a proxy (ideal + n*mean delay).

| n | method | success | makespan s | SoC s | collisions/run | runs w/ collision |
|---|---|---|---|---|---|---|
| 4 | pp | 100% | 26.5 | 75.2 | 0.00 | 0% |
| 4 | ecbs | 100% | 27.2 | 74.5 | 0.00 | 0% |
| 4 | vo | 88% | 21.8 | 57.6 | 0.00 | 0% |
| 4 | mppi_prior | 75% | 29.0 | 78.6 | 0.00 | 0% |
| 8 | pp | 100% | 32.1 | 151.9 | 0.00 | 0% |
| 8 | ecbs | 100% | 32.1 | 149.0 | 0.00 | 0% |
| 8 | vo | 88% | 26.8 | 120.7 | 0.00 | 0% |
| 8 | mppi_prior | 88% | 36.9 | 163.6 | 0.00 | 0% |
| 12 | pp | 100% | 32.0 | 232.5 | 0.00 | 0% |
| 12 | ecbs | 100% | 32.4 | 225.5 | 0.00 | 0% |
| 12 | vo | 75% | 26.5 | 188.8 | 0.00 | 0% |
| 12 | mppi_prior | 75% | 35.4 | 251.0 | 0.00 | 0% |

## Paired makespan ratio vs ECBS (instances both solved)

| n | method | ratio (median) | paired runs |
|---|---|---|---|
| 4 | vo | 0.85 | 7 |
| 4 | mppi_prior | 1.12 | 6 |
| 8 | vo | 0.85 | 7 |
| 8 | mppi_prior | 1.14 | 7 |
| 12 | vo | 0.82 | 6 |
| 12 | mppi_prior | 1.11 | 6 |

## Conclusions (honest)

- Centralized PP/ECBS: 100% success, zero collisions, at every n, by construction. Decentralized fails 12-25% of runs (deadlock/timeouts in the 1-cell-wide gaps; zero collisions because the sim blocks overlapping moves, so failure shows up as stalling, not crashing).
- Caveat on makespan: the decentralized robots move continuously and diagonally; the grid planners are limited to 4-connected unit steps. So vo's ~15-18% shorter makespan is mostly a motion-model advantage, not coordination quality. Do not read it as "decentralized beats CBS".
- mppi_prior is ~11-14% slower than ECBS on paired successes and is no more reliable than plain vo here; the extra sampling does not pay off on this sparse random grid.
- Centralized needs global knowledge, a shared planner and a lock-step executor; planning time was excluded (ECBS/PP finished in ~0 s at these sizes, so it is not the bottleneck yet). Decentralized wins only on deployability (no communication, no replanning on delay or noise); it loses on guarantees and success rate. Expect the gap to widen in narrow corridors and swap-heavy scenarios and to narrow under execution noise/delays, which lock-step plans do not tolerate (not tested here).
- Small sample (8 seeds/n, failures are counts of 1-3 runs): trends, not significance.
