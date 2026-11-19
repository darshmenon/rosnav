# Social-nav adversary results

6 random crowds vs top-5 CEM-adversarial crowds per planner (6 peds, speed 0.3-1.5 m/s, 15 s episodes, 2 noise seeds each, held-out from the search seeds). Near-miss = surface clearance < 0.2 m. CEM pop=12, iters=4.

| planner | scenarios | collision | near-miss | intrusion s | success | stalled s |
|---|---|---|---|---|---|---|
| reactive | random | 0.50 | 0.67 | 3.7 | 0.33 | 2.9 |
| reactive | adversarial(own) | 1.00 | 1.00 | 8.2 | 0.00 | 6.7 |
| predictive | random | 0.08 | 0.17 | 1.7 | 0.00 | 0.3 |
| predictive | adversarial(own) | 0.30 | 0.40 | 2.4 | 0.00 | 0.5 |
| risk | random | 0.00 | 0.00 | 0.4 | 0.08 | 0.0 |
| risk | adversarial(own) | 0.00 | 0.00 | 0.6 | 0.00 | 0.0 |
| conf-risk | random | 0.00 | 0.00 | 0.3 | 0.08 | 0.0 |
| conf-risk | adversarial(own) | 0.10 | 0.20 | 1.5 | 0.00 | 0.1 |

## Transfer: rows = planner the crowd was tuned against, columns = planner run (near-miss rate / intrusion s)

| tuned against | reactive | predictive | risk | conf-risk |
|---|---|---|---|---|
| reactive | 1.00 / 8.7 | 0.00 / 3.4 | 0.00 / 0.5 | 0.00 / 0.7 |
| predictive | 1.00 / 5.2 | 0.40 / 2.3 | 0.00 / 0.3 | 0.00 / 0.1 |
| risk | 1.00 / 4.3 | 0.20 / 2.1 | 0.00 / 0.5 | 0.00 / 0.6 |
| conf-risk | 1.00 / 4.7 | 0.40 / 2.8 | 0.00 / 0.7 | 0.20 / 1.5 |

Atlas images: `atlas_<planner>.png` (left random, right adversarial). Caveat: crowds are constant-velocity bouncing walkers and some adversarial layouts may be near-unavoidable; the conformal budget is judged against the fixed-sigma `risk` planner, see the numbers above rather than a claim here.
