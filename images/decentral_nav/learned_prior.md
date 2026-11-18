Learned MPPI prior v2 (MLP 28-64-64-2). Expert = best-of-4 mppi_prior samples + vo + pref (+yield side) chosen by 2 s lookahead cost; features include yield flag/side + 4-dir wall clearance; DAgger 3 rounds (dataset sizes []); train seeds 100-107; eval n=8, held-out seeds 0-9, max_t=60s

| scenario | controller | success % | makespan s | delay s | collision pairs |
|---|---|---|---|---|---|
| swap | vo | 100 | 12.5 | 0.7 | 0.00 |
| swap | mppi_prior | 100 | 16.7 | 4.7 | 0.00 |
| swap | mppi_learned | 100 | 20.2 | 7.6 | 0.00 |
| swap | mppi_learned_lo | 100 | 17.1 | 4.5 | 0.00 |
| swap | mppi_learned_yield | 100 | 17.1 | 4.5 | 0.10 |
| maze | vo | 90 | 39.4 | 0.5 | 0.00 |
| maze | mppi_prior | 70 | 43.2 | 5.4 | 0.00 |
| maze | mppi_learned | 70 | 46.9 | 7.4 | 0.50 |
| maze | mppi_learned_lo | 60 | 38.8 | 4.2 | 0.40 |
| maze | mppi_learned_yield | 60 | 38.8 | 4.2 | 0.40 |

Previous version (v1, MLP 21-64-64-2 imitating mppi_prior output, 43625 samples, no yield flag), same eval:

| scenario | controller | success % | makespan s | delay s | collision pairs |
|---|---|---|---|---|---|
| swap | mppi_learned (v1) | 100 | 22.4 | 9.1 | 2.00 |
| maze | mppi_learned (v1) | 50 | 48.0 | 10.1 | 0.20 |
