# Offline frontier-policy benchmark (sensor 3.5 m)

12 paired episodes per map (spawn seeds 1000..1011, identical across policies), `FrontierSelectEnv` k=8, max_steps=150, sensor 3.5 m. Cells are mean [95% bootstrap CI]. Steps-to-90% is censored at max_steps+1 for episodes that never reach 90% (reach rate in the last column). Path length = sum of straight-line hop lengths (env semantics), metres. Offline env: no Nav2, no path planning, no real failures; "failed cooldown" is emulated by blacklisting a goal whose hop revealed nothing.

## cafe

| policy | final coverage | steps to 90% | path to 90% (m) | total path (m) | reached 90% |
|---|---|---|---|---|---|
| nearest | 0.999 [0.999, 1.000] | 6.8 [6.2, 7.3] | 24 [22, 27] | 34 [31, 37] | 12/12 |
| max_gain | 0.998 [0.997, 1.000] | 5.2 [4.8, 5.5] | 19 [17, 20] | 42 [36, 47] | 12/12 |
| gain/dist | 0.998 [0.997, 1.000] | 5.2 [4.8, 5.5] | 18 [17, 20] | 42 [36, 48] | 12/12 |
| gain/dist(w=0.5) | 0.998 [0.997, 1.000] | 5.2 [4.8, 5.5] | 19 [17, 20] | 42 [36, 48] | 12/12 |
| gain/dist(w=2) | 0.998 [0.997, 1.000] | 5.2 [4.8, 5.5] | 18 [17, 20] | 42 [36, 48] | 12/12 |
| live | 1.000 [0.999, 1.000] | 6.8 [6.1, 7.4] | 23 [21, 26] | 34 [32, 37] | 12/12 |
| live-no-hyst | 1.000 [0.999, 1.000] | 6.8 [6.1, 7.4] | 23 [21, 26] | 34 [32, 37] | 12/12 |
| cost_aware | 0.998 [0.996, 1.000] | 5.4 [5.0, 5.8] | 20 [18, 22] | 40 [35, 46] | 12/12 |
| random | 1.000 [0.999, 1.000] | 7.1 [6.6, 7.5] | 28 [25, 30] | 41 [38, 44] | 12/12 |
| learned | 1.000 [1.000, 1.000] | 5.5 [5.0, 6.0] | 21 [18, 23] | 44 [41, 47] | 12/12 |

Paired difference vs `live` (policy - live; 95% CI), steps / path-to-90% (m) / coverage:

- nearest: steps +0.1 [-0.2, +0.5] ns; path90 +0.8 [-0.6, +2.3] ns; coverage -0.001 [-0.001, -0.000] SIG
- max_gain: steps -1.6 [-2.3, -0.8] SIG; path90 -4.7 [-7.5, -1.9] SIG; coverage -0.002 [-0.003, -0.000] SIG
- gain/dist: steps -1.6 [-2.3, -0.8] SIG; path90 -4.9 [-7.7, -2.1] SIG; coverage -0.001 [-0.003, +0.000] ns
- gain/dist(w=0.5): steps -1.6 [-2.3, -0.8] SIG; path90 -4.8 [-7.7, -2.0] SIG; coverage -0.001 [-0.003, +0.000] ns
- gain/dist(w=2): steps -1.6 [-2.3, -0.8] SIG; path90 -4.9 [-7.7, -2.1] SIG; coverage -0.001 [-0.003, +0.000] ns
- live-no-hyst: steps +0.0 [+0.0, +0.0] ns; path90 +0.0 [+0.0, +0.0] ns; coverage +0.000 [+0.000, +0.000] ns
- cost_aware: steps -1.3 [-2.2, -0.4] SIG; path90 -3.6 [-7.2, +0.2] ns; coverage -0.002 [-0.003, -0.000] SIG
- random: steps +0.3 [-0.7, +1.3] ns; path90 +4.3 [+0.4, +7.8] SIG; coverage -0.000 [-0.001, +0.000] ns
- learned: steps -1.2 [-2.1, -0.4] SIG; path90 -2.9 [-6.2, +0.4] ns; coverage +0.000 [-0.000, +0.000] ns

## hospital

| policy | final coverage | steps to 90% | path to 90% (m) | total path (m) | reached 90% |
|---|---|---|---|---|---|
| nearest | 1.000 [0.999, 1.000] | 26.8 [25.6, 28.2] | 99 [93, 106] | 145 [140, 150] | 12/12 |
| max_gain | 1.000 [1.000, 1.000] | 22.2 [21.7, 22.8] | 88 [84, 93] | 168 [153, 184] | 12/12 |
| gain/dist | 1.000 [1.000, 1.000] | 22.4 [21.7, 23.2] | 87 [83, 92] | 160 [146, 174] | 12/12 |
| gain/dist(w=0.5) | 1.000 [0.999, 1.000] | 22.3 [21.7, 23.1] | 87 [83, 92] | 160 [148, 172] | 12/12 |
| gain/dist(w=2) | 1.000 [1.000, 1.000] | 22.4 [21.7, 23.2] | 87 [83, 92] | 161 [148, 174] | 12/12 |
| live | 1.000 [0.999, 1.000] | 26.8 [25.8, 27.8] | 96 [92, 100] | 130 [122, 138] | 12/12 |
| live-no-hyst | 1.000 [0.999, 1.000] | 26.8 [25.8, 27.8] | 96 [92, 100] | 130 [122, 138] | 12/12 |
| cost_aware | 0.999 [0.999, 1.000] | 21.5 [20.8, 22.3] | 78 [74, 83] | 154 [139, 170] | 12/12 |
| random | 1.000 [0.999, 1.000] | 26.7 [25.9, 27.3] | 101 [97, 106] | 166 [157, 174] | 12/12 |
| learned | 1.000 [0.999, 1.000] | 21.8 [20.9, 22.5] | 81 [76, 86] | 156 [144, 169] | 12/12 |

Paired difference vs `live` (policy - live; 95% CI), steps / path-to-90% (m) / coverage:

- nearest: steps +0.1 [-0.8, +0.8] ns; path90 +3.3 [-1.7, +8.1] ns; coverage +0.000 [-0.000, +0.000] ns
- max_gain: steps -4.5 [-5.7, -3.2] SIG; path90 -8.0 [-13.6, -1.9] SIG; coverage +0.000 [-0.000, +0.001] ns
- gain/dist: steps -4.3 [-5.6, -3.0] SIG; path90 -8.7 [-14.5, -2.4] SIG; coverage +0.000 [-0.000, +0.001] ns
- gain/dist(w=0.5): steps -4.4 [-5.6, -3.2] SIG; path90 -8.8 [-14.5, -2.5] SIG; coverage +0.000 [-0.000, +0.001] ns
- gain/dist(w=2): steps -4.3 [-5.6, -3.0] SIG; path90 -8.8 [-14.5, -2.4] SIG; coverage +0.000 [-0.000, +0.001] ns
- live-no-hyst: steps +0.0 [+0.0, +0.0] ns; path90 +0.0 [+0.0, +0.0] ns; coverage +0.000 [+0.000, +0.000] ns
- cost_aware: steps -5.2 [-6.5, -4.2] SIG; path90 -17.6 [-22.7, -13.0] SIG; coverage -0.000 [-0.001, +0.000] ns
- random: steps -0.1 [-1.4, +1.4] ns; path90 +5.6 [+0.3, +11.8] SIG; coverage +0.000 [-0.000, +0.001] ns
- learned: steps -5.0 [-6.2, -3.5] SIG; path90 -14.9 [-20.1, -7.7] SIG; coverage -0.000 [-0.001, +0.001] ns

## maze

| policy | final coverage | steps to 90% | path to 90% (m) | total path (m) | reached 90% |
|---|---|---|---|---|---|
| nearest | 1.000 [0.999, 1.000] | 15.8 [15.2, 16.5] | 57 [54, 61] | 88 [83, 93] | 12/12 |
| max_gain | 1.000 [0.999, 1.000] | 12.7 [12.2, 13.1] | 48 [45, 52] | 104 [97, 112] | 12/12 |
| gain/dist | 1.000 [0.999, 1.000] | 12.8 [12.3, 13.2] | 49 [45, 54] | 102 [98, 105] | 12/12 |
| gain/dist(w=0.5) | 1.000 [0.999, 1.000] | 12.8 [12.3, 13.2] | 49 [45, 54] | 100 [97, 104] | 12/12 |
| gain/dist(w=2) | 1.000 [0.999, 1.000] | 12.8 [12.3, 13.2] | 49 [45, 54] | 103 [99, 106] | 12/12 |
| live | 0.999 [0.997, 1.000] | 15.9 [15.2, 16.8] | 59 [55, 63] | 81 [77, 85] | 12/12 |
| live-no-hyst | 0.999 [0.997, 1.000] | 15.9 [15.2, 16.8] | 59 [55, 63] | 81 [77, 85] | 12/12 |
| cost_aware | 1.000 [0.999, 1.000] | 12.4 [12.0, 12.8] | 44 [42, 46] | 96 [90, 103] | 12/12 |
| random | 1.000 [1.000, 1.000] | 16.1 [15.3, 16.9] | 62 [57, 66] | 97 [92, 102] | 12/12 |
| learned | 1.000 [0.999, 1.000] | 13.4 [12.8, 14.0] | 49 [45, 53] | 94 [87, 100] | 12/12 |

Paired difference vs `live` (policy - live; 95% CI), steps / path-to-90% (m) / coverage:

- nearest: steps -0.1 [-0.9, +0.8] ns; path90 -1.1 [-5.8, +3.3] ns; coverage +0.001 [-0.000, +0.002] ns
- max_gain: steps -3.2 [-4.1, -2.4] SIG; path90 -10.4 [-15.5, -5.2] SIG; coverage +0.001 [-0.000, +0.002] ns
- gain/dist: steps -3.2 [-4.1, -2.2] SIG; path90 -9.4 [-15.7, -2.5] SIG; coverage +0.001 [-0.000, +0.002] ns
- gain/dist(w=0.5): steps -3.2 [-4.1, -2.2] SIG; path90 -9.4 [-15.7, -2.5] SIG; coverage +0.001 [-0.000, +0.002] ns
- gain/dist(w=2): steps -3.2 [-4.1, -2.2] SIG; path90 -9.4 [-15.7, -2.5] SIG; coverage +0.001 [-0.000, +0.002] ns
- live-no-hyst: steps +0.0 [+0.0, +0.0] ns; path90 +0.0 [+0.0, +0.0] ns; coverage +0.000 [+0.000, +0.000] ns
- cost_aware: steps -3.5 [-4.4, -2.6] SIG; path90 -14.7 [-18.6, -10.3] SIG; coverage +0.001 [-0.000, +0.002] ns
- random: steps +0.2 [-0.8, +1.3] ns; path90 +2.9 [-3.9, +10.2] ns; coverage +0.001 [-0.000, +0.003] ns
- learned: steps -2.5 [-3.5, -1.4] SIG; path90 -9.7 [-15.0, -3.8] SIG; coverage +0.001 [-0.000, +0.002] ns

## obstacles

| policy | final coverage | steps to 90% | path to 90% (m) | total path (m) | reached 90% |
|---|---|---|---|---|---|
| nearest | 0.995 [0.987, 1.000] | 1.8 [1.4, 2.1] | 6 [5, 7] | 9 [8, 10] | 12/12 |
| max_gain | 0.997 [0.991, 1.000] | 1.4 [1.2, 1.7] | 5 [4, 6] | 9 [8, 10] | 12/12 |
| gain/dist | 0.997 [0.991, 1.000] | 1.4 [1.2, 1.7] | 5 [4, 6] | 9 [8, 10] | 12/12 |
| gain/dist(w=0.5) | 0.997 [0.991, 1.000] | 1.4 [1.2, 1.7] | 5 [4, 6] | 9 [8, 10] | 12/12 |
| gain/dist(w=2) | 0.997 [0.991, 1.000] | 1.4 [1.2, 1.7] | 5 [4, 6] | 9 [8, 10] | 12/12 |
| live | 0.996 [0.989, 1.000] | 1.8 [1.4, 2.1] | 6 [5, 7] | 9 [8, 10] | 12/12 |
| live-no-hyst | 0.996 [0.989, 1.000] | 1.8 [1.4, 2.1] | 6 [5, 7] | 9 [8, 10] | 12/12 |
| cost_aware | 0.997 [0.991, 1.000] | 1.4 [1.2, 1.7] | 5 [4, 6] | 9 [8, 10] | 12/12 |
| random | 0.996 [0.990, 1.000] | 1.7 [1.4, 1.9] | 6 [5, 7] | 10 [8, 11] | 12/12 |
| learned | 0.997 [0.991, 1.000] | 1.5 [1.2, 1.8] | 5 [4, 6] | 10 [8, 11] | 12/12 |

Paired difference vs `live` (policy - live; 95% CI), steps / path-to-90% (m) / coverage:

- nearest: steps +0.0 [+0.0, +0.0] ns; path90 -0.0 [-0.0, -0.0] SIG; coverage -0.001 [-0.004, +0.000] ns
- max_gain: steps -0.3 [-0.6, -0.1] SIG; path90 -1.1 [-2.0, -0.3] SIG; coverage +0.001 [-0.000, +0.003] ns
- gain/dist: steps -0.3 [-0.6, -0.1] SIG; path90 -1.1 [-2.0, -0.3] SIG; coverage +0.001 [-0.000, +0.003] ns
- gain/dist(w=0.5): steps -0.3 [-0.6, -0.1] SIG; path90 -1.1 [-2.0, -0.3] SIG; coverage +0.001 [-0.000, +0.003] ns
- gain/dist(w=2): steps -0.3 [-0.6, -0.1] SIG; path90 -1.1 [-2.0, -0.3] SIG; coverage +0.001 [-0.000, +0.003] ns
- live-no-hyst: steps +0.0 [+0.0, +0.0] ns; path90 +0.0 [+0.0, +0.0] ns; coverage +0.000 [+0.000, +0.000] ns
- cost_aware: steps -0.3 [-0.6, -0.1] SIG; path90 -1.1 [-2.0, -0.3] SIG; coverage +0.001 [-0.000, +0.003] ns
- random: steps -0.1 [-0.2, +0.0] ns; path90 -0.0 [-0.8, +0.5] ns; coverage +0.001 [-0.000, +0.002] ns
- learned: steps -0.2 [-0.5, +0.0] ns; path90 -0.9 [-1.7, +0.0] ns; coverage +0.001 [+0.000, +0.003] ns
