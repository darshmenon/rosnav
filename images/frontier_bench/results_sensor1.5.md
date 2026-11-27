# Offline frontier-policy benchmark (sensor 1.5 m)

12 paired episodes per map (spawn seeds 1000..1011, identical across policies), `FrontierSelectEnv` k=8, max_steps=150, sensor 1.5 m. Cells are mean [95% bootstrap CI]. Steps-to-90% is censored at max_steps+1 for episodes that never reach 90% (reach rate in the last column). Path length = sum of straight-line hop lengths (env semantics), metres. Offline env: no Nav2, no path planning, no real failures; "failed cooldown" is emulated by blacklisting a goal whose hop revealed nothing.

## cafe

| policy | final coverage | steps to 90% | path to 90% (m) | total path (m) | reached 90% |
|---|---|---|---|---|---|
| nearest | 1.000 [1.000, 1.000] | 38.8 [38.1, 39.5] | 58 [56, 60] | 84 [79, 88] | 12/12 |
| max_gain | 0.999 [0.998, 1.000] | 33.9 [33.2, 34.8] | 60 [58, 63] | 135 [129, 142] | 12/12 |
| gain/dist | 1.000 [0.999, 1.000] | 34.7 [33.8, 35.5] | 60 [57, 63] | 121 [118, 124] | 12/12 |
| gain/dist(w=0.5) | 0.999 [0.998, 1.000] | 34.1 [33.4, 34.9] | 59 [57, 62] | 124 [119, 128] | 12/12 |
| gain/dist(w=2) | 0.999 [0.999, 1.000] | 35.0 [34.2, 35.8] | 59 [57, 61] | 121 [117, 126] | 12/12 |
| live | 1.000 [0.999, 1.000] | 38.0 [37.2, 38.8] | 57 [55, 60] | 77 [75, 80] | 12/12 |
| live-no-hyst | 1.000 [0.999, 1.000] | 38.0 [37.2, 38.8] | 57 [55, 60] | 77 [75, 80] | 12/12 |
| cost_aware | 0.999 [0.999, 1.000] | 34.6 [33.4, 35.5] | 58 [55, 60] | 117 [113, 121] | 12/12 |
| random | 1.000 [0.999, 1.000] | 37.7 [36.7, 38.7] | 65 [60, 69] | 107 [102, 113] | 12/12 |
| learned | 0.999 [0.998, 1.000] | 33.1 [32.2, 33.9] | 57 [54, 60] | 129 [125, 133] | 12/12 |

Paired difference vs `live` (policy - live; 95% CI), steps / path-to-90% (m) / coverage:

- nearest: steps +0.8 [+0.0, +1.6] ns; path90 +1.0 [-1.3, +3.5] ns; coverage +0.000 [-0.000, +0.000] ns
- max_gain: steps -4.1 [-4.9, -3.1] SIG; path90 +3.2 [-0.0, +6.8] ns; coverage -0.001 [-0.001, -0.000] SIG
- gain/dist: steps -3.3 [-4.4, -2.2] SIG; path90 +2.9 [-0.3, +6.4] ns; coverage -0.000 [-0.000, +0.000] ns
- gain/dist(w=0.5): steps -3.9 [-4.8, -3.0] SIG; path90 +2.3 [-0.9, +5.9] ns; coverage -0.001 [-0.001, -0.000] SIG
- gain/dist(w=2): steps -3.0 [-4.0, -2.0] SIG; path90 +2.1 [-0.7, +5.1] ns; coverage -0.000 [-0.001, +0.000] ns
- live-no-hyst: steps +0.0 [+0.0, +0.0] ns; path90 +0.0 [+0.0, +0.0] ns; coverage +0.000 [+0.000, +0.000] ns
- cost_aware: steps -3.4 [-4.3, -2.3] SIG; path90 +0.8 [-2.2, +3.8] ns; coverage -0.000 [-0.001, +0.000] ns
- random: steps -0.3 [-1.8, +1.0] ns; path90 +7.8 [+2.0, +13.4] SIG; coverage -0.000 [-0.000, +0.000] ns
- learned: steps -4.9 [-6.2, -3.8] SIG; path90 +0.0 [-4.7, +4.2] ns; coverage -0.001 [-0.002, +0.000] ns

## hospital

| policy | final coverage | steps to 90% | path to 90% (m) | total path (m) | reached 90% |
|---|---|---|---|---|---|
| nearest | 0.914 [0.904, 0.924] | 147.0 [145.4, 148.5] | 222 [218, 227] | 228 [224, 231] | 10/12 |
| max_gain | 0.978 [0.974, 0.981] | 126.6 [124.8, 128.5] | 214 [208, 220] | 278 [274, 282] | 12/12 |
| gain/dist | 0.972 [0.967, 0.977] | 130.2 [128.8, 131.7] | 218 [213, 223] | 268 [264, 273] | 12/12 |
| gain/dist(w=0.5) | 0.969 [0.963, 0.974] | 129.3 [127.3, 131.5] | 218 [211, 225] | 271 [267, 275] | 12/12 |
| gain/dist(w=2) | 0.964 [0.955, 0.971] | 132.1 [129.7, 134.4] | 216 [209, 223] | 255 [252, 257] | 12/12 |
| live | 0.926 [0.916, 0.936] | 145.2 [143.4, 146.9] | 215 [212, 219] | 223 [222, 225] | 11/12 |
| live-no-hyst | 0.926 [0.916, 0.936] | 145.2 [143.4, 146.9] | 215 [212, 219] | 223 [222, 225] | 11/12 |
| cost_aware | 0.964 [0.957, 0.970] | 130.8 [127.8, 133.8] | 214 [207, 221] | 255 [251, 258] | 12/12 |
| random | 0.936 [0.930, 0.943] | 142.8 [141.1, 144.5] | 245 [240, 251] | 264 [259, 270] | 12/12 |
| learned | 0.975 [0.971, 0.979] | 129.2 [126.6, 131.8] | 221 [211, 232] | 280 [275, 285] | 12/12 |

Paired difference vs `live` (policy - live; 95% CI), steps / path-to-90% (m) / coverage:

- nearest: steps +1.8 [-0.6, +4.1] ns; path90 +7.1 [+2.0, +12.2] SIG; coverage -0.012 [-0.023, -0.000] SIG
- max_gain: steps -18.6 [-21.1, -16.0] SIG; path90 -1.7 [-8.0, +5.7] ns; coverage +0.051 [+0.040, +0.063] SIG
- gain/dist: steps -14.9 [-17.8, -11.9] SIG; path90 +3.1 [-4.6, +10.9] ns; coverage +0.046 [+0.034, +0.059] SIG
- gain/dist(w=0.5): steps -15.8 [-19.3, -12.2] SIG; path90 +2.4 [-6.9, +12.2] ns; coverage +0.042 [+0.030, +0.054] SIG
- gain/dist(w=2): steps -13.1 [-15.7, -10.5] SIG; path90 +0.7 [-6.5, +8.7] ns; coverage +0.037 [+0.028, +0.047] SIG
- live-no-hyst: steps +0.0 [+0.0, +0.0] ns; path90 +0.0 [+0.0, +0.0] ns; coverage +0.000 [+0.000, +0.000] ns
- cost_aware: steps -14.3 [-17.9, -11.0] SIG; path90 -1.3 [-9.8, +6.6] ns; coverage +0.038 [+0.027, +0.049] SIG
- random: steps -2.4 [-4.8, +0.1] ns; path90 +30.2 [+23.9, +37.1] SIG; coverage +0.010 [-0.001, +0.021] ns
- learned: steps -16.0 [-19.0, -13.0] SIG; path90 +5.6 [-4.8, +16.2] ns; coverage +0.049 [+0.038, +0.060] SIG

## maze

| policy | final coverage | steps to 90% | path to 90% (m) | total path (m) | reached 90% |
|---|---|---|---|---|---|
| nearest | 1.000 [1.000, 1.000] | 90.3 [88.7, 92.0] | 139 [133, 146] | 195 [189, 201] | 12/12 |
| max_gain | 1.000 [0.999, 1.000] | 76.4 [75.5, 77.2] | 131 [128, 134] | 276 [269, 283] | 12/12 |
| gain/dist | 1.000 [0.999, 1.000] | 77.6 [76.3, 78.8] | 130 [126, 133] | 251 [242, 259] | 12/12 |
| gain/dist(w=0.5) | 1.000 [0.999, 1.000] | 77.2 [76.0, 78.5] | 132 [128, 136] | 262 [255, 271] | 12/12 |
| gain/dist(w=2) | 0.999 [0.999, 1.000] | 78.5 [77.0, 79.9] | 130 [126, 135] | 246 [239, 253] | 12/12 |
| live | 1.000 [1.000, 1.000] | 88.0 [86.7, 89.6] | 132 [129, 137] | 174 [168, 179] | 12/12 |
| live-no-hyst | 1.000 [1.000, 1.000] | 88.0 [86.7, 89.6] | 132 [129, 137] | 174 [168, 179] | 12/12 |
| cost_aware | 1.000 [1.000, 1.000] | 77.2 [76.1, 78.6] | 128 [124, 132] | 252 [246, 257] | 12/12 |
| random | 1.000 [1.000, 1.000] | 86.1 [85.2, 87.0] | 146 [142, 149] | 237 [228, 247] | 12/12 |
| learned | 1.000 [1.000, 1.000] | 76.2 [74.6, 78.2] | 130 [123, 137] | 261 [254, 267] | 12/12 |

Paired difference vs `live` (policy - live; 95% CI), steps / path-to-90% (m) / coverage:

- nearest: steps +2.3 [-0.2, +4.8] ns; path90 +6.5 [-1.9, +14.1] ns; coverage -0.000 [-0.000, +0.000] ns
- max_gain: steps -11.6 [-13.3, -9.8] SIG; path90 -1.8 [-7.3, +4.1] ns; coverage -0.000 [-0.001, -0.000] SIG
- gain/dist: steps -10.4 [-12.2, -8.7] SIG; path90 -2.6 [-8.3, +2.8] ns; coverage -0.000 [-0.001, -0.000] SIG
- gain/dist(w=0.5): steps -10.8 [-12.5, -9.1] SIG; path90 -0.4 [-6.3, +5.3] ns; coverage -0.000 [-0.001, -0.000] SIG
- gain/dist(w=2): steps -9.5 [-11.6, -7.2] SIG; path90 -2.1 [-7.5, +3.8] ns; coverage -0.000 [-0.001, -0.000] SIG
- live-no-hyst: steps +0.0 [+0.0, +0.0] ns; path90 +0.0 [+0.0, +0.0] ns; coverage +0.000 [+0.000, +0.000] ns
- cost_aware: steps -10.8 [-12.8, -8.7] SIG; path90 -4.8 [-12.3, +1.8] ns; coverage -0.000 [-0.000, +0.000] ns
- random: steps -1.9 [-3.9, +0.0] ns; path90 +13.2 [+6.5, +18.5] SIG; coverage -0.000 [-0.000, +0.000] ns
- learned: steps -11.8 [-14.1, -9.3] SIG; path90 -2.8 [-10.9, +5.2] ns; coverage -0.000 [-0.000, -0.000] SIG

## obstacles

| policy | final coverage | steps to 90% | path to 90% (m) | total path (m) | reached 90% |
|---|---|---|---|---|---|
| nearest | 0.999 [0.999, 1.000] | 11.9 [11.1, 12.7] | 18 [17, 19] | 29 [26, 31] | 12/12 |
| max_gain | 1.000 [1.000, 1.000] | 10.4 [10.2, 10.7] | 18 [17, 18] | 41 [38, 45] | 12/12 |
| gain/dist | 1.000 [1.000, 1.000] | 10.6 [10.3, 10.8] | 17 [17, 18] | 37 [35, 39] | 12/12 |
| gain/dist(w=0.5) | 1.000 [1.000, 1.000] | 10.5 [10.2, 10.8] | 17 [17, 18] | 39 [37, 41] | 12/12 |
| gain/dist(w=2) | 1.000 [1.000, 1.000] | 10.5 [10.2, 10.8] | 17 [17, 18] | 37 [34, 40] | 12/12 |
| live | 1.000 [0.999, 1.000] | 11.5 [10.9, 12.1] | 17 [16, 18] | 27 [24, 29] | 12/12 |
| live-no-hyst | 1.000 [0.999, 1.000] | 11.5 [10.9, 12.1] | 17 [16, 18] | 27 [24, 29] | 12/12 |
| cost_aware | 1.000 [0.999, 1.000] | 10.9 [10.5, 11.3] | 18 [17, 18] | 35 [32, 37] | 12/12 |
| random | 0.999 [0.998, 1.000] | 11.4 [10.8, 12.1] | 19 [18, 21] | 31 [28, 34] | 12/12 |
| learned | 1.000 [1.000, 1.000] | 10.4 [9.8, 10.9] | 19 [17, 20] | 42 [38, 45] | 12/12 |

Paired difference vs `live` (policy - live; 95% CI), steps / path-to-90% (m) / coverage:

- nearest: steps +0.4 [+0.0, +0.9] ns; path90 +0.7 [+0.1, +1.3] SIG; coverage -0.000 [-0.001, +0.000] ns
- max_gain: steps -1.1 [-1.7, -0.6] SIG; path90 +0.5 [-0.4, +1.5] ns; coverage +0.000 [-0.000, +0.000] ns
- gain/dist: steps -0.9 [-1.5, -0.3] SIG; path90 +0.2 [-0.9, +1.3] ns; coverage +0.000 [-0.000, +0.000] ns
- gain/dist(w=0.5): steps -1.0 [-1.6, -0.4] SIG; path90 +0.2 [-0.8, +1.4] ns; coverage +0.000 [-0.000, +0.000] ns
- gain/dist(w=2): steps -1.0 [-1.6, -0.5] SIG; path90 -0.1 [-1.1, +0.9] ns; coverage +0.000 [-0.000, +0.000] ns
- live-no-hyst: steps +0.0 [+0.0, +0.0] ns; path90 +0.0 [+0.0, +0.0] ns; coverage +0.000 [+0.000, +0.000] ns
- cost_aware: steps -0.6 [-1.2, +0.0] ns; path90 +0.4 [-0.5, +1.4] ns; coverage -0.000 [-0.001, +0.000] ns
- random: steps -0.1 [-0.9, +0.8] ns; path90 +2.3 [+0.6, +4.1] SIG; coverage -0.001 [-0.002, -0.000] SIG
- learned: steps -1.1 [-1.9, -0.2] SIG; path90 +1.4 [-0.7, +3.4] ns; coverage -0.000 [-0.000, +0.000] ns
