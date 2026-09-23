# Arm characterization 2026-09-22 18:00

Command: `arm_characterize.py all` (forward_position_controller, unloaded)

## Standstill noise (joint_states, holding still)

174 samples over 20 s.

| joint | std (deg) | peak-to-peak (deg) | distinct readings | reported speed std (deg/s) |
|---|---|---|---|---|
| 1 | 0.004 | 0.030 | 4 | 0.002 |
| 2 | 0.004 | 0.030 | 4 | 0.018 |
| 3 | 0.005 | 0.020 | 3 | 0.000 |
| 4 | 0.005 | 0.010 | 2 | 0.000 |
| 5 | 0.005 | 0.020 | 3 | 0.000 |
| 6 | 0.000 | 0.000 | 1 | 0.000 |

## Step response (per joint, both directions)

| joint | step (deg) | delay (s) | rise 10-90% (s) | overshoot (deg) | settle to 0.3 deg (s) | abs steady-state error (deg) | backlash / hysteresis (deg) |
|---|---|---|---|---|---|---|---|
| 1 | 5 | 0.30 ± 0.05 | 0.42 ± 0.06 | 0.03 ± 0.02 | 0.89 ± 0.06 | 0.012 ± 0.008 | 0.015 ± 0.016 |
| 2 | 5 | 0.29 ± 0.04 | 0.42 ± 0.05 | 0.01 ± 0.01 | 0.87 ± 0.05 | 0.012 ± 0.006 | -0.002 ± 0.019 |
| 3 | 5 | 0.27 ± 0.03 | 0.39 ± 0.05 | 0.01 ± 0.01 | 0.83 ± 0.04 | 0.009 ± 0.007 | -0.003 ± 0.007 |
| 4 | 5 | 0.30 ± 0.05 | 0.34 ± 0.04 | 0.01 ± 0.01 | 0.68 ± 0.05 | 0.008 ± 0.009 | -0.009 ± 0.008 |
| 5 | 5 | 0.16 ± 0.03 | 0.23 ± 0.02 | 0.02 ± 0.01 | 0.50 ± 0.03 | 0.011 ± 0.010 | -0.022 ± 0.006 |
| 6 | 5 | 0.17 ± 0.04 | 0.19 ± 0.05 | 0.01 ± 0.01 | 0.51 ± 0.04 | 0.002 ± 0.002 | -0.000 ± 0.001 |

## Speed and acceleration (large steps)

| joint | step (deg) | peak speed (deg/s) | acceleration (deg/s²) | delay (s) |
|---|---|---|---|---|
| 1 | 15 | 22.5 ± 1.1 | 41 ± 5 | 0.30 ± 0.04 |
| 2 | 15 | 21.4 ± 0.9 | 41 ± 6 | 0.28 ± 0.03 |
| 3 | 15 | 22.0 ± 0.5 | 42 ± 4 | 0.28 ± 0.04 |
| 4 | 15 | 27.7 ± 1.4 | 78 ± 7 | 0.29 ± 0.03 |
| 5 | 15 | 37.2 ± 1.7 | 109 ± 9 | 0.19 ± 0.03 |
| 6 | 15 | 38.9 ± 2.4 | 123 ± 17 | 0.15 ± 0.02 |

## Moving target (constant velocity, streamed at 30 Hz)

| joint | direction | target speed (deg/s) | lag (deg) | lag (s) | ripple std (deg) | ripple p2p (deg) | measured speed (deg/s) | overshoot at stop (deg) |
|---|---|---|---|---|---|---|---|---|
| 1 | out | 5.0 | 1.54 | 0.31 | 0.099 | 0.359 | 4.96 ± 0.89 | 0.01 |
| 1 | back | 5.0 | 1.67 | 0.33 | 0.121 | 0.401 | -5.09 ± 1.18 | 0.02 |
| 2 | out | 5.0 | 1.56 | 0.31 | 0.051 | 0.193 | 4.97 ± 0.33 | 0.00 |
| 2 | back | 5.0 | 1.43 | 0.29 | 0.052 | 0.199 | -5.03 ± 0.46 | 0.02 |
| 3 | out | 5.0 | 1.29 | 0.26 | 0.061 | 0.209 | 4.79 ± 0.47 | 0.02 |
| 3 | back | 5.0 | 1.26 | 0.25 | 0.052 | 0.173 | -5.00 ± 0.50 | 0.01 |
| 4 | out | 5.0 | 1.10 | 0.22 | 0.046 | 0.169 | 4.85 ± 0.51 | 0.01 |
| 4 | back | 5.0 | 1.08 | 0.22 | 0.049 | 0.180 | -4.90 ± 0.51 | 0.02 |
| 5 | out | 5.0 | 0.85 | 0.17 | 0.118 | 0.532 | 4.85 ± 1.72 | 0.04 |
| 5 | back | 5.0 | 0.96 | 0.19 | 0.141 | 0.544 | -4.85 ± 1.58 | 0.01 |
| 6 | out | 5.0 | 0.88 | 0.18 | 0.157 | 0.664 | 4.92 ± 1.69 | 0.02 |
| 6 | back | 5.0 | 0.89 | 0.18 | 0.126 | 0.486 | -4.85 ± 1.56 | 0.11 |

## Control-loop timing (joint_states stamps, whole run)

rate_hz 7.69, dt_mean_ms 130.09, dt_std_ms 334.69, dt_p99_ms 140.58, dt_max_ms 19770.74, freezes_over_300ms 23.00

Notes: positions are the servos' own readings (no external reference); time resolution is one control-loop period (see rate above).
