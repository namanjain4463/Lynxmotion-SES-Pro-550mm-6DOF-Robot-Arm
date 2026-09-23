# Arm characterization 2026-09-22 19:47

Command: `arm_characterize.py ramp 6 5 3 0` (forward_position_controller, unloaded)

## Moving target (constant velocity, streamed at 30 Hz)

| joint | direction | target speed (deg/s) | lead (s) | lag (deg) | lag (s) | ripple std (deg) | ripple p2p (deg) | measured speed (deg/s) | overshoot at stop (deg) |
|---|---|---|---|---|---|---|---|---|---|
| 6 | out | 5.0 | 0.00 | 0.95 | 0.19 | 0.067 | 0.234 | 4.88 ± 0.78 | 0.01 |
| 6 | back | 5.0 | 0.00 | 0.87 | 0.17 | 0.090 | 0.359 | -5.00 ± 1.18 | 0.01 |
## Control-loop timing (joint_states stamps, whole run)

rate_hz 8.37, dt_mean_ms 119.54, dt_std_ms 7.88, dt_p99_ms 132.96, dt_max_ms 135.15, freezes_over_300ms 0.00

Notes: positions are the servos' own readings (no external reference); time resolution is one control-loop period (see rate above).
