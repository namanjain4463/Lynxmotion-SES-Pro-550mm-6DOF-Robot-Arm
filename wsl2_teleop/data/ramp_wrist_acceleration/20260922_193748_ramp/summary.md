# Arm characterization 2026-09-22 19:37

Command: `arm_characterize.py ramp 6 5 3 0` (forward_position_controller, unloaded)

## Moving target (constant velocity, streamed at 30 Hz)

| joint | direction | target speed (deg/s) | lead (s) | lag (deg) | lag (s) | ripple std (deg) | ripple p2p (deg) | measured speed (deg/s) | overshoot at stop (deg) |
|---|---|---|---|---|---|---|---|---|---|
| 6 | out | 5.0 | 0.00 | 0.89 | 0.18 | 0.153 | 0.701 | 4.87 ± 1.54 | 0.00 |
| 6 | back | 5.0 | 0.00 | 0.79 | 0.16 | 0.091 | 0.338 | -5.10 ± 1.38 | 0.06 |
## Control-loop timing (joint_states stamps, whole run)

rate_hz 7.09, dt_mean_ms 141.10, dt_std_ms 251.83, dt_p99_ms 256.60, dt_max_ms 2593.73, freezes_over_300ms 1.00

Notes: positions are the servos' own readings (no external reference); time resolution is one control-loop period (see rate above).
