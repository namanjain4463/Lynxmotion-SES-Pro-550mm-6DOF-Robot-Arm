# Arm characterization 2026-09-22 19:37

Command: `arm_characterize.py ramp 5 5 3 0` (forward_position_controller, unloaded)

## Moving target (constant velocity, streamed at 30 Hz)

| joint | direction | target speed (deg/s) | lead (s) | lag (deg) | lag (s) | ripple std (deg) | ripple p2p (deg) | measured speed (deg/s) | overshoot at stop (deg) |
|---|---|---|---|---|---|---|---|---|---|
| 5 | out | 5.0 | 0.00 | 1.00 | 0.20 | 0.099 | 0.373 | 5.09 ± 1.06 | 0.01 |
| 5 | back | 5.0 | 0.00 | 1.02 | 0.20 | 0.119 | 0.451 | -4.93 ± 1.38 | 0.08 |
## Control-loop timing (joint_states stamps, whole run)

rate_hz 8.58, dt_mean_ms 116.61, dt_std_ms 8.47, dt_p99_ms 137.36, dt_max_ms 137.54, freezes_over_300ms 0.00

Notes: positions are the servos' own readings (no external reference); time resolution is one control-loop period (see rate above).
