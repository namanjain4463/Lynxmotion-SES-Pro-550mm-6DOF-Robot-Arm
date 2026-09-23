# Arm characterization 2026-09-22 19:29

Command: `arm_characterize.py ramp 6 5 3 0` (forward_position_controller, unloaded)

## Moving target (constant velocity, streamed at 30 Hz)

| joint | direction | target speed (deg/s) | lead (s) | lag (deg) | lag (s) | ripple std (deg) | ripple p2p (deg) | measured speed (deg/s) | overshoot at stop (deg) |
|---|---|---|---|---|---|---|---|---|---|
| 6 | out | 5.0 | 0.00 | 0.84 | 0.17 | 0.153 | 0.501 | 4.85 ± 1.90 | 0.01 |
| 6 | back | 5.0 | 0.00 | 0.82 | 0.16 | 0.169 | 0.672 | -4.96 ± 1.99 | 0.00 |
## Control-loop timing (joint_states stamps, whole run)

rate_hz 7.14, dt_mean_ms 140.07, dt_std_ms 304.30, dt_p99_ms 120.38, dt_max_ms 3197.90, freezes_over_300ms 1.00

Notes: positions are the servos' own readings (no external reference); time resolution is one control-loop period (see rate above).
