# Arm characterization 2026-09-22 19:29

Command: `arm_characterize.py ramp 5 5 3 0` (forward_position_controller, unloaded)

## Moving target (constant velocity, streamed at 30 Hz)

| joint | direction | target speed (deg/s) | lead (s) | lag (deg) | lag (s) | ripple std (deg) | ripple p2p (deg) | measured speed (deg/s) | overshoot at stop (deg) |
|---|---|---|---|---|---|---|---|---|---|
| 5 | out | 5.0 | 0.00 | 0.84 | 0.17 | 0.087 | 0.325 | 4.93 ± 1.28 | 0.02 |
| 5 | back | 5.0 | 0.00 | 0.93 | 0.19 | 0.142 | 0.639 | -4.72 ± 1.42 | 0.01 |
## Control-loop timing (joint_states stamps, whole run)

rate_hz 9.03, dt_mean_ms 110.77, dt_std_ms 5.45, dt_p99_ms 122.23, dt_max_ms 126.12, freezes_over_300ms 0.00

Notes: positions are the servos' own readings (no external reference); time resolution is one control-loop period (see rate above).
