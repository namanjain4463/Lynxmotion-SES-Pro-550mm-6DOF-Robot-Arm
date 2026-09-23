# Arm characterization 2026-09-22 19:37

Command: `arm_characterize.py ramp 5 5 3 0.12` (forward_position_controller, unloaded)

## Moving target (constant velocity, streamed at 30 Hz)

| joint | direction | target speed (deg/s) | lead (s) | lag (deg) | lag (s) | ripple std (deg) | ripple p2p (deg) | measured speed (deg/s) | overshoot at stop (deg) |
|---|---|---|---|---|---|---|---|---|---|
| 5 | out | 5.0 | 0.12 | 0.98 | 0.20 | 0.122 | 0.603 | 5.21 ± 1.62 | - |
| 5 | back | 5.0 | 0.12 | 1.06 | 0.21 | 0.164 | 0.641 | -5.12 ± 1.75 | 0.00 |
## Control-loop timing (joint_states stamps, whole run)

rate_hz 8.74, dt_mean_ms 114.36, dt_std_ms 4.80, dt_p99_ms 122.62, dt_max_ms 122.68, freezes_over_300ms 0.00

Notes: positions are the servos' own readings (no external reference); time resolution is one control-loop period (see rate above).
