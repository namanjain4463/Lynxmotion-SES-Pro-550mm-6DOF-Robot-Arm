# Arm characterization 2026-09-22 19:29

Command: `arm_characterize.py ramp 6 5 3 0.12` (forward_position_controller, unloaded)

## Moving target (constant velocity, streamed at 30 Hz)

| joint | direction | target speed (deg/s) | lead (s) | lag (deg) | lag (s) | ripple std (deg) | ripple p2p (deg) | measured speed (deg/s) | overshoot at stop (deg) |
|---|---|---|---|---|---|---|---|---|---|
| 6 | out | 5.0 | 0.12 | 0.87 | 0.17 | 0.185 | 0.768 | 4.94 ± 1.90 | - |
| 6 | back | 5.0 | 0.12 | 0.87 | 0.17 | 0.122 | 0.463 | -5.16 ± 1.40 | 0.00 |
## Control-loop timing (joint_states stamps, whole run)

rate_hz 8.81, dt_mean_ms 113.54, dt_std_ms 5.10, dt_p99_ms 124.58, dt_max_ms 130.04, freezes_over_300ms 0.00

Notes: positions are the servos' own readings (no external reference); time resolution is one control-loop period (see rate above).
