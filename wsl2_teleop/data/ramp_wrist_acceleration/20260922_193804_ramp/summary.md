# Arm characterization 2026-09-22 19:38

Command: `arm_characterize.py ramp 6 5 3 0.12` (forward_position_controller, unloaded)

## Moving target (constant velocity, streamed at 30 Hz)

| joint | direction | target speed (deg/s) | lead (s) | lag (deg) | lag (s) | ripple std (deg) | ripple p2p (deg) | measured speed (deg/s) | overshoot at stop (deg) |
|---|---|---|---|---|---|---|---|---|---|
| 6 | out | 5.0 | 0.12 | 0.84 | 0.17 | 0.149 | 0.623 | 4.95 ± 1.81 | - |
| 6 | back | 5.0 | 0.12 | 0.77 | 0.15 | 0.123 | 0.427 | -4.96 ± 1.76 | - |
## Control-loop timing (joint_states stamps, whole run)

rate_hz 9.29, dt_mean_ms 107.60, dt_std_ms 5.37, dt_p99_ms 120.13, dt_max_ms 121.92, freezes_over_300ms 0.00

Notes: positions are the servos' own readings (no external reference); time resolution is one control-loop period (see rate above).
