# Arm characterization 2026-09-22 19:47

Command: `arm_characterize.py ramp 5 5 3 0` (forward_position_controller, unloaded)

## Moving target (constant velocity, streamed at 30 Hz)

| joint | direction | target speed (deg/s) | lead (s) | lag (deg) | lag (s) | ripple std (deg) | ripple p2p (deg) | measured speed (deg/s) | overshoot at stop (deg) |
|---|---|---|---|---|---|---|---|---|---|
| 5 | out | 5.0 | 0.00 | 0.97 | 0.19 | 0.071 | 0.233 | 5.06 ± 0.87 | 0.01 |
| 5 | back | 5.0 | 0.00 | 1.09 | 0.22 | 0.078 | 0.249 | -4.84 ± 0.67 | 0.00 |
## Control-loop timing (joint_states stamps, whole run)

rate_hz 6.32, dt_mean_ms 158.10, dt_std_ms 316.27, dt_p99_ms 501.75, dt_max_ms 3123.61, freezes_over_300ms 1.00

Notes: positions are the servos' own readings (no external reference); time resolution is one control-loop period (see rate above).
