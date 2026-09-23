# Arm characterization 2026-09-22 19:53

Command: `arm_characterize.py ramp 6 5 3 0` (forward_position_controller, unloaded)

## Moving target (constant velocity, streamed at 30 Hz)

| joint | direction | target speed (deg/s) | lead (s) | lag (deg) | lag (s) | ripple std (deg) | ripple p2p (deg) | measured speed (deg/s) | overshoot at stop (deg) |
|---|---|---|---|---|---|---|---|---|---|
| 6 | out | 5.0 | 0.00 | 0.98 | 0.20 | 0.057 | 0.192 | 4.88 ± 0.42 | 0.02 |
| 6 | back | 5.0 | 0.00 | 0.90 | 0.18 | 0.042 | 0.222 | -4.95 ± 0.46 | 0.02 |
## Control-loop timing (joint_states stamps, whole run)

rate_hz 6.88, dt_mean_ms 145.45, dt_std_ms 329.05, dt_p99_ms 191.51, dt_max_ms 3402.18, freezes_over_300ms 1.00

Notes: positions are the servos' own readings (no external reference); time resolution is one control-loop period (see rate above).
