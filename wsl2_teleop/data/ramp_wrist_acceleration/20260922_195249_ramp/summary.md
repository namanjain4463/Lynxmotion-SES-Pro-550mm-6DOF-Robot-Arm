# Arm characterization 2026-09-22 19:52

Command: `arm_characterize.py ramp 5 5 3 0` (forward_position_controller, unloaded)

## Moving target (constant velocity, streamed at 30 Hz)

| joint | direction | target speed (deg/s) | lead (s) | lag (deg) | lag (s) | ripple std (deg) | ripple p2p (deg) | measured speed (deg/s) | overshoot at stop (deg) |
|---|---|---|---|---|---|---|---|---|---|
| 5 | out | 5.0 | 0.00 | 0.99 | 0.20 | 0.067 | 0.207 | 4.99 ± 0.53 | 0.03 |
| 5 | back | 5.0 | 0.00 | 1.12 | 0.22 | 0.062 | 0.262 | -5.03 ± 0.58 | 0.07 |
## Control-loop timing (joint_states stamps, whole run)

rate_hz 8.93, dt_mean_ms 112.02, dt_std_ms 6.49, dt_p99_ms 129.71, dt_max_ms 134.29, freezes_over_300ms 0.00

Notes: positions are the servos' own readings (no external reference); time resolution is one control-loop period (see rate above).
