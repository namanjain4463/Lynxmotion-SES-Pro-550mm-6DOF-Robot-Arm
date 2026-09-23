# Arm characterization 2026-09-22 18:25

Command: `arm_characterize.py play 30` (forward_position_controller, unloaded)

## Hand-push test (play)

Largest reading change per joint while pushing: j1 0.06 deg, j2 0.06 deg, j3 0.06 deg, j4 0.03 deg, j5 0.03 deg, j6 0.01 deg

## Control-loop timing (joint_states stamps, whole run)

rate_hz 8.45, dt_mean_ms 118.27, dt_std_ms 106.71, dt_p99_ms 127.71, dt_max_ms 1861.81, freezes_over_300ms 1.00

Notes: positions are the servos' own readings (no external reference); time resolution is one control-loop period (see rate above).
