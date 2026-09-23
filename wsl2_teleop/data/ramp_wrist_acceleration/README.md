# Wrist servo acceleration vs. speed ripple (2026-09-22)

`tools/arm_characterize.py ramp JOINT 5 3 [LEAD_S]` on joints 5 and 6: a 5 deg/s target streamed
at 30 Hz through `forward_position_controller`, 15° out and back, ready pose, no load. Only the
servo acceleration of joints 5–6 was changed between groups, via
`real_arm_tuned.launch.py wrist_acceleration:=...` (the arm repo is unchanged). Summary and
conclusions: `../../gate_a_phase0_results.md` and the main README, section 7.5.

"speed ripple" = the ± after the measured speed in each `summary.md` (std of the measured joint
speed while following the ramp).

| run | joint | acceleration (deg/s²) | lead (s) | speed ripple out / back (deg/s) |
|---|---|---|---|---|
| 20260922_192915_ramp | 5 | 100 | 0 | 1.28 / 1.42 |
| 20260922_193723_ramp | 5 | 100 | 0 | 1.06 / 1.38 |
| 20260922_193736_ramp | 5 | 100 | 0.12 | 1.62 / 1.75 |
| 20260922_192931_ramp | 6 | 100 | 0 | 1.90 / 1.99 |
| 20260922_193748_ramp | 6 | 100 | 0 | 1.54 / 1.38 |
| 20260922_192946_ramp | 6 | 100 | 0.12 | 1.90 / 1.40 |
| 20260922_193804_ramp | 6 | 100 | 0.12 | 1.81 / 1.76 |
| 20260922_194741_ramp | 5 | 50 | 0 | 0.87 / 0.67 |
| 20260922_194756_ramp | 6 | 50 | 0 | 0.78 / 1.18 |
| 20260922_195249_ramp | 5 | 30 | 0 | 0.53 / 0.58 |
| 20260922_195300_ramp | 6 | 30 | 0 | 0.42 / 0.46 |

Four other runs from the same session were aborted before any ramp finished and are not included.
