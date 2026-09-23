# Gate A Phase 0 (unloaded) — measured servo response, noise, backlash, speed limits, jitter

Physical Lynxmotion SES-Pro 550, 2026-09-22, no load, arm at the ready pose
(joint_2 −50°, joint_3 −60°, joint_5 −40°, others 0). Protocol:
`docs/lynxmotion_physical_gate_a_protocol.md` in the (private) Thesis repo
Phase 0 step 1, plus standstill noise, loop timing, a moving-target (jitter) test and a
hand-push test. Tool: [`tools/arm_characterize.py`](tools/arm_characterize.py). Raw samples and
the tool's own summaries: [`data/`](data) (`20260922_180034_all`, `20260922_182544_play`).

Commands went through `forward_position_controller`, i.e. straight to the servos with no
trajectory shaping. **All positions are the servos' own encoder readings** — nothing here sees
play that sits outside the encoders (see "Backlash" below). Time resolution is one control-loop
period (~0.12 s), so delays are upper bounds.

## Results

**Standstill noise (20 s, 174 samples):** std ≤ 0.005°, peak-to-peak ≤ 0.03° on every joint,
2–4 distinct readings per joint (reading resolution 0.01°). Sensor noise is negligible.

**Step response (±5°, 5 reps each direction):**

| joint | delay (s) | rise 10–90% (s) | overshoot (deg) | settle to 0.3° (s) | abs steady-state error (deg) | hysteresis (deg) |
|---|---|---|---|---|---|---|
| 1 | 0.30 ± 0.05 | 0.42 ± 0.06 | 0.03 ± 0.02 | 0.89 ± 0.06 | 0.012 ± 0.008 | 0.015 ± 0.016 |
| 2 | 0.29 ± 0.04 | 0.42 ± 0.05 | 0.01 ± 0.01 | 0.87 ± 0.05 | 0.012 ± 0.006 | −0.002 ± 0.019 |
| 3 | 0.27 ± 0.03 | 0.39 ± 0.05 | 0.01 ± 0.01 | 0.83 ± 0.04 | 0.009 ± 0.007 | −0.003 ± 0.007 |
| 4 | 0.30 ± 0.05 | 0.34 ± 0.04 | 0.01 ± 0.01 | 0.68 ± 0.05 | 0.008 ± 0.009 | −0.009 ± 0.008 |
| 5 | 0.16 ± 0.03 | 0.23 ± 0.02 | 0.02 ± 0.01 | 0.50 ± 0.03 | 0.011 ± 0.010 | −0.022 ± 0.006 |
| 6 | 0.17 ± 0.04 | 0.19 ± 0.05 | 0.01 ± 0.01 | 0.51 ± 0.04 | 0.002 ± 0.002 | −0.000 ± 0.001 |

**Speed / acceleration (±15° steps, 2 reps each direction):**

| joint | measured peak speed (deg/s) | configured acceleration (deg/s²) | predicted peak √(a·15°) (deg/s) |
|---|---|---|---|
| 1 | 22.5 ± 1.1 | 30 | 21.2 |
| 2 | 21.4 ± 0.9 | 30 | 21.2 |
| 3 | 22.0 ± 0.5 | 30 | 21.2 |
| 4 | 27.7 ± 1.4 | 50 | 27.4 |
| 5 | 37.2 ± 1.7 | 100 | 38.7 |
| 6 | 38.9 ± 2.4 | 100 | 38.7 |

Configured values are from `pro_arm.ros2_control` in the arm repo. The script's own acceleration
estimate (41/78/109–123 deg/s² in the raw summary) is biased high by the ~8 Hz sampling; the
configured values predict the measured peaks within 5%. The configured `max_speed` (65–90 deg/s)
was never reached in 15° steps.

**Moving target (5 deg/s, streamed at 30 Hz, 15° out and back):**

| joint | lag (s) | ripple std / p2p (deg) | measured speed (deg/s) |
|---|---|---|---|
| 1 | 0.31–0.33 | 0.10–0.12 / 0.36–0.40 | 5.0 ± 0.9–1.2 |
| 2 | 0.29–0.31 | 0.05 / 0.19–0.20 | 5.0 ± 0.3–0.5 |
| 3 | 0.25–0.26 | 0.05–0.06 / 0.17–0.21 | 4.8–5.0 ± 0.5 |
| 4 | 0.22 | 0.05 / 0.17–0.18 | 4.9 ± 0.5 |
| 5 | 0.17–0.19 | 0.12–0.14 / 0.53–0.54 | 4.85 ± 1.6–1.7 |
| 6 | 0.18 | 0.13–0.16 / 0.49–0.66 | 4.9 ± 1.6–1.7 |

**Control-loop timing** (joint_states stamps, prompt pauses excluded): 8.1 Hz average (not the
configured 30 Hz), p99 period 0.14 s, **19 freezes longer than 0.3 s in ~12 min of testing,
longest 2.9 s**; in the 30 s hand-push run, 8.45 Hz and one 1.9 s freeze. (The raw
`20260922_180034_all/summary.md` reports 23 freezes and a 19.8 s maximum because it still
counted the waits at the script's prompts; the tool is fixed.)

**Hand-push test (30 s, pushing the tip by hand):** encoder readings changed by at most 0.06°
(joints 1–3), 0.03° (4–5), 0.01° (6). The operator could move the tip only "some mm", and only
felt very small movements.

## What this means

- **Actuator model for `Y_G` (the Gate A question):** each joint behaves like a pure delay
  (≤ 0.3 s for joints 1–4, ≤ 0.17 s for 5–6, upper bounds) followed by an acceleration-limited
  move with the configured acceleration (30 / 50 / 100 deg/s²), negligible overshoot
  (≤ 0.03°) and a steady-state error of about one reading count (≤ 0.012°). This is the
  "torque-velocity saturation actuator model, magnitude measured" that CLAUDE.md calls for,
  with acceleration as the binding limit at these amplitudes.
- **Backlash as seen by the servos: none measurable.** The settled position depends on the
  approach direction by at most 0.02° (joint 5: −0.022 ± 0.006°, possibly real but at the
  resolution limit). The hand-push test agrees: the servos hold stiffly and their readings barely
  change. So the "backlash" felt at the tip (a few mm) must be **mechanical play outside the
  encoders** (gear train after the encoder, link mounts, base) — measuring it needs an external
  reference (dial gauge or ruler at the tip, or the D455), not joint_states.
- **The jitter felt in teleop is a stop-go motion, not sensor noise.** While following a smooth
  5 deg/s target, the measured joint speed fluctuates by ±0.3–0.5 deg/s on joints 2–4 but by
  ±1.6–1.7 deg/s (about ±33%) on joints 5–6. Likely mechanism (consistent with the numbers, not
  yet proven): the driver loop runs at ~8 Hz, so each servo gets a new target only every
  ~0.12 s, about 0.6° ahead. Joints 5–6 (acceleration 100) can cover 0.6° in ~0.15 s — about one
  loop period — so they nearly stop between updates; joints 1–3 (acceleration 30) need ~0.28 s
  and blend the steps into smooth motion. Joint 1's ripple is also higher and not explained by
  this; its return ramp contained a loop freeze. The frequent 0.3–2.9 s loop freezes add larger,
  irregular stalls on top. **Confirmed by the follow-up test below:** lowering only the wrist
  acceleration to 30 brings joints 5–6 down to the level of joints 2–4.
- **Lag**: 0.17–0.33 s per joint behind a moving target, which is most of the ~0.35–0.45 s
  teleop lag.

## Follow-up: wrist servo acceleration (same day)

Same ramp test (5 deg/s, joints 5 and 6) with only the servo acceleration of joints 5–6 changed,
through `spacenav_arm_bridge/launch/real_arm_tuned.launch.py wrist_acceleration:=...` (the arm
repo is not modified). Raw data and per-run numbers:
[`data/ramp_wrist_acceleration/`](data/ramp_wrist_acceleration).

| acceleration (deg/s²) | runs | speed ripple, joints 5–6 (deg/s) | position ripple std (deg) |
|---|---|---|---|
| 100 (arm default) | 4 | ±1.1–2.0 | 0.09–0.17 |
| 100, command 0.12 s ahead | 3 | ±1.4–1.9 | 0.12–0.19 |
| 50 | 2 | ±0.7–1.2 | 0.07–0.09 |
| 30 | 2 | ±0.4–0.6 | 0.04–0.07 |

- The ripple falls with the acceleration, and at 30 matches joints 2–4 (±0.3–0.5), which also
  run at 30. This supports the stop-go mechanism above: a gentler servo blends the ~8 Hz target
  updates instead of stopping between them.
- Aiming the command 0.12 s ahead along the motion (the other candidate fix) did not help.
- Lag stayed in the same range (0.15–0.22 s). The cost is slower short moves: by the model above,
  a 15° wrist step now peaks near √(30·15) ≈ 21 deg/s instead of ~39 deg/s (predicted, not
  re-measured), and the wrist needs ~1.3 s to reach the teleop's 40°/s top speed.
- 30 deg/s² is now the default for joints 5–6 in the teleop setup (`real_arm_tuned.launch.py`).
  For `Y_G`, use the acceleration of the configuration that was actually run.

## Not measured yet

- Phase 0 step 2 (known mass attached), step 3 (multi-joint moves).
- Mechanical play outside the encoders (needs an external reference at the tip).
- The README's J2/J3 coupled limit.
- Whether a faster loop (native Ubuntu instead of WSL2/usbipd) reduces the ripple further.
- Phase 0 at wrist acceleration 30 for the other measures (steps, speed); only the ramp was rerun.
