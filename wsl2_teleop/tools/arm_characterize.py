#!/usr/bin/env python3
"""Lynxmotion SES-Pro arm characterization - Thesis physical Gate A, Phase 0 (unloaded).

Commands go through forward_position_controller, straight to the servos (no trajectory shaping).
  rest  SECONDS                 noise at standstill + control-loop timing (rate, jitter, freezes)
  steps JOINT AMP_DEG REPS      +-AMP steps both ways: delay, rise, overshoot, settle, steady-state
                                error, and backlash/hysteresis (settled position depends on approach side)
  speed JOINT AMP_DEG REPS      large steps: peak joint speed and acceleration (saturation)
  ramp  JOINT DEG_PER_S SECONDS [LEAD_S]  constant-velocity target streamed at 30 Hz, out and
                                back: tracking lag and motion ripple (jitter while moving); LEAD_S aims
                                the command that far ahead (as the teleop does), default 0
  play  SECONDS                 live joint readout while you push the arm by hand
  all                           rest, then steps + speed + ramp for every joint (asks before each)

Needs: arm launch running, forward_position_controller ACTIVE, teleop node NOT running,
arm in the ready pose (hold the left SpaceMouse button in teleop). Moves: steps <= 10 deg, speed <= 20 deg, ramps <= 20 deg.
Every run writes samples.csv + summary.md to ~/arm_characterization/<date_time>/.
"""
import csv
import math
import os
import sys
import time
from datetime import datetime

import numpy as np
import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINTS = [f"pro_arm_joint_{i}" for i in range(1, 7)]
TOPIC = "/forward_position_controller/commands"
LIMITS_DEG = [(-175, 175), (-85, 85), (-110, 110), (-125, 155), (-100, 175), (-175, 175)]  # README minus 5 deg
MAX_AMP = {"steps": 10.0, "speed": 20.0, "ramp": 20.0}
MOVE_TOL = math.radians(0.1)    # "has started moving"
SETTLE_TOL = math.radians(0.3)  # "has settled"
D = math.degrees
NAN = float("nan")


# ---------------- analysis (pure functions) ----------------
def step_metrics(t, m, t_cmd, start, target):
    """One step of one joint. t: receive times (s), m: measured positions (rad)."""
    out = dict(delay=NAN, rise=NAN, overshoot=NAN, settle=NAN, ss_err=NAN, settled=NAN, peak_vel=NAN)
    if len(t) < 5:
        return out
    tr, move = t - t_cmd, target - start
    moved = np.nonzero(np.abs(m - start) > MOVE_TOL)[0]
    if len(moved):
        out["delay"] = tr[moved[0]]
    if abs(move) > 2 * MOVE_TOL:
        prog = (m - start) / move
        i10, i90 = np.nonzero(prog >= 0.1)[0], np.nonzero(prog >= 0.9)[0]
        if len(i10) and len(i90):
            out["rise"] = tr[i90[0]] - tr[i10[0]]
        out["overshoot"] = D(max(0.0, float(np.max((m - target) * np.sign(move)))))
    outside = np.nonzero(np.abs(m - target) > SETTLE_TOL)[0]
    if not len(outside):
        out["settle"] = 0.0
    elif outside[-1] < len(m) - 3:
        out["settle"] = tr[outside[-1] + 1]
    out["settled"] = float(np.mean(m[tr > tr[-1] - 0.5]))
    out["ss_err"] = D(out["settled"] - target)
    v = np.diff(m) / np.maximum(np.diff(t), 1e-3)
    if len(v):
        out["peak_vel"] = D(float(np.max(np.abs(v))))
        peak_i = int(np.argmax(np.abs(v)))
        rising = np.nonzero(np.abs(v[:peak_i + 1]) >= 0.9 * abs(v[peak_i]))[0]
        if len(moved) and len(rising) and tr[rising[0] + 1] > out["delay"]:
            out["accel"] = out["peak_vel"] / (tr[rising[0] + 1] - out["delay"])
    return out


def ramp_metrics(t, m, cmd, t0, t_end, rate):
    """Streaming ramp: steady window from t0+1 s to t_end. rate in rad/s."""
    w = (t >= t0 + 1.0) & (t <= t_end)
    if w.sum() < 6:
        return dict(lag_deg=NAN, lag_s=NAN, ripple_std=NAN, ripple_p2p=NAN, vel_mean=NAN, vel_std=NAN)
    tw, mw, cw = t[w], m[w], cmd[w]
    lag = float(np.mean((cw - mw) * np.sign(rate)))
    fit = np.polyfit(tw, mw, 1)
    resid = mw - np.polyval(fit, tw)
    v = np.diff(mw) / np.maximum(np.diff(tw), 1e-3)
    return dict(lag_deg=D(lag), lag_s=lag / abs(rate), ripple_std=D(float(np.std(resid))),
                ripple_p2p=D(float(np.ptp(resid))), vel_mean=D(float(np.mean(v))), vel_std=D(float(np.std(v))))


def timing_metrics(stamps, breaks=()):
    """Loop timing from joint_states stamps; gaps at `breaks` (row indices after a prompt) are skipped."""
    dt = np.diff(np.asarray(stamps))
    keep = np.array([i + 1 not in breaks for i in range(len(dt))], dtype=bool)
    dt = dt[keep & (dt > 0)]
    if len(dt) < 3:
        return {}
    return dict(rate_hz=1.0 / float(np.mean(dt)), dt_mean_ms=1e3 * float(np.mean(dt)),
                dt_std_ms=1e3 * float(np.std(dt)), dt_p99_ms=1e3 * float(np.percentile(dt, 99)),
                dt_max_ms=1e3 * float(np.max(dt)), freezes_over_300ms=int(np.sum(dt > 0.3)))


def fmt(x, nd=2):
    return "-" if x is None or (isinstance(x, float) and math.isnan(x)) else f"{x:.{nd}f}"


def mean_std(vals, nd=2):
    v = [x for x in vals if not (isinstance(x, float) and math.isnan(x))]
    if not v:
        return "-"
    return f"{np.mean(v):.{nd}f} ± {np.std(v):.{nd}f}" if len(v) > 1 else f"{v[0]:.{nd}f}"


# ---------------- hardware side ----------------
class Rig(Node):
    def __init__(self):
        super().__init__("arm_characterize")
        self.pos = self.vel = self.cmd = None
        self.phase = "idle"
        self.rows = []  # (t_recv, t_stamp, phase, cmd[6] or None, pos[6], vel[6])
        self.breaks = set()  # row indices recorded right after waiting at a prompt
        self.t0 = time.monotonic()
        self.create_subscription(JointState, "/joint_states", self.on_js, 50)
        self.pub = self.create_publisher(Float64MultiArray, TOPIC, 10)
        self.ctrl_cli = self.create_client(ListControllers, "/controller_manager/list_controllers")

    def now(self):
        return time.monotonic() - self.t0

    def on_js(self, msg):
        d = dict(zip(msg.name, msg.position))
        if not all(j in d for j in JOINTS):
            return
        v = dict(zip(msg.name, msg.velocity)) if len(msg.velocity) == len(msg.name) else {}
        self.pos = np.array([d[j] for j in JOINTS])
        self.vel = np.array([v.get(j, NAN) for j in JOINTS])
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.rows.append((self.now(), stamp, self.phase, None if self.cmd is None else self.cmd.copy(),
                          self.pos.copy(), self.vel.copy()))

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def send(self, q):
        self.cmd = np.array(q, dtype=float)
        self.pub.publish(Float64MultiArray(data=[float(x) for x in self.cmd]))

    def hold(self, q, seconds, phase):
        self.phase = phase
        self.send(q)
        end, resend = time.monotonic() + seconds, time.monotonic() + 0.5
        while time.monotonic() < end:
            self.spin_for(0.02)
            if time.monotonic() > resend:  # guard against a lost message; same target, no effect on motion
                self.send(q)
                resend += 0.5

    def check_setup(self):
        if not self.ctrl_cli.wait_for_service(timeout_sec=5.0):
            print("controller_manager not found - is the arm launch running?")
            return False
        fut = self.ctrl_cli.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        res = fut.result()
        state = {c.name: c.state for c in res.controller}.get("forward_position_controller", "not loaded") if res else "?"
        if state != "active":
            print(f"forward_position_controller is {state}. Start the teleop launch once (it loads and activates it),\n"
                  "then Ctrl-C the teleop, then run this script.")
            return False
        t_pub = time.monotonic() + 3.0  # a node that just exited can stay visible for a moment
        while self.count_publishers(TOPIC) > 1 and time.monotonic() < t_pub:
            self.spin_for(0.2)
        if self.count_publishers(TOPIC) > 1:
            print("Something else is commanding the arm (teleop still running?). Ctrl-C it first.")
            return False
        t_end = time.monotonic() + 5.0
        while self.pos is None and time.monotonic() < t_end:
            self.spin_for(0.1)
        if self.pos is None:
            print("No /joint_states - is the arm launch running?")
            return False
        return True

    def segment(self, i0, j):
        rows = self.rows[i0:]
        return (np.array([r[0] for r in rows]), np.array([r[4][j] for r in rows]),
                np.array([np.nan if r[3] is None else r[3][j] for r in rows]))

    def safe(self, j, lo_rad, hi_rad):
        lo, hi = LIMITS_DEG[j]
        if D(lo_rad) < lo or D(hi_rad) > hi:
            print(f"  joint_{j + 1}: test range {D(lo_rad):.0f}..{D(hi_rad):.0f} deg leaves the safe range {lo}..{hi}. "
                  "Move the arm to the ready pose first.")
            return False
        return True


def run_rest(rig, seconds, report):
    print(f"\nREST: recording {seconds:.0f} s at standstill - don't touch the arm")
    q0 = rig.pos.copy()
    i0 = len(rig.rows)
    rig.hold(q0, seconds, "rest")
    rows = rig.rows[i0:]
    pos = np.array([r[4] for r in rows])
    vel = np.array([r[5] for r in rows])
    lines = ["## Standstill noise (joint_states, holding still)", "",
             f"{len(rows)} samples over {seconds:.0f} s.", "",
             "| joint | std (deg) | peak-to-peak (deg) | distinct readings | reported speed std (deg/s) |",
             "|---|---|---|---|---|"]
    for j in range(6):
        lines.append(f"| {j + 1} | {fmt(D(np.std(pos[:, j])), 3)} | {fmt(D(np.ptp(pos[:, j])), 3)} | "
                     f"{len(np.unique(np.round(pos[:, j], 6)))} | {fmt(D(np.nanstd(vel[:, j])), 3)} |")
    report.extend(lines + [""])
    print("\n".join(lines))


def run_steps(rig, j, amp_deg, reps, report, kind="steps", hold=3.0):
    amp_deg = min(abs(amp_deg), MAX_AMP[kind])
    a = math.radians(amp_deg)
    q0 = rig.pos.copy()
    if not rig.safe(j, q0[j] - a, q0[j] + a):
        return
    print(f"\n{kind.upper()} joint_{j + 1}: +-{amp_deg:.0f} deg, {reps} reps, {hold:.1f} s hold")
    rig.hold(q0, 1.0, f"{kind}_j{j + 1}_start")
    res = {k: [] for k in ("up", "back_from_above", "down", "back_from_below")}
    for r in range(reps):
        for label, off in (("up", a), ("back_from_above", 0.0), ("down", -a), ("back_from_below", 0.0)):
            q = q0.copy()
            q[j] = q0[j] + off
            start, i0, t_cmd = float(rig.pos[j]), len(rig.rows), rig.now()
            rig.hold(q, hold, f"{kind}_j{j + 1}_{label}")
            t, m, _ = rig.segment(i0, j)
            res[label].append(step_metrics(t, m, t_cmd, start, q[j]))
        print(f"  rep {r + 1}/{reps} done")
    moves = res["up"] + res["down"] + res["back_from_above"] + res["back_from_below"]
    hyst = [D(a_["settled"] - b_["settled"]) for a_, b_ in zip(res["back_from_above"], res["back_from_below"])]
    if kind == "steps":
        report.append(f"| {j + 1} | {amp_deg:.0f} | {mean_std([x['delay'] for x in moves])} | "
                      f"{mean_std([x['rise'] for x in moves])} | {mean_std([x['overshoot'] for x in moves])} | "
                      f"{mean_std([x['settle'] for x in moves])} | {mean_std([abs(x['ss_err']) for x in moves], 3)} | "
                      f"{mean_std(hyst, 3)} |")
    else:
        report.append(f"| {j + 1} | {amp_deg:.0f} | {mean_std([x['peak_vel'] for x in moves], 1)} | "
                      f"{mean_std([x.get('accel', NAN) for x in moves], 0)} | {mean_std([x['delay'] for x in moves])} |")
    print("  " + report[-1])


def run_ramp(rig, j, rate_dps, seconds, report, lead=0.0):
    total = min(abs(rate_dps) * seconds, MAX_AMP["ramp"])
    seconds = total / abs(rate_dps)
    rate = math.radians(abs(rate_dps))
    q0 = rig.pos.copy()
    if not rig.safe(j, q0[j] - math.radians(total), q0[j] + math.radians(total)):
        return
    print(f"\nRAMP joint_{j + 1}: {abs(rate_dps):.1f} deg/s for {seconds:.1f} s out, then back, lead {lead:.2f} s")
    rig.hold(q0, 1.0, f"ramp_j{j + 1}_start")
    for label, sign in (("out", 1.0), ("back", -1.0)):
        base = rig.cmd[j]
        i0, t_start = len(rig.rows), rig.now()
        rig.phase = f"ramp_j{j + 1}_{label}"
        while rig.now() - t_start < seconds:
            q = q0.copy()
            q[j] = base + sign * rate * min(rig.now() - t_start + lead, seconds)
            rig.send(q)
            rig.spin_for(1.0 / 30.0)
        q = q0.copy()
        q[j] = base + sign * rate * seconds
        t_end = rig.now()
        rig.hold(q, 2.0, f"ramp_j{j + 1}_{label}_settle")
        t, m, c = rig.segment(i0, j)
        met = ramp_metrics(t, m, c, t_start, t_end, sign * rate)
        after = t >= t_end
        end_step = step_metrics(t[after], m[after], t_end, float(m[after][0]) if after.any() else NAN, q[j]) \
            if after.sum() > 5 else {}
        report.append(f"| {j + 1} | {label} | {abs(rate_dps):.1f} | {lead:.2f} | {fmt(met['lag_deg'])} | {fmt(met['lag_s'])} | "
                      f"{fmt(met['ripple_std'], 3)} | {fmt(met['ripple_p2p'], 3)} | {fmt(met['vel_mean'])} ± "
                      f"{fmt(met['vel_std'])} | {fmt(end_step.get('overshoot', NAN))} |")
        print("  " + report[-1])


def run_play(rig, seconds, report):
    print(f"\nPLAY: {seconds:.0f} s. Gently push the arm tip by hand in different directions and release.\n"
          "Watch whether the numbers change while you feel the arm move (degrees moved since start):")
    q0 = rig.pos.copy()
    rig.phase = "play"
    rig.send(q0)
    t_start, nxt, i0 = rig.now(), 0.0, len(rig.rows)
    while rig.now() - t_start < seconds:
        rig.spin_for(0.02)
        if rig.now() - t_start >= nxt:
            print("  " + "  ".join(f"j{k + 1} {D(rig.pos[k] - q0[k]):+6.2f}" for k in range(6)))
            nxt += 0.25
    pos = np.array([r[4] for r in rig.rows[i0:]])
    report.extend(["## Hand-push test (play)", "",
                   "Largest reading change per joint while pushing: " +
                   ", ".join(f"j{k + 1} {D(np.max(np.abs(pos[:, k] - q0[k]))):.2f} deg" for k in range(6)), ""])


def main():
    args = sys.argv[1:]
    if not args or args[0] not in ("rest", "steps", "speed", "ramp", "play", "all"):
        print(__doc__)
        return
    out_dir = os.path.expanduser(f"~/arm_characterization/{datetime.now():%Y%m%d_%H%M%S}_{args[0]}")
    rclpy.init()
    rig = Rig()
    report = [f"# Arm characterization {datetime.now():%Y-%m-%d %H:%M}", "",
              f"Command: `arm_characterize.py {' '.join(args)}` (forward_position_controller, unloaded)", ""]
    step_hdr = ["## Step response (per joint, both directions)", "",
                "| joint | step (deg) | delay (s) | rise 10-90% (s) | overshoot (deg) | settle to 0.3 deg (s) | "
                "abs steady-state error (deg) | backlash / hysteresis (deg) |", "|---|---|---|---|---|---|---|---|"]
    speed_hdr = ["## Speed and acceleration (large steps)", "",
                 "| joint | step (deg) | peak speed (deg/s) | acceleration (deg/s²) | delay (s) |", "|---|---|---|---|---|"]
    ramp_hdr = ["## Moving target (constant velocity, streamed at 30 Hz)", "",
                "| joint | direction | target speed (deg/s) | lead (s) | lag (deg) | lag (s) | ripple std (deg) | ripple p2p (deg) | "
                "measured speed (deg/s) | overshoot at stop (deg) |", "|---|---|---|---|---|---|---|---|---|---|"]
    try:
        if not rig.check_setup():
            return
        cmd = args[0]
        if cmd == "rest":
            run_rest(rig, float(args[1]) if len(args) > 1 else 20.0, report)
        elif cmd == "play":
            run_play(rig, float(args[1]) if len(args) > 1 else 30.0, report)
        elif cmd in ("steps", "speed"):
            j = int(args[1]) - 1
            report += step_hdr if cmd == "steps" else speed_hdr
            run_steps(rig, j, float(args[2]) if len(args) > 2 else (5.0 if cmd == "steps" else 15.0),
                      int(args[3]) if len(args) > 3 else (5 if cmd == "steps" else 2), report, cmd,
                      3.0 if cmd == "steps" else 2.5)
        elif cmd == "ramp":
            report += ramp_hdr
            run_ramp(rig, int(args[1]) - 1, float(args[2]) if len(args) > 2 else 5.0,
                     float(args[3]) if len(args) > 3 else 3.0, report, float(args[4]) if len(args) > 4 else 0.0)
        else:  # all
            run_rest(rig, 20.0, report)
            sections = {"steps": list(step_hdr), "speed": list(speed_hdr), "ramp": list(ramp_hdr)}
            for j in range(6):
                rig.breaks.add(len(rig.rows))
                a = input(f"\nJoint {j + 1}: about 2 minutes of small moves. Enter = test, s = skip, q = finish: ").strip().lower()
                if a == "q":
                    break
                if a == "s":
                    continue
                run_steps(rig, j, 5.0, 5, sections["steps"], "steps", 3.0)
                run_steps(rig, j, 15.0, 2, sections["speed"], "speed", 2.5)
                run_ramp(rig, j, 5.0, 3.0, sections["ramp"])
            rig.breaks.add(len(rig.rows))
            for s in sections.values():
                if len(s) > 4:
                    report += s + [""]
    except KeyboardInterrupt:
        print("\nStopped - the arm holds its last commanded position.")
    finally:
        if rig.rows:
            tm = timing_metrics([r[1] for r in rig.rows], rig.breaks)
            report += ["## Control-loop timing (joint_states stamps, whole run)", "",
                       ", ".join(f"{k} {fmt(v)}" for k, v in tm.items()), "",
                       "Notes: positions are the servos' own readings (no external reference); time resolution "
                       "is one control-loop period (see rate above)."]
            os.makedirs(out_dir, exist_ok=True)
            with open(os.path.join(out_dir, "samples.csv"), "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t_recv_s", "t_stamp_s", "phase"] + [f"cmd{k}_deg" for k in range(1, 7)] +
                           [f"pos{k}_deg" for k in range(1, 7)] + [f"vel{k}_dps" for k in range(1, 7)])
                for t, st, ph, c, p, v in rig.rows:
                    w.writerow([f"{t:.4f}", f"{st:.4f}", ph] +
                               ([f"{D(x):.4f}" for x in c] if c is not None else [""] * 6) +
                               [f"{D(x):.4f}" for x in p] + [f"{D(x):.4f}" for x in v])
            with open(os.path.join(out_dir, "summary.md"), "w") as f:
                f.write("\n".join(report) + "\n")
            print(f"\n{'=' * 70}\n" + "\n".join(report) + f"\n{'=' * 70}\nSaved to {out_dir}")
        rig.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
