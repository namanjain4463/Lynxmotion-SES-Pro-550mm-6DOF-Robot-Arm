"""Grasp event detection for the DH CGE-10-10 from its only two sensors: the grip state register
and the finger position (per mille, 1000 = open). Pure Python, no ROS, so it can replay logs.

What real data showed (2026-09-23, 51 mm and other cylinders, force 20 %):
- A normal hold creeps 10-15 per mille in the first ~1.5 s (the object seating under the squeeze),
  then stays flat.
- An object pulled out of the grip shows as the fingers closing further in steps (656 -> 604 ->
  524 -> 204 -> 0 over 1.6 s) while the grip state still reads 2 (caught); the state only changes
  once the fingers have closed on nothing. So the position is the slip sensor; the state lags.
- Right after a new command the grip state can still show the previous result for ~0.4 s.
- Finger speed separates the two cases cleanly (speed setting 50 %): an object sliding while still
  touched lets the fingers creep at 10-130 per mille/s; once it is gone they close freely at
  1600-3100 per mille/s (same as an empty close). So "fingers suddenly fast" = contact lost,
  0.2-0.4 s before they reach the closed end or the grip state changes.

Events (dicts with "event" and details), in the order they can happen:
  CAUGHT    fingers stopped on an object during a close: position, estimated diameter, close time
  SEATED    end of the seating window: how far the fingers crept while the object settled
  SLIP      in-contact creep beyond seating: reported at slip_permille, then each time it doubles
  LOST      object gone without an open command, detected when the fingers speed up past
            contact_loss_speed (or the state/position says so): GRADUAL if it slid in contact for at
            least gradual_s first, else SUDDEN; with the in-contact slip amount and duration, and
            arm_moving, if known, so a loss during arm motion reads as a drop and one at rest as a
            removal/hand-over
  RELEASED  opened on purpose while holding: hold time, creep, hold horizons survived
  MISSED    a close reached the closed end without touching anything
The gripper cannot measure force; force_pct in each event is the COMMANDED squeeze (current limit).
"""
import math

GRIP_MOVING, GRIP_AT_TARGET, GRIP_CAUGHT, GRIP_DROPPED = 0, 1, 2, 3
HOLD_HORIZONS_S = (0.12, 0.5, 2.0)  # the simulator benchmark's hold horizons


class GraspMonitor:
    def __init__(self, open_pos=1000, closed_pos=0, seat_time=1.5, slip_permille=20,
                 diameter_open_mm=float("nan"), diameter_closed_mm=float("nan"),
                 contact_loss_speed=500.0, gradual_s=0.2):
        self.open_pos, self.closed_pos = open_pos, closed_pos
        self.seat_time = seat_time          # s after the catch that creep counts as seating
        self.slip_step = slip_permille      # creep beyond the seated position that counts as slip
        self.d_open, self.d_closed = diameter_open_mm, diameter_closed_mm
        self.loss_speed = contact_loss_speed  # per mille/s of closing that means "closing on air"
        self.gradual_s = gradual_s            # in-contact sliding at least this long = GRADUAL
        self.prev = None                      # (t, pos) of the previous sample
        self.phase = "idle"                 # idle | closing | opening | holding | lost
        self.target = None
        self.t_cmd = 0.0
        self.seen_moving = False
        self.hold = None                    # details of the current hold

    def diameter_mm(self, pos):
        """Object diameter from the finger position (linear between the two calibration points)."""
        if math.isnan(self.d_open) or math.isnan(self.d_closed):
            return float("nan")
        frac = (pos - self.closed_pos) / float(self.open_pos - self.closed_pos)
        return round(self.d_closed + frac * (self.d_open - self.d_closed), 1)

    def command(self, t, target, force_pct, pos):
        """Call when a new target is sent. Returns a list of events."""
        events = []
        if self.phase == "holding" and target > pos:
            events.append(self._end_hold(t, "RELEASED", pos, force_pct))
        self.target = target
        self.t_cmd = t
        self.seen_moving = False
        self.phase = "closing" if target < pos else "opening"
        return events

    def sample(self, t, grip_state, pos, force_pct, arm_moving=None):
        """Call for every state reading. Returns a list of events."""
        events = []
        prev, self.prev = self.prev, (t, pos)
        closing_speed = (prev[1] - pos) / (t - prev[0]) if prev is not None and t > prev[0] else 0.0
        fresh = self.seen_moving or t - self.t_cmd > 0.5  # older states may be left over
        self.seen_moving |= grip_state == GRIP_MOVING
        if self.phase == "closing":
            if grip_state == GRIP_CAUGHT and fresh:
                self.phase = "holding"
                self.hold = {"t_catch": t, "p_catch": pos, "p_seated": None, "slip_steps": 0,
                             "p_min": pos, "t_slip": None, "p_contact": pos}
                events.append({"event": "CAUGHT", "t": round(t, 3), "position": pos,
                               "diameter_mm": self.diameter_mm(pos), "close_time_s": round(t - self.t_cmd, 3),
                               "force_pct": force_pct})
            elif fresh and grip_state != GRIP_MOVING and abs(pos - self.target) <= 15 \
                    and self.target <= self.closed_pos + 15:
                self.phase = "idle"
                events.append({"event": "MISSED", "t": round(t, 3), "position": pos, "force_pct": force_pct,
                               "note": "closed fully without touching anything"})
        elif self.phase == "holding":
            h = self.hold
            h["p_min"] = min(h["p_min"], pos)
            held = t - h["t_catch"]
            # Gone = the gripper says so, or the fingers reached the closed end. Checked first, so a
            # loss between two readings (no creep seen before it) counts as SUDDEN, not as slip.
            free = closing_speed > self.loss_speed  # closing on air: no longer touching the object
            gone = free or grip_state in (GRIP_AT_TARGET, GRIP_DROPPED) or pos <= self.closed_pos + 15
            if not gone:
                h["p_contact"] = pos  # last position with the object still touched
            if (not gone and h["p_seated"] is None and h["t_slip"] is None and held >= self.seat_time
                    and grip_state == GRIP_CAUGHT):
                h["p_seated"] = pos
                events.append({"event": "SEATED", "t": round(t, 3), "position": pos,
                               "seat_creep": h["p_catch"] - pos, "force_pct": force_pct})
            ref = h["p_seated"] if h["p_seated"] is not None else h["p_catch"] - self.slip_step
            creep = ref - pos  # before seating, allow one slip step of settling first
            if not gone and creep >= self.slip_step * (2 ** h["slip_steps"]):
                while creep >= self.slip_step * (2 ** h["slip_steps"]):
                    h["slip_steps"] += 1
                h["t_slip"] = h["t_slip"] or t
                events.append({"event": "SLIP", "t": round(t, 3), "position": pos,
                               "creep_since_catch": h["p_catch"] - pos, "held_s": round(held, 3),
                               "slipping_for_s": round(t - h["t_slip"], 3),
                               "force_pct": force_pct, "arm_moving": arm_moving})
            if gone:
                ev = self._end_hold(t, "LOST", pos, force_pct)
                in_contact_s = (prev[0] - h["t_slip"]) if h["t_slip"] is not None and prev is not None else 0.0
                ev["kind"] = "GRADUAL" if in_contact_s >= self.gradual_s else "SUDDEN"
                ev["in_contact_slip_s"] = round(max(0.0, in_contact_s), 3)
                ev["in_contact_slip_permille"] = h["p_catch"] - h["p_contact"]
                ev["detected_by"] = ("finger speed" if free else "grip state" if grip_state in
                                     (GRIP_AT_TARGET, GRIP_DROPPED) else "closed end")
                ev["arm_moving"] = arm_moving
                ev["likely"] = ("dropped during arm motion" if arm_moving else
                                "removed at rest (hand-over or pulled out)" if arm_moving is False else
                                "unknown (no arm motion data)")
                events.append(ev)
                self.phase = "lost"
        return events

    def _end_hold(self, t, name, pos, force_pct):
        h = self.hold
        held = t - h["t_catch"]
        return {"event": name, "t": round(t, 3), "position": pos, "held_s": round(held, 3),
                "creep_total": h["p_catch"] - h["p_min"],
                "seat_creep": (h["p_catch"] - h["p_seated"]) if h["p_seated"] is not None else None,
                "horizons_survived": [x for x in HOLD_HORIZONS_S if held >= x], "force_pct": force_pct}
