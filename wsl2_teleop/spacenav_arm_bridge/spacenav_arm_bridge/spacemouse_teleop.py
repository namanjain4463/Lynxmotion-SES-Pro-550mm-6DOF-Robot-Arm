import time

import numpy as np
import rclpy
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import State
from moveit_msgs.srv import GetStateValidity
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64MultiArray, String
from visualization_msgs.msg import InteractiveMarkerFeedback

from spacenav_arm_bridge.arm_kinematics import (ArmKinematics, dls_step, path_min_height,
                                               rotation_error, rotation_from_vector, smoothstep)

JOINTS = [f"pro_arm_joint_{i}" for i in range(1, 7)]
ROT_GAIN = 0.2  # fraction of any tip-orientation error corrected per step
# Puck inputs in the arm frame (after axis_map / rot_axis_map), in the order command() returns them
INPUTS = ["lin_x", "lin_y", "lin_z", "ang_x", "ang_y", "ang_z"]
INPUT_TEXT = {"lin_x": "push away/toward", "lin_y": "push left/right", "lin_z": "lift/press",
              "ang_x": "tilt left/right", "ang_y": "tilt away/toward", "ang_z": "twist"}
JOINT_TEXT = ["base turn", "shoulder", "elbow", "forearm roll", "wrist bend", "gripper roll"]


class SpacemouseTeleop(Node):
    """SpaceMouse teleop, streaming joint positions to forward_position_controller. Two modes:

    JOINT mode: each puck motion drives one joint (only the strongest motion counts), so every
    pose inside the joint limits can be reached. TIP mode: the puck moves the arm tip in
    straight lines and turns the gripper, as described below.
    Buttons (SpaceMouse Compact): left short = switch mode, left hold = go to the ready pose;
    right short = gripper open/close (no gripper driver yet), right hold = precision (slow) mode.

    Every step starts from the last COMMANDED joints (open loop): restarting from the measured
    position loses over 90% of the motion on this arm because of the ~0.35 s servo delay.
    TIP mode computes the joint motion from the arm's Jacobian (damped least squares); near the
    edge of reach or a singularity the tip slides or slows instead of freezing. The arm repo's
    URDF gives every joint +-180 deg, so the real limits (arm repo README) are enforced here in
    both modes. Each step is collision-checked by MoveIt before it is sent.
    """

    def __init__(self):
        super().__init__("spacemouse_teleop")
        p = self.declare_parameter
        self.rate = p("rate", 30.0).value
        self.linear_speed = p("linear_speed", 0.1).value       # m/s at full puck deflection
        self.angular_speed = p("angular_speed", 0.5).value     # rad/s at full deflection (~29 deg/s), 0 = off
        self.deadband = p("deadband", 0.1).value
        # spacenav_node divides by 512 but the SpaceMouse Compact only reaches ~350 (~0.68): this counts as full
        self.full_deflection = p("full_deflection", 0.65).value
        # Response curve: 0 = straight line, 1 = cubic. 0.8: 25% puck -> ~6% speed, 50% -> ~20%, 100% -> 100%
        self.expo = p("expo", 0.8).value
        self.mode = p("start_mode", "joint").value              # "joint" or "tip"
        # JOINT mode: which puck input drives each joint (names from INPUTS, arm frame) and its top speed.
        # Directions are worked out at the ready pose so each joint moves the way the puck moves there.
        self.joint_inputs = list(p("joint_inputs", ["ang_z", "lin_y", "lin_z", "lin_x", "ang_x", "ang_y"]).value)
        self.joint_speeds = np.radians(list(p("joint_speeds_deg", [25.0, 25.0, 25.0, 40.0, 40.0, 40.0]).value))
        # 0 = automatic direction; set +1/-1 per joint to override
        self.joint_signs = [float(x) for x in p("joint_signs", [0.0] * 6).value]
        self.switch_margin = p("switch_margin", 1.3).value   # another motion must be this much stronger to take over
        # Buttons: /spacenav/joy index, on the SpaceMouse Compact 0 = right, 1 = left (checked on the hardware)
        self.mode_button = p("mode_button", 1).value      # short: switch mode, hold: ready pose
        self.gripper_button = p("gripper_button", 0).value  # short: gripper, hold: precision mode
        self.hold_time = p("hold_time", 1.0).value
        self.precision_scale = p("precision_scale", 0.3).value
        # Tip axis x, y, z <- puck axis number (1=x, 2=y, 3=z); negative number = reversed
        self.axis_map = list(p("axis_map", [1, 2, 3]).value)
        # Same for the gripper rotation axes (twist/tilt of the puck), calibrated separately
        self.rot_axis_map = list(p("rot_axis_map", [1, 2, 3]).value)
        # Only move OR rotate at any moment: whichever the puck is doing more wins
        self.one_mode = p("one_mode_at_a_time", True).value
        # Aim each command this far ahead along the current motion so the servos, which only get a new
        # target ~8 times a second, keep moving instead of stopping between targets (0 = off)
        # "wrist": tilt moves only joints 4-6 (arm body still, tip swings a little) and twist spins
        # the gripper about its own axis (joint 6 only);
        # "tip": the gripper turns about its tip point and the shoulder/elbow move to keep it still
        self.rotate_about = p("rotate_about", "wrist").value
        self.lead_time = p("lead_time", 0.0).value  # measured: 0.12 s did not reduce the wrist ripple
        self.max_joint_speed = p("max_joint_speed", 0.6).value  # rad/s cap per joint
        self.damping = p("damping", 0.02).value                 # higher = calmer near singularities
        self.rot_weight = p("rot_weight", 0.5).value            # how firmly the tip orientation is held
        self.resync_after = p("resync_after", 1.0).value        # s idle before re-reading the real pose
        ready_deg = list(p("ready_pose_deg", [0.0, -50.0, -60.0, 0.0, -40.0, 0.0]).value)
        self.home_speed = p("home_speed", 0.2).value            # rad/s average of the joint that moves most
        self.min_height = p("min_height", 0.05).value           # m; homing refused if the path goes lower
        self.base_frame = p("base_frame", "pro_arm_base_link").value
        self.ee_link = p("ee_link", "pro_arm_ee").value
        self.group = p("group", "pro_arm").value
        # Real joint limits (deg) as [lo1, hi1, ..., lo6, hi6], from the arm repo README.
        # J6 is untested there; +-180 is also where the driver rejects commands.
        default_limits = [-180.0, 180.0, -90.0, 90.0, -115.0, 115.0,
                          -130.0, 160.0, -105.0, 180.0, -180.0, 180.0]
        limits_deg = list(p("joint_limits_deg", default_limits).value)
        margin = np.radians(p("limit_margin_deg", 5.0).value)

        if sorted(abs(int(m)) for m in self.axis_map) != [1, 2, 3]:
            self.get_logger().error(f"Bad axis_map {self.axis_map}, using [1, 2, 3]")
            self.axis_map = [1, 2, 3]
        if sorted(abs(int(m)) for m in self.rot_axis_map) != [1, 2, 3]:
            self.get_logger().error(f"Bad rot_axis_map {self.rot_axis_map}, using [1, 2, 3]")
            self.rot_axis_map = [1, 2, 3]
        if self.mode not in ("joint", "tip"):
            self.get_logger().error(f"Bad start_mode {self.mode}, using joint")
            self.mode = "joint"
        if len(self.joint_inputs) != 6 or any(i not in INPUTS for i in self.joint_inputs):
            self.get_logger().error(f"Bad joint_inputs {self.joint_inputs}, using defaults")
            self.joint_inputs = ["ang_z", "lin_y", "lin_z", "lin_x", "ang_x", "ang_y"]
        if len(self.joint_speeds) != 6 or len(self.joint_signs) != 6:
            self.get_logger().error("joint_speeds_deg and joint_signs need 6 values; using defaults")
            self.joint_speeds = np.radians([25.0, 25.0, 25.0, 40.0, 40.0, 40.0])
            self.joint_signs = [0.0] * 6
        if len(limits_deg) != 12:
            self.get_logger().error(f"joint_limits_deg needs 12 values, got {len(limits_deg)}; using defaults")
            limits_deg = default_limits
        self.limits = [(np.radians(limits_deg[2 * i]) + margin, np.radians(limits_deg[2 * i + 1]) - margin)
                       for i in range(6)]
        self.ready = np.radians(ready_deg) if len(ready_deg) == 6 else None
        if self.ready is None or not all(lo <= r <= hi for r, (lo, hi) in zip(self.ready, self.limits)):
            self.get_logger().error(f"ready_pose_deg {ready_deg} is not 6 values inside the joint limits; "
                                    "home button disabled")
            self.ready = None

        self.kin = None
        self.measured = None
        self.q_cmd = None
        self.r_hold = None
        self.twist = Twist()
        self.twist_time = 0.0
        self.buttons = []
        self.last_active = 0.0
        self.last_sync = 0.0
        self.last_step = 0.0
        self.busy = False
        self.stopped = False
        self.homing = None  # (start time, duration, q_start, q_goal)
        self.problems = {}  # health problems by source; teleop pauses while any exist
        self.qdot = np.zeros(6)   # recent commanded joint velocity, for the lead
        self.lead_active = False  # the last command sent was ahead of q_cmd
        self.precision = False
        self.active_joint = None  # JOINT mode: the joint currently being driven
        self.press_start = {}     # button index -> time it went down (None once its hold action fired)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, "/robot_description", self.on_description, latched)
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.create_subscription(Twist, "/spacenav/twist", self.on_twist, 10)
        self.create_subscription(Joy, "/spacenav/joy", self.on_joy, 10)
        self.create_subscription(InteractiveMarkerFeedback, "/emergency_stop_button/feedback",
                                 self.on_estop, 10)
        self.pub = self.create_publisher(Float64MultiArray, "/forward_position_controller/commands", 10)
        self.valid_cli = self.create_client(GetStateValidity, "/check_state_validity")
        self.hw_cli = self.create_client(ListHardwareComponents, "/controller_manager/list_hardware_components")
        self.ctrl_cli = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.create_timer(1.0 / self.rate, self.tick)
        self.create_timer(2.0, self.check_health)
        self.get_logger().info("Waiting for /robot_description and /joint_states ...")

    # ---------- inputs ----------
    def on_description(self, msg):
        if self.kin is not None:
            return
        try:
            self.kin = ArmKinematics(msg.data, self.base_frame, self.ee_link, JOINTS)
        except ValueError as e:
            self.get_logger().error(f"Can't use /robot_description: {e}")
            return
        self.setup_joint_signs()
        self.get_logger().info("SpaceMouse teleop ready. LEFT button: short = switch JOINT/TIP mode, "
                               f"hold {self.hold_time:.0f} s = go to the ready pose. RIGHT button: short = gripper, "
                               f"hold {self.hold_time:.0f} s = precision (slow) mode.")
        self.announce_mode()

    def setup_joint_signs(self):
        """Direction of each joint in JOINT mode: at the ready pose, the joint moves the tip (or turns the
        gripper) the same way as its puck input."""
        q = self.ready if self.ready is not None else np.zeros(6)
        _, jac, _ = self.kin.jacobian(q)
        signs = []
        for j, name in enumerate(self.joint_inputs):
            if self.joint_signs[j] != 0.0:
                signs.append(1.0 if self.joint_signs[j] > 0 else -1.0)
                continue
            row = INPUTS.index(name)  # rows of the Jacobian are in the same order as INPUTS
            signs.append(1.0 if jac[row, j] >= 0.0 else -1.0)
        self.joint_signs = signs

    def announce_mode(self):
        if self.mode == "joint":
            parts = [f"{INPUT_TEXT[i]} = J{j + 1} {JOINT_TEXT[j]}" for j, i in enumerate(self.joint_inputs)]
            self.get_logger().info("MODE: JOINT (one joint at a time): " + ", ".join(parts))
        else:
            self.get_logger().info("MODE: TIP: push = tip moves in straight lines, tilt = gripper points, "
                                   "twist = gripper spins")

    def on_js(self, msg):
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in JOINTS):
            self.measured = [d[j] for j in JOINTS]

    def on_twist(self, msg):
        self.twist = msg
        self.twist_time = time.monotonic()

    def on_joy(self, msg):
        prev, self.buttons = self.buttons, list(msg.buttons)
        now = time.monotonic()
        for b in (self.mode_button, self.gripper_button):
            down = b < len(self.buttons) and bool(self.buttons[b])
            was = b < len(prev) and bool(prev[b])
            if down and not was:
                if self.homing is not None:  # any press stops homing
                    self.homing = None
                    self.get_logger().info("Homing stopped")
                    self.press_start[b] = None
                else:
                    self.press_start[b] = now
            elif was and not down:
                start = self.press_start.pop(b, None)
                if start is not None:  # released before the hold time: short press
                    self.short_press(b)

    def check_holds(self, now):
        for b, start in list(self.press_start.items()):
            if start is not None and now - start >= self.hold_time:
                self.press_start[b] = None  # the release must not also count as a short press
                self.hold_press(b)

    def short_press(self, b):
        if b == self.mode_button:
            self.mode = "tip" if self.mode == "joint" else "joint"
            self.active_joint = None
            self.announce_mode()
        elif b == self.gripper_button:
            self.get_logger().warn("Gripper button: no gripper driver yet, nothing happens")

    def hold_press(self, b):
        if b == self.mode_button:
            self.start_homing()
        elif b == self.gripper_button:
            self.precision = not self.precision
            self.get_logger().info(f"Precision mode {'ON: all speeds x' + str(self.precision_scale) if self.precision else 'OFF: full speed'}")

    def on_estop(self, msg):
        if msg.event_type != InteractiveMarkerFeedback.BUTTON_CLICK or msg.marker_name != "emergency_stop":
            return
        self.stopped = not self.stopped
        self.homing = None
        self.q_cmd = None  # re-read the real pose before moving again
        if self.stopped:
            self.get_logger().warn("RViz STOP pressed: teleop paused. Click STOP again to resume.")
        else:
            self.get_logger().info("RViz STOP released: teleop resumed.")

    # ---------- health: is the arm actually able to take commands? ----------
    def check_health(self):
        if self.hw_cli.service_is_ready():
            self.hw_cli.call_async(ListHardwareComponents.Request()).add_done_callback(self.on_hw)
        if self.ctrl_cli.service_is_ready():
            self.ctrl_cli.call_async(ListControllers.Request()).add_done_callback(self.on_ctrl)

    def on_hw(self, fut):
        res = fut.result()
        if res is None:
            return
        bad = [c.name for c in res.component if c.state.id != State.PRIMARY_STATE_ACTIVE]
        self.set_problem("hardware", ", ".join(bad) and
                         f"arm driver stopped ({', '.join(bad)}): the USB link to the arm was probably lost. "
                         "Ctrl-C all ROS terminals, run 'wsl --shutdown' in PowerShell, re-attach, relaunch.")

    def on_ctrl(self, fut):
        res = fut.result()
        if res is None:
            return
        state = {c.name: c.state for c in res.controller}.get("forward_position_controller", "not loaded")
        self.set_problem("controller", state != "active" and
                         f"forward_position_controller is {state}, so the arm will not move.")

    def set_problem(self, source, text):
        if text:
            self.get_logger().warn(f"Teleop paused: {text}", throttle_duration_sec=10.0)
            if source not in self.problems:
                self.homing = None
            self.problems[source] = text
        elif source in self.problems:
            del self.problems[source]
            if not self.problems:
                self.q_cmd = None  # re-read the real pose before moving again
                self.get_logger().info("Arm reachable again: teleop resumed.")

    # ---------- main loop ----------
    @staticmethod
    def mapped(values, axis_map):
        return [values[abs(int(m)) - 1] * (1.0 if int(m) > 0 else -1.0) for m in axis_map]

    def shape(self, x):
        """Deadband, then the response curve: 0 at the deadband edge, 1 at full deflection."""
        s = (abs(x) - self.deadband) / max(1e-6, self.full_deflection - self.deadband)
        if s <= 0.0:
            return 0.0
        s = min(1.0, s)
        return float(np.copysign((1.0 - self.expo) * s + self.expo * s ** 3, x))

    def puck(self, now):
        """Shaped puck inputs in the arm frame: (linear x, y, z, angular x, y, z), each -1..1."""
        if now - self.twist_time > 0.2:
            return np.zeros(3), np.zeros(3)
        t = self.twist
        lin = [self.shape(v) for v in (t.linear.x, t.linear.y, t.linear.z)]
        ang = [self.shape(v) for v in (t.angular.x, t.angular.y, t.angular.z)]
        return np.array(self.mapped(lin, self.axis_map)), np.array(self.mapped(ang, self.rot_axis_map))

    def command(self, now):
        lin, ang = self.puck(now)
        if self.angular_speed <= 0.0:
            ang = np.zeros(3)
        if self.one_mode:  # the puck always mixes push and tilt a little: keep only the stronger one
            if np.linalg.norm(lin) >= np.linalg.norm(ang):
                ang = np.zeros(3)
            else:
                lin = np.zeros(3)
        scale = self.precision_scale if self.precision else 1.0
        return lin * self.linear_speed * scale, ang * self.angular_speed * scale

    def tick(self):
        if self.kin is None or self.measured is None or self.stopped or self.problems:
            return
        now = time.monotonic()
        if self.homing is not None:
            self.homing_step(now)
            return
        self.check_holds(now)
        if self.homing is not None:
            return
        if self.busy:
            return
        if self.mode == "joint":
            self.joint_tick(now)
            return
        v, w = self.command(now)
        if not v.any() and not w.any():
            self.drop_lead()  # puck released: send the true target so the arm stops where it should
            # Idle: refresh the start point from the real arm so the next push starts at once
            if now - self.last_active > self.resync_after and now - self.last_sync > 1.0:
                self.sync_from_measured(now)
            return
        self.last_active = now
        if self.q_cmd is None:
            self.sync_from_measured(now)
        dt = now - self.last_step
        if dt <= 0.0 or dt > 0.2:
            dt = 1.0 / self.rate
        self.last_step = now
        # The step turns the gripper by w*dt and also corrects any leftover orientation error
        # against r_hold; r_hold itself advances only once the step is accepted (no wind-up).
        wrist_only = self.rotate_about == "wrist" and w.any() and not v.any()
        if wrist_only and self.is_twist(w):
            self.twist_step(w, dt)
            return
        lock = (0, 1, 2) if wrist_only else ()
        q_new, locked, dx, scale = dls_step(self.kin, self.q_cmd, v * dt, w * dt, self.r_hold, self.limits,
                                            self.max_joint_speed * dt, self.damping, self.rot_weight, ROT_GAIN,
                                            lock=lock, pos_weight=0.0 if wrist_only else 1.0)
        self.report_if_blocked(v * dt, dx, locked, scale)
        at_limit = [i for i in locked if i not in lock]
        if w.any() and at_limit:
            self.get_logger().warn("Can't turn further that way: " + ", ".join(f"joint_{i + 1}" for i in at_limit)
                                   + " at its limit", throttle_duration_sec=1.0)
        if np.max(np.abs(q_new - self.q_cmd)) <= 1e-7:
            self.drop_lead()  # blocked: don't let the servos run on toward an old lead point
            return
        r_hold_new = rotation_from_vector(w * dt) @ self.r_hold if w.any() else self.r_hold
        qdot = 0.5 * self.qdot + 0.5 * (q_new - self.q_cmd) / dt
        q_pub = np.array([min(max(q + self.lead_time * qd, lo), hi)
                          for q, qd, (lo, hi) in zip(q_new, qdot, self.limits)])
        self.check_and_send(q_new, q_pub, qdot, r_hold_new)

    def joint_tick(self, now):
        """JOINT mode: the strongest puck motion drives its joint; the others are ignored."""
        lin, ang = self.puck(now)
        inputs = dict(zip(INPUTS, list(lin) + list(ang)))
        strength = np.array([abs(inputs[name]) for name in self.joint_inputs])
        if not strength.any():
            self.active_joint = None
            if now - self.last_active > self.resync_after and now - self.last_sync > 1.0:
                self.sync_from_measured(now)
            return
        best = int(np.argmax(strength))
        # Keep the current joint unless another motion is clearly stronger (no flicker between two)
        if self.active_joint is None or strength[best] > self.switch_margin * strength[self.active_joint]:
            self.active_joint = best
        j = self.active_joint
        if strength[j] == 0.0:
            return
        self.last_active = now
        if self.q_cmd is None:
            self.sync_from_measured(now)
        dt = now - self.last_step
        if dt <= 0.0 or dt > 0.2:
            dt = 1.0 / self.rate
        self.last_step = now
        scale = self.precision_scale if self.precision else 1.0
        step = self.joint_signs[j] * inputs[self.joint_inputs[j]] * self.joint_speeds[j] * scale * dt
        lo, hi = self.limits[j]
        qj = min(max(self.q_cmd[j] + step, lo), hi)
        if abs(qj - self.q_cmd[j]) <= 1e-7:
            self.get_logger().warn(f"Can't turn further that way: joint_{j + 1} at its limit",
                                   throttle_duration_sec=1.0)
            return
        q_new = self.q_cmd.copy()
        q_new[j] = qj
        qdot = np.zeros(6)
        qdot[j] = (qj - self.q_cmd[j]) / dt
        self.check_and_send(q_new, q_new, qdot, self.kin.fk(q_new)[0][:3, :3])

    def twist_index(self):
        """Index in w (tip x, y, z) that carries the puck twist (puck axis 3)."""
        return [abs(int(m)) for m in self.rot_axis_map].index(3)

    def is_twist(self, w):
        k = self.twist_index()
        tilt = np.delete(w, k)
        return abs(w[k]) >= np.linalg.norm(tilt)  # twist stronger than tilt: spin the gripper only

    def twist_step(self, w, dt):
        """Spin the gripper about its own axis: joint 6 only, no other joint moves. Direction: seen
        from above, a counter-clockwise puck twist turns the gripper counter-clockwise."""
        _, axes, _ = self.kin.fk(self.q_cmd)
        up = axes[5][2]  # how much joint 6 points up (+) or down (-)
        sign = 1.0 if up >= 0.0 else -1.0
        step = sign * w[self.twist_index()] * dt
        step = max(-self.max_joint_speed * dt, min(self.max_joint_speed * dt, step))
        lo, hi = self.limits[5]
        q6 = min(max(self.q_cmd[5] + step, lo), hi)
        if abs(q6 - self.q_cmd[5]) <= 1e-7:
            self.get_logger().warn("Can't turn further that way: joint_6 at its limit", throttle_duration_sec=1.0)
            self.drop_lead()
            return
        q_new = self.q_cmd.copy()
        q_new[5] = q6
        qdot = np.zeros(6)
        qdot[5] = (q6 - self.q_cmd[5]) / dt
        self.check_and_send(q_new, q_new, qdot, self.kin.fk(q_new)[0][:3, :3])

    def drop_lead(self):
        if self.lead_active and self.q_cmd is not None:
            self.send(self.q_cmd)
        self.lead_active = False
        self.qdot = np.zeros(6)

    def sync_from_measured(self, now):
        self.last_sync = now
        self.q_cmd = np.array(self.measured, dtype=float)
        self.r_hold = self.kin.fk(self.q_cmd)[0][:3, :3]
        self.qdot = np.zeros(6)

    def report_if_blocked(self, dp, dx, locked, scale):
        n = float(np.dot(dp, dp))
        if n < 1e-14 or float(np.dot(dx, dp)) / n >= 0.3:
            return
        if locked:
            reason = ", ".join(f"joint_{i + 1}" for i in locked) + " at its limit"
        elif scale < 0.5:
            reason = "near a singularity (joints would have to move too fast)"
        else:
            reason = "edge of reach or singularity (if the arm is fully stretched, hold the left button for the ready pose)"
        self.get_logger().warn(f"Can't go further that way: {reason}", throttle_duration_sec=1.0)

    def check_and_send(self, q_new, q_pub, qdot, r_hold_new):
        if not self.valid_cli.service_is_ready():
            self.get_logger().warn("Waiting for /check_state_validity (is the arm launch running?)",
                                   throttle_duration_sec=5.0)
            return
        req = GetStateValidity.Request()
        req.group_name = self.group
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = [float(x) for x in q_pub]  # check what is actually sent
        self.busy = True
        self.valid_cli.call_async(req).add_done_callback(
            lambda f: self.on_valid(f, q_new, q_pub, qdot, r_hold_new))

    def on_valid(self, fut, q_new, q_pub, qdot, r_hold_new):
        self.busy = False
        if self.stopped or self.homing is not None or self.q_cmd is None:
            return
        res = fut.result()
        if res is None or not res.valid:
            self.get_logger().warn("Can't go further that way: the move would collide (MoveIt collision model)",
                                   throttle_duration_sec=1.0)
            self.drop_lead()
            return
        self.q_cmd, self.qdot = q_new, qdot
        r_now = self.kin.fk(q_new)[0][:3, :3]
        # If the arm could not follow the requested rotation (limit, reach), hold what it reached
        if np.linalg.norm(rotation_error(r_hold_new, r_now)) > np.radians(5.0):
            r_hold_new = r_now
        self.r_hold = r_hold_new
        self.send(q_pub)
        self.lead_active = bool(np.any(q_pub != q_new))

    def send(self, q):
        self.pub.publish(Float64MultiArray(data=[float(x) for x in q]))

    # ---------- home button ----------
    def start_homing(self):
        if self.stopped or self.problems or self.kin is None or self.measured is None:
            return
        if self.ready is None:
            self.get_logger().warn("Home button disabled (bad ready_pose_deg)")
            return
        q_start = np.array(self.q_cmd if self.q_cmd is not None else self.measured, dtype=float)
        if float(np.max(np.abs(self.ready - q_start))) < 0.01:
            self.get_logger().info("Already at the ready pose")
            return
        lowest = path_min_height(self.kin, q_start, self.ready)
        if lowest < self.min_height:
            self.get_logger().warn(f"Not homing: on the way a joint or the tip would come down to "
                                   f"{lowest * 100:.0f} cm above the base (minimum {self.min_height * 100:.0f} cm). "
                                   "Move the arm up first.")
            return
        duration = max(2.0, float(np.max(np.abs(self.ready - q_start))) / self.home_speed)
        self.homing = (time.monotonic(), duration, q_start, self.ready.copy())
        self.get_logger().info(f"Moving to the ready pose ({duration:.1f} s). Press any button to stop.")

    def homing_step(self, now):
        t0, duration, q_start, q_goal = self.homing
        s = smoothstep((now - t0) / duration)
        q = q_start + s * (q_goal - q_start)
        self.q_cmd = q
        self.send(q)
        if s >= 1.0:
            self.homing = None
            self.r_hold = self.kin.fk(q)[0][:3, :3]
            self.last_active = now  # re-read the real pose once the arm has settled
            self.get_logger().info("At the ready pose")


def main():
    rclpy.init()
    node = SpacemouseTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
