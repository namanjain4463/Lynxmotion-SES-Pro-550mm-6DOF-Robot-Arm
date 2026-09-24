"""ROS 2 node for the DH-Robotics CGE-10-10 gripper (Modbus RTU over a USB-RS485 adapter).

Services (std_srvs/Trigger): /gripper/open, /gripper/close, /gripper/toggle
Topics in:  /gripper/set_position (std_msgs/Int32, 0 = closed ... 1000 = open, per mille)
            /gripper/set_force    (std_msgs/Int32, 20-100 %)
Topics out (rate Hz): /gripper/position (Int32, per mille), /gripper/grip_state (Int32:
            0 moving, 1 at target, 2 object caught, 3 object dropped), /gripper/object_caught (Bool),
            /gripper/diameter_mm (Float32, estimated object size while holding, NaN otherwise)
Grasp events (grasp_monitor.py): /gripper/event (std_msgs/String, JSON): CAUGHT, SEATED, SLIP,
            LOST (GRADUAL/SUDDEN, with arm motion from /joint_states), RELEASED, MISSED
Diagnostics: /diagnostics (connection, Modbus errors, read time, poll rate, grasp phase)
Optional CSV logs (log_dir): every state sample, every grasp event, and the diagnostics (1 Hz).

On start it initialises the gripper if needed (the fingers move). On shutdown it only closes the
serial port: the gripper keeps its last position, so a held object is not dropped.
No WSL-specific code: set `port` for the adapter (e.g. /dev/ttyUSB0, or a udev symlink).
"""
import csv
import json
import math
import os
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Int32, String
from std_srvs.srv import Trigger

from spacenav_arm_bridge.dh_gripper import (GRIP_CAUGHT, GRIP_TEXT, DHGripper, ModbusError)
from spacenav_arm_bridge.grasp_monitor import GraspMonitor


class GripperNode(Node):
    def __init__(self):
        super().__init__("gripper")
        p = self.declare_parameter
        self.port = p("port", "/dev/ttyUSB0").value
        self.baud = p("baud", 115200).value
        self.slave_id = p("slave_id", 1).value
        self.force_pct = p("force_pct", 20).value      # 20-100; the grip force limit
        self.speed_pct = p("speed_pct", 50).value      # 1-100
        self.open_pos = p("open_position", 1000).value   # per mille (1000 = fully open, measured)
        self.closed_pos = p("closed_position", 0).value
        rate = p("rate", 10.0).value
        log_dir = p("log_dir", "").value               # "" = no CSV log
        # Object diameter (mm) at the open and closed ends, for the size estimate. PROVISIONAL
        # (2026-09-23): one 51 mm cylinder caught at ~930-985 per mille plus DH's 10 mm stroke per
        # jaw (20 mm in diameter); confirm with a second known object.
        d_open = p("diameter_open_mm", 51.6).value
        d_closed = p("diameter_closed_mm", 31.6).value
        self.monitor = GraspMonitor(self.open_pos, self.closed_pos,
                                    seat_time=p("seat_time", 1.5).value,
                                    slip_permille=p("slip_permille", 20).value,
                                    diameter_open_mm=d_open, diameter_closed_mm=d_closed,
                                    # measured at speed 50 %: in-contact creep <= 130, free close >= 1600
                                    contact_loss_speed=p("contact_loss_speed", 500.0).value)
        self.arm_moving_deg = p("arm_moving_deg", 0.5).value  # joint change in 0.5 s that counts as moving

        self.grip = None
        self.target = None       # last commanded position
        self.last_state = None
        self.pub_pos = self.create_publisher(Int32, "/gripper/position", 10)
        self.pub_state = self.create_publisher(Int32, "/gripper/grip_state", 10)
        self.pub_caught = self.create_publisher(Bool, "/gripper/object_caught", 10)
        self.pub_diam = self.create_publisher(Float32, "/gripper/diameter_mm", 10)
        self.pub_event = self.create_publisher(String, "/gripper/event", 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
        self.arm_hist = []          # (t, joint positions) for the last ~0.5 s
        self.stats = {"polls": 0, "errors": 0, "reconnects": 0, "read_s": 0.0, "last_error": ""}
        self.diag_t = time.monotonic()
        self.create_service(Trigger, "/gripper/open", lambda req, res: self.srv_move(res, self.open_pos, "open"))
        self.create_service(Trigger, "/gripper/close", lambda req, res: self.srv_move(res, self.closed_pos, "close"))
        self.create_service(Trigger, "/gripper/toggle", self.srv_toggle)
        self.create_subscription(Int32, "/gripper/set_position", self.on_set_position, 10)
        self.create_subscription(Int32, "/gripper/set_force", self.on_set_force, 10)

        self.log = None
        if log_dir:
            os.makedirs(os.path.expanduser(log_dir), exist_ok=True)
            path = os.path.join(os.path.expanduser(log_dir), time.strftime("%Y%m%d_%H%M%S") + "_gripper.csv")
            self.log_file = open(path, "w", newline="")
            self.log = csv.writer(self.log_file)
            self.log.writerow(["t_s", "commanded_position", "force_pct", "initialised", "grip_state", "position"])
            self.event_file = open(path.replace("_gripper.csv", "_gripper_events.csv"), "w", newline="")
            self.event_log = csv.writer(self.event_file)
            self.event_log.writerow(["t_s", "event", "details_json"])
            self.diag_file = open(path.replace("_gripper.csv", "_gripper_diagnostics.csv"), "w", newline="")
            self.diag_log = csv.writer(self.diag_file)
            self.diag_log_header = False
            self.get_logger().info(f"Logging to {path} (+ _events.csv, _diagnostics.csv)")
        self.t0 = time.monotonic()
        self.create_timer(1.0 / rate, self.poll)
        self.connect()

    # ---------- connection ----------
    def connect(self):
        try:
            self.grip = DHGripper(self.port, self.baud, self.slave_id)
            initialised, grip_state, pos = self.grip.state()
            if not initialised:
                self.get_logger().info("Gripper not initialised: initialising now (the fingers move)")
                self.grip.initialise()
                t0 = time.monotonic()
                while not self.grip.state()[0]:
                    if time.monotonic() - t0 > 15.0:
                        raise ModbusError("initialisation did not finish within 15 s")
                    time.sleep(0.2)
            self.grip.set_force(self.force_pct)
            self.grip.set_speed(self.speed_pct)
            pos = self.grip.state()[2]
            self.target = pos
            self.last_pos = pos
            self.get_logger().info(f"Gripper ready on {self.port} (ID {self.slave_id}, {self.baud} baud): "
                                   f"position {pos} per mille, force {self.force_pct} %, speed {self.speed_pct} %")
            return True
        except (OSError, ModbusError) as e:  # serial.SerialException is an OSError
            self.get_logger().warn(f"Gripper not reachable on {self.port}: {e}. Retrying every 2 s.",
                                   throttle_duration_sec=10.0)
            self.stats["last_error"] = str(e)
            self.drop_connection()
            return False

    def drop_connection(self):
        if self.grip is not None:
            try:
                self.grip.close()
            except OSError:
                pass
        self.grip = None

    # ---------- state ----------
    def poll(self):
        if not self.context.ok():  # shutting down: nothing may be published any more
            return
        if self.grip is None:
            if time.monotonic() - getattr(self, "last_try", 0.0) > 2.0:
                self.last_try = time.monotonic()
                self.connect()
            return
        t_read = time.monotonic()
        try:
            initialised, grip_state, pos = self.grip.state()
        except (OSError, ModbusError) as e:
            self.get_logger().warn(f"Lost the gripper: {e}. Reconnecting.")
            self.stats["errors"] += 1
            self.stats["reconnects"] += 1
            self.stats["last_error"] = str(e)
            self.drop_connection()
            return
        now = time.monotonic()
        self.stats["polls"] += 1
        self.stats["read_s"] += now - t_read
        self.last_pos = pos
        self.pub_pos.publish(Int32(data=pos))
        self.pub_state.publish(Int32(data=grip_state))
        self.pub_caught.publish(Bool(data=grip_state == GRIP_CAUGHT))
        if grip_state != self.last_state and grip_state != 0:
            self.get_logger().info(f"Gripper: {GRIP_TEXT.get(grip_state, grip_state)} at {pos} per mille")
        self.last_state = grip_state
        t = round(now - self.t0, 3)
        if self.log is not None:
            self.log.writerow([t, self.target, self.force_pct, int(initialised), grip_state, pos])
        self.emit(self.monitor.sample(t, grip_state, pos, self.force_pct, self.arm_moving(now)))
        holding = self.monitor.phase == "holding"
        self.pub_diam.publish(Float32(data=self.monitor.diameter_mm(pos) if holding else float("nan")))
        if now - self.diag_t >= 1.0:
            self.publish_diagnostics(now, initialised, grip_state, pos)

    # ---------- grasp events and diagnostics ----------
    def emit(self, events):
        for ev in events:
            self.pub_event.publish(String(data=json.dumps(ev)))
            text = ", ".join(f"{k}={v}" for k, v in ev.items() if k not in ("event", "t"))
            if ev["event"] in ("LOST", "SLIP"):
                self.get_logger().warn(f"GRASP {ev['event']}: {text}")
            else:
                self.get_logger().info(f"GRASP {ev['event']}: {text}")
            if self.log is not None:
                self.event_log.writerow([ev["t"], ev["event"], json.dumps(ev)])
                self.event_file.flush()

    def on_joint_states(self, msg):
        if not msg.name or msg.name == ["gripper"]:
            return
        now = time.monotonic()
        self.arm_hist.append((now, list(msg.position)))
        while self.arm_hist and now - self.arm_hist[0][0] > 0.5:
            self.arm_hist.pop(0)

    def arm_moving(self, now):
        """True/False from /joint_states over the last 0.5 s; None if there is no recent arm data."""
        if not self.arm_hist or now - self.arm_hist[-1][0] > 1.0:
            return None
        first, last = self.arm_hist[0][1], self.arm_hist[-1][1]
        if len(first) != len(last):
            return None
        return max(abs(a - b) for a, b in zip(first, last)) > math.radians(self.arm_moving_deg)

    def publish_diagnostics(self, now, initialised, grip_state, pos):
        polls = self.stats["polls"]
        st = DiagnosticStatus(name="gripper: DH CGE-10-10", hardware_id=f"{self.port} id {self.slave_id}")
        st.level = DiagnosticStatus.OK if initialised else DiagnosticStatus.WARN
        st.message = f"{GRIP_TEXT.get(grip_state, grip_state)}, grasp phase {self.monitor.phase}"
        values = {"position_permille": pos, "force_pct": self.force_pct, "speed_pct": self.speed_pct,
                  "poll_rate_hz": round(polls / (now - self.diag_t), 1),
                  "mean_read_ms": round(1000 * self.stats["read_s"] / max(1, polls), 1),
                  "modbus_errors_total": self.stats["errors"], "reconnects_total": self.stats["reconnects"],
                  "last_error": self.stats["last_error"]}
        st.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        if self.log is not None:
            if not self.diag_log_header:
                self.diag_log.writerow(["t_s", "level", "message"] + list(values))
                self.diag_log_header = True
            self.diag_log.writerow([round(now - self.t0, 3), int.from_bytes(st.level, "little")
                                    if isinstance(st.level, bytes) else int(st.level), st.message]
                                   + list(values.values()))
            self.diag_file.flush()
            self.log_file.flush()  # the state log too, so a killed node loses at most ~1 s of samples
        arr = DiagnosticArray(status=[st])
        arr.header.stamp = self.get_clock().now().to_msg()
        self.pub_diag.publish(arr)
        self.diag_t = now
        self.stats["polls"] = 0
        self.stats["read_s"] = 0.0

    # ---------- commands ----------
    def move(self, target, what):
        if self.grip is None:
            return False, f"gripper not connected ({self.port})"
        try:
            self.grip.set_position(target)
        except (OSError, ModbusError) as e:
            self.stats["errors"] += 1
            self.stats["last_error"] = str(e)
            self.drop_connection()
            return False, f"gripper command failed: {e}"
        self.target = target
        self.emit(self.monitor.command(round(time.monotonic() - self.t0, 3), target, self.force_pct,
                                       getattr(self, "last_pos", target)))
        self.get_logger().info(f"Gripper {what} (target {target} per mille, force {self.force_pct} %)")
        return True, what

    def srv_move(self, res, target, what):
        res.success, res.message = self.move(target, what)
        return res

    def srv_toggle(self, req, res):
        # Close unless the last command already closed it (or it is holding something)
        closing = not (self.target is not None and self.target <= (self.open_pos + self.closed_pos) / 2
                       or self.last_state == GRIP_CAUGHT)
        return self.srv_move(res, self.closed_pos if closing else self.open_pos, "close" if closing else "open")

    def on_set_position(self, msg):
        ok, text = self.move(int(msg.data), "move")
        if not ok:
            self.get_logger().warn(text)

    def on_set_force(self, msg):
        if self.grip is None:
            self.get_logger().warn("gripper not connected")
            return
        try:
            self.grip.set_force(msg.data)
            self.force_pct = int(min(100, max(20, msg.data)))
            self.get_logger().info(f"Gripper force {self.force_pct} %")
        except (OSError, ModbusError) as e:
            self.get_logger().warn(f"gripper force command failed: {e}")
            self.drop_connection()

    def shutdown(self):
        self.drop_connection()  # the gripper keeps its position; nothing held is dropped
        if self.log is not None:
            self.log_file.close()
            self.event_file.close()
            self.diag_file.close()


def main():
    rclpy.init()
    node = GripperNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
