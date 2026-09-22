#!/usr/bin/env python3
"""Usage (source the arm workspace first):
  python3 ~/arm_timer.py move JOINT TARGET_RAD SECONDS          one timed move (Servo stopped)
  python3 ~/arm_timer.py stream JOINT DEG_PER_S SECONDS MODE    MODE = open or anchored (Servo stopped)
  python3 ~/arm_timer.py watch SECONDS                          live joint changes
  python3 ~/arm_timer.py servo SECONDS                          Servo command vs actual (Servo running)
"""
import math
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = [f"pro_arm_joint_{i}" for i in range(1, 7)]
TOPIC = "/arm_trajectory_controller/joint_trajectory"
MAX_STEP_RAD = 0.6  # refuse moves bigger than ~34 deg for safety
PERIOD = 0.1        # same update period Servo uses now
deg = math.degrees


class ArmTimer(Node):
    def __init__(self):
        super().__init__("arm_timer")
        self.pos = None
        self.cmd = None
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.create_subscription(JointTrajectory, TOPIC, self.on_cmd, 10)
        self.pub = self.create_publisher(JointTrajectory, TOPIC, 10)

    def on_js(self, msg):
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in JOINTS):
            self.pos = [d[j] for j in JOINTS]

    def on_cmd(self, msg):
        if msg.points:
            d = dict(zip(msg.joint_names, msg.points[0].positions))
            if all(j in d for j in JOINTS):
                self.cmd = [d[j] for j in JOINTS]

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def wait_state(self):
        while self.pos is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        return list(self.pos)

    def wait_subscriber(self):
        while self.pub.get_subscription_count() < 2:  # controller + our own echo subscription
            self.spin_for(0.1)

    def send(self, positions, seconds):
        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        msg.points = [pt]
        self.pub.publish(msg)


def move(node, joint, target, seconds):
    i = joint - 1
    start = node.wait_state()
    if abs(target - start[i]) > MAX_STEP_RAD:
        print(f"Refusing: joint_{joint} at {start[i]:.3f}, target {target:.3f} is too far.")
        return
    node.wait_subscriber()
    goal = list(start)
    goal[i] = target
    print(f"joint_{joint}: {deg(start[i]):.1f} -> {deg(target):.1f} deg, commanded in {seconds:.2f} s")
    t0 = time.monotonic()
    node.send(goal, seconds)
    reached, last_print = None, -1.0
    while time.monotonic() - t0 < seconds + 10:
        node.spin_for(0.02)
        t = time.monotonic() - t0
        if t - last_print >= 0.25:
            print(f"  t={t:5.2f}s  joint_{joint}={deg(node.pos[i]):7.2f} deg")
            last_print = t
        if abs(node.pos[i] - target) < 0.01:
            reached = t
            break
    moved = deg(abs(node.pos[i] - start[i]))
    if reached is None:
        print(f"NOT reached within {seconds + 10:.0f} s. Moved {moved:.1f} deg.")
    else:
        print(f"REACHED in {reached:.2f} s (commanded {seconds:.2f} s). Moved {moved:.1f} deg, avg {moved / reached:.1f} deg/s")


def stream(node, joint, deg_s, seconds, mode):
    i = joint - 1
    start = node.wait_state()
    if math.radians(abs(deg_s)) * seconds > MAX_STEP_RAD:
        print("Refusing: total motion too big.")
        return
    node.wait_subscriber()
    goal = list(start)
    step = math.radians(deg_s) * PERIOD
    print(f"{mode} stream: joint_{joint} at {deg_s:+.1f} deg/s for {seconds:.1f} s, new target every {PERIOD} s")
    t0, next_pub, last_print = time.monotonic(), 0.0, -1.0
    while True:
        t = time.monotonic() - t0
        if t >= seconds + 3.0:
            break
        if t < seconds and t >= next_pub:
            base = node.pos[i] if mode == "anchored" else goal[i]
            goal[i] = base + step
            node.send(goal, PERIOD)
            next_pub += PERIOD
        if t - last_print >= 0.5:
            print(f"  t={t:4.1f}s  actual={deg(node.pos[i]):7.2f}  target={deg(goal[i]):7.2f} deg")
            last_print = t
        node.spin_for(0.005)
    moved = deg(node.pos[i] - start[i])
    asked = deg_s * seconds
    print(f"RESULT {mode}: asked {asked:+.1f} deg, arm moved {moved:+.1f} deg ({100 * moved / asked:.0f}%)")


def watch(node, seconds):
    start = node.wait_state()
    print(f"Watching {seconds:.0f} s. Degrees moved since start (j1..j6):")
    t0, last_print = time.monotonic(), -1.0
    while time.monotonic() - t0 < seconds:
        node.spin_for(0.02)
        t = time.monotonic() - t0
        if t - last_print >= 0.5:
            print(f"  t={t:5.1f}s  " + "  ".join(f"{deg(p - s):+6.1f}" for p, s in zip(node.pos, start)))
            last_print = t


def servo(node, seconds):
    start = node.wait_state()
    print(f"Logging {seconds:.0f} s. 'moved' = deg since start, 'lead' = Servo command minus actual (deg), j1..j6")
    t0, last_print = time.monotonic(), -1.0
    while time.monotonic() - t0 < seconds:
        node.spin_for(0.02)
        t = time.monotonic() - t0
        if t - last_print >= 0.5:
            moved = " ".join(f"{deg(p - s):+6.1f}" for p, s in zip(node.pos, start))
            lead = " ".join(f"{deg(c - p):+5.2f}" for c, p in zip(node.cmd, node.pos)) if node.cmd else "no Servo commands yet"
            print(f"  t={t:4.1f}s  moved: {moved}  | lead: {lead}")
            last_print = t


def main():
    rclpy.init()
    node = ArmTimer()
    a = sys.argv
    try:
        if len(a) == 5 and a[1] == "move":
            move(node, int(a[2]), float(a[3]), float(a[4]))
        elif len(a) == 6 and a[1] == "stream" and a[5] in ("open", "anchored"):
            stream(node, int(a[2]), float(a[3]), float(a[4]), a[5])
        elif len(a) == 3 and a[1] == "watch":
            watch(node, float(a[2]))
        elif len(a) == 3 and a[1] == "servo":
            servo(node, float(a[2]))
        else:
            print(__doc__)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
