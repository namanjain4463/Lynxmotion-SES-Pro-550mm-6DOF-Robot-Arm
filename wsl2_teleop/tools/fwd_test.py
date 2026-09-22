#!/usr/bin/env python3
"""python3 ~/fwd_test.py JOINT DEG_PER_S SECONDS MODE   (MODE = open or anchored)
Streams positions to forward_position_controller at 30 Hz and reports how far the arm followed."""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINTS = [f"pro_arm_joint_{i}" for i in range(1, 7)]
TOPIC = "/forward_position_controller/commands"
RATE = 30.0
MAX_TOTAL_RAD = 0.6
deg = math.degrees


class FwdTest(Node):
    def __init__(self):
        super().__init__("fwd_test")
        self.pos = None
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.pub = self.create_publisher(Float64MultiArray, TOPIC, 10)

    def on_js(self, msg):
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in JOINTS):
            self.pos = [d[j] for j in JOINTS]

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def send(self, positions):
        self.pub.publish(Float64MultiArray(data=list(positions)))


def main():
    a = sys.argv
    if len(a) != 5 or a[4] not in ("open", "anchored"):
        print(__doc__)
        return
    joint, deg_s, seconds, mode = int(a[1]), float(a[2]), float(a[3]), a[4]
    rclpy.init()
    node = FwdTest()
    try:
        while node.pos is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        if math.radians(abs(deg_s)) * seconds > MAX_TOTAL_RAD:
            print("Refusing: total motion too big.")
            return
        t_wait = time.monotonic()
        while node.pub.get_subscription_count() == 0:
            if time.monotonic() - t_wait > 5:
                print("forward_position_controller not found. Did the spawner step work?")
                return
            node.spin_for(0.1)
        i = joint - 1
        start = list(node.pos)
        goal = list(start)
        step = math.radians(deg_s) / RATE
        print(f"{mode} stream (forward controller): joint_{joint} at {deg_s:+.1f} deg/s for {seconds:.1f} s")
        t0, next_pub, last_print = time.monotonic(), 0.0, -1.0
        while True:
            t = time.monotonic() - t0
            if t >= seconds + 3.0:
                break
            if t < seconds and t >= next_pub:
                base = node.pos[i] if mode == "anchored" else goal[i]
                goal[i] = base + step
                node.send(goal)
                next_pub += 1.0 / RATE
            if t - last_print >= 0.5:
                print(f"  t={t:4.1f}s  actual={deg(node.pos[i]):7.2f}  target={deg(goal[i]):7.2f} deg")
                last_print = t
            node.spin_for(0.005)
        moved = deg(node.pos[i] - start[i])
        asked = deg_s * seconds
        print(f"RESULT {mode}: asked {asked:+.1f} deg, arm moved {moved:+.1f} deg ({100 * moved / asked:.0f}%)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
