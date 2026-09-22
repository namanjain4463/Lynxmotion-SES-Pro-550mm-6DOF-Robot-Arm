#!/usr/bin/env python3
"""python3 ~/tip_watch.py SECONDS
Every 0.5 s prints the SpaceMouse push (puck x/y/z) and how far the arm tip has moved
since the start, in cm, in the arm base frame (pro_arm_base_link)."""
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from moveit_msgs.srv import GetPositionFK
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = [f"pro_arm_joint_{i}" for i in range(1, 7)]


class TipWatch(Node):
    def __init__(self):
        super().__init__("tip_watch")
        self.pos = None
        self.twist = Twist()
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.create_subscription(Twist, "/spacenav/twist", self.on_twist, 10)
        self.fk = self.create_client(GetPositionFK, "/compute_fk")

    def on_js(self, msg):
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in JOINTS):
            self.pos = [d[j] for j in JOINTS]

    def on_twist(self, msg):
        self.twist = msg

    def tip(self):
        req = GetPositionFK.Request()
        req.header.frame_id = "pro_arm_base_link"
        req.fk_link_names = ["pro_arm_ee"]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = list(self.pos)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)
        res = fut.result()
        if res is None or res.error_code.val != 1 or not res.pose_stamped:
            return None
        p = res.pose_stamped[0].pose.position
        return (p.x, p.y, p.z)


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) == 2 else 40.0
    rclpy.init()
    node = TipWatch()
    try:
        while node.pos is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.fk.wait_for_service(timeout_sec=5.0):
            print("/compute_fk not available - is the arm launch running?")
            return
        start = node.tip()
        if start is None:
            print("FK failed")
            return
        print(f"Watching {seconds:.0f} s. puck = SpaceMouse push, tip = cm moved since start (arm base frame)")
        t0 = time.monotonic()
        next_print = 0.0
        while time.monotonic() - t0 < seconds:
            rclpy.spin_once(node, timeout_sec=0.02)
            t = time.monotonic() - t0
            if t >= next_print:
                cur = node.tip()
                tw = node.twist.linear
                if cur is not None:
                    dx, dy, dz = ((c - s) * 100.0 for c, s in zip(cur, start))
                    print(f"  t={t:5.1f}s  puck x{tw.x:+.2f} y{tw.y:+.2f} z{tw.z:+.2f}  |  "
                          f"tip x{dx:+6.1f} y{dy:+6.1f} z{dz:+6.1f} cm")
                next_print += 0.5
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
