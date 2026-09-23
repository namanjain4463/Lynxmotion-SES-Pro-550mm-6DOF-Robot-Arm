#!/usr/bin/env python3
"""Interactive axis_map calibration for the spacenav_arm_bridge teleop node.

Needs: arm launch running in PLANNING mode (arm_trajectory_controller active, i.e. teleop
NOT running), spacenav_node running, arm in a bent pose, arm's reach clear.

1. Moves the tip 3 cm along the arm's +x, +y, +z axes (and back) and asks you which way
   it moved from where you sit.
2. Asks you to push the puck away from you, to your left, and up, and records which puck
   axis each produces.
3. Prints the axis_map that makes the tip follow your hand.
"""
import copy
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, Twist
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = [f"pro_arm_joint_{i}" for i in range(1, 7)]
BASE, EE, GROUP = "pro_arm_base_link", "pro_arm_ee", "pro_arm"
STEP_M = 0.03           # tip test move
MOVE_S = 2.0            # seconds per test move
MAX_JOINT_CHANGE = 0.35  # rad; refuse test moves that swing a joint more than this
AXES = "xyz"
# answer letter -> (operator direction, sign)
ANSWERS = {"a": ("away", 1), "t": ("away", -1), "l": ("left", 1),
           "r": ("left", -1), "u": ("up", 1), "d": ("up", -1)}
PUSH_TEXT = {"away": "AWAY from you", "left": "to your LEFT", "up": "UP (pull the cap up)"}


class Calib(Node):
    def __init__(self):
        super().__init__("axis_calib")
        self.pos = None
        self.twist = Twist()
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.create_subscription(Twist, "/spacenav/twist", self.on_twist, 10)
        self.pub = self.create_publisher(JointTrajectory, "/arm_trajectory_controller/joint_trajectory", 10)
        self.fk_cli = self.create_client(GetPositionFK, "/compute_fk")
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik")

    def on_js(self, msg):
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in JOINTS):
            self.pos = [d[j] for j in JOINTS]

    def on_twist(self, msg):
        self.twist = msg

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def call(self, client, req):
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        return fut.result()

    def fk_pose(self, q):
        req = GetPositionFK.Request()
        req.header.frame_id = BASE
        req.fk_link_names = [EE]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = list(q)
        res = self.call(self.fk_cli, req)
        if res is None or res.error_code.val != 1 or not res.pose_stamped:
            return None
        return res.pose_stamped[0].pose

    def ik(self, pose, seed):
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = GROUP
        r.ik_link_name = EE
        r.avoid_collisions = True
        r.robot_state.joint_state.name = JOINTS
        r.robot_state.joint_state.position = list(seed)
        ps = PoseStamped()
        ps.header.frame_id = BASE
        ps.pose = pose
        r.pose_stamped = ps
        r.timeout = Duration(sec=0, nanosec=200_000_000)
        res = self.call(self.ik_cli, req)
        if res is None or res.error_code.val != 1:
            return None
        d = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [d[j] for j in JOINTS]

    def move_joints(self, q):
        pt = JointTrajectoryPoint()
        pt.positions = list(q)
        pt.time_from_start = Duration(sec=int(MOVE_S), nanosec=int((MOVE_S % 1) * 1e9))
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        msg.points = [pt]
        self.pub.publish(msg)
        self.spin_for(MOVE_S + 1.5)  # the arm starts ~0.4 s late

    def record_push(self):
        """Wait for a clear push, average it for 1 s, return (axis index, sign)."""
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            self.spin_for(0.02)
            lin = [self.twist.linear.x, self.twist.linear.y, self.twist.linear.z]
            if max(abs(v) for v in lin) > 0.25:
                break
        else:
            return None
        sums, end = [0.0, 0.0, 0.0], time.monotonic() + 1.0
        while time.monotonic() < end:
            self.spin_for(0.02)
            for k, v in enumerate((self.twist.linear.x, self.twist.linear.y, self.twist.linear.z)):
                sums[k] += v
        idx = max(range(3), key=lambda k: abs(sums[k]))
        return idx, (1 if sums[idx] > 0 else -1)

    def wait_release(self):
        end = time.monotonic() + 10.0
        while time.monotonic() < end:
            self.spin_for(0.05)
            lin = [self.twist.linear.x, self.twist.linear.y, self.twist.linear.z]
            if max(abs(v) for v in lin) < 0.1:
                self.spin_for(0.5)
                return


def ask(prompt, valid):
    while True:
        a = input(prompt).strip().lower()[:1]
        if a in valid:
            return a
        print(f"  please type one of: {', '.join(valid)}")


def run(n):
    while n.pos is None:
        rclpy.spin_once(n, timeout_sec=0.1)
    if not (n.fk_cli.wait_for_service(timeout_sec=10.0) and n.ik_cli.wait_for_service(timeout_sec=10.0)):
        print("/compute_fk or /compute_ik not available - is the arm launch running?")
        return
    t0 = time.monotonic()
    while n.pub.get_subscription_count() == 0:
        if time.monotonic() - t0 > 5.0:
            print("arm_trajectory_controller not found - is the arm launch running?")
            return
        n.spin_for(0.1)

    q0 = list(n.pos)
    p0 = n.fk_pose(q0)
    if p0 is None:
        print("FK failed")
        return

    print("\nPART 1: the arm moves its tip 3 cm along each of its own axes, then back.")
    print("Teleop must NOT be running (planning mode). Keep the arm's reach clear.")
    input("Press Enter to start... ")

    base_dir = {}  # arm axis k -> (operator direction, sign): moving +k looks like sign*direction
    for k in range(3):
        q, step_sign = None, 0
        for sign in (1, -1):  # if +3 cm is out of reach from this pose, -3 cm tells us the same thing
            target = copy.deepcopy(p0)
            setattr(target.position, AXES[k], getattr(p0.position, AXES[k]) + sign * STEP_M)
            sol = n.ik(target, q0)
            if sol is not None and max(abs(a - b) for a, b in zip(sol, q0)) <= MAX_JOINT_CHANGE:
                q, step_sign = sol, sign
                break
        if q is None:
            print(f"\nCan't move the tip 3 cm along the arm's {AXES[k]} axis from this pose.\n"
                  "Move the arm to the calibration pose in RViz and rerun:\n"
                  "  joint_1 = 0, joint_2 = -50 deg, joint_3 = -60 deg, joint_4 = 0, joint_5 = -40 deg, joint_6 = 0")
            return
        print(f"\nMoving the tip 3 cm along the arm's {'+' if step_sign > 0 else '-'}{AXES[k]} axis - watch the tip...")
        n.move_joints(q)
        a = ask("Which way did the tip move?  [a]way from you / [t]oward you / [l]eft / "
                "[r]ight / [u]p / [d]own / [n]othing: ", "atlrudn")
        print("Moving back...")
        n.move_joints(q0)
        if a == "n":
            print("The arm did not move. Stop the teleop launch, switch back to planning mode:\n"
                  "  ros2 control switch_controllers --deactivate forward_position_controller "
                  "--activate arm_trajectory_controller\nthen rerun.")
            return
        d, s = ANSWERS[a]
        base_dir[k] = (d, s * step_sign)  # convert to what the +axis direction looks like

    dirs = sorted(d for d, _ in base_dir.values())
    if dirs != ["away", "left", "up"]:
        print(f"\nThe three answers should be three different directions, got {dirs}. Rerun and watch closely.")
        return
    dir_to_base = {d: (k, s) for k, (d, s) in base_dir.items()}

    print("\nPART 2: SpaceMouse. For each prompt, push the puck firmly in that direction and hold "
          "about 2 seconds, then let go. (The arm will not move.)")
    dir_to_puck = {}
    for d in ("away", "left", "up"):
        print(f"\nPush the puck {PUSH_TEXT[d]} and hold...")
        got = n.record_push()
        if got is None:
            print("No push detected in 20 s - is spacenav_node running?")
            return
        idx, s = got
        print(f"  recorded: puck {'+' if s > 0 else '-'}{AXES[idx]}")
        dir_to_puck[d] = got
        n.wait_release()
    if sorted(i for i, _ in dir_to_puck.values()) != [0, 1, 2]:
        print("Two pushes landed on the same puck axis. Rerun and push more cleanly.")
        return

    axis_map = [0, 0, 0]
    for d in ("away", "left", "up"):
        k, sb = dir_to_base[d]
        idx, sp = dir_to_puck[d]
        axis_map[k] = sb * sp * (idx + 1)

    print("\nRESULT")
    for d in ("away", "left", "up"):
        k, sb = dir_to_base[d]
        print(f"  {d:>5} from you = arm {'+' if sb > 0 else '-'}{AXES[k]}")
    print(f"  axis_map = {axis_map}")
    print("Put this in launch/spacemouse_teleop.launch.py:")
    print(f'  parameters=[{{"linear_speed": 0.05, "angular_speed": 0.0, "axis_map": {axis_map}}}],')


def main():
    rclpy.init()
    n = Calib()
    try:
        run(n)
    except (KeyboardInterrupt, EOFError):
        print("\nStopped.")
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
