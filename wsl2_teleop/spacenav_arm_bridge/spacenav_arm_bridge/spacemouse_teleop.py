import math
import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, Twist
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import InteractiveMarkerFeedback

JOINTS = [f"pro_arm_joint_{i}" for i in range(1, 7)]


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_from_rotvec(wx, wy, wz):
    angle = math.sqrt(wx * wx + wy * wy + wz * wz)
    if angle < 1e-9:
        return (0.0, 0.0, 0.0, 1.0)
    s = math.sin(angle / 2) / angle
    return (wx * s, wy * s, wz * s, math.cos(angle / 2))


def quat_nlerp(a, b, s):
    if sum(x * y for x, y in zip(a, b)) < 0.0:
        b = tuple(-x for x in b)
    q = [x + s * (y - x) for x, y in zip(a, b)]
    n = math.sqrt(sum(x * x for x in q))
    return tuple(x / n for x in q)


class SpacemouseTeleop(Node):
    """Keeps its own tip target (open loop) and streams IK solutions to forward_position_controller.

    It does not restart from the measured position every cycle: on this arm that loses
    over 90% of the motion because of the ~0.35 s servo response delay.
    """

    def __init__(self):
        super().__init__("spacemouse_teleop")
        p = self.declare_parameter
        self.rate = p("rate", 30.0).value
        self.linear_speed = p("linear_speed", 0.05).value      # m/s at full puck deflection
        self.angular_speed = p("angular_speed", 0.0).value     # rad/s at full deflection, 0 = no rotation
        self.deadband = p("deadband", 0.05).value
        # Tip axis x, y, z <- puck axis number (1=x, 2=y, 3=z); negative number = reversed
        self.axis_map = list(p("axis_map", [1, 2, 3]).value)
        self.max_joint_speed = p("max_joint_speed", 0.6).value  # rad/s; faster requests are slowed down
        self.flip_threshold = p("flip_threshold", 0.1).value    # rad in one step = IK flip, refused
        self.resync_after = p("resync_after", 1.0).value        # s idle before re-reading the real position
        self.base_frame = p("base_frame", "pro_arm_base_link").value
        self.ee_link = p("ee_link", "pro_arm_ee").value
        self.group = p("group", "pro_arm").value

        if sorted(abs(int(m)) for m in self.axis_map) != [1, 2, 3]:
            self.get_logger().error(f"Bad axis_map {self.axis_map}, using [1, 2, 3]")
            self.axis_map = [1, 2, 3]

        self.measured = None
        self.q_target = None
        self.pose = None
        self.twist = Twist()
        self.twist_time = 0.0
        self.last_active = 0.0
        self.last_sync = 0.0
        self.last_step = 0.0
        self.busy = False
        self.stopped = False

        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.create_subscription(Twist, "/spacenav/twist", self.on_twist, 10)
        self.create_subscription(InteractiveMarkerFeedback, "/emergency_stop_button/feedback",
                                 self.on_estop, 10)
        self.pub = self.create_publisher(Float64MultiArray, "/forward_position_controller/commands", 10)
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info("SpaceMouse teleop ready. Push the puck to move the arm tip. "
                               "RViz STOP pauses teleop.")

    def on_js(self, msg):
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in JOINTS):
            self.measured = [d[j] for j in JOINTS]

    def on_twist(self, msg):
        self.twist = msg
        self.twist_time = time.monotonic()

    def on_estop(self, msg):
        if msg.event_type != InteractiveMarkerFeedback.BUTTON_CLICK or msg.marker_name != "emergency_stop":
            return
        self.stopped = not self.stopped
        self.pose = None
        if self.stopped:
            self.get_logger().warn("RViz STOP pressed: teleop paused. Click STOP again to resume.")
        else:
            self.get_logger().info("RViz STOP released: teleop resumed.")

    def mapped(self, values):
        out = []
        for m in self.axis_map:
            m = int(m)
            out.append(values[abs(m) - 1] * (1.0 if m > 0 else -1.0))
        return out

    def command(self, now):
        if now - self.twist_time > 0.2:
            return [0.0] * 3, [0.0] * 3
        db = lambda x: x if abs(x) > self.deadband else 0.0
        lin = [db(self.twist.linear.x), db(self.twist.linear.y), db(self.twist.linear.z)]
        ang = [db(self.twist.angular.x), db(self.twist.angular.y), db(self.twist.angular.z)]
        v = [x * self.linear_speed for x in self.mapped(lin)]
        w = [x * self.angular_speed for x in self.mapped(ang)]
        return v, w

    def tick(self):
        if self.busy or self.measured is None or self.stopped:
            return
        if not (self.fk.service_is_ready() and self.ik.service_is_ready()):
            self.get_logger().warn("Waiting for /compute_fk and /compute_ik (is the arm launch running?)",
                                   throttle_duration_sec=5.0)
            return
        now = time.monotonic()
        v, w = self.command(now)
        if not any(v + w):
            # Idle: refresh the start point from the real arm so the next push starts at once
            if now - self.last_active > self.resync_after and now - self.last_sync > 1.0:
                self.sync_from_measured(now)
            return
        self.last_active = now
        if self.pose is None:
            self.sync_from_measured(now)
            return
        dt = now - self.last_step
        if dt <= 0.0 or dt > 0.2:
            dt = 1.0 / self.rate
        self.last_step = now
        pos, quat = self.pose
        new_pos = tuple(pos[k] + v[k] * dt for k in range(3))
        new_quat = quat_multiply(quat_from_rotvec(w[0] * dt, w[1] * dt, w[2] * dt), quat)
        self.request_ik(new_pos, new_quat, dt)

    def sync_from_measured(self, now):
        self.last_sync = now
        q = list(self.measured)
        req = GetPositionFK.Request()
        req.header.frame_id = self.base_frame
        req.fk_link_names = [self.ee_link]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = q
        self.busy = True
        self.fk.call_async(req).add_done_callback(lambda f: self.on_fk(f, q))

    def on_fk(self, fut, q):
        self.busy = False
        res = fut.result()
        if res is None or res.error_code.val != 1 or not res.pose_stamped:
            self.get_logger().warn("FK failed", throttle_duration_sec=1.0)
            return
        p = res.pose_stamped[0].pose
        self.pose = ((p.position.x, p.position.y, p.position.z),
                     (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
        self.q_target = q

    def request_ik(self, new_pos, new_quat, dt):
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = self.group
        r.ik_link_name = self.ee_link
        r.avoid_collisions = True
        r.robot_state.joint_state.name = JOINTS
        r.robot_state.joint_state.position = list(self.q_target)
        ps = PoseStamped()
        ps.header.frame_id = self.base_frame
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = new_pos
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = new_quat
        r.pose_stamped = ps
        r.timeout = Duration(sec=0, nanosec=20_000_000)
        self.busy = True
        self.ik.call_async(req).add_done_callback(lambda f: self.on_ik(f, new_pos, new_quat, dt))

    def on_ik(self, fut, new_pos, new_quat, dt):
        self.busy = False
        if self.stopped or self.pose is None:
            return
        res = fut.result()
        if res is None or res.error_code.val != 1:
            self.get_logger().warn("Can't go further that way (reach, joint limit or collision)",
                                   throttle_duration_sec=1.0)
            return
        d = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        q = [d[j] for j in JOINTS]
        step = max(abs(a - b) for a, b in zip(q, self.q_target))
        if step > self.flip_threshold:
            self.get_logger().warn(f"IK wants to flip the arm ({step:.2f} rad jump) - not moving",
                                   throttle_duration_sec=1.0)
            return
        max_step = self.max_joint_speed * dt
        if step > max_step:
            # Too fast for one cycle: keep the direction but go only part of the way
            s = max_step / step
            q = [b + s * (a - b) for a, b in zip(q, self.q_target)]
            pos, quat = self.pose
            new_pos = tuple(p0 + s * (p1 - p0) for p0, p1 in zip(pos, new_pos))
            new_quat = quat_nlerp(quat, new_quat, s)
        self.q_target = q
        self.pose = (new_pos, new_quat)
        self.pub.publish(Float64MultiArray(data=q))


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
