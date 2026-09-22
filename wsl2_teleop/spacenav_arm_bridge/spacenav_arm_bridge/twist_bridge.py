import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class SpacenavTwistBridge(Node):
    def __init__(self):
        super().__init__('spacenav_twist_bridge')
        self.declare_parameter('frame_id', 'pro_arm_base_link')
        self.declare_parameter('output_topic', '/servo_node/delta_twist_cmds')
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale', 1.0)

        self.frame_id = self.get_parameter('frame_id').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value

        output_topic = self.get_parameter('output_topic').value
        self.pub = self.create_publisher(TwistStamped, output_topic, 10)
        self.sub = self.create_subscription(Twist, '/spacenav/twist', self.on_twist, 10)

    def on_twist(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.twist.linear.x = msg.linear.x * self.linear_scale
        out.twist.linear.y = msg.linear.y * self.linear_scale
        out.twist.linear.z = msg.linear.z * self.linear_scale
        out.twist.angular.x = msg.angular.x * self.angular_scale
        out.twist.angular.y = msg.angular.y * self.angular_scale
        out.twist.angular.z = msg.angular.z * self.angular_scale
        self.pub.publish(out)


def main():
    rclpy.init()
    node = SpacenavTwistBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
