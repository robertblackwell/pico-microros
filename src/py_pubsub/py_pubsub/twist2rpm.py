import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

# The topic on which the twist messages will arrive
TWIST_TOPIC = "cmd_vel"
# The topic on which we will publish the rpm string message
RPM_TOPIC = "pico_subscriber_topic"

def convert_twistmsg_to_rpm_string(twist_msg: Twist) -> str:
    # twist messages are assumed to have linear speeds in meters per second

    print(f"convert in linear {twist_msg.linear.x} {twist_msg.linear.y} {twist_msg.linear.z} angular {twist_msg.angular.x} {twist_msg.angular.y} {twist_msg.angular.z}")
    speed_mm_s = twist_msg.linear.x * 1000.0
    wheel_rps = speed_mm_s / (60.0 * math.pi)
    wheel_rpm = wheel_rps * 60.0
    motor_rpm = (wheel_rpm * 229.0)
    print(f"speed_mm_s {speed_mm_s} wheel_rps: {wheel_rps} wheel_rpm: {wheel_rpm} motor_rpm: {motor_rpm} ")
    return f"rpm {motor_rpm} {motor_rpm}"


class TwistToRpm(Node):

    def __init__(self):
        super().__init__('twist2rpm_pubsub_node')

        self.publisher_ = self.create_publisher(String, RPM_TOPIC, 10)

        self.subscription = self.create_subscription(
            Twist,
            TWIST_TOPIC,
            self.listener_callback,
            10)

        timer_period = 10  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        s = convert_twistmsg_to_rpm_string(msg)
        self.get_logger().info(f"I heard: {s}")
        self.publish_string(s)

    def publish_string(self, s: str):
        msg = String()
        msg.data = s #'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def timer_callback(self):
        return
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    pubsub_node = TwistToRpm()

    rclpy.spin(pubsub_node)
    pubsub_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
