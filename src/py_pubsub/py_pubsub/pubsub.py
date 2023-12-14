import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PubSubNode(Node):

    def __init__(self):
        super().__init__('pubsub_node')

        self.publisher_ = self.create_publisher(String, 'pico_subscriber', 10)

        self.subscription = self.create_subscription(
            String,
            'pico_publisher',
            self.listener_callback,
            10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    pubsub_node = PubSubNode()

    rclpy.spin(pubsub_node)
    pubsub_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
