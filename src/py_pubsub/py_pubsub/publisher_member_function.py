import rclpy
import time
import threading
from rclpy.node import Node, GuardCondition

from std_msgs.msg import String, Int32


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.count = 0
        self.publisher_ = self.create_publisher(String, 'pico_subscriber_topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.fake_timer = self.create_timer(0, self.timer_trigger_callback)
        self.fake_timer.cancel()
        self.guard = self.create_guard_condition(self.gc_callback)
        self.i = 0
        self.main_thread_ident = threading.current_thread().ident
        self.t = threading.Thread(target=self.thread_function)
        self.t.start()

    def thread_function(self):
        while True:
            time.sleep(5)
            # self.fake_timer.reset()
            self.guard.trigger()

    def gc_callback(self):
        id = threading.current_thread().ident
        id2 = self.t.ident
        self.get_logger().info(f"gc__callback current thread:{id}  background:{id2}  main:{self.main_thread_ident}")
        if id != self.main_thread_ident:
            self.get_logger().info("gc_callback called on the background thread - this is wrong")            

    def timer_trigger_callback(self):
        self.get_logger().info('timer_trigger_callback')
        self.fake_timer.cancel()

    def timer_callback(self):
        msg = String()
        msg.data =  f"Here we are from python publisher {self.i}"
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()