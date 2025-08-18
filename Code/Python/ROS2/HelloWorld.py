import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StringSubscriber(Node):

    def __init__(self):
        super().__init__('string_subscriber')
        self.subscription = self.create_subscription(
            String,
            'gui_output',  # Replace with your topic name
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    string_subscriber = StringSubscriber()
    rclpy.spin(string_subscriber)
    string_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()