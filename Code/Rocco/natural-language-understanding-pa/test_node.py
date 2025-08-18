'''
ros2 topic pub --once /nlu_input std_msgs/msg/String "{data: 'start recording'}"
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NLUInputPublisher(Node):

    def __init__(self):
        super().__init__('nlu_input_publisher')
        self.publisher_ = self.create_publisher(String, '/nlu_input', 10)

    def publish_message(self, message):
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    nlu_input_publisher = NLUInputPublisher()

    try:
        while rclpy.ok():
            message = input("Enter a message to publish: ")
            nlu_input_publisher.publish_message(message)
    except KeyboardInterrupt:
        pass

    nlu_input_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()