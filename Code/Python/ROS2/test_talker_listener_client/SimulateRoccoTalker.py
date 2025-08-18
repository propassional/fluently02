import sys
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    intents = [ "Next", "Last"] # , "GotoStart"]

    # intents = [
    #     "Next",
    #     "Instructions",
    #     "Setup",
    #     "PreviewBasic",
    #     "FullScan",
    #     "SliceScan",
    #     "SnakeScan",
    #     "Registration",
    #     "CommandHub",
    #     "AutopilotOn",
    #     "AutopilotOff"
    # ]
    iterator = iter(intents)

    def __init__(self):
        super().__init__('Rocco_talker')
        self.i = 0
        #self.publisher = self.create_publisher(String, 'gui_output', 10) # chatter




        self.publisher = self.create_publisher(String, 'test_topic', 10)  # chatter




        timer_period = 1.0
        self.tmr = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        message = String()
        try:
            item = next(self.iterator)
        except StopIteration:
            self.iterator = iter(self.intents)
            item = next(self.iterator)
        #msg.data = 'Rocco intent: {0}'.format(self.i)
        message.data = item
        self.i += 1
        self.get_logger().info('Publishing: "{0}"'.format(message.data))
        self.publisher.publish(message)

def main(args=None):
    rclpy.init(args=args)

    node = Talker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
