# FLROS2Talker.py, server

import sys
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import socket

class Talker(Node):
    conn = None
    addr = None
    server_socket = None

    def __init__(self):
        super().__init__('talker')
        self.i = 0
        self.pub = self.create_publisher(String, 'chatter', 10)
        timer_period = 1.0
        self.tmr = self.create_timer(timer_period, self.timer_callback)

        # Setup and connect to the server
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('localhost', 65432))
        self.server_socket.listen()
        print('Server: listening on port 65432...')
        self.conn, self.addr = self.server_socket.accept()

    def timer_callback(self):
        msg = String()
        msg.data = 'Server Hello World: {0}'.format(self.i)
        self.i += 1
        self.get_logger().info('Server Publishing: "{0}"'.format(msg.data))
        self.pub.publish(msg)

        # Server connections
        # self.send_message(message_ros2.data)
        # self.recv_message()

    def send_message(self, message):
        message = "HelloFromServer"
        self.server_socket.sendall(message.encode())

    def recv_message(self):
        data = self.server_socket.recv(1024)
        print('Server: received from server:', data)
        # if data:
        #     number = int(data.decode())
        #     print('Client: received from server:', number)
        #     doubled_number = number * 2
        #     self.server_socket.sendall(str(doubled_number).encode())

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


#
# import sys
# import rclpy
# from rclpy.executors import ExternalShutdownException
# from rclpy.node import Node
# from std_msgs.message_ros2 import String
#
# class Talker(Node):
#
#     def __init__(self):
#         super().__init__('talker')
#         self.i = 0
#         self.publisher = self.create_publisher(String, 'chatter', 10)
#         timer_period = 1.0
#         self.tmr = self.create_timer(timer_period, self.timer_callback)
#
#     def timer_callback(self):
#         message_ros2 = String()
#         message_ros2.data = 'Hello World: {0}'.format(self.i)
#         self.i += 1
#         self.get_logger().info('Publishing: "{0}"'.format(message_ros2.data))
#         self.publisher.publish(message_ros2)
#
# def main(args=None):
#     rclpy.init(args=args)
#
#     node = Talker()
#
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     except ExternalShutdownException:
#         sys.exit(1)
#     finally:
#         node.destroy_node()
#         rclpy.try_shutdown()
#
#
# if __name__ == '__main__':
#     main()
