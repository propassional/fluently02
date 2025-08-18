# Execute in pycharm >C:\Python38\python.exe D:\Banfi\Github\Fluently\Code\Python\ROS2\FLROS2ListenerSocket.py

import sys
import rclpy # ROS2
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import socket
import json
import threading

topic_read = 'gui_output'  # gui_output, chatter
topic_write = '/sm_state'

class Listener(Node):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def __init__(self):
        # ROS2 node
        super().__init__('EBuddy')  # Node name must be a single word

        # ROS2 read
        self.sub = self.create_subscription(String, topic_read, self.topic_read_callback, 10)
        print("Created ROS2 listener on topic " + topic_read)

        #ROS2 write
        self.publisher_ = self.create_publisher(String, topic_write, 10)
        print("Created ROS2 publisher on topic " + topic_write)

        # Socket
        self.connect_to_server_ebuddy()

        # Thread for socket polling
        self.thread_socket = threading.Thread(target=self.socket_poll)
        self.thread_socket.daemon = True
        self.thread_socket.start()

    def topic_read_callback(self, msg):
        intent = msg.data
        message = str(intent)
        self.get_logger().info('Intent read from NLU: %s' % intent)
        # if "GoToTab" in message: #nnn
        #     message = message + "\t" + message
        self.get_logger().info('Command sent to EBuddy: %s' % message)
        self.client_socket.sendall(message.encode())

    def topic_read_callback_advanced(self, msg):
        try:
            if not msg.data:
                raise ValueError("Client: ROS2 received empty message")

            self.get_logger().info('Client: ROS2 received message: %s' % msg.data)

            data_list = json.loads(msg.data)  # Convert string to list
            print("data_list = ", data_list)
            message = data_list['transcription']
            self.get_logger().info('Client: ROS2 message extracted: %s' % message)

            # Send message to server
            self.client_socket.sendall(message.encode())  # Converts string into bytes
            self.get_logger().info('Client: ROS2 message sent to server: %s' % message)
        except json.JSONDecodeError as e:
            self.get_logger().error('Client: JSON decode error: %s' % str(e))
        except ValueError as e:
            self.get_logger().error('Client: Value error: %s' % str(e))
        except socket.error as e:
            self.get_logger().error('Client: Socket error: %s' % str(e))

    def connect_to_server_ebuddy(self):
        try:
            self.client_socket.connect(('localhost', 65432))
            print("Client: connection with EBuddy server successful")
        except socket.error as e:
            print(f"Client: connection failed: {e}")

    def exchange_messages(self, ros2_message):
        self.client_socket.sendall(ros2_message.encode())

    def ros2_poll(self):
        print(f"Starting ROS2 Listener")
        rclpy.init(args=sys.argv)  # ROS2 init
        try:
            rclpy.spin(self)  # Infinite method, but repeated each time a new message is read
        except KeyboardInterrupt:
            pass
        except ExternalShutdownException:
            sys.exit(1)
        finally:
            self.destroy_node()
            rclpy.try_shutdown()

    # Poll on receive msg
    def socket_poll(self):
        print("Starting socket Listener")
        while True:
            msg = self.client_socket.recv(1024)
            if not msg:
                break
            decoded_msg = msg.decode()
            print(f'Socket received: {decoded_msg}')

            ros_msg = String()
            ros_msg.data = decoded_msg
            self.publisher_.publish(ros_msg)

            self.get_logger().info(f'ROS2 publishing: "{ros_msg}"')

if __name__ == '__main__':
    rclpy.init(args=sys.argv)  # Initialize ROS2
    my_listener = Listener()
    try:
        rclpy.spin(my_listener)  # Keep the main thread alive
    except KeyboardInterrupt:
        pass
    finally:
        my_listener.destroy_node()
        rclpy.shutdown()