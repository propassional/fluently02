# This module uses C:\Python38\python.exe, a fix-sourced environment
# It must be run from the terminal of the EBuddy project with >C:\Python38\python.exe D:\Banfi\Github\Fluently\Code\Python\ROS2\FLROS2TalkerListenerClient.py
# It connects to the EBuddy socket server, it connects to ROS2
# It listens EBuddy server, and sends to ROS2
# It listens ROS2, and sends to EBuddy server
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket

class TalkerListenerClient(Node):
    ROS2_TIMER = 1.0
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def __init__(self):
        self.i = 0
        self.message_ros2 = String()
        self.message_ros2.data = "No msg"

        # Init ROS2
        rclpy.init()
        super().__init__('ebuddy_talker_listener') # Define node name

        # Listen to Rocco intents, the listener frequency depends on the publisher, it can not be set
        self.subscription = self.create_subscription(String,'gui_output', self.ros2_listen, 10)

        # Publish EBuddy states to Rocco
        self.publisher = self.create_publisher(String, 'sm_state', 10)
        self.tmr = self.create_timer(self.ROS2_TIMER, self.ros2_talk)

        #self.subscription  # prevent unused variable warning
        # self.timer = self.create_timer(1.0, self.ros2_talker_callback)
        # self.message_ros2 = "Default message_ros2"
        # print("TalkerListenerClient init done")

        # Init sockets
        self.socket_connect_to_server_ebuddy()

    # Receive socket message_ros2 from EBuddy server, such as EBuddy state, send it to ROS2
    def socket_receive_ros2_send(self):
        while True:
            data = self.client_socket.recv(1024)
            message = str(data.decode())
            if message:
                print('Client received from server this EBuddy state: ', message)
                self.ros2_set_message(message)
            time.sleep(node.ROS2_TIMER)

    def socket_connect_to_server_ebuddy(self):
        try:
            self.client_socket.connect(('localhost', 65432))
            print("Client: connection successful")
        except socket.error as e:
            print(f"Client: connection failed: {e}")
            exit(-1)

    # Send state to Rocco
    def ros2_set_message(self, message_input):
        self.message_ros2.data = message_input

    def ros2_listen(self, message):
        self.get_logger().info(f'ROS2 listener: "{message.data}"')
        print("Client send method called with message_ros2: " + message.data)
        self.client_socket.sendall(message.data.encode())

    def ros2_talk(self):
        #self.message_ros2.data = 'Hello World: {0}'.format(self.i)
        self.i += 1
        self.get_logger().info('Publishing: "{0}"'.format(self.message_ros2.data))
        self.publisher.publish(self.message_ros2)

    def ros2_spin(self):
        rclpy.spin(self) # Neverending ROS2 loop

if __name__ == '__main__':
    node = TalkerListenerClient() # Neverending loop

    # Create a thread for the socket receive mechanism
    thread = threading.Thread(target=node.socket_receive_ros2_send)
    thread.start()

    node.ros2_spin()

    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()