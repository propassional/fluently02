# This code uses ROS2, thus it has to be executed with a python sourced environment such as C:\Python38\python.exe, which does not need to be sourced, it is already sourced

import socket
import time

from fluently.FLROS2TalkerListener import TalkerListener


class Client():
    client_socket = None

    def __init__(self):
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect(('localhost', 65432))
            #self.client_socket.setblocking(False)  # Set the socket to non-blocking mode
            print("Connection successful!")
        except socket.error as e:
            print(f"Connection failed: {e}")

        self.node = TalkerListener()

    # Receive socket msg from EBuddy server, such as EBuddy state, send it to ROS2
    def recv(self):
        data = self.client_socket.recv(1024)
        message = str(data.decode())
        if message:
            print('Client received from server this EBuddy state: ', message)
            self.node.set_message(message)

    def send(self, message):
        #print('Client: sending intent to server:', message)
        print("Client send method called with message: " + message)
        self.client_socket.sendall(message.encode())

    def close(self):
        self.client_socket.close()

if __name__ == '__main__':
    intents = ['Next', 'CheckArtecStudioIsOff', 'StartArtecStudio', 'StartScanMenu', 'VerifyCameraSetupAdvanced1', 'VerifyCameraSetupAdvanced2', 'StartPreview', 'CheckScannerIsOn', 'VerifyCameraSetupBasic', 'Auto']
    my_client = Client()
    count = 0
    while True:
        my_client.recv()
        time.sleep(0.1)

    # while count < 7:
    #     count += 1
    #     my_client.recv()
    #     time.sleep(1)
    #     my_client.send(intents[count])

    #my_client.close()