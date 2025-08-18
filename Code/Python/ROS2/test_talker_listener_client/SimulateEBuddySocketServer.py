# This module simulates EBuddy, starting a socket server that sends state names

import socket
import time

class Server():
    def __init__(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('localhost', 65432))
        self.server_socket.listen()
        print('Server: listening on port 65432...')
        self.conn, addr = self.server_socket.accept()
        print('Server: connected by', addr)

    def send(self, message):
        self.conn.sendall(message.encode())

    def recv(self):
        data = self.conn.recv(1024)
        print('Server received intent:', data.decode())

    def close(self):
        self.conn.close()
        self.server_socket.close()

if __name__ == '__main__':
    # state to be published: current tab cat current state
    states = [
        "InstructionsStart",
        "InstructionsReady",
        "InstructionsRototiltInstalled",
        "InstructionsMarkersApplied",
        "InstructionsRingBelowScanned",
        "InstructionsRingIntermediateScanned",
        "InstructionsRingAboveScanned",
        "InstructionsEnd"
    ]
    my_server = Server()
    count = 0
    while count < 7:
        my_server.send(states[count])
        print("Server socket sent: " + states[count])
        time.sleep(1)
        my_server.recv()
        count += 1
        if count == 6: count = 0

    #my_server.close()