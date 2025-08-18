# EBuddy > Socket server > Sockets > Socket client > ROS2
# Socket server waits until a socket client connects, then it sends via socket the state name at each state change

import os
import socket
import threading
import time
import subprocess

from fluently.FLCommands import FLCommands
from fluently.FLConstants import SOCKET_SERVER_LAUNCHES_CLIENT, SOCKET_SERVER_IS_USED, TIME_WAIT_READ_NEXT_COMMAND, \
    SM_STATE_SEPARATOR, COMMAND_FOR_EBUDDY, COMMAND_AUTOPILOT_ON, COMMAND_AUTOPILOT_OFF, ROS2_INTERPRETER, \
    ROS2_SOCKET_LISTENER


class Server():
    def __init__(self):
        self.message = ""
        self.SM = ""
        self.state = ""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setblocking(False)  # Set the socket to non-blocking mode
        self.server_socket.bind(('localhost', 65432))
        self.server_socket.listen()
        print('Server: listening on port 65432...')

        if SOCKET_SERVER_LAUNCHES_CLIENT:
            #command = r'C:\Python38\python.exe D:\Banfi\Github\Fluently\Code\Python\ROS2\FLROS2ListenerSocket.py'
            command = f"{ROS2_INTERPRETER} {ROS2_SOCKET_LISTENER}"
            process = subprocess.Popen(command, shell=True)
            print(r'Server: socket client launched now, more infos in D:\Banfi\Github\Fluently\Releases\2025_01_10_EBuddy\_readme.txt')
            time.sleep(2)
            # result = subprocess.run(command, shell=True)
            # print("Return Code:", result.returncode)
            # print("Output:", result.stdout)
            # print("Error:", result.stderr)
        else:
            print(r'Server: manual mode is on, so please launch now a client as indicated by D:\Banfi\Github\Fluently\Releases\2025_01_10_EBuddy\_readme.txt')

        # If setup is non-blocking, at this moment the client must be up otherwise you get following error:
        # BlockingIOError: [WinError 10035] A non-blocking socket operation could not be completed immediately

        self.conn, addr = self.server_socket.accept() # Now run in cmd>C:\Python38\python.exe D:\Banfi\Github\Fluently\Code\Python\ROS2\FLROS2ListenerSocket.py
        print('Server: connected by', addr)

        # Start a new thread to handle incoming messages
        self.thread = threading.Thread(target=self.run_forever)
        self.thread.daemon = True
        self.thread.start()

    def singularize_command(self, command):
        # Check for repeated commands and return a single instance
        if len(command) % 2 == 0:
            mid = len(command) // 2
            if command[:mid] == command[mid:]:
                return command[:mid]
        return command

    def message_set(self, message_in):
        self.message = message_in

        if self.message != "":
            # Separe SM name (will be used later) from state
            separated_strings = message_in.split(SM_STATE_SEPARATOR)
            # Check if the split resulted in the expected number of parts
            if len(separated_strings) > 2:
                raise ValueError(f"Input message must contain exactly one '{SM_STATE_SEPARATOR}' separator")
            # Assign the separated parts to the respective attributes
            self.SM = separated_strings[0].strip()  # Use strip() to remove any leading/trailing whitespace
            self.state = separated_strings[1].strip()

    def send(self, state):
        self.conn.sendall(state.encode())

    # Receive from client, thus from ROS2
    def recv(self):
        # data = self.conn.recv(1024)
        # command = data.decode()

        try:
            data = self.conn.recv(1024)
            if not data:
                print("Connection closed by the client.")
                self.conn.close()
            else:
                try:
                    command = data.decode("utf-8")
                    # command = self.singularize_command(command)
                except UnicodeDecodeError:
                    print("Received data is not valid UTF-8.")
                    # Handle accordingly or close the connection
                    self.conn.close()
                else:
                    # Proceed with processing the command
                    print(f"Received command: {command}")
        except BlockingIOError:
            return
        except socket.error as e:
            print(f"Socket error occurred: {e}")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")

        print('Server received intent:', command)

        command_extended = ""
        # Add the autopilot flag if needed
        if command.__contains__(COMMAND_AUTOPILOT_ON):
            command_extended = command + r"\n" + "True"
        elif command.__contains__(COMMAND_AUTOPILOT_OFF):
            command_extended = command + r"\n" + "False"
        else:
            command_extended = command

        command_complete = self.SM + r"\n" + command_extended
        command_file_name = self.SM + "_" + command + ".txt"
        path_sink = os.path.join(COMMAND_FOR_EBUDDY, command_file_name)
        try:
            with open(path_sink, "w") as sink_file:
                sink_file.write(command_complete)
                # This will be done automatically by exiting the with statement: sink_file.close()
                #window_command['-FILE_SAVED-'].update(path_sink)
                file_written = True
        except FileNotFoundError:
            pass
            #sg.popup(f"GUI_command_send: file '{command}' not found in source directory.", title="Error")
        except Exception as e:
            pass
            #sg.popup(f"GUI_command_send: an error occurred while copying '{command}': {str(e)}", title="Error")

    def run_forever(self):
        try:
            while True:
                #nnn
                self.recv() # Blocking, so not consuming resources
                time.sleep(TIME_WAIT_READ_NEXT_COMMAND)
                # Send is called directly by the state machine that changes state
        except KeyboardInterrupt:
            print('Server: shutting down...')
            self.close()

    def close(self):
        self.conn.close()
        self.server_socket.close()

if __name__ == '__main__':
    states = [
        "Start",
        "Ready",
        "PreviewIsOn",
        "CobotStarted",
        "RecordingIsOn",
        "PauseIsOn",
        "RecordingIsOff",
        "SaveFile"
    ]
    my_server = Server()
    count = 0
    while count < 7:
        my_server.send(states[count])
        time.sleep(1)
        my_server.recv()
        count += 1

    #my_server.close()

# import socket
# import time
#
# class Server():
#     server_socket = None
#
#     def __init__(self):
#         server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         server_socket.bind(('localhost', 65432))
#         server_socket.listen()
#
#         print('Server: listening on port 65432...')
#         self.server_socket, addr = server_socket.accept()
#         with self.server_socket:
#             print('Server: connected by', addr)
#             pass
#
#     def send(self, message):
#         while True:
#             self.server_socket.sendall(message.encode())
#
#     def recv(self):
#         data = self.server_socket.recv(1024)
#         # if not data:
#         #     break
#         print('Server: received:', data.decode())
#
#     def close(self):
#         self.server_socket.close()
#
# if __name__ == '__main__':
#     my_server = Server()
#     count = 0
#     while count < 10:
#         print(f"This is iteration number {count + 1}")
#         count += 1
#         my_server.send("Hello")
#         time.sleep(1)
#         my_server.recv()
#
#     my_server.close()