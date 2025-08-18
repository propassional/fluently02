import os
import socket
import time

# Constants
TIME_WAIT_FOR_CHECK_AKNOWLEDGE = 1
BUFFER_SIZE = 1024
PORT = 22
addressIP = "192.168.1.131"  # Replace with the desired IP address
cmdForSikulixPath = "/path/to/cmdForSikulix/"  # Replace with the actual path
cmdFromSikulixPath = "/path/to/cmdFromSikulix/"  # Replace with the actual path
feedbackFileName = "feedback.txt"  # Replace with the actual feedback file name
commandFileName = "command.txt"  # Replace with the actual command file name

# Functions
def aknowledge_sikulix_check(client_message, sikulix_path):
    cmd_file_found = False
    while cmd_file_found:
        # Check Sikulix cmd acknowledge
        time.sleep(TIME_WAIT_FOR_CHECK_AKNOWLEDGE)
        cmd_file_found = any(os.listdir(sikulix_path))
        if cmd_file_found:
            print(f"Server: waiting for Sikulix to execute last command: {client_message}")

def command_file_create(file_name, command):
    try:
        with open(file_name, 'w') as command_file:
            command_file.write(command)
    except IOError:
        print(f"Server error: failed to create the file: {file_name}")
        print("Server: error, press Enter to exit Server program")
        input()
        do_exit()

def do_exit():
    print("Server: exiting server app")
    # Add additional cleanup if needed
    exit(1)

def string_to_lowercase(buffer):
    return buffer.lower()

def string_remove_comma(buffer):
    return buffer.replace(',', '')

def string_remove_bad_characters(string_in):
    unacceptable_chars = ",\\/:*?\"<>|¦#$^!@&*()_+-%=."
    return ''.join(char for char in string_in if char not in unacceptable_chars)

def string_remove_spaces(input_str):
    return input_str.strip()

# ... (Other functions remain unchanged)

def workspace_setup():
    global setup_workspace_done
    setup_workspace_done = False
    cmd_set_glob = {"preview", "record", "pause", "stop", "tools", "global registration", "fast fusion", "save", "close", "shutdown"}

    if not setup_workspace_done:
        res = logger.log_file_create()
        if res < 0:
            print("Server: logfile error, press Enter to exit Server program")
            input()
            exit(-1)
    else:
        logger.log_file_write(f"{__LINE__} Server: workspace has already been set up previously")

    return 0

def main():
    workspace_setup()

    # Check which windows socket implementation is needed
    try:
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind((addressIP, PORT))
        server_socket.listen(5)
        print(f"Server: listening on port {PORT}")

        client_socket, _ = server_socket.accept()
        print("Server: accepted client connection")

        while True:
            buffer = client_socket.recv(BUFFER_SIZE).decode()
            if not buffer:
                print("Server: error receiving SM_JSON_data from client")
                break

            whisper_message_unfiltered = buffer
            print(f"Server: received whisper message: {whisper_message_unfiltered}")

            buffer = string_to_lowercase(buffer)
            buffer = string_remove_bad_characters(buffer)
            client_message1 = buffer
            print(f"Server: filtered whisper message: {client_message1}")

            cmd_file_name = cmd_for_sikulix_path + command_file_name
            print(f"Server: saved cmd file to: {cmd_file_name}")
            command_file_create(cmd_file_name, client_message1)

            # ... (Remaining code remains unchanged)

    except Exception as e:
        print(f"An error occurred: {str(e)}")
    finally: # This part is always executed
        if 'client_socket' in locals():
            client_socket.close()
        if 'server_socket' in locals():
            server_socket.close()

if __name__ == "__main__":
    main()
