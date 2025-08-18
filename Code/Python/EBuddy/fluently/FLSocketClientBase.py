import socket

def start_client():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect(('localhost', 65432))

    try:
        while True:
            data = client_socket.recv(1024)
            if not data:
                break
            number = int(data.decode())
            print('Client: received from server:', number)
            doubled_number = number * 2
            client_socket.sendall(str(doubled_number).encode())
    finally:
        client_socket.close()

if __name__ == '__main__':
    start_client()