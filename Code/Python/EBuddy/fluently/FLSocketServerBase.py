import socket

def start_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('localhost', 65432))
    server_socket.listen()

    print('Server: listening on port 65432...')
    conn, addr = server_socket.accept()
    with conn:
        print('Server: connected by', addr)
        counter = 0
        while True:
            conn.sendall(str(counter).encode())  # Send the current counter value
            data = conn.recv(1024)
            if not data:
                break
            print('Server: received:', data.decode())
            counter += 1  # Increment the counter

if __name__ == '__main__':
    start_server()