import socket

def client_program():
    host = '192.168.178.10'  # Replace with Computer 2's IP address
    port = 12345  # Same port as the server

    client_socket = socket.socket()  # Instantiate
    client_socket.connect((host, port))  # Connect to the server

    while True:
        data_to_send = input('Enter data to send to Computer 2: ')  # Example: Get input data

        client_socket.send(data_to_send.encode())  # Send data

        # Receive the output from Computer 2
        data = client_socket.recv(1024).decode()
        print(f"Received from Computer 2: {data}")  # Output received data

        # If you want to stop the loop
        if data_to_send.lower() == 'exit':
            break

    client_socket.close()  # Close the connection


if __name__ == '__main__':
    client_program()
