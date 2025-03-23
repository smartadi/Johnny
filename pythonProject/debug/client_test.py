# import socket
#
#
# def connect_to_server():
#     # Create a socket object (IPv4, TCP)
#     client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#
#     # Server IP address (replace with the actual server's IP)
#     server_ip = '192.168.0.38'  # Replace with server's IP address
#     port = 12345
#
#     try:
#         # Connect to the server
#         client_socket.connect((server_ip, port))
#         print("Connected to the server.")
#
#         # Send a message to the server
#         message = "Hello, Server!"
#         client_socket.send(message.encode('utf-8'))
#
#         # Receive the server's response
#         response = client_socket.recv(1024).decode('utf-8')
#         print(f"Received from server: {response}")
#
#     except Exception as e:
#         print(f"Error: {e}")
#     finally:
#         # Close the connection
#         client_socket.close()
#
#
# if __name__ == "__main__":
#     connect_to_server()
import socket
import numpy as np
import pickle

def connect_to_server():
    # Create a socket object (IPv4, TCP)
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Server IP address (replace with the actual server's IP)
    server_ip = '192.168.0.38'  # Replace with the server's IP address
    port = 12345

    try:
        # Connect to the server
        client_socket.connect((server_ip, port))
        print("Connected to the server.")

        # Create a NumPy array to send
        array_to_send = np.random.rand(5, 5)
        print(f"Sending array:\n{array_to_send}")

        # Serialize the NumPy array
        data = pickle.dumps(array_to_send)

        # Send the serialized data
        client_socket.sendall(data)

        # Receive the server's response
        response = b""
        while True:
            packet = client_socket.recv(4096)
            if not packet:
                break
            response += packet

        # Deserialize the response
        result = pickle.loads(response)
        print(f"Received result from server: {result}")

    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Close the connection
        client_socket.close()

if __name__ == "__main__":
    connect_to_server()
