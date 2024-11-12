import socket

# Set up IP and port for listening
UDP_IP = "0.0.0.0"  # Listen on all available interfaces
UDP_PORT = 5005     # Must match the port used by the transmitting script

# Initialize the UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
print(f"Listening for UDP data on {UDP_IP}:{UDP_PORT}")

try:
    while True:
        # Receive data from the transmitter
        data, addr = sock.recvfrom(4096)  # Buffer size is 4096 bytes
        print(f"Received data from {addr}: {data.decode()}")
except KeyboardInterrupt:
    print("Socket receiver test interrupted by user.")
finally:
    sock.close()
    print("Socket closed.")
