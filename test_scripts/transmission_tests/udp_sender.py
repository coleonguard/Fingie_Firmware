import socket
import time

# Set up the IP and port for your Mac
UDP_IP = "10.52.174.57"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

counter = 0
while True:
    # Example data to send
    message = f"Ping {counter}"
    sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
    print(f"Sent: {message}")
    counter += 1
    time.sleep(1)  # Send data every second
