# receiver_script.py

import socket

UDP_IP = "10.52.174.57"  # IP of Mac
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

while True:
    data, _ = sock.recvfrom(4096)  # Buffer size
    print("Received:", data.decode())
