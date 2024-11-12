# imu_sensor.py

import socket
import time
import random

UDP_IP = "127.0.0.1"
UDP_PORT = 6003
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    data = " ".join(f"{random.uniform(-180, 180):.2f}" for _ in range(9))
    sock.sendto(data.encode(), (UDP_IP, UDP_PORT))
    time.sleep(0.01)  # 100 Hz
