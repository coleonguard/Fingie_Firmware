# emg_sensor.py

import socket
import time
import random

UDP_IP = "127.0.0.1"
UDP_PORT = 6002
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    dt_msg = int(time.time() * 1000)  # milliseconds
    dt_read = dt_msg + random.randint(0, 5)  # Simulate slight delay
    emg_data = " ".join(str(random.randint(0, 1023)) for _ in range(8))
    sync = random.randint(0, 1)  # SYNC byte
    data = f"{dt_msg} {dt_read} {emg_data} {sync}"
    sock.sendto(data.encode(), (UDP_IP, UDP_PORT))
    time.sleep(0.01)  # 100 Hz
