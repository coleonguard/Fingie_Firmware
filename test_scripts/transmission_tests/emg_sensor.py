# emg_sensor.py

import socket
import time

UDP_IP = "127.0.0.1"
UDP_PORT = 6002
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    dt_msg = 0  # Timestamp replaced with zero
    dt_read = 0  # Simulate slight delay replaced with zero
    emg_data = " ".join("0" for _ in range(8))  # EMG data filled with zeros
    sync = 0  # SYNC byte replaced with zero
    data = f"{dt_msg} {dt_read} {emg_data} {sync}"
    sock.sendto(data.encode(), (UDP_IP, UDP_PORT))
    time.sleep(0.01)  # 100 Hz
