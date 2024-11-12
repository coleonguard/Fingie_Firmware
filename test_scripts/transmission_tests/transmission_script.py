# transmission_script.py

import socket
import time

# IP and port for Mac (receiver)
UDP_IP = "10.52.174.57"
UDP_PORT = 5005

# Local ports for each sensor data source
SENSOR_PORTS = {
    "proximity": 6000,
    "pressure": 6001,
    "emg": 6002,
    "imu": 6003
}

# Setup receiving sockets for each sensor
sensors = {}
for sensor, port in SENSOR_PORTS.items():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(0.01)  # Non-blocking socket with a short timeout
    sensors[sensor] = sock

# Main socket to send data to the receiver
send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    data_package = {}

    # Collect data from each sensor socket
    for sensor, sock in sensors.items():
        try:
            data, _ = sock.recvfrom(1024)  # Adjust buffer size as needed
            data_package[sensor] = data.decode()
        except socket.timeout:
            data_package[sensor] = "No Data"

    # Send aggregated data to the receiver
    message = f"Data Package: {data_package}"
    send_sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
    print(f"Sent: {message}")

    time.sleep(0.01)  # Adjust for the desired send frequency (e.g., 100 Hz)
