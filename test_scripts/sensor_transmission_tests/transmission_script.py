# transmission_script.py

import socket
import time
import sys

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

# Check for arguments to determine which sensors to listen to
if len(sys.argv) > 1:
    enabled_sensors = sys.argv[1:]
else:
    enabled_sensors = list(SENSOR_PORTS.keys())

# Debug: Print enabled sensors
print(f"Enabled sensors: {enabled_sensors}")

# Setup receiving sockets only for enabled sensors
sensors = {}
for sensor in enabled_sensors:
    sensor = sensor.strip()  # Remove any leading/trailing whitespace
    if sensor in SENSOR_PORTS:
        port = SENSOR_PORTS[sensor]
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", port))
        sock.settimeout(0.01)  # Non-blocking socket with a short timeout
        sensors[sensor] = sock
        print(f"Listening for {sensor} data on port {port}")
    else:
        print(f"Unknown sensor '{sensor}' specified.")

# Main socket to send data to the receiver
send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    data_package = {}

    # Collect data from each enabled sensor socket
    for sensor, sock in sensors.items():
        try:
            data, _ = sock.recvfrom(1024)  # Adjust buffer size as needed
            data_package[sensor] = data.decode()
        except socket.timeout:
            # Instead of assigning fake data, we can skip or assign None
            data_package[sensor] = None

    # Only send data if at least one sensor has valid data
    if any(value is not None for value in data_package.values()):
        message = f"Data Package: {data_package}"
        send_sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
        print(f"Sent: {message}")
    else:
        print("No data received from sensors.")

    time.sleep(0.01)  # Adjust for the desired send frequency (e.g., 100 Hz)