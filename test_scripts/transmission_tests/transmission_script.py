import socket
import time
import sys
import struct
import fcntl

# Function to get local IP address
def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Connect to an external host; it doesn't actually send any data
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
    finally:
        s.close()
    return local_ip

# Function to get subnet mask
def get_subnet_mask(ifname="eth0"):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Retrieve the subnet mask of the specified interface
        subnet_mask = socket.inet_ntoa(
            fcntl.ioctl(
                s.fileno(),
                0x891b,  # SIOCGIFNETMASK
                struct.pack("256s", ifname[:15].encode("utf-8"))
            )[20:24]
        )
    finally:
        s.close()
    return subnet_mask

# Function to calculate broadcast address
def calculate_broadcast_address(ip, mask):
    ip_bytes = struct.unpack("!I", socket.inet_aton(ip))[0]
    mask_bytes = struct.unpack("!I", socket.inet_aton(mask))[0]
    broadcast_bytes = ip_bytes | ~mask_bytes & 0xFFFFFFFF
    return socket.inet_ntoa(struct.pack("!I", broadcast_bytes))

# Get local IP and subnet mask
local_ip = get_local_ip()
subnet_mask = get_subnet_mask("eth0")  # Use your actual network interface, e.g., "wlan0" for Wi-Fi
UDP_IP = calculate_broadcast_address(local_ip, subnet_mask)
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

# Main socket to send data to all devices on the network
send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
send_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  # Enable broadcasting

print(f"Broadcasting to: {UDP_IP}")

while True:
    data_package = {}

    # Collect data from each enabled sensor socket
    for sensor, sock in sensors.items():
        try:
            data, _ = sock.recvfrom(1024)  # Adjust buffer size as needed
            data_package[sensor] = data.decode()
        except socket.timeout:
            data_package[sensor] = "No Data"

    # Send aggregated data to all devices on the network
    message = f"Data Package: {data_package}"
    send_sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
    print(f"Sent: {message}")

    time.sleep(0.01)  # Adjust for the desired send frequency (e.g., 100 Hz)
