import socket
import time
import sys
import struct
import fcntl

# Function to get local IP address
def get_local_ip(interface="wlan0"):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Connect to an external host; it doesn't actually send any data
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
    finally:
        s.close()
    return local_ip

# Function to get subnet mask
def get_subnet_mask(ifname="wlan0"):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
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

# Get local IP and subnet mask, using wlan0 by default
interface = "wlan0"
local_ip = get_local_ip(interface)
subnet_mask = get_subnet_mask(interface)
UDP_IP = calculate_broadcast_address(local_ip, subnet_mask)
UDP_PORT = 5005

# Debug: Print calculated broadcast information
print(f"Local IP: {local_ip}")
print(f"Subnet Mask: {subnet_mask}")
print(f"Broadcast IP: {UDP_IP}")

# Setup main socket for broadcasting
send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
send_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

# Placeholder sensor data (simulating what might be gathered)
data_package = {
    "proximity": "100",
    "pressure": "1.2",
    "emg": "1 2 3 4 5 6 7 8",
    "imu": "1.0 2.0 3.0 4.0 5.0 6.0 7.0 8.0 9.0"
}

try:
    while True:
        # Send the data package over the broadcast address
        message = f"Data Package: {data_package}"
        send_sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
        print(f"Sent: {message} to {UDP_IP}:{UDP_PORT}")
        
        time.sleep(2)  # Adjust for testing every 2 seconds

except KeyboardInterrupt:
    print("Transmission interrupted by user.")
finally:
    send_sock.close()
    print("Socket closed.")
