import socket

UDP_IP = "0.0.0.0"  # Bind to all interfaces
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    sock.bind((UDP_IP, UDP_PORT))
    print(f"Listening on {UDP_IP}:{UDP_PORT}")
except Exception as e:
    print(f"Failed to bind: {e}")
