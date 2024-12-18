import socket
import rospy
import sys
from std_msgs.msg import String  # Adjust if needed for specific data types
import json  # Import JSON module

# Default IP for UDP receiving (binds to all interfaces if no IP is provided)
DEFAULT_UDP_IP = "0.0.0.0"
UDP_PORT = 5005

# Parse command-line arguments
if len(sys.argv) > 1:
    UDP_IP = sys.argv[1]
    print(f"Using provided IP address: {UDP_IP}")
else:
    UDP_IP = DEFAULT_UDP_IP
    print(f"No IP address provided. Binding to all interfaces (0.0.0.0)")

# Initialize UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    sock.bind((UDP_IP, UDP_PORT))
    print(f"Listening for UDP data on {UDP_IP}:{UDP_PORT}")
except Exception as e:
    print(f"Failed to bind socket: {e}")
    sys.exit(1)  # Exit if binding fails

# Initialize ROS node after confirming socket is bound
rospy.init_node('fingie_sensors', anonymous=True)

# Define ROS publishers with the `fingie_sensors` namespace
publishers = {
    'proximity': rospy.Publisher('/fingie_sensors/proximity', String, queue_size=10),
    'pressure': rospy.Publisher('/fingie_sensors/pressure', String, queue_size=10),
    'emg': rospy.Publisher('/fingie_sensors/emg', String, queue_size=10),
    'imu': rospy.Publisher('/fingie_sensors/imu', String, queue_size=10)
}

def process_and_publish(data_package):
    """Publish each piece of sensor data to the respective ROS topic."""
    for sensor, data in data_package.items():
        if sensor in publishers:
            try:
                publishers[sensor].publish(data)  # Publish data as a string
                rospy.loginfo(f"Published {sensor} data: {data}")
            except rospy.ROSInterruptException:
                pass  # Handle ROS exceptions

while not rospy.is_shutdown():
    try:
        # Receive UDP data
        data, _ = sock.recvfrom(4096)
        decoded_data = data.decode()
        print(f"Received: {decoded_data}")

        # Deserialize the JSON data
        data_package = json.loads(decoded_data)
        process_and_publish(data_package)

    except KeyboardInterrupt:
        print("Shutting down receiver.")
        break
    except Exception as e:
        print(f"Error receiving or processing data: {e}")
        
# Close socket on exit
sock.close()
