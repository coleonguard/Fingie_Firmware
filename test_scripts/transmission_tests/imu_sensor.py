# imu_sensor.py

import os
import time
import socket
import mscl
from scipy.spatial.transform import Rotation as R

# Constants
IMU_RATE = 200  # Adjust to desired sampling rate
IMU_ID = "133931"  # The IMU ID from the device
UDP_IP = "127.0.0.1"
UDP_PORT = 6003

# Socket setup for UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

class IMUData:
    """Simple class to hold IMU data."""
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

class IMU:
    """Class to manage a single IMU and extract roll, pitch, yaw data."""
    
    def __init__(self, imu_id, rate=IMU_RATE):
        self.imu_id = imu_id
        self.rate = rate
        self._enable_port()
        self.imu_node = self._set_up_imu()

    def _enable_port(self):
        """Enables access to the IMU serial port. Adjust if using a different device node."""
        os.system('sudo chmod 777 /dev/ttyACM0') 

    def _set_up_imu(self):
        """Sets up the IMU connection and data channels."""
        node = mscl.InertialNode(mscl.Connection.Serial('/dev/ttyACM0'))  # Adjust if needed
        ahrs_channels = mscl.MipChannels()
        ahrs_channels.append(mscl.MipChannel(mscl.MipTypes.CH_FIELD_SENSOR_EULER_ANGLES, mscl.SampleRate.Hertz(self.rate)))
        
        node.setToIdle()
        node.setActiveChannelFields(mscl.MipTypes.CLASS_AHRS_IMU, ahrs_channels)
        node.enableDataStream(mscl.MipTypes.CLASS_AHRS_IMU)
        node.resume()
        
        return node

    def get_data(self):
        """Fetches roll, pitch, and yaw data from the IMU."""
        imu_data = IMUData()

        packets = self.imu_node.getDataPackets(0)
        if packets:
            for data_point in packets[-1].data():
                if data_point.field() == mscl.MipTypes.CH_FIELD_SENSOR_EULER_ANGLES:
                    if data_point.qualifier() == 6:
                        imu_data.roll = data_point.as_double()
                    elif data_point.qualifier() == 7:
                        imu_data.pitch = data_point.as_double()
                    elif data_point.qualifier() == 8:
                        imu_data.yaw = data_point.as_double()
        return imu_data

if __name__ == "__main__":
    imu = IMU(IMU_ID, IMU_RATE)
    # Sending at 100 Hz (adjust as needed)
    send_interval = 1.0 / 100.0

    while True:
        data = imu.get_data()
        # Format the message as needed; here we just send roll, pitch, yaw
        msg = f"{data.roll:.2f} {data.pitch:.2f} {data.yaw:.2f}"
        sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))
        time.sleep(send_interval)
