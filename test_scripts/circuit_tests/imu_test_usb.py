import os
import time
import mscl
import serial.tools.list_ports
from scipy.spatial.transform import Rotation as R

# Constants
IMU_RATE = 200  # Sampling rate in Hz
IMU_IDS = ["133931", "133932"]  # List of IMU IDs

class IMUData:
    """Simple class to hold IMU data without dataclasses."""
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

class IMU:
    """Class to manage a single IMU and extract roll, pitch, yaw data."""
    
    def __init__(self, port, rate=IMU_RATE):
        self.port = port
        self.rate = rate
        self._enable_port()
        self.imu_node = self._set_up_imu()

    def _enable_port(self):
        """Enables access to the IMU serial port."""
        os.system(f'sudo chmod 777 {self.port}')

    def _set_up_imu(self):
        """Sets up the IMU connection and data channels."""
        node = mscl.InertialNode(mscl.Connection.Serial(self.port))
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

    def get_imu_id(self):
        """Fetches the IMU identifier."""
        # Example logic: Read IMU info packet to get ID (simplified for illustration)
        info = self.imu_node.getDeviceInfo()
        return info.deviceSerialNumber()

def auto_detect_ports():
    """Automatically detects available USB ports."""
    ports = [port.device for port in serial.tools.list_ports.comports()]
    print(f"Detected ports: {ports}")
    return ports

def detect_imus():
    """Detect and assign IMUs to their respective ports."""
    imu_mapping = {}
    available_ports = auto_detect_ports()
    for port in available_ports:
        try:
            imu = IMU(port)
            imu_id = imu.get_imu_id()
            if imu_id in IMU_IDS:
                imu_mapping[imu_id] = port
        except Exception as e:
            print(f"Failed to communicate with IMU on port {port}: {e}")
    return imu_mapping

def main():
    # Detect IMUs and assign them to their respective ports
    imu_mapping = detect_imus()
    if len(imu_mapping) != len(IMU_IDS):
        print("Could not detect all IMUs. Check connections.")
        return

    # Initialize IMUs
    imus = {imu_id: IMU(port) for imu_id, port in imu_mapping.items()}

    while True:
        for imu_id, imu in imus.items():
            data = imu.get_data()
            print(f'IMU {imu_id} - Roll: {data.roll:.2f}, Pitch: {data.pitch:.2f}, Yaw: {data.yaw:.2f}')
        time.sleep(1 / IMU_RATE)  # Adjust sleep for desired sampling rate

if __name__ == "__main__":
    main()
