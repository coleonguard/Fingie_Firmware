import os
import time
import mscl
from scipy.spatial.transform import Rotation as R

# Constants
IMU_RATE = 200  # Sampling rate in Hz
IMU_IDS = {"back of hand": "133931", "wrist": "156124"}  # IMU IDs for identification

class IMUData:
    """Simple class to hold IMU data without dataclasses."""
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

class IMU:
    """Class to manage a single IMU and extract roll, pitch, yaw data."""
    
    def __init__(self, port, imu_id, rate=IMU_RATE):
        self.port = port
        self.imu_id = imu_id
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

def identify_imu(port):
    """Identifies the IMU by ID and returns the corresponding role."""
    node = mscl.InertialNode(mscl.Connection.Serial(port))
    imu_id = node.deviceName()  # Assuming deviceName fetches the ID
    for role, id_value in IMU_IDS.items():
        if imu_id.endswith(id_value):
            return role, id_value
    raise ValueError(f"Unknown IMU ID: {imu_id}")

def main():
    imu_ports = ["/dev/ttyACM0", "/dev/ttyACM1"]  # List of potential IMU ports
    imu_instances = {}

    # Identify and initialize IMUs
    for port in imu_ports:
        try:
            role, imu_id = identify_imu(port)
            imu_instances[role] = IMU(port, imu_id)
            print(f"{role.capitalize()} initialized on port {port}.")
        except Exception as e:
            print(f"Failed to initialize IMU on port {port}: {e}")

    if len(imu_instances) < 2:
        raise RuntimeError("Both IMUs (back of hand and wrist) must be connected.")

    # Read and print data from both IMUs
    while True:
        for role, imu in imu_instances.items():
            data = imu.get_data()
            print(f"{role.capitalize()} - Roll: {data.roll:.2f}, Pitch: {data.pitch:.2f}, Yaw: {data.yaw:.2f}")
        time.sleep(1 / IMU_RATE)  # Adjust sleep for desired sampling rate

if __name__ == "__main__":
    main()
