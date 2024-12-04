import os
import time
import mscl
from scipy.spatial.transform import Rotation as R

# Constants
IMU_RATE = 200  # Sampling rate in Hz
BACK_OF_HAND_IMU_ID = "133931"  # Back of hand
WRIST_IMU_ID = "156124"         # Wrist

class IMUData:
    """Simple class to hold IMU data without dataclasses."""
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

class IMU:
    """Class to manage a single IMU and extract roll, pitch, yaw data."""
    
    def __init__(self, imu_id, device_port, rate=IMU_RATE):
        self.imu_id = imu_id
        self.rate = rate
        self.device_port = device_port
        self._enable_port()
        self.imu_node = self._set_up_imu()

    def _enable_port(self):
        """Enables access to the IMU serial port."""
        os.system(f'sudo chmod 777 {self.device_port}')

    def _set_up_imu(self):
        """Sets up the IMU connection and data channels."""
        node = mscl.InertialNode(mscl.Connection.Serial(self.device_port))
        ahrs_channels = mscl.MipChannels()
        ahrs_channels.append(mscl.MipChannel(mscl.MipTypes.CH_FIELD_SENSOR_EULER_ANGLES,
                                             mscl.SampleRate.Hertz(self.rate)))
        
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

def main():
    # Initialize both IMUs
    imu_back = IMU(BACK_OF_HAND_IMU_ID, "/dev/ttyACM0", IMU_RATE)
    imu_wrist = IMU(WRIST_IMU_ID, "/dev/ttyACM1", IMU_RATE)

    while True:
        data_back = imu_back.get_data()
        data_wrist = imu_wrist.get_data()

        # Print data, labeling based on IMU ID
        # Since we know BACK_OF_HAND_IMU_ID is 133931, we print back_of_hand for that one
        print(f'back of hand: Roll: {data_back.roll:.2f}, Pitch: {data_back.pitch:.2f}, Yaw: {data_back.yaw:.2f}')
        print(f'wrist:       Roll: {data_wrist.roll:.2f}, Pitch: {data_wrist.pitch:.2f}, Yaw: {data_wrist.yaw:.2f}')

        time.sleep(1 / IMU_RATE)  # Adjust sleep for desired sampling rate

if __name__ == "__main__":
    main()
