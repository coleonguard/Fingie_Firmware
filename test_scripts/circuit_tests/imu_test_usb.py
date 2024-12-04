import os
import time
import mscl

# Constants
IMU_RATE = 200  # Sampling rate in Hz
PORT_BACK = "/dev/ttyACM0"  # Expected port for back of hand
PORT_WRIST = "/dev/ttyACM1"  # Expected port for wrist

class IMUData:
    """Simple class to hold IMU data."""
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

class IMU:
    """Class to manage a single IMU and extract roll, pitch, yaw data."""
    def __init__(self, device_port, rate=IMU_RATE):
        self.rate = rate
        self.device_port = device_port
        self.imu_node = None
        self._enable_port()
        self._set_up_imu()

    def _enable_port(self):
        """Enables access to the IMU serial port."""
        os.system(f'sudo chmod 777 {self.device_port}')

    def _set_up_imu(self):
        """Sets up the IMU connection and data channels."""
        try:
            connection = mscl.Connection.Serial(self.device_port)
            self.imu_node = mscl.InertialNode(connection)

            # Configure data stream
            ahrs_channels = mscl.MipChannels()
            ahrs_channels.append(
                mscl.MipChannel(mscl.MipTypes.CH_FIELD_SENSOR_EULER_ANGLES,
                                mscl.SampleRate.Hertz(self.rate))
            )

            self.imu_node.setToIdle()
            self.imu_node.setActiveChannelFields(mscl.MipTypes.CLASS_AHRS_IMU, ahrs_channels)
            self.imu_node.enableDataStream(mscl.MipTypes.CLASS_AHRS_IMU)
            self.imu_node.resume()

        except mscl.Error as e:
            raise RuntimeError(f"Failed to initialize IMU on {self.device_port}: {e}")

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

def initialize_imus():
    """Try initializing both IMUs and assign roles based on success."""
    imu_back, imu_wrist = None, None

    try:
        # Try initializing back of hand IMU
        print(f"Attempting to initialize back of hand IMU on {PORT_BACK}...")
        imu_back = IMU(PORT_BACK)
        print("Back of hand IMU initialized successfully.")
    except RuntimeError:
        print(f"Failed to initialize back of hand IMU on {PORT_BACK}. Trying wrist IMU here instead...")
        imu_wrist = IMU(PORT_BACK)
        print("Wrist IMU initialized successfully on back of hand port.")

    try:
        # Try initializing wrist IMU
        print(f"Attempting to initialize wrist IMU on {PORT_WRIST}...")
        imu_wrist = IMU(PORT_WRIST)
        print("Wrist IMU initialized successfully.")
    except RuntimeError:
        print(f"Failed to initialize wrist IMU on {PORT_WRIST}. Trying back of hand IMU here instead...")
        imu_back = IMU(PORT_WRIST)
        print("Back of hand IMU initialized successfully on wrist port.")

    return imu_back, imu_wrist

def main():
    # Initialize IMUs
    imu_back, imu_wrist = initialize_imus()

    if not imu_back or not imu_wrist:
        print("Error: Could not initialize both IMUs. Please check connections.")
        return

    print("IMUs initialized and assigned correctly:")
    print(f"Back of hand IMU: {imu_back.device_port}")
    print(f"Wrist IMU: {imu_wrist.device_port}")

    # Read and print data from both IMUs
    while True:
        try:
            data_back = imu_back.get_data()
            data_wrist = imu_wrist.get_data()

            print(f"back of hand: Roll: {data_back.roll:.2f}, Pitch: {data_back.pitch:.2f}, Yaw: {data_back.yaw:.2f}")
            print(f"wrist:       Roll: {data_wrist.roll:.2f}, Pitch: {data_wrist.pitch:.2f}, Yaw: {data_wrist.yaw:.2f}")

            time.sleep(1 / IMU_RATE)  # Adjust sleep for desired sampling rate
        except RuntimeError as e:
            print(f"Error reading IMU data: {e}")
            break

if __name__ == "__main__":
    main()
