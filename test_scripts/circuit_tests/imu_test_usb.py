import os
import mscl

# Constants
IMU_RATE = 200  # Sampling rate in Hz
PORT_BACK = "/dev/ttyACM0"  # Expected port for back of hand
PORT_WRIST = "/dev/ttyACM1"  # Expected port for wrist

class IMU:
    """Class to manage a single IMU connection."""
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
        """Attempts to set up the IMU connection."""
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
    imu_back, imu_wrist = initialize_imus()

    if not imu_back or not imu_wrist:
        print("Error: Could not initialize both IMUs. Please check connections.")
        return

    print("IMUs initialized and assigned correctly:")
    print(f"Back of hand IMU: {imu_back.device_port}")
    print(f"Wrist IMU: {imu_wrist.device_port}")

if __name__ == "__main__":
    main()
