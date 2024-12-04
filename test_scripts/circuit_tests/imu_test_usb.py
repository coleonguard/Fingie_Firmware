import os
import time
import mscl

# Constants
IMU_RATE = 200  # Sampling rate in Hz
IMU_PORT = "/dev/ttyACM0"  # Adjust as needed

class IMUData:
    """Simple class to hold IMU data."""
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

def main():
    # Set permissions on port (optional if you run as sudo or have proper permissions)
    os.system(f'sudo chmod 777 {IMU_PORT}')

    # Connect to the IMU
    node = mscl.InertialNode(mscl.Connection.Serial(IMU_PORT))

    # Get device info
    info = node.getDeviceInfo()
    print("Connected to IMU with serial number:", info.serialNumber())

    # Set up Euler angle stream
    ahrs_channels = mscl.MipChannels()
    ahrs_channels.append(mscl.MipChannel(mscl.MipTypes.CH_FIELD_SENSOR_EULER_ANGLES,
                                         mscl.SampleRate.Hertz(IMU_RATE)))

    node.setToIdle()
    node.setActiveChannelFields(mscl.MipTypes.CLASS_AHRS_IMU, ahrs_channels)
    node.enableDataStream(mscl.MipTypes.CLASS_AHRS_IMU)
    node.resume()

    # Continuously read and print data
    while True:
        packets = node.getDataPackets(0)
        if packets:
            data_points = packets[-1].data()
            imu_data = IMUData()
            for dp in data_points:
                if dp.field() == mscl.MipTypes.CH_FIELD_SENSOR_EULER_ANGLES:
                    if dp.qualifier() == 6:   # Roll
                        imu_data.roll = dp.as_double()
                    elif dp.qualifier() == 7: # Pitch
                        imu_data.pitch = dp.as_double()
                    elif dp.qualifier() == 8: # Yaw
                        imu_data.yaw = dp.as_double()

            # Print the latest Euler angles
            print(f"Roll: {imu_data.roll:.2f}, Pitch: {imu_data.pitch:.2f}, Yaw: {imu_data.yaw:.2f}")

        time.sleep(1 / IMU_RATE)

if __name__ == "__main__":
    main()
