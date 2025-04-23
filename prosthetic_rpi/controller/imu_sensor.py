# IMU Sensor Module: Handles IMU orientation data collection for hand tracking

import os
import time
import threading
import numpy as np
from collections import deque
import mscl  # MicroStrain Communication Library

# Constants
IMU_RATE = 100  # Sampling rate in Hz
PORT_BACK = "/dev/ttyACM0"  # Expected port for back of hand
PORT_WRIST = "/dev/ttyACM1"  # Expected port for wrist

class IMUData:
    """Simple class to hold IMU data."""
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.timestamp = time.time()
        
    def to_dict(self):
        """Convert IMU data to dictionary for logging."""
        return {
            "roll": self.roll,
            "pitch": self.pitch,
            "yaw": self.yaw,
            "timestamp": self.timestamp
        }

class IMU:
    """Class to manage a single IMU and extract roll, pitch, yaw data."""
    def __init__(self, device_port, rate=IMU_RATE, name="unnamed"):
        self.rate = rate
        self.device_port = device_port
        self.imu_node = None
        self.name = name  # Identifier for this IMU (e.g., "back", "wrist")
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

            # Configure data stream for Euler angles
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
        imu_data.timestamp = time.time()
        
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
        
    def close(self):
        """Close the IMU connection properly."""
        if self.imu_node:
            try:
                self.imu_node.setToIdle()
            except:
                pass

class IMUManager:
    """Manages multiple IMUs and provides orientation data for the hand."""
    
    def __init__(self, sampling_rate=IMU_RATE):
        """Initialize the IMU manager
        
        Args:
            sampling_rate: Frequency to sample IMU data (Hz)
        """
        self.sampling_rate = sampling_rate
        self.sampling_interval = 1.0 / sampling_rate
        
        # IMU references
        self.imu_back = None
        self.imu_wrist = None
        
        # Data storage with timestamp
        self.back_data = IMUData()
        self.wrist_data = IMUData()
        
        # For filtering (simple moving average)
        self.filter_window = 5
        self.back_history = {
            "roll": deque(maxlen=self.filter_window),
            "pitch": deque(maxlen=self.filter_window),
            "yaw": deque(maxlen=self.filter_window)
        }
        self.wrist_history = {
            "roll": deque(maxlen=self.filter_window),
            "pitch": deque(maxlen=self.filter_window),
            "yaw": deque(maxlen=self.filter_window)
        }
        
        # Control variables
        self.running = False
        self.thread = None
    
    def initialize_imus(self):
        """Try initializing both IMUs and assign roles based on success."""
        try:
            # Try initializing back of hand IMU
            print(f"Attempting to initialize back of hand IMU on {PORT_BACK}...")
            self.imu_back = IMU(PORT_BACK, rate=self.sampling_rate, name="back")
            print("Back of hand IMU initialized successfully.")
        except RuntimeError as e:
            print(f"Failed to initialize back of hand IMU: {e}")
            self.imu_back = None

        try:
            # Try initializing wrist IMU
            print(f"Attempting to initialize wrist IMU on {PORT_WRIST}...")
            self.imu_wrist = IMU(PORT_WRIST, rate=self.sampling_rate, name="wrist")
            print("Wrist IMU initialized successfully.")
        except RuntimeError as e:
            print(f"Failed to initialize wrist IMU: {e}")
            self.imu_wrist = None
            
        return (self.imu_back is not None) or (self.imu_wrist is not None)

    def _update_filtered_data(self, data, history):
        """Apply a simple moving average filter to IMU data."""
        history["roll"].append(data.roll)
        history["pitch"].append(data.pitch)
        history["yaw"].append(data.yaw)
        
        # Calculate averages
        data.roll = sum(history["roll"]) / len(history["roll"])
        data.pitch = sum(history["pitch"]) / len(history["pitch"])
        data.yaw = sum(history["yaw"]) / len(history["yaw"])
        
        return data
    
    def _imu_reading_thread(self):
        """Background thread for continuous IMU reading."""
        next_sample_time = time.time()
        
        while self.running:
            current_time = time.time()
            
            # Check if it's time for the next sample
            if current_time >= next_sample_time:
                # Read from IMUs if available
                if self.imu_back:
                    try:
                        self.back_data = self.imu_back.get_data()
                        self.back_data = self._update_filtered_data(self.back_data, self.back_history)
                    except Exception as e:
                        print(f"Error reading back IMU: {e}")
                        
                if self.imu_wrist:
                    try:
                        self.wrist_data = self.imu_wrist.get_data()
                        self.wrist_data = self._update_filtered_data(self.wrist_data, self.wrist_history)
                    except Exception as e:
                        print(f"Error reading wrist IMU: {e}")
                
                # Calculate next sample time
                next_sample_time = current_time + self.sampling_interval
            
            # Small sleep to prevent CPU hogging
            time.sleep(0.001)
    
    def start(self):
        """Start the IMU reading thread."""
        if not self.running:
            # Initialize IMUs first
            if not self.initialize_imus():
                print("Warning: No IMUs could be initialized")
                return False
                
            self.running = True
            self.thread = threading.Thread(target=self._imu_reading_thread)
            self.thread.daemon = True
            self.thread.start()
            print("IMU manager started")
            return True
        return False
    
    def stop(self):
        """Stop the IMU reading thread."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        
        # Clean up
        if self.imu_back:
            self.imu_back.close()
        if self.imu_wrist:
            self.imu_wrist.close()
            
        print("IMU manager stopped")
    
    def get_orientation(self, location="back"):
        """Get the current orientation data for a specific IMU.
        
        Args:
            location: Which IMU to get data from ("back" or "wrist")
            
        Returns:
            IMUData object with current orientation
        """
        if location == "back":
            return self.back_data
        elif location == "wrist":
            return self.wrist_data
        else:
            raise ValueError(f"Unknown IMU location: {location}")
    
    def get_all_data(self):
        """Get all IMU data as a dictionary for logging.
        
        Returns:
            Dictionary with all IMU data
        """
        return {
            "back": self.back_data.to_dict() if self.imu_back else None,
            "wrist": self.wrist_data.to_dict() if self.imu_wrist else None
        }