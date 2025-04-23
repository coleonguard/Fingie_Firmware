# Sensor Module: Handles proximity sensor readings, filtering, and processing

import smbus2
import time
import threading
import numpy as np
from collections import deque

# ------------------------------------------------------------------------
# I²C Addresses
# ------------------------------------------------------------------------
VL6180X_ADDRESS = 0x29   # Hardcoded address of each VL6180X sensor

# Multiplexer addresses
MUX1_ADDRESS = 0x70      # MULT1
MUX2_ADDRESS = 0x71      # MULT2

# ------------------------------------------------------------------------
# Sensor-to-Multiplexer Mapping
# ------------------------------------------------------------------------
SENSORS = [
    # (multiplexer_addr, channel, "DescriptiveName")
    (MUX1_ADDRESS, 0, "Thumb1"),
    (MUX1_ADDRESS, 1, "Thumb2"),
    (MUX1_ADDRESS, 2, "Index1"),
    (MUX1_ADDRESS, 3, "Index2"),
    (MUX1_ADDRESS, 4, "Middle1"),
    (MUX2_ADDRESS, 0, "Middle2"),
    (MUX2_ADDRESS, 1, "Ring1"),
    (MUX2_ADDRESS, 2, "Ring2"),
    (MUX2_ADDRESS, 3, "Pinky1"),
    (MUX2_ADDRESS, 4, "Pinky2"),
]

# MCP joint sensors (the ones we primarily care about)
MCP_SENSORS = [
    "Thumb1",
    "Index1",
    "Middle1",
    "Ring1",
    "Pinky1"
]

class KalmanFilter:
    """Simple Kalman filter for smoothing sensor readings"""
    
    def __init__(self, process_variance=1e-3, measurement_variance=1e-1, initial_value=30):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = initial_value
        self.estimate_error = 1.0
        
    def update(self, measurement):
        """Update filter with new measurement"""
        # Prediction step
        prediction_error = self.estimate_error + self.process_variance
        
        # Update step
        kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.estimate_error = (1 - kalman_gain) * prediction_error
        
        return self.estimate

class ProximitySensorManager:
    """Manages all proximity sensors, including reading and filtering"""
    
    def __init__(self, sampling_rate=20):  # 20Hz default sampling rate
        """Initialize the sensor manager
        
        Args:
            sampling_rate: Frequency to sample all sensors (Hz)
        """
        self.bus = smbus2.SMBus(1)  # Raspberry Pi I2C bus
        self.sampling_rate = sampling_rate
        self.sampling_interval = 1.0 / sampling_rate
        
        # Initialize sensor data dictionaries
        self.raw_values = {sensor_name: 30 for _, _, sensor_name in SENSORS}  # Default 30mm
        self.filtered_values = {sensor_name: 30 for _, _, sensor_name in SENSORS}
        
        # Create Kalman filters for each sensor
        self.filters = {sensor_name: KalmanFilter() for _, _, sensor_name in SENSORS}
        
        # Control variables
        self.running = False
        self.thread = None
        
        # Threshold for meaningful proximity (mm)
        self.proximity_threshold = 30
        self.contact_threshold = 5
    
    def select_channel(self, mux_address, channel):
        """Enable a specific channel on the TCA9548A multiplexer."""
        self.bus.write_byte(mux_address, 1 << channel)
        time.sleep(0.01)  # Small delay for stable switching
    
    def write_byte(self, device_address, reg, data):
        """Write a single byte to a register (16-bit address) on the VL6180X."""
        self.bus.write_i2c_block_data(device_address,
                               (reg >> 8) & 0xFF,
                               [(reg & 0xFF), data])
    
    def read_byte(self, device_address, reg):
        """Read a single byte from a register (16-bit address) on the VL6180X."""
        self.bus.write_i2c_block_data(device_address,
                               (reg >> 8) & 0xFF,
                               [(reg & 0xFF)])
        time.sleep(0.01)
        return self.bus.read_byte(device_address)
    
    def initialize_vl6180x(self):
        """Initialize VL6180X sensor if needed."""
        if self.read_byte(VL6180X_ADDRESS, 0x016) == 1:
            # Full initialization sequence (only when needed)
            # Register settings for optimal proximity sensing
            write_registers = [
                (0x0207, 0x01), (0x0208, 0x01), (0x0096, 0x00),
                (0x0097, 0xfd), (0x00e3, 0x00), (0x00e4, 0x04),
                (0x00e5, 0x02), (0x00e6, 0x01), (0x00e7, 0x03),
                (0x00f5, 0x02), (0x00d9, 0x05), (0x00db, 0xce),
                (0x00dc, 0x03), (0x00dd, 0xf8), (0x009f, 0x00),
                (0x00a3, 0x3c), (0x00b7, 0x00), (0x00bb, 0x3c),
                (0x00b2, 0x09), (0x00ca, 0x09), (0x0198, 0x01),
                (0x01b0, 0x17), (0x01ad, 0x00), (0x00ff, 0x05),
                (0x0100, 0x05), (0x0199, 0x05), (0x01a6, 0x1b),
                (0x01ac, 0x3e), (0x01a7, 0x1f), (0x0030, 0x00),
                
                # Public registers
                (0x0011, 0x10), (0x010a, 0x30), (0x003f, 0x46),
                (0x0031, 0xFF), (0x0040, 0x63), (0x002e, 0x01),
                (0x001b, 0x09), (0x003e, 0x31), (0x0014, 0x24),
                
                # Final initialization bit
                (0x016, 0x00)
            ]
            
            for reg, value in write_registers:
                self.write_byte(VL6180X_ADDRESS, reg, value)
    
    def get_distance(self):
        """Get distance measurement (in mm)."""
        # Start range measurement
        self.write_byte(VL6180X_ADDRESS, 0x018, 0x01)
        
        # Poll until measurement is ready
        retry_count = 0
        max_retries = 10  # Prevent infinite loops
        
        while retry_count < max_retries:
            status = self.read_byte(VL6180X_ADDRESS, 0x04F)
            range_status = status & 0x07
            if range_status == 0x04:  # Measurement ready
                break
            time.sleep(0.01)
            retry_count += 1
            
        # Read distance value
        dist = self.read_byte(VL6180X_ADDRESS, 0x062)  # Range result register
        
        # Clear interrupts
        self.write_byte(VL6180X_ADDRESS, 0x015, 0x07)
        
        return dist
    
    def _sensor_reading_thread(self):
        """Background thread for continuous sensor reading"""
        next_sample_time = time.time()
        
        while self.running:
            current_time = time.time()
            
            # Check if it's time for the next sample
            if current_time >= next_sample_time:
                # Read all sensors
                self._read_all_sensors()
                
                # Calculate next sample time
                next_sample_time = current_time + self.sampling_interval
            
            # Small sleep to prevent CPU hogging
            time.sleep(0.001)
    
    def _read_all_sensors(self):
        """Read and filter all proximity sensors"""
        for mux_addr, channel, sensor_name in SENSORS:
            # We're only interested in MCP sensors (named with "1" suffix)
            if sensor_name in MCP_SENSORS:
                # Select the multiplexer channel
                self.select_channel(mux_addr, channel)
                
                # Initialize sensor if needed
                self.initialize_vl6180x()
                
                # Read raw distance
                raw_distance = self.get_distance()
                self.raw_values[sensor_name] = raw_distance
                
                # Apply Kalman filter
                filtered_distance = self.filters[sensor_name].update(raw_distance)
                self.filtered_values[sensor_name] = filtered_distance
    
    def start(self):
        """Start the sensor reading thread"""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._sensor_reading_thread)
            self.thread.daemon = True  # Allow program to exit even if thread is running
            self.thread.start()
    
    def stop(self):
        """Stop the sensor reading thread"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)  # Wait for thread to finish
            
        # Clean up
        self.bus.write_byte(MUX1_ADDRESS, 0x00)  # Disable all channels
        self.bus.write_byte(MUX2_ADDRESS, 0x00)
        self.bus.close()
    
    def get_sensor_value(self, sensor_name, filtered=True):
        """Get the latest value for a specific sensor
        
        Args:
            sensor_name: Name of the sensor to read
            filtered: Whether to return filtered (True) or raw (False) value
            
        Returns:
            Current sensor reading in mm
        """
        if filtered:
            return self.filtered_values.get(sensor_name, 30)  # Default 30mm if not found
        else:
            return self.raw_values.get(sensor_name, 30)
