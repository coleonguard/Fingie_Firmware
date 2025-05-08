#!/usr/bin/env python3
"""
Test for I2C multiplexer and VL6180X sensors.

This test verifies the communication with VL6180X proximity sensors 
through I2C multiplexers, which will be crucial for hardware integration.
"""

import sys
import os
import unittest
import time
import logging
import threading
import random
from unittest.mock import MagicMock, patch

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("I2CMuxTest")

# Create mocks for hardware dependencies
class MockSMBus:
    """Mock SMBus for I2C communication"""
    
    def __init__(self, bus=None):
        """Initialize the mock SMBus"""
        self.written_data = {}  # (addr, reg) -> data
        self.read_data = {}     # (addr, reg) -> data
        self.debug_mode = False
        
    def write_byte_data(self, addr, reg, data):
        """Write a byte to a device register"""
        if self.debug_mode:
            logger.debug(f"I2C write: addr=0x{addr:02x}, reg=0x{reg:02x}, data=0x{data:02x}")
        self.written_data[(addr, reg)] = data
        
        # Special handling for multiplexer
        if addr == 0x70 and reg == 0x00:  # TCA9548 command byte
            # Record the selected channel
            self.current_channel = data
            
    def read_byte_data(self, addr, reg):
        """Read a byte from a device register"""
        # Return specified data if provided
        if (addr, reg) in self.read_data:
            data = self.read_data[(addr, reg)]
            if self.debug_mode:
                logger.debug(f"I2C read: addr=0x{addr:02x}, reg=0x{reg:02x} -> 0x{data:02x}")
            return data
            
        # Default behaviors for different devices
        if addr == 0x29:  # VL6180X default address
            if reg == 0x00:  # Model ID register
                return 0xB4  # VL6180X model ID
            elif reg == 0x01:  # Model revision
                return 0x01
            elif reg == 0x0f:  # ROM version check
                return 0x01
            elif reg == 0x4f:  # Range status
                return 0x00  # No errors
            elif reg == 0x50:  # Result register (range value)
                # Return a random value between 5 and 200
                return random.randint(5, 200)
                
        # Default response for unknown registers
        if self.debug_mode:
            logger.debug(f"I2C read (default): addr=0x{addr:02x}, reg=0x{reg:02x} -> 0x00")
        return 0x00
        
    def read_i2c_block_data(self, addr, reg, length):
        """Read a block of data from a device register"""
        # Return a block of data
        block = []
        for i in range(length):
            if (addr, reg + i) in self.read_data:
                block.append(self.read_data[(addr, reg + i)])
            else:
                block.append(0)
        
        if self.debug_mode:
            logger.debug(f"I2C block read: addr=0x{addr:02x}, reg=0x{reg:02x}, len={length} -> {block}")
        return block
        
    def write_i2c_block_data(self, addr, reg, data):
        """Write a block of data to a device register"""
        if self.debug_mode:
            logger.debug(f"I2C block write: addr=0x{addr:02x}, reg=0x{reg:02x}, data={data}")
        
        for i, value in enumerate(data):
            self.written_data[(addr, reg + i)] = value
    
    def simulate_bus_error(self, probability=0.05):
        """
        Enable random I2C bus errors for testing error handling.
        
        This overrides normal methods to occasionally raise exceptions.
        """
        original_write = self.write_byte_data
        original_read = self.read_byte_data
        
        def write_with_errors(addr, reg, data):
            if random.random() < probability:
                raise IOError("Simulated I2C write error")
            return original_write(addr, reg, data)
            
        def read_with_errors(addr, reg):
            if random.random() < probability:
                raise IOError("Simulated I2C read error")
            return original_read(addr, reg)
            
        self.write_byte_data = write_with_errors
        self.read_byte_data = read_with_errors
    
    def set_device_not_present(self, addr):
        """Simulate a device not being present by always raising errors"""
        def raise_error(*args, **kwargs):
            raise IOError(f"Device not present at address 0x{addr:02x}")
            
        # Store original methods for this address
        self._original_methods = {
            "write": self.write_byte_data,
            "read": self.read_byte_data
        }
        
        # Override methods to raise errors for this address only
        original_write = self.write_byte_data
        original_read = self.read_byte_data
        
        def write_with_missing_device(a, reg, data):
            if a == addr:
                raise IOError(f"Device not present at address 0x{addr:02x}")
            return original_write(a, reg, data)
            
        def read_with_missing_device(a, reg):
            if a == addr:
                raise IOError(f"Device not present at address 0x{addr:02x}")
            return original_read(a, reg)
            
        self.write_byte_data = write_with_missing_device
        self.read_byte_data = read_with_missing_device
    
    def restore_device(self, addr):
        """Restore normal behavior for a device"""
        if hasattr(self, '_original_methods'):
            self.write_byte_data = self._original_methods["write"]
            self.read_byte_data = self._original_methods["read"]
            delattr(self, '_original_methods')

class I2CMultiplexer:
    """
    I2C Multiplexer (TCA9548A) interface.
    
    This class manages an I2C multiplexer to allow communication with multiple 
    devices that share the same I2C address.
    """
    
    def __init__(self, bus, address=0x70):
        """
        Initialize the multiplexer.
        
        Args:
            bus: The SMBus object for I2C communication
            address: The I2C address of the multiplexer (default: 0x70)
        """
        self.bus = bus
        self.address = address
        self.current_channel = None
        self.retry_count = 3
        self.retry_delay = 0.05  # 50ms
        
    def select_channel(self, channel):
        """
        Select a channel on the multiplexer.
        
        Args:
            channel: Channel number (0-7)
            
        Returns:
            True if successful, False otherwise
        """
        if not 0 <= channel <= 7:
            logger.error(f"Invalid channel: {channel}, must be 0-7")
            return False
            
        # Channel is selected by writing a byte with the corresponding bit set
        channel_byte = 1 << channel
        
        # Try with retries
        for attempt in range(self.retry_count):
            try:
                self.bus.write_byte_data(self.address, 0x00, channel_byte)
                self.current_channel = channel
                return True
            except IOError as e:
                logger.warning(f"I2C error selecting channel {channel} (attempt {attempt+1}): {e}")
                if attempt < self.retry_count - 1:
                    time.sleep(self.retry_delay * (attempt + 1))  # Exponential backoff
                else:
                    logger.error(f"Failed to select channel {channel} after {self.retry_count} attempts")
                    return False
    
    def get_current_channel(self):
        """Get the currently selected channel"""
        return self.current_channel
        
    def disable_all_channels(self):
        """Disable all channels"""
        try:
            self.bus.write_byte_data(self.address, 0x00, 0x00)
            self.current_channel = None
            return True
        except IOError as e:
            logger.error(f"I2C error disabling all channels: {e}")
            return False
            
    def scan_for_devices(self, channels=None):
        """
        Scan all channels for I2C devices.
        
        Args:
            channels: List of channels to scan (default: all channels 0-7)
            
        Returns:
            Dictionary mapping channels to lists of detected device addresses
        """
        if channels is None:
            channels = range(8)
            
        found_devices = {}
        
        for channel in channels:
            if not self.select_channel(channel):
                logger.warning(f"Skipping channel {channel} (could not select)")
                continue
                
            found_devices[channel] = []
            
            # Scan address range (1-127)
            for addr in range(1, 128):
                # Skip the multiplexer's own address
                if addr == self.address:
                    continue
                    
                try:
                    # Try to read from device
                    self.bus.read_byte_data(addr, 0)
                    found_devices[channel].append(addr)
                    logger.info(f"Found device at channel {channel}, address 0x{addr:02x}")
                except IOError:
                    # No device at this address
                    pass
                    
        return found_devices

class VL6180X:
    """
    VL6180X proximity sensor interface.
    
    This class provides an interface to VL6180X proximity sensors
    attached to I2C multiplexers.
    """
    
    def __init__(self, bus, address=0x29, mux=None, mux_channel=None):
        """
        Initialize the sensor.
        
        Args:
            bus: The SMBus object for I2C communication
            address: The I2C address of the sensor (default: 0x29)
            mux: Optional I2CMultiplexer object if connected through a multiplexer
            mux_channel: Channel on the multiplexer if connected through one
        """
        self.bus = bus
        self.address = address
        self.mux = mux
        self.mux_channel = mux_channel
        self.is_initialized = False
        self.retry_count = 3
        self.retry_delay = 0.05  # 50ms
        
        # Cache for readings to avoid constant I2C operations
        self.cached_reading = None
        self.cache_time = 0
        self.cache_duration = 0.01  # 10ms
        
    def initialize(self):
        """
        Initialize the sensor.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            # Select multiplexer channel if connected through a multiplexer
            if self.mux and self.mux_channel is not None:
                if not self.mux.select_channel(self.mux_channel):
                    logger.error("Failed to select multiplexer channel")
                    return False
            
            # Check if the sensor is present by reading model ID
            model_id = self._read_register(0x00)
            if model_id != 0xB4:  # VL6180X model ID
                logger.error(f"Invalid model ID: 0x{model_id:02x}, expected 0xB4")
                return False
                
            # Initialize sensor registers (simplified for test)
            # In a real implementation, this would set up the sensor configuration
            self._write_register(0x0A, 0x01)  # Set enable
                
            self.is_initialized = True
            return True
            
        except IOError as e:
            logger.error(f"Failed to initialize VL6180X: {e}")
            return False
    
    def read_distance(self):
        """
        Read distance from the sensor.
        
        Returns:
            Distance in mm or None if measurement fails
        """
        # Check if we have a recent cached reading
        if self.cached_reading is not None and time.time() - self.cache_time < self.cache_duration:
            return self.cached_reading
            
        # Select multiplexer channel if connected through a multiplexer
        if self.mux and self.mux_channel is not None:
            if not self.mux.select_channel(self.mux_channel):
                logger.error("Failed to select multiplexer channel")
                return None
                
        # Try with retries
        for attempt in range(self.retry_count):
            try:
                # In a real implementation, this would:
                # 1. Start a measurement
                # 2. Wait for completion
                # 3. Read the result
                
                # For the mock, we just read from the range result register
                distance = self._read_register(0x50)  # Range result register
                
                # Check range status register for errors
                status = self._read_register(0x4f)
                if status != 0:
                    logger.warning(f"Range status error: 0x{status:02x}")
                    # In a real implementation, we would handle different error types
                    
                # Update cache
                self.cached_reading = distance
                self.cache_time = time.time()
                
                return distance
                
            except IOError as e:
                logger.warning(f"I2C error reading distance (attempt {attempt+1}): {e}")
                if attempt < self.retry_count - 1:
                    time.sleep(self.retry_delay * (attempt + 1))  # Exponential backoff
                else:
                    logger.error(f"Failed to read distance after {self.retry_count} attempts")
                    return None
    
    def _write_register(self, register, value):
        """Write a value to a register"""
        self.bus.write_byte_data(self.address, register, value)
        
    def _read_register(self, register):
        """Read a value from a register"""
        return self.bus.read_byte_data(self.address, register)
        
    def _write_register_16bit(self, register, value):
        """Write a 16-bit value to a register"""
        # Split value into high and low bytes
        high_byte = (value >> 8) & 0xFF
        low_byte = value & 0xFF
        self.bus.write_i2c_block_data(self.address, register, [high_byte, low_byte])
        
    def _read_register_16bit(self, register):
        """Read a 16-bit value from a register"""
        data = self.bus.read_i2c_block_data(self.address, register, 2)
        return (data[0] << 8) | data[1]

class TestI2CMux(unittest.TestCase):
    """Test cases for I2C multiplexer and VL6180X sensors"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.bus = MockSMBus()
        self.mux = I2CMultiplexer(self.bus)
        
        # Configure the mock bus to simulate presence of sensors
        for channel in range(5):  # 5 sensors on different channels
            for reg in [0x00, 0x01, 0x0f]:  # Model ID, revision, ROM version
                self.bus.read_data[(0x29, reg)] = 0xB4 if reg == 0x00 else 0x01
    
    def test_multiplexer_channel_selection(self):
        """Test multiplexer channel selection"""
        # Test selecting valid channels
        for channel in range(8):
            result = self.mux.select_channel(channel)
            self.assertTrue(result, f"Failed to select channel {channel}")
            self.assertEqual(self.mux.get_current_channel(), channel)
            
        # Test invalid channel
        result = self.mux.select_channel(8)
        self.assertFalse(result, "Should fail with invalid channel")
        
        # Test disabling all channels
        result = self.mux.disable_all_channels()
        self.assertTrue(result)
        self.assertIsNone(self.mux.get_current_channel())
    
    def test_sensor_initialization(self):
        """Test sensor initialization through multiplexer"""
        # Create sensors on 5 different channels
        sensors = []
        for channel in range(5):
            sensor = VL6180X(self.bus, mux=self.mux, mux_channel=channel)
            sensors.append(sensor)
            
        # Initialize all sensors
        for channel, sensor in enumerate(sensors):
            result = sensor.initialize()
            self.assertTrue(result, f"Failed to initialize sensor on channel {channel}")
            self.assertTrue(sensor.is_initialized)
            
        # Verify multiplexer was used to select channels
        # In real implementation, this would be verified through bus traffic analysis
    
    def test_sensor_reading(self):
        """Test reading from sensors through multiplexer"""
        # Create sensors on 5 different channels
        sensors = []
        for channel in range(5):
            sensor = VL6180X(self.bus, mux=self.mux, mux_channel=channel)
            sensor.initialize()
            sensors.append(sensor)
            
        # Read from all sensors in multiple rounds
        for _ in range(10):
            for channel, sensor in enumerate(sensors):
                distance = sensor.read_distance()
                self.assertIsNotNone(distance, f"Failed to read from sensor on channel {channel}")
                self.assertTrue(5 <= distance <= 200, f"Distance value out of range: {distance}")
    
    def test_error_handling(self):
        """Test error handling for I2C issues"""
        # Create a sensor
        sensor = VL6180X(self.bus, mux=self.mux, mux_channel=0)
        sensor.initialize()
        
        # First, make sure normal reading works
        distance = sensor.read_distance()
        self.assertIsNotNone(distance)
        
        # Now, simulate bus errors
        self.bus.simulate_bus_error(probability=1.0)  # 100% error rate
        
        # Reading should fail after retries
        distance = sensor.read_distance()
        self.assertIsNone(distance, "Reading should fail with simulated bus errors")
        
        # Simulate a missing device
        self.bus = MockSMBus()  # Create new bus
        self.mux = I2CMultiplexer(self.bus)
        self.bus.set_device_not_present(0x29)  # VL6180X default address
        
        sensor = VL6180X(self.bus, mux=self.mux, mux_channel=0)
        result = sensor.initialize()
        self.assertFalse(result, "Initialization should fail with missing device")
    
    def test_concurrent_access(self):
        """Test concurrent access to multiple sensors"""
        # Create sensors on 5 different channels
        sensors = []
        for channel in range(5):
            sensor = VL6180X(self.bus, mux=self.mux, mux_channel=channel)
            sensor.initialize()
            sensors.append(sensor)
            
        # Create threads to read sensors concurrently
        threads = []
        results = {}
        
        def sensor_reader(sensor_idx, count):
            """Thread function for reading from a sensor"""
            readings = []
            for _ in range(count):
                reading = sensors[sensor_idx].read_distance()
                readings.append(reading)
                time.sleep(0.01)  # Small delay
            results[sensor_idx] = readings
            
        # Create and start threads
        for i in range(len(sensors)):
            thread = threading.Thread(target=sensor_reader, args=(i, 10))
            threads.append(thread)
            thread.start()
            
        # Wait for all threads to complete
        for thread in threads:
            thread.join()
            
        # Verify all sensors were read correctly
        for sensor_idx, readings in results.items():
            self.assertEqual(len(readings), 10, f"Sensor {sensor_idx} should have 10 readings")
            self.assertTrue(all(r is not None for r in readings), 
                         f"Sensor {sensor_idx} had failed readings")

if __name__ == "__main__":
    unittest.main()