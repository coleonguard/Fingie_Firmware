#!/usr/bin/env python3
"""
Proximity Sensor Manager for Prosthetic Control.

This module implements the ProximityManager class which handles:
- Initialization and control of multiplexers
- Reading from VL6180X sensors 
- Kalman filtering of sensor data
- Fallback mechanisms for sensor errors
"""

import smbus2
import time
import threading
from enum import Enum
import logging
from collections import deque
import numpy as np

# Local imports
from .kalman_filter import KalmanFilter

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("ProximityManager")

# I²C Addresses and Sensor Mapping
VL6180X_ADDRESS = 0x29   # VL6180X sensor address

# Default multiplexer addresses
MUX1_ADDRESS = 0x70  # First multiplexer (typically on mcp joints)
MUX2_ADDRESS = 0x71  # Second multiplexer (typically on pip joints)

# Default sensor layout
# Format: (multiplexer_addr, channel, "DescriptiveName")
DEFAULT_SENSORS = [
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

# Default fallback mapping for sensor substitution
DEFAULT_FALLBACK = {
    "Thumb1": ["Thumb2", "Index1", "Index2", "Middle1"],
    "Thumb2": ["Thumb1", "Index2", "Index1", "Middle2"],
    "Index1": ["Index2", "Thumb2", "Middle1", "Middle2"],
    "Index2": ["Index1", "Middle2", "Thumb2", "Middle1"],
    "Middle1": ["Middle2", "Index1", "Ring1", "Ring2"],
    "Middle2": ["Middle1", "Index2", "Ring2", "Ring1"],
    "Ring1": ["Ring2", "Middle1", "Pinky1", "Pinky2"],
    "Ring2": ["Ring1", "Middle2", "Pinky2", "Pinky1"],
    "Pinky1": ["Pinky2", "Ring1", "Ring2", "Middle1"],
    "Pinky2": ["Pinky1", "Ring2", "Ring1", "Middle2"],
}

# MCP joint sensors - primary control sensors
DEFAULT_MCP_SENSORS = ["Thumb1", "Index1", "Middle1", "Ring1", "Pinky1"]


class ProximityManager:
    """
    Manages all proximity sensors, including reading, filtering, and error handling.
    
    This class implements:
    1. VL6180X sensor initialization and reading
    2. Multiplexer control for accessing multiple sensors
    3. Kalman filtering to smooth sensor readings
    4. Neighbor substitution when sensors fail
    5. Asynchronous reading via background thread
    """
    
    def __init__(
        self,
        sampling_rate=20,  # Hz
        sensors=None,
        fallback_map=None,
        bus_num=1,
        max_ready_ms=20,  # ms
        n_retries=3,
        approach_threshold=40,  # mm
        contact_threshold=5,   # mm
    ):
        """
        Initialize the proximity sensor manager.
        
        Args:
            sampling_rate: Frequency to sample all sensors (Hz)
            sensors: Sensor configuration list (default uses DEFAULT_SENSORS)
            fallback_map: Sensor substitution map (default uses DEFAULT_FALLBACK)
            bus_num: I2C bus number
            max_ready_ms: Maximum time to wait for sensor ready (ms)
            n_retries: Number of retries before considering a sensor reading failed
            approach_threshold: Distance threshold for approach phase (mm)
            contact_threshold: Distance threshold for contact phase (mm)
        """
        # Save parameters
        self.sampling_rate = sampling_rate
        self.sampling_interval = 1.0 / sampling_rate
        self.sensors = sensors or DEFAULT_SENSORS
        self.fallback_map = fallback_map or DEFAULT_FALLBACK
        self.max_ready_ms = max_ready_ms
        self.n_retries = n_retries
        
        # Thresholds (in mm)
        self.approach_threshold = approach_threshold
        self.contact_threshold = contact_threshold
        
        # Initialize I2C bus
        self.bus = smbus2.SMBus(bus_num)
        self.bus_lock = threading.RLock()  # Thread-safe lock for I2C bus access
        
        # Sensor data structures
        self.sensor_names = [name for _, _, name in self.sensors]
        self.raw_values = {name: 100 for name in self.sensor_names}  # Default to 100mm instead of 30mm
        self.filtered_values = {name: 100 for name in self.sensor_names}  # Default to 100mm instead of 30mm
        self.filtered_derivatives = {name: 0.0 for name in self.sensor_names}  # mm/s
        self.status = {name: "OK" for name in self.sensor_names}  # OK, SUB, BAD
        
        # Initialize filters for each sensor
        self.filters = {name: KalmanFilter() for name in self.sensor_names}
        
        # Initialize history for derivative calculation
        self.history = {name: deque(maxlen=5) for name in self.sensor_names}
        self.timestamp_history = {name: deque(maxlen=5) for name in self.sensor_names}
        
        # Control variables
        self.running = False
        self.shutdown_event = threading.Event()  # Event to signal shutdown
        self.thread = None
        self.init_done = {(mux, ch): False for mux, ch, _ in self.sensors}
        
        # Track current mux/channel for better error reporting
        self.current_mux = None
        self.current_channel = None
        
        # Track sensor health and recent errors
        self.error_counts = {name: 0 for name in self.sensor_names}
        self.last_valid_reading_time = {name: 0 for name in self.sensor_names}
        
        # Cache of last valid readings to handle intermittent failures
        self.reading_cache = {name: collections.deque(maxlen=5) for name in self.sensor_names}
    
    def select_channel(self, mux_address, channel):
        """
        Enable a specific channel on the TCA9548A multiplexer.
        
        Args:
            mux_address: I2C address of the multiplexer
            channel: Channel number to select (0-7)
            
        Raises:
            RuntimeError: If attempting to access hardware during shutdown
        """
        # Check if we're in shutdown
        if self.shutdown_event.is_set():
            raise RuntimeError("Cannot access hardware during shutdown")
            
        try:
            # Track which mux/channel we're currently using for better error reporting
            self.current_mux = mux_address
            self.current_channel = channel
            
            with self.bus_lock:
                self.bus.write_byte(mux_address, 1 << channel)
                time.sleep(0.0005)  # Small delay for stable switching
        except Exception as e:
            logger.error(f"Failed to select channel {channel} on multiplexer {hex(mux_address)}: {e}")
            raise
    
    def write_byte(self, reg, val):
        """
        Write a single byte to a register on the VL6180X.
        
        Args:
            reg: 16-bit register address
            val: 8-bit value to write
            
        Raises:
            RuntimeError: If attempting to access hardware during shutdown
        """
        # Check if we're in shutdown
        if self.shutdown_event.is_set():
            raise RuntimeError("Cannot access hardware during shutdown")
            
        try:
            with self.bus_lock:
                hi, lo = reg >> 8 & 0xFF, reg & 0xFF
                self.bus.write_i2c_block_data(VL6180X_ADDRESS, hi, [lo, val])
        except Exception as e:
            logger.error(f"Failed to write {val} to register {hex(reg)}: {e}")
            raise
    
    def read_byte(self, reg):
        """
        Read a single byte from a register on the VL6180X.
        
        Args:
            reg: 16-bit register address
            
        Returns:
            The 8-bit value read from the register
            
        Raises:
            RuntimeError: If attempting to access hardware during shutdown
        """
        # Check if we're in shutdown
        if self.shutdown_event.is_set():
            raise RuntimeError("Cannot access hardware during shutdown")
            
        try:
            with self.bus_lock:
                hi, lo = reg >> 8 & 0xFF, reg & 0xFF
                self.bus.write_i2c_block_data(VL6180X_ADDRESS, hi, [lo])
                time.sleep(0.0003)
                return self.bus.read_byte(VL6180X_ADDRESS)
        except Exception as e:
            logger.error(f"Failed to read from register {hex(reg)}: {e}")
            raise
    
    def initialize_vl6180x(self):
        """
        Initialize VL6180X sensor if needed.
        
        Returns:
            True if initialization was performed, False if already initialized
        """
        # Check if initialization is needed (fresh-out-of-reset)
        if self.read_byte(0x016) != 1:
            return False
            
        # Full initialization sequence
        init_settings = [
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
        
        for reg, value in init_settings:
            self.write_byte(reg, value)
            
        return True
    
    def start_range(self):
        """Start a range measurement"""
        self.write_byte(0x018, 0x01)
    
    def clear_interrupts(self):
        """Clear all interrupts"""
        self.write_byte(0x015, 0x07)
    
    def wait_ready(self):
        """
        Wait for range measurement to be ready.
        
        Raises:
            TimeoutError: If measurement not ready within max_ready_ms
        """
        t0 = time.time()
        while (self.read_byte(0x04F) & 0x07) != 0x04:
            if (time.time() - t0) * 1000 > self.max_ready_ms:
                raise TimeoutError("Timeout waiting for range ready")
            time.sleep(0.0003)
    
    def get_distance(self):
        """
        Get distance measurement from the current sensor.
        
        This performs a complete measurement cycle (start, wait, read, clear).
        
        Returns:
            Distance in mm or None if all retry attempts fail
            
        Raises:
            RuntimeError: If attempting to access hardware during shutdown
        """
        # Check if we're in shutdown
        if self.shutdown_event.is_set():
            raise RuntimeError("Cannot access hardware during shutdown")
        
        # Enhanced version of the fallback_test.py get_distance() function with exponential backoff
        base_delay = 0.002  # Base delay between retries (same as original)
        max_delay = 0.05    # Maximum delay between retries
        
        # Determine if we're in a high-load scenario (motors moving)
        # We'll check if we've had recent failures and increase retries in that case
        dynamic_retries = self.n_retries
        
        for attempt in range(dynamic_retries):
            try:
                # Clear any previous errors before starting
                try:
                    self.clear_interrupts()
                except Exception:
                    # Ignore errors here - we're just trying to reset the sensor
                    pass
                
                # Start the range measurement
                self.start_range()
                
                # Wait for the measurement to complete
                self.wait_ready()
                
                # Read the distance value
                d = self.read_byte(0x062)  # Range result register
                
                # Clear interrupts after successful read
                self.clear_interrupts()
                
                # Return valid reading
                return d
                
            except TimeoutError:
                # Calculate exponential backoff delay
                delay = min(base_delay * (2 ** attempt), max_delay)
                
                # Add a small random component to reduce chance of synchronized retries
                jitter = delay * 0.2 * (0.5 + 0.5 * (time.time() % 1.0))
                time.sleep(delay + jitter)
                
            except RuntimeError as e:
                # If we're shutting down, propagate the exception
                if "shutdown" in str(e):
                    raise
                
                # For I2C-specific errors, use a longer delay
                if "I/O error" in str(e) or "I2C" in str(e):
                    # This indicates bus contention - use longer delay
                    time.sleep(max_delay)
                else:
                    # For other errors, use the same exponential backoff as timeout
                    delay = min(base_delay * (2 ** attempt), max_delay)
                    time.sleep(delay)
        
        # All retries failed - log which sensor was being read if possible
        sensor_info = "unknown sensor"
        for mux, ch, name in self.sensors:
            try:
                # Check which sensors use the current mux/channel
                if (mux, ch) == (self.current_mux, self.current_channel):
                    sensor_info = f"{name} (mux: {hex(mux)}, ch: {ch})"
                    break
            except:
                pass
                
        logger.warning(f"All retries failed in get_distance() for {sensor_info}")
        return None
    
    def get_sensor_value(self, sensor_name, filtered=True, with_status=False):
        """
        Get the latest value for a specific sensor.
        
        Args:
            sensor_name: Name of the sensor to read
            filtered: Whether to return filtered (True) or raw (False) value
            with_status: Whether to return status along with value
            
        Returns:
            Current sensor reading in mm, or tuple (value, status) if with_status=True
        """
        if filtered:
            value = self.filtered_values.get(sensor_name, 100)  # Default to 100mm instead of 30mm
        else:
            value = self.raw_values.get(sensor_name, 100)  # Default to 100mm instead of 30mm
            
        if with_status:
            status = self.status.get(sensor_name, "BAD")
            return value, status
        else:
            return value
    
    def get_all_values(self, filtered=True):
        """
        Get all sensor values.
        
        Args:
            filtered: Whether to return filtered (True) or raw (False) values
            
        Returns:
            Dictionary mapping sensor names to current values
        """
        if filtered:
            return self.filtered_values.copy()
        else:
            return self.raw_values.copy()
    
    def _read_all_sensors(self):
        """
        Read all proximity sensors and apply filtering and fallback.
        
        This method:
        1. Reads raw values from all sensors
        2. Applies Kalman filtering to raw values
        3. Applies fallback substitution for failed readings
        4. Updates status for each sensor
        
        Behavior is modeled after fallback_test.py for consistency.
        """
        # Step 1: Read raw values from all sensors (similar to fallback_test.py)
        raw = {}
        ok = []
        
        for mux, ch, name in self.sensors:
            try:
                # Select the correct channel on the multiplexer
                self.select_channel(mux, ch)
                
                # Initialize sensor if needed
                if not self.init_done[(mux, ch)]:
                    if self.initialize_vl6180x():
                        logger.info(f"Initialized sensor {name} on mux {hex(mux)}, channel {ch}")
                    self.init_done[(mux, ch)] = True
                
                # Read raw distance
                distance = self.get_distance()
                
                # Store valid readings and update cache
                if distance is not None:
                    # Valid reading - update cache and raw values
                    raw[name] = distance
                    self.raw_values[name] = distance
                    ok.append(name)  # Mark as OK
                    
                    # Update cache and tracking
                    self.reading_cache[name].append(distance)
                    self.last_valid_reading_time[name] = time.time()
                    self.error_counts[name] = 0  # Reset error count on success
                else:
                    # Reading failed - track error
                    self.error_counts[name] += 1
                    
                    # Use cached readings if available and not too stale
                    cached_values = list(self.reading_cache[name])
                    if cached_values and (time.time() - self.last_valid_reading_time[name] < 1.0):
                        # Use average of recent valid readings
                        avg_distance = sum(cached_values) / len(cached_values)
                        logger.debug(f"Using cached value for {name}: {avg_distance:.1f}mm")
                        raw[name] = int(avg_distance)
                        ok.append(name)  # Consider this a success since we have a value
                    else:
                        # No usable cached values - mark as failed
                        raw[name] = None
                        
                        # If errors are persistent, gradually fade to max distance (100mm)
                        if self.error_counts[name] > 5 and name in self.raw_values:
                            # Gradually increase towards 100mm
                            current = self.raw_values[name]
                            if current < 100:
                                # Move 10% closer to 100mm
                                new_value = current + min(5, int((100 - current) * 0.1))
                                logger.debug(f"Fading {name} to max: {current}mm -> {new_value}mm")
                                self.raw_values[name] = new_value
            except Exception as e:
                logger.warning(f"Error reading sensor {name}: {e}")
                raw[name] = None
        
        # Step 2: Apply Kalman filtering to valid readings
        timestamp = time.time()
        for name in self.sensor_names:
            if raw[name] is not None:
                # Apply filter
                filtered = self.filters[name].update(raw[name])
                
                # Update filtered value
                self.filtered_values[name] = filtered
                
                # Update history for derivative calculation
                self.history[name].append(filtered)
                self.timestamp_history[name].append(timestamp)
                
                # Calculate derivative if we have enough history
                if len(self.history[name]) >= 2:
                    times = list(self.timestamp_history[name])
                    values = list(self.history[name])
                    dt = times[-1] - times[0]
                    if dt > 0:
                        dv = values[-1] - values[0]
                        self.filtered_derivatives[name] = dv / dt
                
                # Update status - any valid reading is OK (like fallback_test.py)
                self.status[name] = "OK"
        
        # Step 3: Apply fallback substitution for failed readings (like fallback_test.py)
        final = {}
        substituted = []
        bad = []
        
        for name in self.sensor_names:
            if raw[name] is not None:
                # Direct reading available
                final[name] = self.filtered_values[name]
            else:
                # Try neighbors in fallback map (exactly like fallback_test.py)
                for nb in self.fallback_map.get(name, []):
                    if raw.get(nb) is not None:
                        # Use neighbor's value
                        final[name] = self.filtered_values[nb]
                        substituted.append(name)
                        self.status[name] = "SUB"
                        break
                else:
                    # No substitution available
                    # Even for BAD readings, we'll set the distance to maximum (100mm)
                    # This ensures values don't get stuck at the last detected position
                    final[name] = 100  # Maximum distance when no reading is available
                    self.filtered_values[name] = 100  # Update filtered values too
                    # Update raw values too to ensure consistent state
                    self.raw_values[name] = 100
                    bad.append(name)
                    self.status[name] = "BAD"
        
        # Log substitution summary (like fallback_test.py)
        if substituted or bad:
            logger.debug(f"Summary: OK={ok} SUB={substituted} BAD={bad}")
    
    def _sensor_reading_thread(self):
        """
        Background thread for continuous sensor reading.
        
        This method runs in a separate thread and continuously reads 
        all sensors at the specified sampling rate.
        """
        next_sample_time = time.time()
        error_count = 0  # Track consecutive errors for dynamic rate adjustment
        current_interval = self.sampling_interval  # Start with normal interval
        
        # Constants for dynamic rate adjustment
        MIN_INTERVAL = self.sampling_interval  # Fastest rate (normal)
        MAX_INTERVAL = self.sampling_interval * 2  # Slowest rate (during errors)
        
        while self.running and not self.shutdown_event.is_set():
            current_time = time.time()
            
            # Check if it's time for the next sample
            if current_time >= next_sample_time:
                try:
                    # Read all sensors if not shutting down
                    if not self.shutdown_event.is_set():
                        self._read_all_sensors()
                    
                    # Successful read - gradually move back to normal rate if we were slowed down
                    if current_interval > MIN_INTERVAL:
                        current_interval = max(MIN_INTERVAL, current_interval * 0.9)
                        logger.debug(f"Adjusting sampling interval: {current_interval:.4f}s")
                    
                    # Reset error count on success
                    error_count = 0
                    
                    # Calculate next sample time
                    next_sample_time = current_time + current_interval
                    
                except RuntimeError as e:
                    # Handle shutdown-related errors gracefully
                    if "shutdown" in str(e):
                        logger.info("Sensor thread detected shutdown, exiting...")
                        break
                    else:
                        # Handle other errors by backing off
                        error_count += 1
                        logger.error(f"Error in sensor thread: {e}")
                        
                        # Adjust sampling rate based on errors (slow down during high error periods)
                        if error_count > 3:
                            # Exponential backoff up to MAX_INTERVAL
                            backoff_factor = min(2.0, 1.0 + (error_count - 3) * 0.2)
                            current_interval = min(MAX_INTERVAL, current_interval * backoff_factor)
                            logger.info(f"Reducing sampling rate due to errors: {current_interval:.4f}s")
                        
                        # Add some delay after errors to avoid rapid retries
                        time.sleep(0.05)
                        
                        # Set next sample time accounting for the delay we just added
                        next_sample_time = time.time() + current_interval
                        
                except Exception as e:
                    # Handle other exceptions similarly
                    error_count += 1
                    logger.error(f"Error in sensor thread: {e}")
                    
                    # Adjust sampling rate based on errors
                    if error_count > 3:
                        current_interval = min(MAX_INTERVAL, current_interval * 1.2)
                        logger.info(f"Reducing sampling rate due to errors: {current_interval:.4f}s")
                    
                    # Add some delay after errors to avoid rapid retries
                    time.sleep(0.05)
                    
                    # Set next sample time accounting for the delay we just added
                    next_sample_time = time.time() + current_interval
            
            # Check shutdown between iterations with small delay
            if self.shutdown_event.wait(0.001):  # Will wait 1ms and return True if event is set
                logger.info("Sensor thread received shutdown event")
                break
    
    def start(self):
        """Start the sensor reading thread"""
        if not self.running:
            # Clear shutdown event if it was previously set
            self.shutdown_event.clear()
            
            # Set running flag
            self.running = True
            
            # Start reading thread
            self.thread = threading.Thread(target=self._sensor_reading_thread)
            self.thread.daemon = True  # Allow program to exit even if thread is running
            self.thread.start()
            logger.info("Proximity manager started")
    
    def stop(self):
        """Stop the sensor reading thread and clean up resources"""
        logger.info("Stopping proximity manager...")
        
        # First, signal shutdown to prevent new hardware access
        self.shutdown_event.set()
        
        # Clear running flag to stop the thread loop
        self.running = False
        
        # Wait for thread to finish with timeout
        if self.thread and self.thread.is_alive():
            logger.info("Waiting for sensor thread to exit...")
            try:
                self.thread.join(timeout=2.0)  # Longer timeout for safer shutdown
                
                if self.thread.is_alive():
                    logger.warning("Sensor thread did not exit gracefully within timeout")
                else:
                    logger.info("Sensor thread exited gracefully")
            except Exception as e:
                logger.error(f"Error while joining thread: {e}")
        
        # Clean up I2C resources with lock to prevent conflicts
        try:
            # Use a new bus handle for cleanup to avoid "already closed" issues
            bus_is_open = True
            try:
                # Try to access the bus to see if it's still open - using hasattr to check if the file descriptor exists
                with self.bus_lock:
                    # Check if the bus has a valid file descriptor
                    if hasattr(self.bus, '_fd') and self.bus._fd is not None:
                        bus_is_open = True
                    else:
                        bus_is_open = False
                        logger.debug("I2C bus file descriptor is None, bus appears to be closed")
            except Exception as e:
                bus_is_open = False
                logger.debug(f"I2C bus appears to be already closed: {e}")
            
            # Only perform cleanup if the bus is still open
            if bus_is_open:
                with self.bus_lock:
                    try:
                        # Get unique mux addresses
                        mux_addresses = set(mux for mux, _, _ in self.sensors)
                        
                        # Disable all channels on each mux
                        for mux in mux_addresses:
                            try:
                                logger.debug(f"Disabling multiplexer at {hex(mux)}")
                                self.bus.write_byte(mux, 0x00)
                            except Exception as e:
                                logger.warning(f"Error disabling multiplexer {hex(mux)}: {e}")
                        
                        # Close the bus
                        logger.debug("Closing I2C bus")
                        self.bus.close()
                    except Exception as e:
                        logger.warning(f"Error during I2C operations: {e}")
        except Exception as e:
            logger.warning(f"Error during I2C cleanup: {e}")
        
        logger.info("Proximity manager stopped")
    
    def test_sensors(self, duration=10):
        """
        Run a test to print all sensor values for a specified duration.
        
        Args:
            duration: Test duration in seconds
        """
        # Start the sensor manager if not already running
        was_running = self.running
        if not was_running:
            self.start()
        
        try:
            end_time = time.time() + duration
            while time.time() < end_time:
                print("\n---- Sensor Readings ----")
                for sensor_name in self.sensor_names:
                    raw = self.get_sensor_value(sensor_name, filtered=False)
                    filtered = self.get_sensor_value(sensor_name, filtered=True)
                    status = self.status.get(sensor_name, "UNKNOWN")
                    print(f"{sensor_name}: Raw={raw}mm, Filtered={filtered:.1f}mm, Status={status}")
                time.sleep(1.0)
        
        except KeyboardInterrupt:
            print("\nTest interrupted by user")
        
        finally:
            # Only stop if we started it in this method
            if not was_running:
                self.stop()

# Simple test code
if __name__ == "__main__":
    print("Starting proximity sensor test...")
    manager = ProximityManager()
    print("Running sensor test for 10 seconds...")
    manager.test_sensors(10)
    print("Test complete.")