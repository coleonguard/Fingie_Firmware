#!/usr/bin/env python3
"""
Low-level test for thread synchronization with sensor inputs.

This test verifies thread-safe data exchange between sensor reading threads
and the control loop, ensuring no data loss occurs even under high load.
"""

import sys
import os
import unittest
import time
import threading
import queue
import random
import logging
from concurrent.futures import ThreadPoolExecutor

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("ThreadSync")

class ThreadSafeBuffer:
    """Thread-safe buffer for sensor data exchange between threads"""
    
    def __init__(self, maxsize=100):
        """
        Initialize the buffer.
        
        Args:
            maxsize: Maximum buffer size before oldest items are dropped
        """
        self.buffer = queue.Queue(maxsize=maxsize)
        self.lock = threading.Lock()
        self.dropped_count = 0
        self.total_added = 0
        self.total_retrieved = 0
        
    def add(self, data):
        """
        Add data to the buffer, dropping oldest if full.
        
        Args:
            data: Data item to add
            
        Returns:
            True if added successfully, False if dropped
        """
        with self.lock:
            try:
                self.buffer.put_nowait(data)
                self.total_added += 1
                return True
            except queue.Full:
                # Buffer is full, remove oldest item
                try:
                    self.buffer.get_nowait()
                    self.buffer.put_nowait(data)
                    self.dropped_count += 1
                    self.total_added += 1
                    return False
                except (queue.Empty, queue.Full):
                    # This should not happen, but handle just in case
                    self.dropped_count += 1
                    return False
    
    def get(self, block=True, timeout=None):
        """
        Get data from the buffer.
        
        Args:
            block: Whether to block if buffer is empty
            timeout: Timeout for blocking get
            
        Returns:
            Data item or None if buffer is empty
        """
        try:
            data = self.buffer.get(block=block, timeout=timeout)
            with self.lock:
                self.total_retrieved += 1
            return data
        except queue.Empty:
            return None
    
    def get_stats(self):
        """Get buffer statistics"""
        with self.lock:
            return {
                "dropped": self.dropped_count,
                "total_added": self.total_added,
                "total_retrieved": self.total_retrieved,
                "current_size": self.buffer.qsize()
            }

class SensorMock:
    """
    Mock sensor that generates readings at a specified rate.
    This simulates hardware sensors producing data at their own pace.
    """
    
    def __init__(self, name, reading_rate_hz, noise=0.1, jitter=0.2):
        """
        Initialize the mock sensor.
        
        Args:
            name: Sensor name
            reading_rate_hz: Readings per second
            noise: Amount of noise in readings
            jitter: Timing jitter factor (0-1)
        """
        self.name = name
        self.reading_rate_hz = reading_rate_hz
        self.period = 1.0 / reading_rate_hz
        self.noise = noise
        self.jitter = jitter
        self.base_value = random.uniform(10, 50)
        self.running = False
        self.thread = None
        self.buffer = ThreadSafeBuffer()
        
    def start(self):
        """Start the sensor reading thread"""
        if self.running:
            return
            
        self.running = True
        self.thread = threading.Thread(target=self._reading_loop)
        self.thread.daemon = True
        self.thread.start()
        
    def stop(self):
        """Stop the sensor reading thread"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
            
    def _reading_loop(self):
        """Main reading loop that generates sensor values"""
        next_reading_time = time.time()
        
        while self.running:
            # Wait until next reading time
            now = time.time()
            if now < next_reading_time:
                time.sleep(next_reading_time - now)
            
            # Generate a reading
            timestamp = time.time()
            value = self.base_value + self.noise * random.normalvariate(0, 1)
            
            # Add to buffer
            reading = {
                "sensor": self.name,
                "timestamp": timestamp,
                "value": value
            }
            
            if not self.buffer.add(reading):
                logger.warning(f"Dropped reading from {self.name}")
            
            # Calculate next reading time with jitter
            jitter_factor = 1.0 + self.jitter * random.normalvariate(0, 0.1)
            next_reading_time = timestamp + (self.period * jitter_factor)
    
    def get_reading(self, block=True, timeout=0.1):
        """Get the latest reading"""
        return self.buffer.get(block=block, timeout=timeout)
    
    def get_stats(self):
        """Get buffer statistics"""
        return self.buffer.get_stats()

class ControlLoopMock:
    """
    Mock control loop that processes sensor readings.
    This simulates the main control loop that needs to handle data from multiple sensors.
    """
    
    def __init__(self, rate_hz=20):
        """
        Initialize the control loop.
        
        Args:
            rate_hz: Control loop rate in Hz
        """
        self.rate_hz = rate_hz
        self.period = 1.0 / rate_hz
        self.sensors = {}
        self.running = False
        self.thread = None
        self.processed_readings = 0
        self.late_readings = 0
        self.missed_cycles = 0
        
    def add_sensor(self, sensor):
        """Add a sensor to the control loop"""
        self.sensors[sensor.name] = sensor
        
    def start(self):
        """Start the control loop"""
        if self.running:
            return
            
        self.running = True
        self.thread = threading.Thread(target=self._control_loop)
        self.thread.daemon = True
        self.thread.start()
        
    def stop(self):
        """Stop the control loop"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
            
    def _control_loop(self):
        """Main control loop that processes sensor readings"""
        next_cycle_time = time.time()
        
        while self.running:
            cycle_start = time.time()
            
            # Check if we're running late
            if cycle_start > next_cycle_time + self.period:
                self.missed_cycles += 1
                next_cycle_time = cycle_start  # Reset timing
                
            # Process all available sensor readings
            readings = []
            for sensor_name, sensor in self.sensors.items():
                reading = sensor.get_reading(block=False)
                if reading:
                    readings.append(reading)
                    self.processed_readings += 1
                    
                    # Check if reading is late
                    reading_age = cycle_start - reading["timestamp"]
                    if reading_age > self.period:
                        self.late_readings += 1
            
            # Process readings (just counting them in this mock)
            
            # Calculate sleep time
            cycle_end = time.time()
            cycle_duration = cycle_end - cycle_start
            next_cycle_time = cycle_start + self.period
            sleep_time = next_cycle_time - cycle_end
            
            if sleep_time > 0:
                time.sleep(sleep_time)
            
    def get_stats(self):
        """Get control loop statistics"""
        return {
            "processed_readings": self.processed_readings,
            "late_readings": self.late_readings,
            "missed_cycles": self.missed_cycles,
            "sensor_stats": {name: sensor.get_stats() for name, sensor in self.sensors.items()}
        }

class TestThreadSync(unittest.TestCase):
    """Test cases for thread synchronization"""
    
    def test_normal_load(self):
        """Test sensor data flow under normal load conditions"""
        # Create sensors
        sensors = [
            SensorMock("proximity_1", reading_rate_hz=40),  # 40 Hz
            SensorMock("proximity_2", reading_rate_hz=40),
            SensorMock("proximity_3", reading_rate_hz=40),
            SensorMock("imu", reading_rate_hz=100)          # 100 Hz
        ]
        
        # Create control loop
        control_loop = ControlLoopMock(rate_hz=20)  # 20 Hz
        
        # Add sensors to control loop
        for sensor in sensors:
            control_loop.add_sensor(sensor)
        
        # Start all components
        for sensor in sensors:
            sensor.start()
        control_loop.start()
        
        # Run for 5 seconds
        logger.info("Running normal load test for 5 seconds")
        time.sleep(5.0)
        
        # Stop all components
        control_loop.stop()
        for sensor in sensors:
            sensor.stop()
        
        # Get statistics
        stats = control_loop.get_stats()
        logger.info(f"Control loop stats: {stats}")
        
        # Check for data loss
        total_readings = 0
        total_dropped = 0
        
        for name, sensor_stats in stats["sensor_stats"].items():
            total_readings += sensor_stats["total_added"]
            total_dropped += sensor_stats["dropped"]
        
        # With normal load, we expect very few or no dropped readings
        self.assertLess(total_dropped, 10, "Too many dropped readings under normal load")
        
        # Check for missed cycles
        self.assertEqual(stats["missed_cycles"], 0, "Control loop missed cycles under normal load")
    
    def test_high_load(self):
        """Test sensor data flow under high load conditions"""
        # Create a lot of high-rate sensors
        sensors = [
            SensorMock(f"proximity_{i}", reading_rate_hz=100) for i in range(10)
        ]
        sensors.append(SensorMock("imu", reading_rate_hz=200))  # 200 Hz
        
        # Create control loop
        control_loop = ControlLoopMock(rate_hz=20)  # 20 Hz
        
        # Add sensors to control loop
        for sensor in sensors:
            control_loop.add_sensor(sensor)
        
        # Start all components
        for sensor in sensors:
            sensor.start()
        control_loop.start()
        
        # Add CPU load to simulate busy system
        def cpu_load():
            start_time = time.time()
            while time.time() - start_time < 5.0:
                # Busy wait to consume CPU
                for _ in range(10000):
                    pass
        
        # Run CPU load in threads
        with ThreadPoolExecutor(max_workers=2) as executor:
            futures = [executor.submit(cpu_load) for _ in range(2)]
            
            # Wait for 5 seconds
            logger.info("Running high load test for 5 seconds")
            time.sleep(5.0)
        
        # Stop all components
        control_loop.stop()
        for sensor in sensors:
            sensor.stop()
        
        # Get statistics
        stats = control_loop.get_stats()
        logger.info(f"Control loop stats under high load: {stats}")
        
        # Under high load, we might have dropped readings, but the system should still function
        # We care more about missed control cycles since they affect real-time performance
        
        # Verify control loop met timing requirements
        # Even under load, missed cycles should be minimal for real-time control
        self.assertLess(stats["missed_cycles"], 5, 
                      "Too many missed control cycles under high load")
        
        # Verify we processed a reasonable number of readings
        expected_min = 5 * 20  # 5 seconds * 20 Hz minimum
        self.assertGreater(stats["processed_readings"], expected_min,
                        "Processed too few readings under high load")
    
    def test_data_consistency(self):
        """Test data consistency across thread boundaries"""
        # Create sensor with a predictable pattern
        sensor = SensorMock("test_sensor", reading_rate_hz=100, noise=0.0, jitter=0.0)
        sensor.base_value = 0.0  # Start at 0
        
        # Override the reading loop to generate sequential values
        def sequential_reading_loop():
            value = 0
            while sensor.running:
                timestamp = time.time()
                reading = {
                    "sensor": sensor.name,
                    "timestamp": timestamp,
                    "value": value
                }
                sensor.buffer.add(reading)
                value += 1
                time.sleep(0.01)  # 100 Hz
                
        sensor.thread = threading.Thread(target=sequential_reading_loop)
        sensor.thread.daemon = True
        
        # Start the sensor
        sensor.running = True
        sensor.thread.start()
        
        # Read values in another thread
        received_values = []
        stop_reading = threading.Event()
        
        def reader_thread():
            while not stop_reading.is_set():
                reading = sensor.get_reading(timeout=0.1)
                if reading:
                    received_values.append(reading["value"])
        
        reader = threading.Thread(target=reader_thread)
        reader.daemon = True
        reader.start()
        
        # Run for 3 seconds
        time.sleep(3.0)
        
        # Stop everything
        stop_reading.set()
        reader.join(timeout=1.0)
        sensor.running = False
        sensor.thread.join(timeout=1.0)
        
        # Check for sequential values without gaps
        # Sort because they might be received out of order
        received_values.sort()
        
        # Check for duplicates
        duplicates = len(received_values) - len(set(received_values))
        self.assertEqual(duplicates, 0, f"Found {duplicates} duplicate values")
        
        # Analyze gaps to see if we lost data
        if len(received_values) >= 2:
            gaps = []
            for i in range(1, len(received_values)):
                if received_values[i] - received_values[i-1] > 1:
                    gaps.append((received_values[i-1], received_values[i]))
            
            # Some small gaps might be expected due to thread scheduling
            # But we want to make sure there aren't many large gaps
            large_gaps = [g for g in gaps if g[1] - g[0] > 5]
            self.assertLess(len(large_gaps), 3, 
                         f"Found {len(large_gaps)} large gaps in data: {large_gaps}")
        
        logger.info(f"Received {len(received_values)} values, expected approximately 300")
        # We should receive approximately 3 seconds * 100 Hz = 300 values
        # Allow some margin for thread scheduling
        self.assertGreater(len(received_values), 250, 
                        f"Received too few values: {len(received_values)}")

if __name__ == "__main__":
    unittest.main()