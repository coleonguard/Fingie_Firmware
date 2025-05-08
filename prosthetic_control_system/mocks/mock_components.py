#!/usr/bin/env python3
"""
Mock components for testing the Prosthetic Control System.

This module provides mock implementations of the system's components
for use in automated testing.
"""

import time
import threading
import logging
import math
import numpy as np
from typing import Dict, List, Optional, Union
from enum import Enum
import queue

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("MockComponents")

class FakeClock:
    """
    Fake clock for deterministic time in tests.
    
    This class allows control of time in tests to ensure deterministic behavior.
    """
    
    def __init__(self, start_time=0.0, tick_rate=1.0):
        """
        Initialize the fake clock.
        
        Args:
            start_time: Initial time value
            tick_rate: How fast the clock advances relative to real time
        """
        self.current_time = start_time
        self.tick_rate = tick_rate
        self.real_start_time = time.time()
    
    def time(self):
        """Get the current fake time"""
        if self.tick_rate > 0:
            real_elapsed = time.time() - self.real_start_time
            return self.current_time + (real_elapsed * self.tick_rate)
        else:
            return self.current_time
    
    def sleep(self, seconds):
        """
        Sleep for the specified number of fake seconds.
        
        This actually sleeps for less real time based on the tick rate.
        """
        if self.tick_rate > 0:
            real_sleep = seconds / self.tick_rate
            time.sleep(real_sleep)
        self.current_time += seconds
    
    def advance(self, seconds):
        """Advance the clock by the specified number of seconds"""
        self.current_time += seconds

class MockVL6180X:
    """Mock VL6180X proximity sensor"""
    
    def __init__(self, initial_distance=30, drift=0.0, noise=0.0, fail_rate=0.0):
        """
        Initialize the mock sensor.
        
        Args:
            initial_distance: Initial distance value in mm
            drift: Rate of drift in mm/s
            noise: Standard deviation of noise in mm
            fail_rate: Probability of read failure (0-1)
        """
        self.distance = initial_distance
        self.drift = drift
        self.noise = noise
        self.fail_rate = fail_rate
        self.last_update = time.time()
    
    def read(self):
        """Read the current distance value"""
        # Apply drift based on elapsed time
        now = time.time()
        elapsed = now - self.last_update
        self.distance += self.drift * elapsed
        self.last_update = now
        
        # Simulate failures
        if np.random.random() < self.fail_rate:
            # Return 0 or 255 to simulate common failure modes
            return np.random.choice([0, 255])
        
        # Apply noise
        if self.noise > 0:
            noisy_value = self.distance + np.random.normal(0, self.noise)
            # Ensure value is in valid range
            noisy_value = max(0, min(255, noisy_value))
            return int(round(noisy_value))
        else:
            return int(round(self.distance))
    
    def set_distance(self, distance):
        """Set the current distance value"""
        self.distance = distance
        self.last_update = time.time()

class MockMux:
    """Mock I2C multiplexer"""
    
    def __init__(self, sensor_map=None):
        """
        Initialize the mock multiplexer.
        
        Args:
            sensor_map: Dictionary mapping channel numbers to MockVL6180X objects
        """
        self.sensor_map = sensor_map or {}
        self.selected_channel = None
    
    def select_channel(self, channel):
        """Select a channel on the multiplexer"""
        if channel in self.sensor_map:
            self.selected_channel = channel
            return True
        else:
            self.selected_channel = None
            return False
    
    def read_sensor(self):
        """Read from the currently selected sensor"""
        if self.selected_channel is None:
            raise RuntimeError("No channel selected")
        
        if self.selected_channel not in self.sensor_map:
            raise RuntimeError(f"No sensor on channel {self.selected_channel}")
        
        return self.sensor_map[self.selected_channel].read()

class MockIMU:
    """Mock IMU for testing"""
    
    def __init__(self, data_file=None):
        """
        Initialize the mock IMU.
        
        Args:
            data_file: Optional CSV file with IMU data for replay
        """
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = -9.81  # Default to gravity
        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0
        
        # Data for replay
        self.data_points = []
        self.current_index = 0
        
        # Load data file if provided
        if data_file:
            self._load_data(data_file)
    
    def _load_data(self, data_file):
        """Load IMU data from CSV file"""
        try:
            import pandas as pd
            data = pd.read_csv(data_file)
            
            # Convert to list of dictionaries
            self.data_points = data.to_dict('records')
            logger.info(f"Loaded {len(self.data_points)} IMU data points from {data_file}")
        except Exception as e:
            logger.error(f"Failed to load IMU data: {e}")
    
    def next_data_point(self):
        """Get the next data point in the sequence"""
        if not self.data_points:
            return None
            
        if self.current_index >= len(self.data_points):
            self.current_index = 0
            
        point = self.data_points[self.current_index]
        self.current_index += 1
        
        return point
    
    def update(self):
        """Update IMU data - either from loaded data or simulated"""
        # If we have data points, use them
        point = self.next_data_point()
        if point:
            # Update values from data point
            for key, value in point.items():
                if key == 'roll':
                    self.roll = value
                elif key == 'pitch':
                    self.pitch = value
                elif key == 'yaw':
                    self.yaw = value
                elif key == 'accel_x':
                    self.accel_x = value
                elif key == 'accel_y':
                    self.accel_y = value
                elif key == 'accel_z':
                    self.accel_z = value
                elif key == 'gyro_x':
                    self.gyro_x = value
                elif key == 'gyro_y':
                    self.gyro_y = value
                elif key == 'gyro_z':
                    self.gyro_z = value
        else:
            # Simulate some gentle movement
            t = time.time()
            self.roll = 5 * math.sin(t * 0.3)
            self.pitch = 8 * math.sin(t * 0.5)
            self.yaw = 3 * math.sin(t * 0.7)
            
            # Simulate some noise in acceleration
            self.accel_x = 0.2 * math.sin(t * 2.0)
            self.accel_y = 0.3 * math.sin(t * 3.0)
            self.accel_z = -9.81 + 0.1 * math.sin(t * 4.0)
            
            # Simulate some noise in gyro
            self.gyro_x = 1.0 * math.sin(t * 5.0)
            self.gyro_y = 1.5 * math.sin(t * 6.0)
            self.gyro_z = 0.8 * math.sin(t * 7.0)
    
    def simulate_impact(self, magnitude=5.0, duration=0.2):
        """Simulate an impact event"""
        # Store original z acceleration
        original_z = self.accel_z
        
        # Add impact spike
        self.accel_z = original_z - magnitude
        
        # Wait for duration
        time.sleep(duration)
        
        # Restore original value
        self.accel_z = original_z
    
    def simulate_lowering(self, duration=1.0):
        """Simulate lowering motion"""
        # Store original z acceleration
        original_z = self.accel_z
        
        # Set to lowering acceleration
        self.accel_z = -5.0  # Strong negative z acceleration
        
        # Wait for duration
        time.sleep(duration)
        
        # Restore original value
        self.accel_z = original_z

class MockHand:
    """Mock hand for testing"""
    
    def __init__(self, n_fingers=5, position_lag=0.2, current_lag=0.05):
        """
        Initialize the mock hand.
        
        Args:
            n_fingers: Number of fingers
            position_lag: Time constant for position changes (s)
            current_lag: Time constant for current changes (s)
        """
        self.n_fingers = n_fingers
        self.position_lag = position_lag
        self.current_lag = current_lag
        
        # Finger state
        self.positions = [0.0] * n_fingers
        self.velocities = [0.0] * n_fingers
        self.currents = [0.0] * n_fingers
        
        # Target values
        self.target_positions = [0.0] * n_fingers
        self.target_currents = [0.0] * n_fingers
        
        # Control mode (0=position, 1=current)
        self.control_modes = [0] * n_fingers
        
        # Timing
        self.last_update = time.time()
    
    def update(self):
        """Update finger state based on targets and lag"""
        now = time.time()
        dt = now - self.last_update
        self.last_update = now
        
        for i in range(self.n_fingers):
            # Get control mode
            mode = self.control_modes[i]
            
            if mode == 0:  # Position control
                # Apply lag to position
                target = self.target_positions[i]
                current = self.positions[i]
                error = target - current
                
                # First-order lag
                rate = 1.0 / self.position_lag
                delta = error * rate * dt
                self.positions[i] += delta
                
                # Calculate velocity
                self.velocities[i] = delta / dt if dt > 0 else 0.0
                
                # Calculate current (simplified)
                self.currents[i] = 0.01 * abs(error) + 0.002 * abs(self.velocities[i])
                
            elif mode == 1:  # Current control
                # Apply lag to current
                target = self.target_currents[i]
                current = self.currents[i]
                error = target - current
                
                # First-order lag
                rate = 1.0 / self.current_lag
                self.currents[i] += error * rate * dt
                
                # Simulate position/velocity response to current
                # In this simple model, current drives velocity which updates position
                # This is a massive simplification of real motor dynamics
                
                # Current -> force -> acceleration -> velocity -> position
                self.velocities[i] = 20.0 * self.currents[i]  # Simple mapping
                
                # Update position based on velocity
                self.positions[i] += self.velocities[i] * dt
                
                # Clamp position to valid range
                self.positions[i] = max(0.0, min(100.0, self.positions[i]))
    
    def set_position(self, finger_index, position):
        """Set target position for a finger"""
        if 0 <= finger_index < self.n_fingers:
            self.target_positions[finger_index] = position
            self.control_modes[finger_index] = 0  # Position control
    
    def set_current(self, finger_index, current):
        """Set target current for a finger"""
        if 0 <= finger_index < self.n_fingers:
            self.target_currents[finger_index] = current
            self.control_modes[finger_index] = 1  # Current control
    
    def get_position(self, finger_index):
        """Get current position for a finger"""
        if 0 <= finger_index < self.n_fingers:
            return self.positions[finger_index]
        return 0.0
    
    def get_velocity(self, finger_index):
        """Get current velocity for a finger"""
        if 0 <= finger_index < self.n_fingers:
            return self.velocities[finger_index]
        return 0.0
    
    def get_current(self, finger_index):
        """Get current for a finger"""
        if 0 <= finger_index < self.n_fingers:
            return self.currents[finger_index]
        return 0.0
    
    def simulate_blocked_finger(self, finger_index, position=50.0):
        """Simulate a finger being blocked at a specific position"""
        if 0 <= finger_index < self.n_fingers:
            # Force position to the blocked position
            self.positions[finger_index] = position
            
            # Velocity goes to zero
            self.velocities[finger_index] = 0.0
            
            # Current increases due to stall
            self.currents[finger_index] = min(0.8, self.currents[finger_index] * 1.5)
    
    def simulate_slip(self, finger_index, amount=0.2):
        """Simulate a slip event with a current drop"""
        if 0 <= finger_index < self.n_fingers:
            # Current drops during slip
            self.currents[finger_index] = max(0.0, self.currents[finger_index] - amount)
            
            # Position changes slightly
            slip_direction = -1.0 if self.positions[finger_index] > 50 else 1.0
            self.positions[finger_index] += slip_direction * 5.0

# Example usage in a test
if __name__ == "__main__":
    print("Testing mock components...")
    
    # Create mock multiplexer with sensors
    mux = MockMux({
        0: MockVL6180X(initial_distance=30, noise=1.0),
        1: MockVL6180X(initial_distance=20, noise=0.5),
        2: MockVL6180X(initial_distance=10, noise=1.5),
    })
    
    # Create mock IMU
    imu = MockIMU()
    
    # Create mock hand
    hand = MockHand()
    
    # Test reading from sensors
    print("\nTesting proximity sensors:")
    for channel in range(3):
        mux.select_channel(channel)
        distance = mux.read_sensor()
        print(f"Channel {channel}: {distance} mm")
    
    # Test IMU data
    print("\nTesting IMU:")
    imu.update()
    print(f"Orientation: roll={imu.roll:.1f}°, pitch={imu.pitch:.1f}°, yaw={imu.yaw:.1f}°")
    print(f"Acceleration: x={imu.accel_x:.2f}, y={imu.accel_y:.2f}, z={imu.accel_z:.2f} m/s²")
    
    # Test hand control
    print("\nTesting hand control:")
    # Set finger 0 to position control
    hand.set_position(0, 50.0)
    # Set finger 1 to current control
    hand.set_current(1, 0.3)
    
    # Update a few times
    for i in range(5):
        hand.update()
        print(f"Finger 0: pos={hand.get_position(0):.1f}°, cur={hand.get_current(0):.2f}A")
        print(f"Finger 1: pos={hand.get_position(1):.1f}°, cur={hand.get_current(1):.2f}A")
        time.sleep(0.1)
    
    # Test blocking and slip
    hand.simulate_blocked_finger(0)
    hand.update()
    print("\nAfter blocking finger 0:")
    print(f"Finger 0: pos={hand.get_position(0):.1f}°, cur={hand.get_current(0):.2f}A")
    
    hand.simulate_slip(1)
    hand.update()
    print("\nAfter slip on finger 1:")
    print(f"Finger 1: pos={hand.get_position(1):.1f}°, cur={hand.get_current(1):.2f}A")
    
    print("\nTest complete.")