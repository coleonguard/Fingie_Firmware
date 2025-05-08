#!/usr/bin/env python3
"""
Unit tests for the velocity limiter implementation (U-02).

Tests that the motor position changes are rate-limited to 8° per cycle.
"""

import sys
import os
import unittest
import time

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import motor interface to test
from hand.motor_interface import SimulatedMotorInterface, ControlMode

class TestVelocityLimiter(unittest.TestCase):
    """Test cases for the velocity limiter"""
    
    def test_position_rate_limiting(self):
        """Test that position changes are rate-limited (U-02)"""
        # Create motor interface with known rate limit
        motors = SimulatedMotorInterface(control_rate=50)
        motors.max_position_rate = 8.0  # 8 degrees per update
        
        # Start the interface in a controlled way without the thread
        motors.running = True
        
        try:
            # Initial state
            motors.position_targets["Thumb"] = 0.0
            motors.prev_position_targets["Thumb"] = 0.0
            motors.control_modes["Thumb"] = ControlMode.POSITION
            
            # Set finger to a large position change (0→100°) in one command
            motors.set_position("Thumb", 100.0)
            
            # First time through the control loop
            motors._apply_rate_limits()
            
            # Check that the change was limited to max_position_rate
            self.assertEqual(motors.position_targets["Thumb"], 8.0,
                          f"Position change not limited to {motors.max_position_rate}°")
            
            # In normal operation, the control loop applies rate limits, then 
            # the user can call set_position again to request a new target.
            # For testing, we'll keep the same ultimate target (100.0) but
            # update the previous position after each cycle.
            
            # Remember the current limited position (8.0)
            current_pos = motors.position_targets["Thumb"]
            
            # Simulate 4 more control cycles
            for i in range(4):
                # Store current position as previous
                motors.prev_position_targets["Thumb"] = current_pos
                
                # The position target gets reset to 100.0 each time
                motors.position_targets["Thumb"] = 100.0
                
                # Apply limits again
                motors._apply_rate_limits()
                
                # Update our tracking variable with the new limited position
                current_pos = motors.position_targets["Thumb"]
                
                # Check that we've incremented by max_position_rate
                expected = min(8.0 * (i + 2), 100.0)  # i+2 because we start at cycle 1
                self.assertEqual(current_pos, expected,
                             f"Position not incremented correctly on cycle {i+2}")
                
            # After 5 cycles (1 initial + 4 more), we should be at 40 degrees
            self.assertEqual(current_pos, 40.0,
                         "Position not incremented correctly after 5 cycles")
            
        finally:
            motors.running = False
    
    def test_multiple_fingers(self):
        """Test rate limiting on multiple fingers"""
        motors = SimulatedMotorInterface(control_rate=50)
        motors.max_position_rate = 8.0
        
        # Start in a controlled way
        motors.running = True
        
        try:
            # Initialize previous positions
            for finger in motors.fingers:
                motors.position_targets[finger] = 0.0
                motors.prev_position_targets[finger] = 0.0
                motors.control_modes[finger] = ControlMode.POSITION
            
            # Set multiple fingers to different large position changes
            motors.set_position("Thumb", 50.0)
            motors.set_position("Index", 70.0)
            motors.set_position("Middle", 30.0)
            
            # Run one control cycle
            motors._apply_rate_limits()
            
            # Check all fingers were limited to max_position_rate
            self.assertEqual(motors.position_targets["Thumb"], 8.0)
            self.assertEqual(motors.position_targets["Index"], 8.0)
            self.assertEqual(motors.position_targets["Middle"], 8.0)
            
        finally:
            motors.running = False
    
    def test_negative_position_changes(self):
        """Test rate limiting for negative position changes"""
        motors = SimulatedMotorInterface(control_rate=50)
        motors.max_position_rate = 8.0
        
        # Start in a controlled way
        motors.running = True
        
        try:
            # First set up a high position and make it the "previous" position
            motors.position_targets["Thumb"] = 50.0
            motors.prev_position_targets["Thumb"] = 50.0
            motors.control_modes["Thumb"] = ControlMode.POSITION
            
            # Then set a large negative change
            motors.set_position("Thumb", 0.0)
            
            # Run one control cycle
            motors._apply_rate_limits()
            
            # Check that the negative change was limited to -max_position_rate
            self.assertEqual(motors.position_targets["Thumb"], 42.0,
                          "Negative position change not limited correctly")
            
        finally:
            motors.running = False

if __name__ == "__main__":
    unittest.main()