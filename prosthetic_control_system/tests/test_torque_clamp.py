#!/usr/bin/env python3
"""
Unit tests for the torque limiter implementation (U-03).

Tests that torque commands are clamped to a maximum value of 0.6A.
"""

import sys
import os
import unittest

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import motor interface to test
from hand.motor_interface import SimulatedMotorInterface, ControlMode

class TestTorqueClamp(unittest.TestCase):
    """Test cases for the torque clamping functionality"""
    
    def test_torque_clamp(self):
        """Test that torque is clamped to max_torque (U-03)"""
        # Create motor interface with known limits
        motors = SimulatedMotorInterface(control_rate=50)
        motors.max_torque = 0.6  # Maximum torque of 0.6A
        
        # Set torque above the limit
        motors.set_torque("Thumb", 1.0)  # Try to set 1.0A
        
        # Check that it was clamped to max_torque
        self.assertEqual(motors.torque_targets["Thumb"], 0.6,
                       f"Torque not clamped to max_torque {motors.max_torque}A")
    
    def test_negative_torque_clamp(self):
        """Test that negative torque is also clamped"""
        motors = SimulatedMotorInterface(control_rate=50)
        motors.max_torque = 0.6
        
        # Set negative torque beyond limit
        motors.set_torque("Thumb", -1.0)  # Try to set -1.0A
        
        # Check that it was clamped to -max_torque
        self.assertEqual(motors.torque_targets["Thumb"], -0.6,
                       f"Negative torque not clamped to -max_torque {-motors.max_torque}A")
    
    def test_torque_rate_limiting(self):
        """Test that torque changes are rate-limited"""
        motors = SimulatedMotorInterface(control_rate=50)
        motors.max_torque = 0.6
        motors.max_torque_rate = 0.05  # 0.05A per update
        
        # Start in a controlled way
        motors.running = True
        
        try:
            # Initialize previous torque
            motors.torque_targets["Thumb"] = 0.0
            motors.prev_torque_targets["Thumb"] = 0.0
            motors.control_modes["Thumb"] = ControlMode.TORQUE
            
            # Set a large torque change
            motors.set_torque("Thumb", 0.6)  # Target: 0.6A
            
            # Apply rate limits
            motors._apply_rate_limits()
            
            # Check that change was limited to max_torque_rate
            self.assertEqual(motors.torque_targets["Thumb"], 0.05,
                          f"Torque change not limited to {motors.max_torque_rate}A")
            
        finally:
            motors.running = False

if __name__ == "__main__":
    unittest.main()