#!/usr/bin/env python3
"""
Integration test for log schema validation (I-02).

Tests:
- I-02: Log schema - Validates that log entries conform to the expected schema.
"""

import sys
import os
import unittest
import json
import time
from unittest.mock import MagicMock, patch

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import utils.logger directly
from utils.logger import DataLogger

class TestLogSchema(unittest.TestCase):
    """Test cases for the log schema validation"""
    
    def test_log_entry_validation(self):
        """Test that log entries are validated against the schema (I-02)"""
        # Create a data logger with a temporary directory
        temp_dir = "/tmp/prosthetic_test_logs"
        os.makedirs(temp_dir, exist_ok=True)
        
        logger = DataLogger(log_dir=temp_dir, buffer_size=1)  # Small buffer for testing
        
        # Create a valid test entry
        valid_entry = {
            "timestamp": time.time(),
            "proximity": {
                "raw": {"Thumb1": 30, "Index1": 25},
                "filtered": {"Thumb1": 30.5, "Index1": 25.2},
                "status": {"Thumb1": "OK", "Index1": "OK"}
            },
            "imu": {
                "orientation": {"roll": 1.0, "pitch": 2.0, "yaw": 3.0},
                "acceleration": {"x": 0.1, "y": 0.2, "z": 9.8},
                "angular_rate": {"x": 0.5, "y": 0.6, "z": 0.7},
                "motion_state": "STATIC"
            },
            "fingers": {
                "Thumb": {
                    "state": "APPROACH",
                    "position": 0.0,
                    "velocity": 0.0,
                    "current": 0.0,
                    "target_position": 0.0,
                    "target_torque": 0.0
                },
                "Index": {
                    "state": "PROPORTIONAL",
                    "position": 30.0,
                    "velocity": 5.0,
                    "current": 0.1,
                    "target_position": 35.0,
                    "target_torque": 0.0
                }
            },
            "hand_state": "REACH"
        }
        
        # Create an invalid entry (missing required fields)
        invalid_entry = {
            "timestamp": time.time(),
            # Missing proximity
            "imu": {
                "orientation": {"roll": 1.0, "pitch": 2.0, "yaw": 3.0}
                # Missing other required fields
            },
            # Missing fingers
            "hand_state": "REACH"
        }
        
        # Test valid entry
        result = logger._validate_entry(valid_entry)
        self.assertTrue(result, "Valid entry failed validation")
        
        # Test invalid entry
        result = logger._validate_entry(invalid_entry)
        self.assertFalse(result, "Invalid entry passed validation")
        
        # Test with 100 random valid entries
        for i in range(100):
            # Create a variation of the valid entry
            entry = valid_entry.copy()
            entry["timestamp"] = time.time() + i
            entry["proximity"]["raw"]["Thumb1"] = 30 + i % 10
            entry["imu"]["orientation"]["roll"] = i % 360
            
            # Validate
            result = logger._validate_entry(entry)
            self.assertTrue(result, f"Valid entry {i} failed validation")
        
        # Clean up
        logger.stop()
    
    def test_log_file_creation(self):
        """Test that log files are created correctly"""
        # Create a data logger with a temporary directory
        temp_dir = "/tmp/prosthetic_test_logs"
        os.makedirs(temp_dir, exist_ok=True)
        
        logger = DataLogger(log_dir=temp_dir, buffer_size=1)
        logger.start()
        
        # Create a valid test entry
        valid_entry = {
            "timestamp": time.time(),
            "proximity": {
                "raw": {"Thumb1": 30, "Index1": 25},
                "filtered": {"Thumb1": 30.5, "Index1": 25.2},
                "status": {"Thumb1": "OK", "Index1": "OK"}
            },
            "imu": {
                "orientation": {"roll": 1.0, "pitch": 2.0, "yaw": 3.0},
                "acceleration": {"x": 0.1, "y": 0.2, "z": 9.8},
                "angular_rate": {"x": 0.5, "y": 0.6, "z": 0.7},
                "motion_state": "STATIC"
            },
            "fingers": {
                "Thumb": {
                    "state": "APPROACH",
                    "position": 0.0,
                    "velocity": 0.0,
                    "current": 0.0,
                    "target_position": 0.0,
                    "target_torque": 0.0
                }
            },
            "hand_state": "REACH"
        }
        
        # Log 10 entries
        for i in range(10):
            entry = valid_entry.copy()
            entry["timestamp"] = time.time() + i
            logger.log_data(entry)
        
        # Get the log filename
        log_filename = logger.log_filename
        
        # Stop the logger to flush the buffer
        logger.stop()
        
        # Check that the file exists
        self.assertTrue(os.path.exists(log_filename), 
                      f"Log file {log_filename} was not created")
        
        # Check that the file contains valid JSON
        with open(log_filename, 'r') as f:
            lines = f.readlines()
            self.assertEqual(len(lines), 10, 
                           f"Expected 10 log entries, found {len(lines)}")
            
            for i, line in enumerate(lines):
                try:
                    entry = json.loads(line)
                    self.assertTrue("timestamp" in entry, 
                                 f"Log entry {i} missing timestamp")
                    self.assertTrue("proximity" in entry, 
                                 f"Log entry {i} missing proximity data")
                    self.assertTrue("imu" in entry, 
                                 f"Log entry {i} missing IMU data")
                    self.assertTrue("fingers" in entry, 
                                 f"Log entry {i} missing finger data")
                    self.assertTrue("hand_state" in entry, 
                                 f"Log entry {i} missing hand state")
                except json.JSONDecodeError:
                    self.fail(f"Log entry {i} is not valid JSON: {line}")

if __name__ == "__main__":
    unittest.main()