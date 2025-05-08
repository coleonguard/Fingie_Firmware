#!/usr/bin/env python3
"""
Unit tests for the IMU orientation check (U-04).

Tests that the IMU interface checks for correct orientation on startup.
"""

import sys
import os
import unittest
from unittest.mock import MagicMock, patch

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import IMU interface to test
from imu.imu_interface import IMUInterface, OrientationError

class TestIMUOrientation(unittest.TestCase):
    """Test cases for the IMU orientation check"""
    
    @patch('imu.imu_interface.MSCL_AVAILABLE', True)
    def test_orientation_check_failure(self):
        """Test that orientation check raises OrientationError when Z-accel is wrong (U-04)"""
        # Create a mock IMU object with incorrectly oriented Z-axis (positive instead of negative)
        with patch('imu.imu_interface.IMUInterface._initialize_imus') as mock_init:
            # Set up the mock to return incorrect orientation data
            imu = IMUInterface(check_orientation=True)
            
            # Mock the IMU nodes
            imu.primary_imu = MagicMock()
            
            # Mock the _get_imu_data method to return data with wrong Z orientation
            with patch.object(imu, '_get_imu_data') as mock_get_data:
                # Return data with positive Z acceleration (wrong orientation, should be negative)
                mock_data = MagicMock()
                mock_data.accel_z = 9.81  # Positive instead of -9.81
                mock_get_data.return_value = mock_data
                
                # Check orientation should raise OrientationError
                with self.assertRaises(OrientationError):
                    imu._check_orientation()
    
    @patch('imu.imu_interface.MSCL_AVAILABLE', True)
    def test_orientation_check_success(self):
        """Test that orientation check passes when Z-accel is correct"""
        # Create a mock IMU object with correctly oriented Z-axis (negative)
        with patch('imu.imu_interface.IMUInterface._initialize_imus') as mock_init:
            # Set up the mock
            imu = IMUInterface(check_orientation=True)
            
            # Mock the IMU nodes
            imu.primary_imu = MagicMock()
            
            # Mock the _get_imu_data method to return data with correct Z orientation
            with patch.object(imu, '_get_imu_data') as mock_get_data:
                # Return data with negative Z acceleration (correct orientation)
                mock_data = MagicMock()
                mock_data.accel_z = -9.81  # Correct negative value
                mock_get_data.return_value = mock_data
                
                # Check orientation should not raise exception
                try:
                    imu._check_orientation()
                except OrientationError:
                    self.fail("_check_orientation() raised OrientationError unexpectedly!")

if __name__ == "__main__":
    unittest.main()