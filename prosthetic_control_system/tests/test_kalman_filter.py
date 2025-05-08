#!/usr/bin/env python3
"""
Unit tests for the Kalman filter implementation.

This tests the Kalman filter used for sensor smoothing to ensure it
correctly handles glitch values and normal measurements.
"""

import sys
import os
import unittest

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import filter to test
from proximity.kalman_filter import KalmanFilter

class TestKalmanFilter(unittest.TestCase):
    """Test cases for the Kalman filter"""
    
    def test_initialization(self):
        """Test filter initialization"""
        kf = KalmanFilter(initial_value=30.0)
        self.assertEqual(kf.estimate, 30.0)
    
    def test_glitch_masking(self):
        """Test that the filter masks glitch values (U-01)"""
        # Create filter with initial estimate of 30
        kf = KalmanFilter(initial_value=30.0, glitch_values=[0, 255])
        
        # Feed sequence with glitches
        estimates = []
        for value in [30, 0, 0, 28]:
            estimates.append(kf.update(value))
        
        # Verify that output never becomes 0
        self.assertTrue(all(est > 0 for est in estimates), 
                      f"Filter output contained zero: {estimates}")
        
        # Check that final estimate is reasonable
        self.assertAlmostEqual(estimates[-1], 28.6, delta=1.0,
                              msg=f"Final estimate {estimates[-1]} not close to expected")
    
    def test_normal_update(self):
        """Test normal filter updates"""
        kf = KalmanFilter(initial_value=30.0)
        
        # Feed a sequence of decreasing values
        sequence = [28, 26, 24, 22, 20]
        estimates = []
        for value in sequence:
            estimates.append(kf.update(value))
        
        # Verify estimates follow the trend but lag behind
        self.assertTrue(estimates[0] > sequence[0],
                        f"First estimate {estimates[0]} should be above {sequence[0]}")
        
        self.assertTrue(estimates[-1] > sequence[-1],
                        f"Last estimate {estimates[-1]} should be above {sequence[-1]}")
        
        # Check that estimates are monotonically decreasing
        for i in range(1, len(estimates)):
            self.assertTrue(estimates[i] < estimates[i-1],
                           f"Estimates not monotonically decreasing: {estimates}")
    
    def test_measurement_variance(self):
        """Test effect of measurement variance"""
        # Create two filters with different measurement variances
        kf_precise = KalmanFilter(initial_value=30.0, measurement_variance=0.1)
        kf_imprecise = KalmanFilter(initial_value=30.0, measurement_variance=10.0)
        
        # Update both with the same value
        est_precise = kf_precise.update(20.0)
        est_imprecise = kf_imprecise.update(20.0)
        
        # The precise filter should move more toward the measurement
        self.assertTrue(est_precise < est_imprecise,
                       f"Precise filter ({est_precise}) should be closer to measurement than imprecise ({est_imprecise})")
    
    def test_process_variance(self):
        """Test effect of process variance"""
        # Create two filters with different process variances
        kf_stable = KalmanFilter(initial_value=30.0, process_variance=0.001)
        kf_dynamic = KalmanFilter(initial_value=30.0, process_variance=1.0)
        
        # First update both with the same value
        kf_stable.update(25.0)
        kf_dynamic.update(25.0)
        
        # Then update with a glitch value
        est_stable = kf_stable.update(0)
        est_dynamic = kf_dynamic.update(0)
        
        # The dynamic filter's estimate error should grow faster during glitch rejection
        # so it should respond more to the next valid measurement
        kf_stable.update(25.0)
        kf_dynamic.update(25.0)
        
        next_stable = kf_stable.update(20.0)
        next_dynamic = kf_dynamic.update(20.0)
        
        # The dynamic filter should move more toward the new measurement
        self.assertTrue(next_dynamic < next_stable,
                       f"Dynamic filter ({next_dynamic}) should be closer to new measurement than stable filter ({next_stable})")

if __name__ == "__main__":
    unittest.main()