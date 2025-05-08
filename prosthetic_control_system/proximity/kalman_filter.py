#!/usr/bin/env python3
"""
Kalman Filter for proximity sensor data smoothing.

This module implements a simple scalar Kalman filter for smoothing
proximity sensor readings, especially helpful for noisy VL6180X data.
"""

import threading

class KalmanFilter:
    """
    Simple Kalman filter for smoothing one-dimensional sensor readings.
    
    This implementation uses a scalar form of the Kalman filter algorithm,
    optimized for proximity sensor distance measurements.
    """
    
    def __init__(self, 
                 process_variance=1e-3, 
                 measurement_variance=1e-1, 
                 initial_value=30.0,
                 glitch_values=None):
        """
        Initialize the Kalman filter.
        
        Args:
            process_variance: Process noise variance (how quickly the system state can change)
            measurement_variance: Measurement noise variance (how noisy are the measurements)
            initial_value: Initial state estimate in mm
            glitch_values: List of specific measurement values to treat as invalid (e.g., [0, 255])
        """
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = initial_value
        self.estimate_error = 1.0  # Initial uncertainty in the estimate
        
        # Values to mask as measurement errors
        self.glitch_values = glitch_values or [0, 255]  # Common VL6180X error values
        
        # Thread lock for thread safety
        self.lock = threading.Lock()
    
    def update(self, measurement):
        """
        Update the filter with a new measurement.
        
        Args:
            measurement: The new sensor measurement
            
        Returns:
            The updated state estimate
        """
        with self.lock:
            # Check if measurement is a glitch value
            if measurement in self.glitch_values:
                # For glitch values, we don't incorporate the measurement
                # but we still increase the estimated error due to time passing
                self.estimate_error += self.process_variance
                return self.estimate
            
            # Prediction step
            # (No change to estimate in simple model, but error increases)
            prediction_error = self.estimate_error + self.process_variance
            
            # Update step
            kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
            self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
            self.estimate_error = (1 - kalman_gain) * prediction_error
            
            return self.estimate