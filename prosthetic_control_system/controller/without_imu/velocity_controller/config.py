#!/usr/bin/env python3
"""
Configuration settings for velocity-based proximity controller.

This module contains default configuration values specifically tuned for
the velocity-based controller approach.
"""

import os
import copy
from ..config import DEFAULT_CONFIG as BASE_CONFIG

# Start with a copy of the base config
DEFAULT_CONFIG = copy.deepcopy(BASE_CONFIG)

# Override with velocity-specific settings
VELOCITY_CONFIG = {
    # Control parameters
    "control_rate": 50,  # Higher control rate for smoother velocity transitions

    # Velocity control parameters - conservative settings for minimal jerkiness
    "velocity_scale": 25.0,  # Significantly reduced scaling factor for velocity commands (deg/sec)
    "velocity_damping": 0.95,  # Very high damping factor for extremely smooth transitions
    "max_closing_velocity": 25.0,  # Severely limited closing velocity (deg/sec)
    "max_opening_velocity": 30.0,  # Severely limited opening velocity (deg/sec)
    
    # Motion parameters
    "idle_return_factor": 0.2,  # Gentler return to open position when idle
    "position_recalibration_interval": 15.0,  # Extended interval between position recalibrations
    
    # Position estimation
    "position_estimation_enabled": True,  # Whether to track estimated positions
    "position_error_threshold": 10.0,  # Position error that triggers recalibration
    
    # Safety limits
    "emergency_stop_threshold": 5.0,  # Current threshold for emergency velocity reduction
    
    # Finger velocity thresholds for different states - narrow range for smoother control
    "finger_velocities": {
        "idle": -15.0,  # Gentle velocity when returning to open position (neg = opening)
        "approach": 7.0,  # Very slow velocity when approaching
        "proportional_max": 15.0,  # Severely limited maximum velocity in proportional control
        "proportional_min": 3.0,  # Minimum velocity in proportional control for gentle movement
        "contact": 3.0,  # Minimal velocity when in contact
    },
    
    # Distance filtering parameters
    "distance_filter_alpha": 0.15,  # Strong filtering of distance measurements (lower = stronger)
    "consecutive_readings_threshold": 4,  # Require more consistent readings before changing velocities
    
    # Finger angle limits (same as base config for safety)
    "max_finger_angles": {
        "Thumb": 70.0,
        "Index": 90.0,
        "Middle": 90.0,
        "Ring": 90.0,
        "Pinky": 90.0
    },
}

# Update the default config with velocity-specific settings
DEFAULT_CONFIG.update(VELOCITY_CONFIG)

# Mapping from sensors to fingers (same as base config)
FINGER_MAPPING = {
    "Thumb": "Thumb",
    "Index": "Index",
    "Middle": "Middle",
    "Ring": "Ring",
    "Pinky": "Pinky"
}

# Sensor mapping for each finger (same as base config)
MCP_SENSORS = {
    "Thumb": "T1",
    "Index": "I1",
    "Middle": "M1",
    "Ring": "R1",
    "Pinky": "P1"
}

# Fallback sensor mapping (same as base config)
FINGER_FALLBACK_MAP = {
    "T1": ["T2", "I1", "I2", "M1"],
    "I1": ["I2", "T2", "M1", "M2"],
    "M1": ["M2", "I1", "R1", "R2"],
    "R1": ["R2", "M1", "P1", "P2"],
    "P1": ["P2", "R1", "R2", "M1"]
}