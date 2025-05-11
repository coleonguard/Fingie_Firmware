"""Configuration for operating without IMU."""

from enum import Enum

# Default configuration values
DEFAULT_CONFIG = {
    # Control rate (Hz)
    "control_rate": 20,
    
    # Logging
    "enable_logging": True,
    "log_dir": None,  # Default: ~/prosthetic_logs
    
    # Hardware configuration
    "use_simulated_motors": False,
    "motor_interface_kwargs": {},
    "proximity_sampling_rate": 50,  # Higher than control rate
    
    # Feature flags
    "use_imu": False,  # Set to false to disable IMU dependency
    
    # Proximity thresholds (mm)
    "approach_threshold": 40,
    "contact_threshold": 5,
    
    # Controller behavior
    "auto_release_timeout": 5.0,  # seconds
}

# Default multiplexer addresses
MUX_ADDRESSES = {
    "mux1": 0x77,  # For fingers: Thumb1, Thumb2, Index1, Index2, Middle1
    "mux2": 0x73,  # For fingers: Middle2, Ring1, Ring2, Pinky1, Pinky2
}

# Sensor-to-Motor mapping
FINGER_MAPPING = {
    "Thumb1": "Thumb",
    "Index1": "Index",
    "Middle1": "Middle",
    "Ring1": "Ring",
    "Pinky1": "Pinky"
}

# MCP joint sensors - primary control sensors
MCP_SENSORS = ["Thumb1", "Index1", "Middle1", "Ring1", "Pinky1"]

# Distance ranges for control (in mm)
DISTANCE_RANGES = {
    "max_range": 100,  # Maximum sensing range
    "proportional_max": 40,  # Start of proportional control
    "proportional_min": 5,   # End of proportional control (contact threshold)
    "contact": 5,            # Contact threshold
}

# Torque settings
TORQUE_SETTINGS = {
    "default": 0.2,     # Default torque value (A)
    "maximum": 0.6,     # Maximum allowed torque (A)
    "slip_trigger": 0.1  # Torque drop to detect slip (A)
}