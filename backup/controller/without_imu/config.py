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
    "approach_threshold": 25,  # More conservative approach threshold (was 40)
    "contact_threshold": 10,   # Increased contact threshold (was 5)
    
    # Controller behavior
    "auto_release_timeout": 5.0,  # seconds
    
    # Distance filtering
    "consecutive_readings_required": 3,  # Number of consecutive readings before triggering
    "reset_distance": 40,            # Distance to consider object moved away (mm)
    "startup_delay": 2.0,            # Delay after startup before allowing closure (seconds)
    
    # State transition timing (to slow down finger movement)
    "min_time_in_idle": 0.1,         # Minimum time in IDLE state before transitioning (seconds)
    "min_time_in_approach": 0.2,     # Minimum time in APPROACH state before transitioning (seconds)
    "min_time_in_proportional": 0.3, # Minimum time in PROPORTIONAL state before transitioning (seconds)
    "min_time_in_contact": 0.2,      # Minimum time in CONTACT state before transitioning (seconds)
    
    # Motor movement speed control
    "max_position_change_per_second": 15.0,  # Degrees per second (lower = slower movement)
    "max_finger_angles": {              # Maximum allowed closure angle for each finger (to prevent self-detection)
        "Thumb": 35.0,                   # Approx. 50% of full closure
        "Index": 40.0,
        "Middle": 40.0,
        "Ring": 40.0,
        "Pinky": 35.0
    },
    
    # Debug settings
    "verbose_logging": False,         # Set to True for verbose debug logging
}

# Default multiplexer addresses - MAKE SURE THESE MATCH YOUR HARDWARE
MUX_ADDRESSES = {
    "mux1": 0x77,  # First multiplexer (address 0x77) - Thumb1, Thumb2, Index1, Index2, Middle1
    "mux2": 0x73,  # Second multiplexer (address 0x73) - Middle2, Ring1, Ring2, Pinky1, Pinky2
}

# Default sensor configuration matching multiplexer addresses
DEFAULT_SENSORS = [
    (0x77, 0, "Thumb1"),
    (0x77, 1, "Thumb2"),
    (0x77, 2, "Index1"),
    (0x77, 3, "Index2"),
    (0x77, 4, "Middle1"),
    (0x73, 0, "Middle2"),
    (0x73, 1, "Ring1"),
    (0x73, 2, "Ring2"),
    (0x73, 3, "Pinky1"),
    (0x73, 4, "Pinky2"),
]

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

# PIP joint sensors - secondary sensors
PIP_SENSORS = ["Thumb2", "Index2", "Middle2", "Ring2", "Pinky2"]

# Finger-to-finger fallback mapping for when all sensors for a finger fail
FINGER_FALLBACK_MAP = {
    "Thumb": ["Index", "Middle"],         # If thumb sensors fail, try Index then Middle
    "Index": ["Middle", "Thumb"],         # If index sensors fail, try Middle then Thumb
    "Middle": ["Index", "Ring"],          # If middle sensors fail, try Index then Ring
    "Ring": ["Middle", "Pinky"],          # If ring sensors fail, try Middle then Pinky
    "Pinky": ["Ring", "Middle"]           # If pinky sensors fail, try Ring then Middle
}

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