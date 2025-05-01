#!/usr/bin/env python3
# Configuration for Prosthetic Control System

import os

# System Configuration
# --------------------
# Default control rate in Hz
DEFAULT_CONTROL_RATE = 20

# Default proximity sensor sampling rate in Hz (2x control rate recommended)
DEFAULT_PROXIMITY_RATE = DEFAULT_CONTROL_RATE * 2

# Default log directory
DEFAULT_LOG_DIR = os.path.expanduser("~/prosthetic_logs")

# Control Thresholds
# -----------------
# Distance thresholds in mm
APPROACH_THRESHOLD = 40  # Distance beyond which no action occurs
CONTACT_THRESHOLD = 5    # Distance at which contact is detected

# Control Parameters
# -----------------
# Maximum position value (degrees or normalized)
MAX_POSITION = 100.0

# Maximum current during proportional control (A)
MAX_CURRENT = 0.8

# Current during contact for stable grip (A)
CONTACT_CURRENT = 0.5

# Thumb Configuration
# ------------------
# Whether thumb rotation is enabled
ENABLE_THUMB_ROTATION = True

# Maximum thumb rotation angle (degrees or normalized)
MAX_THUMB_ROTATION = 100.0

# Sensor Configuration
# -------------------
# Sensor filtering parameters
KALMAN_PROCESS_VARIANCE = 1e-3
KALMAN_MEASUREMENT_VARIANCE = 1e-1

# I2C Configuration
# ----------------
# I2C bus number
I2C_BUS = 1

# Multiplexer addresses
MUX1_ADDRESS = 0x70
MUX2_ADDRESS = 0x71

# VL6180X sensor address
VL6180X_ADDRESS = 0x29

# Ability Hand Configuration
# -------------------------
# Serial port (None for auto-detection)
ABILITY_HAND_PORT = None

# Serial baud rate
ABILITY_HAND_BAUD_RATE = 460800

# Reply mode (0: Pos,Cur,Touch, 1: Pos,Vel,Touch, 2: Pos,Cur,Vel)
ABILITY_HAND_REPLY_MODE = 2

# Debugging
# ---------
# Enable debug logging
DEBUG = False

# Enable simulation mode (no hardware required)
SIMULATION_MODE = False