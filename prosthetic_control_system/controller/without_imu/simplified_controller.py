#!/usr/bin/env python3
"""
Simplified Proximity Controller for Prosthetic Hand.

This module implements a simplified controller that uses proximity sensors for hand
control without threading. It's based on the architecture of fallback_test.py to
ensure reliable I2C communication with the proximity sensors.

Key differences from the threaded approach:
1. Single-loop architecture that runs sensor reading and motor control in sequence
2. Strict separation of I2C operations from motor control
3. Simple, deterministic error handling without complex retry mechanisms
4. No thread synchronization overhead or context switching delays
"""

import time
import smbus2
import logging
import json
import os
import sys
import signal
from enum import Enum
from typing import Dict, List, Optional, Tuple, Any
from collections import defaultdict, deque

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("SimplifiedController")

# Add parent directories to path if necessary
parent_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

# Local imports
from prosthetic_control_system.hand.motor_interface import MotorInterface, ControlMode, SimulatedMotorInterface
from prosthetic_control_system.hand.ability_hand_interface import AbilityHandInterface
from prosthetic_control_system.controller.state_machines import FingerState, HandState, FingerFSM, HandFSM
from prosthetic_control_system.utils.logger import DataLogger
from prosthetic_control_system.controller.without_imu.config import DEFAULT_CONFIG, FINGER_MAPPING, MCP_SENSORS, FINGER_FALLBACK_MAP

# State definitions
class MotionState(Enum):
    """Motion states for IMU-free operation."""
    STATIC = 0
    MOVING = 1
    IMPACT = 2
    UNKNOWN = 3

# Constants for VL6180X sensor
VL6180X_ADDRESS = 0x29  # I2C address of VL6180X
MAX_READY_MS = 20       # Maximum time to wait for sensor ready
N_RETRIES = 3           # Number of retries for sensor readings

# I2C bus operations
def mux_select(bus, addr, ch):
    """Select a channel on the multiplexer"""
    bus.write_byte(addr, 1 << ch)
    time.sleep(0.0005)  # Small delay for stable switching

def write_byte(bus, reg, val):
    """Write a byte to VL6180X sensor register"""
    hi, lo = reg >> 8 & 0xFF, reg & 0xFF
    bus.write_i2c_block_data(VL6180X_ADDRESS, hi, [lo, val])

def read_byte(bus, reg):
    """Read a byte from VL6180X sensor register"""
    hi, lo = reg >> 8 & 0xFF, reg & 0xFF
    bus.write_i2c_block_data(VL6180X_ADDRESS, hi, [lo])
    time.sleep(0.0003)  # Short delay for stable reading
    return bus.read_byte(VL6180X_ADDRESS)

def vl_init(bus):
    """Initialize the VL6180X sensor"""
    # Check if initialization needed
    if read_byte(bus, 0x016) != 1:
        return False
        
    # Initialize register settings (from datasheet sequence)
    init_settings = [
        (0x0207, 0x01), (0x0208, 0x01), (0x0096, 0x00),
        (0x0097, 0xfd), (0x00e3, 0x00), (0x00e4, 0x04),
        (0x00e5, 0x02), (0x00e6, 0x01), (0x00e7, 0x03),
        (0x00f5, 0x02), (0x00d9, 0x05), (0x00db, 0xce),
        (0x00dc, 0x03), (0x00dd, 0xf8), (0x009f, 0x00),
        (0x00a3, 0x3c), (0x00b7, 0x00), (0x00bb, 0x3c),
        (0x00b2, 0x09), (0x00ca, 0x09), (0x0198, 0x01),
        (0x01b0, 0x17), (0x01ad, 0x00), (0x00ff, 0x05),
        (0x0100, 0x05), (0x0199, 0x05), (0x01a6, 0x1b),
        (0x01ac, 0x3e), (0x01a7, 0x1f), (0x0030, 0x00),
        
        # Public registers
        (0x0011, 0x10), (0x010a, 0x30), (0x003f, 0x46),
        (0x0031, 0xFF), (0x0040, 0x63), (0x002e, 0x01),
        (0x001b, 0x09), (0x003e, 0x31), (0x0014, 0x24),
        
        # Final initialization bit
        (0x016, 0x00)
    ]
    
    for reg, value in init_settings:
        write_byte(bus, reg, value)
        
    return True

def start_range(bus):
    """Start a range measurement"""
    write_byte(bus, 0x018, 0x01)

def clear_int(bus):
    """Clear all interrupts"""
    write_byte(bus, 0x015, 0x07)

def wait_ready(bus):
    """Wait for range measurement to be ready"""
    t0 = time.time()
    while (read_byte(bus, 0x04F) & 0x07) != 0x04:
        if (time.time() - t0) * 1000 > MAX_READY_MS:
            raise TimeoutError("Timeout waiting for range ready")
        time.sleep(0.0003)

def get_distance(bus):
    """Get distance measurement from current sensor"""
    for _ in range(N_RETRIES):
        try:
            start_range(bus)
            wait_ready(bus)
            d = read_byte(bus, 0x062)
            clear_int(bus)
            return d
        except TimeoutError:
            time.sleep(0.002)  # Let the sensor settle before retry
    
    # All retries failed
    return None

class SimplifiedController:
    """
    Simplified proximity controller for prosthetic hand.
    
    This controller uses a single-loop architecture inspired by fallback_test.py
    to ensure reliable I2C communication while controlling the Ability Hand.
    """
    
    def __init__(self, 
                 control_rate: int = DEFAULT_CONFIG["control_rate"], 
                 enable_logging: bool = DEFAULT_CONFIG["enable_logging"],
                 log_dir: str = DEFAULT_CONFIG["log_dir"],
                 use_simulated_motors: bool = DEFAULT_CONFIG["use_simulated_motors"],
                 motor_interface_kwargs: dict = None,
                 approach_threshold: int = DEFAULT_CONFIG["approach_threshold"],
                 contact_threshold: int = DEFAULT_CONFIG["contact_threshold"],
                 consecutive_readings_required: int = DEFAULT_CONFIG["consecutive_readings_required"],
                 reset_distance: int = DEFAULT_CONFIG["reset_distance"],
                 startup_delay: float = DEFAULT_CONFIG["startup_delay"],
                 min_time_in_idle: float = DEFAULT_CONFIG["min_time_in_idle"],
                 min_time_in_approach: float = DEFAULT_CONFIG["min_time_in_approach"],
                 min_time_in_proportional: float = DEFAULT_CONFIG["min_time_in_proportional"],
                 min_time_in_contact: float = DEFAULT_CONFIG["min_time_in_contact"],
                 max_position_change_per_second: float = DEFAULT_CONFIG["max_position_change_per_second"],
                 max_finger_angles: dict = DEFAULT_CONFIG["max_finger_angles"],
                 verbose_logging: bool = DEFAULT_CONFIG["verbose_logging"]):
        """
        Initialize the simplified proximity controller.
        
        Args:
            control_rate: Control loop frequency in Hz
            enable_logging: Whether to enable data logging
            log_dir: Directory for log files (default: ~/prosthetic_logs)
            use_simulated_motors: Whether to use simulated motors
            motor_interface_kwargs: Additional kwargs for motor interface
            approach_threshold: Distance threshold for approach phase (mm)
            contact_threshold: Distance threshold for contact phase (mm)
            consecutive_readings_required: Number of consecutive readings required for state change
            reset_distance: Distance to reset to after contact
            startup_delay: Delay after startup before enabling control
            min_time_in_*: Minimum time required in each state
            max_position_change_per_second: Maximum rate of position change (deg/s)
            max_finger_angles: Maximum angle for each finger
            verbose_logging: Whether to enable verbose logging
        """
        # Save parameters
        self.enable_logging = enable_logging
        self.approach_threshold = approach_threshold
        self.contact_threshold = contact_threshold
        self.consecutive_readings_required = consecutive_readings_required
        self.reset_distance = reset_distance
        self.startup_delay_seconds = startup_delay
        self.verbose_logging = verbose_logging
        
        # State transition timing constraints
        self.min_time_in_idle = min_time_in_idle
        self.min_time_in_approach = min_time_in_approach
        self.min_time_in_proportional = min_time_in_proportional
        self.min_time_in_contact = min_time_in_contact
        
        # Motion constraints
        self.max_position_change_per_second = max_position_change_per_second
        self.max_finger_angles = max_finger_angles
        
        # Get motor parameters
        motor_interface_kwargs = motor_interface_kwargs or {}
        port = motor_interface_kwargs.get('port')
        
        # Create Ability Hand interface directly
        logger.info(f"Creating {'simulated' if use_simulated_motors else 'real'} Ability Hand interface")
        
        if use_simulated_motors:
            # For simulated mode, pass None as port to force simulation
            self.hand = AbilityHandInterface(
                control_rate=control_rate,
                port=None
            )
        else:
            # For real hardware
            self.hand = AbilityHandInterface(
                control_rate=control_rate,
                port=port
            )
        
        # Use the hand interface for motor control
        self.motors = self.hand
        
        # Store control rate for calculations
        self.control_rate = control_rate  # Hz
        self.control_interval = 1.0 / control_rate  # seconds
        
        # Initialize data logger if enabled
        self.logger = None
        if enable_logging:
            log_dir = log_dir or os.path.expanduser("~/prosthetic_logs")
            self.logger = DataLogger(log_dir)
            logger.info(f"Data logging enabled to: {log_dir}")
        
        # Initialize I2C bus
        self.bus = smbus2.SMBus(1)
        
        # Sensor setup
        # Based on fallback_test.py sensor layout
        self.SENSORS = [
            ("Thumb1", 0x77, 0), ("Thumb2", 0x77, 1),  # First multiplexer
            ("Index1", 0x77, 2), ("Index2", 0x77, 3),
            ("Middle1", 0x77, 4), ("Middle2", 0x73, 0),  # Second multiplexer
            ("Ring1", 0x73, 1), ("Ring2", 0x73, 2),
            ("Pinky1", 0x73, 3), ("Pinky2", 0x73, 4),
        ]
        
        # Fallback rules
        self.FALLBACK = {
            "Thumb1": ["Thumb2", "Index1", "Index2", "Middle1"],
            "Thumb2": ["Thumb1", "Index2", "Index1", "Middle2"],
            "Index1": ["Index2", "Thumb2", "Middle1", "Middle2"],
            "Index2": ["Index1", "Middle2", "Thumb2", "Middle1"],
            "Middle1": ["Middle2", "Index1", "Ring1", "Ring2"],
            "Middle2": ["Middle1", "Index2", "Ring2", "Ring1"],
            "Ring1": ["Ring2", "Middle1", "Pinky1", "Pinky2"],
            "Ring2": ["Ring1", "Middle2", "Pinky2", "Pinky1"],
            "Pinky1": ["Pinky2", "Ring1", "Ring2", "Middle1"],
            "Pinky2": ["Pinky1", "Ring2", "Ring1", "Middle2"],
        }
        
        # State machine for each finger
        self.finger_fsm = {
            "Thumb": FingerFSM(
                finger_name="Thumb",
                approach_threshold=approach_threshold,
                contact_threshold=contact_threshold
            ),
            "Index": FingerFSM(
                finger_name="Index",
                approach_threshold=approach_threshold,
                contact_threshold=contact_threshold
            ),
            "Middle": FingerFSM(
                finger_name="Middle",
                approach_threshold=approach_threshold,
                contact_threshold=contact_threshold
            ),
            "Ring": FingerFSM(
                finger_name="Ring",
                approach_threshold=approach_threshold,
                contact_threshold=contact_threshold
            ),
            "Pinky": FingerFSM(
                finger_name="Pinky",
                approach_threshold=approach_threshold,
                contact_threshold=contact_threshold
            ),
        }
        
        # Hand state machine
        self.hand_fsm = HandFSM(
            release_current_threshold=0.2,
            required_fingers_grasping=2
        )
        
        # Initialize sensor state
        self.raw_values = {}  # Raw sensor readings
        self.final_values = {}  # With fallback substitution
        self.filtered_values = {}  # For smoothing
        self.status = {}  # OK, SUB, BAD for each sensor
        
        # Finger position tracking
        self.finger_positions = {
            "Thumb": 0.0,
            "Index": 0.0,
            "Middle": 0.0,
            "Ring": 0.0,
            "Pinky": 0.0,
        }
        
        # System state
        self.running = False
        self.fault_conditions = {
            "hand_timeout": False,
            "system_overload": False,
            "sensor_failure": False,
            "i2c_error": False,
        }
        
        # Sensor to finger mapping
        self.mcp_sensor_mapping = {
            "Thumb1": "Thumb",
            "Index1": "Index",
            "Middle1": "Middle",
            "Ring1": "Ring",
            "Pinky1": "Pinky",
        }
        
        # Track connection between sensor data and finger control
        self.finger_substitutions = {
            "Thumb": None,
            "Index": None,
            "Middle": None,
            "Ring": None,
            "Pinky": None,
        }
        
        # State tracking for each finger
        self.approach_counters = defaultdict(int)
        self.last_state_change_time = {finger: time.time() for finger in self.finger_fsm}
        self.awaiting_reset = {finger: False for finger in self.finger_fsm}
        
        # Performance tracking
        self.last_cycle_time = 0.0
        self.avg_cycle_time = 0.0
        self.cycle_count = 0
        
        # Startup protection
        self.startup_time = time.time()
        
        # Sensor analysis mode (optional)
        self.analyzing_sensors = False
        self.sensor_samples = defaultdict(list)
        self.sensor_analysis_start_time = 0.0
        
        # Initialize sensor states
        self.init_done = defaultdict(bool)  # (mux, ch) → bool
        
    def start(self):
        """Start the controller"""
        if self.running:
            logger.warning("Controller is already running")
            return
            
        logger.info("Starting simplified controller")
        
        # Initialize hardware
        try:
            # Reset multiplexers to known state
            mux_addresses = {addr for _, addr, _ in self.SENSORS}
            for mux in mux_addresses:
                try:
                    self.bus.write_byte(mux, 0x00)  # Disable all channels
                except Exception as e:
                    logger.error(f"Error resetting multiplexer {hex(mux)}: {e}")
            
            # Start hand interface
            self.hand.connect()
            
            # Motors already initialized in __init__
            
            # Set running flag
            self.running = True
            
            # Start main control loop
            logger.info("Controller started successfully")
            
            # Reset startup time
            self.startup_time = time.time()
            
        except Exception as e:
            logger.error(f"Failed to start controller: {e}")
            self._shutdown()
            raise
    
    def stop(self):
        """Stop the controller and release resources"""
        if not self.running:
            logger.warning("Controller is not running")
            return
            
        logger.info("Stopping controller")
        self._shutdown()
        logger.info("Controller stopped")
    
    def _shutdown(self):
        """Shutdown and cleanup resources"""
        self.running = False
        
        # Safely stop hand interface
        try:
            if self.hand:
                self.hand.disconnect()
        except Exception as e:
            logger.error(f"Error disconnecting hand: {e}")
        
        # Close I2C bus
        try:
            # Disable all multiplexers first
            mux_addresses = {addr for _, addr, _ in self.SENSORS}
            for mux in mux_addresses:
                try:
                    self.bus.write_byte(mux, 0x00)
                except Exception:
                    pass  # Ignore errors during shutdown
            
            # Close the bus
            self.bus.close()
        except Exception as e:
            logger.error(f"Error closing I2C bus: {e}")
        
        # Close logger if active
        if self.logger:
            try:
                self.logger.close()
            except Exception as e:
                logger.error(f"Error closing data logger: {e}")
    
    def _read_all_sensors(self):
        """
        Read all proximity sensors with fallback substitution.
        
        This is a direct adaptation of fallback_test.py's approach,
        ensuring reliable I2C communication.
        """
        raw, ok, subs, bad = {}, [], [], []
        
        # 1. Get raw readings from all sensors
        for name, mux, ch in self.SENSORS:
            try:
                # Select the multiplexer channel
                mux_select(self.bus, mux, ch)
                
                # Initialize sensor if needed
                if not self.init_done[(mux, ch)]:
                    if vl_init(self.bus):
                        logger.info(f"Initialized sensor {name}")
                    self.init_done[(mux, ch)] = True
                
                # Get distance reading
                raw[name] = get_distance(self.bus)
                if raw[name] is not None:
                    ok.append(name)
            except Exception as e:
                logger.warning(f"Error reading sensor {name}: {e}")
                raw[name] = None
        
        # 2. Apply fallback substitution where needed
        final = {}
        for name, *_ in self.SENSORS:
            if raw[name] is not None:
                final[name] = raw[name]
                self.status[name] = "OK"
            else:
                # Try fallback sensors
                for nb in self.FALLBACK[name]:
                    if raw.get(nb) is not None:
                        final[name] = raw[nb]
                        subs.append(name)
                        self.status[name] = "SUB"
                        break
                else:
                    # No substitution available
                    final[name] = 100  # Default to max distance
                    bad.append(name)
                    self.status[name] = "BAD"
        
        # Update sensor data
        self.raw_values = raw
        self.final_values = final
        
        # Simple exponential filter for smoothing (alpha = 0.3)
        alpha = 0.3
        for name in self.SENSORS:
            name = name[0]  # Extract sensor name
            if name in self.filtered_values:
                # Update existing value with filtering
                if final[name] is not None:
                    self.filtered_values[name] = (1-alpha) * self.filtered_values[name] + alpha * final[name]
            else:
                # Initialize filtered value
                self.filtered_values[name] = final[name] if final[name] is not None else 100
            
        # Return lists of sensors by status
        return ok, subs, bad
    
    def _update_finger_states(self):
        """
        Update each finger's state based on proximity readings.
        
        Maps the MCP joint sensor to each finger and updates state machines.
        """
        # Track which sensors are controlling which fingers
        self.finger_substitutions = {finger: None for finger in self.finger_fsm}
        
        # Process each finger
        for finger, fsm in self.finger_fsm.items():
            # Determine which sensor to use for this finger
            primary_sensor = f"{finger[0]}1"  # e.g., "T1" for Thumb MCP
            distance, src_sensor = self._get_best_sensor_for_finger(finger, primary_sensor)
            
            # Record substitution if not using primary sensor
            if src_sensor != primary_sensor:
                self.finger_substitutions[finger] = src_sensor
            
            # Update finger state machine
            current_state = fsm.state
            
            # Handle state transitions with hysteresis
            if self.awaiting_reset[finger]:
                # Waiting for reset (large distance)
                if distance > self.reset_distance:
                    self.awaiting_reset[finger] = False
                    # Reset counter when coming out of hysteresis
                    self.approach_counters[finger] = 0
            else:
                # Normal state transitions
                # First, check time constraints
                time_in_state = time.time() - self.last_state_change_time[finger]
                
                # Minimum time requirements by state
                min_time_requirements = {
                    FingerState.IDLE: self.min_time_in_idle,
                    FingerState.APPROACH: self.min_time_in_approach,
                    FingerState.PROPORTIONAL: self.min_time_in_proportional,
                    FingerState.CONTACT: self.min_time_in_contact,
                }
                
                # Only change state if minimum time requirement is met
                if time_in_state >= min_time_requirements[current_state]:
                    # Determine new state based on distance
                    if distance <= self.contact_threshold:
                        # CONTACT: Very close proximity
                        new_state = FingerState.CONTACT
                    elif distance <= self.approach_threshold:
                        # PROPORTIONAL: Medium proximity - control proportionally
                        range_size = self.approach_threshold - self.contact_threshold
                        if range_size > 0:
                            # Normalize distance within the proportional range (0-1)
                            normalized = (distance - self.contact_threshold) / range_size
                            # Use normalized value for proportional control
                            new_state = FingerState.PROPORTIONAL
                        else:
                            # Safety: if thresholds are too close together
                            new_state = FingerState.APPROACH
                    else:
                        # IDLE: No object detected
                        new_state = FingerState.IDLE
                    
                    # State transition logic with consecutive reading requirement
                    if new_state != current_state:
                        # Increment approach counter for this transition
                        if new_state == current_state:
                            # Reset counter if we're staying in the same state
                            self.approach_counters[finger] = 0
                        else:
                            # Increment counter for a potential state change
                            self.approach_counters[finger] += 1
                            
                            # Only change state if we've seen enough consecutive readings
                            if self.approach_counters[finger] >= self.consecutive_readings_required:
                                # Apply state change
                                fsm.force_state(new_state)
                                
                                # Record time of state change
                                self.last_state_change_time[finger] = time.time()
                                
                                # Reset counter
                                self.approach_counters[finger] = 0
                                
                                # For CONTACT transitions, set awaiting reset
                                if new_state == FingerState.CONTACT:
                                    self.awaiting_reset[finger] = True
                    else:
                        # Reset counter if we're staying in the same state
                        self.approach_counters[finger] = 0
    
    def _update_hand_state(self):
        """Update the hand state machine based on finger states"""
        # Count fingers in each state
        state_counts = {state: 0 for state in FingerState}
        
        # Count states
        for finger, fsm in self.finger_fsm.items():
            state_counts[fsm.state] += 1
        
        # Update hand state based on finger states
        current_hand_state = self.hand_fsm.state
        
        # Hand is ACTIVE if any finger is in APPROACH or PROPORTIONAL
        if state_counts[FingerState.APPROACH] > 0 or state_counts[FingerState.PROPORTIONAL] > 0:
            new_hand_state = HandState.ACTIVE
        
        # Hand is HOLDING if any finger is in CONTACT
        elif state_counts[FingerState.CONTACT] > 0:
            new_hand_state = HandState.HOLDING
        
        # Hand is IDLE if all fingers are IDLE
        elif state_counts[FingerState.IDLE] == len(self.finger_fsm):
            new_hand_state = HandState.IDLE
        
        # Otherwise, maintain current state
        else:
            new_hand_state = current_hand_state
        
        # Apply state change if needed
        if new_hand_state != current_hand_state:
            self.hand_fsm.force_state(new_hand_state)
    
    def _update_finger_positions(self):
        """
        Update finger positions based on state and proximity values.
        
        This determines the angle for each finger based on its state
        and sensor readings, then applies rate limiting.
        """
        # Startup protection
        time_since_startup = time.time() - self.startup_time
        startup_protection_active = time_since_startup < self.startup_delay_seconds
        
        # If startup protection is active, keep all fingers at 0
        if startup_protection_active:
            for finger in self.finger_positions:
                self.finger_positions[finger] = 0.0
            return
        
        # Time since last update for rate limiting
        dt = self.last_cycle_time
        
        # Maximum movement allowed this cycle
        max_change = self.max_position_change_per_second * dt
        
        # Process each finger
        for finger, fsm in self.finger_fsm.items():
            # Current finger position
            current_pos = self.finger_positions[finger]
            
            # Calculate target position based on state
            if fsm.state == FingerState.IDLE:
                target_pos = 0.0  # Open position
            elif fsm.state == FingerState.CONTACT:
                target_pos = self.max_finger_angles[finger]  # Full close
            elif fsm.state == FingerState.PROPORTIONAL:
                # Find the appropriate sensor for this finger
                primary_sensor = f"{finger[0]}1"  # e.g., "T1" for Thumb
                distance, _ = self._get_best_sensor_for_finger(finger, primary_sensor)
                
                # Calculate proportional position
                range_size = self.approach_threshold - self.contact_threshold
                if range_size > 0:
                    # Normalize distance within range (0-1)
                    normalized = (distance - self.contact_threshold) / range_size
                    # Invert and scale to finger range (closer = more closed)
                    normalized = 1.0 - max(0.0, min(1.0, normalized))
                    target_pos = normalized * self.max_finger_angles[finger]
                else:
                    # Safety fallback if thresholds are invalid
                    target_pos = 0.5 * self.max_finger_angles[finger]
            elif fsm.state == FingerState.APPROACH:
                target_pos = 0.3 * self.max_finger_angles[finger]  # Partial close
            else:
                # Unknown state, maintain position
                target_pos = current_pos
            
            # Apply rate limiting
            if abs(target_pos - current_pos) <= max_change:
                # Small change, apply directly
                new_pos = target_pos
            else:
                # Limit the maximum change rate
                if target_pos > current_pos:
                    new_pos = current_pos + max_change
                else:
                    new_pos = current_pos - max_change
            
            # Enforce limits
            new_pos = max(0.0, min(new_pos, self.max_finger_angles[finger]))
            
            # Update position
            self.finger_positions[finger] = new_pos
    
    def _send_motor_commands(self):
        """Send position commands to the motors"""
        try:
            # Ensure hand is connected
            if not self.hand.is_connected():
                self.hand.connect()
                logger.info("Reconnected to hand")
            
            # Send position commands for each finger
            for finger, position in self.finger_positions.items():
                # Map finger name to motor name (may be different)
                motor = FINGER_MAPPING.get(finger, finger)
                
                # Send command to motor
                self.hand.set_position(motor, position)
            
            # Execute commands
            self.hand.update()
            
        except Exception as e:
            logger.error(f"Error sending motor commands: {e}")
            self.fault_conditions["hand_timeout"] = True
    
    def _get_best_sensor_for_finger(self, finger, primary_sensor):
        """
        Get the best available proximity reading for a finger.
        
        Args:
            finger: Finger name (e.g., "Thumb")
            primary_sensor: Primary sensor for this finger (e.g., "T1")
            
        Returns:
            Tuple of (distance, sensor_name) with best available reading
        """
        # Get status of primary sensor
        primary_status = self.status.get(primary_sensor, "BAD")
        
        # If primary sensor is ok, use it
        if primary_status == "OK" and primary_sensor in self.filtered_values:
            return self.filtered_values[primary_sensor], primary_sensor
        
        # Determine fallback sensors for this finger (based on mapping)
        primary_code = primary_sensor[0] + "1"  # E.g., "T1" for Thumb MCP
        
        # Find best available alternative
        for sensor_name in self.filtered_values:
            # Check if this sensor starts with the same letter (same finger)
            if sensor_name.startswith(primary_sensor[0]) and sensor_name != primary_sensor:
                return self.filtered_values[sensor_name], sensor_name
        
        # If no same-finger sensor, try neighbors
        if primary_sensor in self.FALLBACK:
            for nb in self.FALLBACK[primary_sensor]:
                if nb in self.filtered_values:
                    return self.filtered_values[nb], nb
        
        # Last resort - use default value
        return 100, None  # 100mm = far away
    
    def run_one_cycle(self):
        """
        Run a single control cycle.
        
        This method:
        1. Reads all sensors
        2. Updates finger state machines
        3. Updates hand state machine
        4. Updates finger positions
        5. Sends motor commands
        
        Returns:
            True if cycle completed successfully, False otherwise
        """
        if not self.running:
            return False
        
        cycle_start = time.time()
        
        try:
            # PHASE 1: Read all sensors
            ok, subs, bad = self._read_all_sensors()
            
            # Log sensor status
            if self.verbose_logging:
                logger.debug(f"Sensors: OK={len(ok)} SUB={len(subs)} BAD={len(bad)}")
            
            # Update fault state based on sensor health
            if len(bad) >= 3:  # If at least 3 sensors are bad
                self.fault_conditions["sensor_failure"] = True
            else:
                self.fault_conditions["sensor_failure"] = False
            
            # PHASE 2: Update state machines
            self._update_finger_states()
            self._update_hand_state()
            
            # PHASE 3: Update finger positions
            self._update_finger_positions()
            
            # PHASE 4: Send motor commands
            self._send_motor_commands()
            
            # Collect sensor analysis data if enabled
            if self.analyzing_sensors:
                self._collect_sensor_data()
            
            # Update cycle time measurements
            cycle_end = time.time()
            self.last_cycle_time = cycle_end - cycle_start
            
            # Update average cycle time
            self.cycle_count += 1
            self.avg_cycle_time = ((self.cycle_count - 1) * self.avg_cycle_time + self.last_cycle_time) / self.cycle_count
            
            # Check for system overload - using control_interval directly
            # (1.5 times the expected cycle time is our threshold)
            if self.last_cycle_time > (1.5 * self.control_interval):
                self.fault_conditions["system_overload"] = True
            else:
                self.fault_conditions["system_overload"] = False
            
            # Log data if enabled
            self._log_data()
            
            return True
            
        except Exception as e:
            logger.error(f"Error in control cycle: {e}")
            return False
    
    def run(self):
        """Run the controller continuously until stopped"""
        if not self.running:
            self.start()
        
        try:
            logger.info("Starting main control loop")
            
            while self.running:
                cycle_start = time.time()
                
                # Run one control cycle
                success = self.run_one_cycle()
                
                if not success:
                    logger.warning("Control cycle failed")
                
                # Calculate sleep time to maintain control rate
                cycle_end = time.time()
                elapsed = cycle_end - cycle_start
                sleep_time = max(0, self.control_interval - elapsed)
                
                # Sleep to maintain control rate
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            logger.info("Controller interrupted by user")
        except Exception as e:
            logger.error(f"Error in main control loop: {e}")
        finally:
            # Make sure to stop the controller
            self.stop()
    
    def get_system_status(self):
        """
        Get the current system status.
        
        Returns:
            Status dictionary with hand, proximity, and debugging info
        """
        # Create status dictionary
        status = {
            "hand": {
                "hand_state": self.hand_fsm.state.name,
                "finger_states": {f: fsm.state.name for f, fsm in self.finger_fsm.items()},
            },
            "proximity": {s[0]: self.filtered_values.get(s[0]) for s in self.SENSORS},
            "positions": self.finger_positions.copy(),
            "faults": self.fault_conditions.copy(),
            "cycle_time": {
                "last": self.last_cycle_time,
                "avg": self.avg_cycle_time,
            },
            "debug": {
                "startup": {
                    "time_since_startup": time.time() - self.startup_time,
                    "startup_protection_active": (time.time() - self.startup_time) < self.startup_delay_seconds,
                },
                "hysteresis": {
                    "awaiting_reset": self.awaiting_reset.copy(),
                    "reset_threshold": self.reset_distance,
                },
                "timing": {
                    "min_times": {
                        "idle": self.min_time_in_idle,
                        "approach": self.min_time_in_approach,
                        "proportional": self.min_time_in_proportional,
                        "contact": self.min_time_in_contact,
                    },
                },
            },
        }
        
        # Add sensor analysis data if active
        if self.analyzing_sensors:
            status["debug"]["sensor_analysis"] = {
                "active": True,
                "duration": time.time() - self.sensor_analysis_start_time,
                "sample_counts": {f: len(samples) for f, samples in self.sensor_samples.items()},
            }
        
        return status
    
    def _log_data(self):
        """Log system data if logging is enabled"""
        if not self.enable_logging or not self.logger:
            return
        
        try:
            # Create log entry
            log_data = {
                "timestamp": time.time(),
                "hand_state": self.hand_fsm.state.name,
                "finger_states": {f: fsm.state.name for f, fsm in self.finger_fsm.items()},
                "finger_positions": self.finger_positions.copy(),
                "proximity_values": {s[0]: self.final_values.get(s[0]) for s in self.SENSORS},
                "raw_values": {s[0]: self.raw_values.get(s[0]) for s in self.SENSORS},
                "sensor_status": self.status.copy(),
                "cycle_time": self.last_cycle_time,
                "faults": self.fault_conditions.copy(),
                "substitutions": self.finger_substitutions.copy(),
            }
            
            # Add to log
            self.logger.log_data(log_data)
            
        except Exception as e:
            logger.error(f"Error logging data: {e}")
    
    def safety_reset(self):
        """Reset the hand to a safe open position"""
        try:
            logger.info("Performing safety reset - opening hand")
            
            # Open all fingers
            for finger in self.finger_positions:
                self.finger_positions[finger] = 0.0
                
                # Reset state machines
                if finger in self.finger_fsm:
                    self.finger_fsm[finger].force_state(FingerState.IDLE)
            
            # Reset hand state
            self.hand_fsm.force_state(HandState.IDLE)
            
            # Send open commands to all motors
            self._send_motor_commands()
            
            # Reset awaiting_reset flags
            self.awaiting_reset = {finger: False for finger in self.finger_fsm}
            
            logger.info("Safety reset completed")
            
        except Exception as e:
            logger.error(f"Error during safety reset: {e}")
    
    def start_sensor_analysis(self):
        """Start collecting sensor data for analysis"""
        self.analyzing_sensors = True
        self.sensor_analysis_start_time = time.time()
        self.sensor_samples = defaultdict(list)
        logger.info("Sensor analysis started")
    
    def stop_sensor_analysis(self):
        """Stop collecting sensor data"""
        self.analyzing_sensors = False
        logger.info("Sensor analysis stopped")
    
    def _collect_sensor_data(self):
        """Collect sensor data for analysis"""
        if not self.analyzing_sensors:
            return
            
        # Collect raw values for each sensor
        for name, mux, ch in self.SENSORS:
            if name in self.raw_values and self.raw_values[name] is not None:
                # Store sample as (timestamp, value)
                self.sensor_samples[name].append((time.time(), self.raw_values[name]))
    
    def get_sensor_analysis_report(self):
        """
        Generate a report of the collected sensor data.
        
        Returns:
            Analysis report dictionary
        """
        if not self.analyzing_sensors and not self.sensor_samples:
            return {"status": "No data available"}
        
        # Generate basic report
        report = {
            "status": "active" if self.analyzing_sensors else "complete",
            "duration": time.time() - self.sensor_analysis_start_time,
            "sample_counts": {f: len(samples) for f, samples in self.sensor_samples.items()},
            "findings": [],
        }
        
        # Only generate detailed analysis if we have enough data
        if sum(len(samples) for samples in self.sensor_samples.values()) < 100:
            report["findings"].append("Not enough data for detailed analysis")
            return report
        
        # Analyze correlations between MCP and PIP sensors
        correlations = {}
        for finger in ["Thumb", "Index", "Middle", "Ring", "Pinky"]:
            mcp_sensor = f"{finger[0]}1"  # e.g., "T1"
            pip_sensor = f"{finger[0]}2"  # e.g., "T2"
            
            if mcp_sensor in self.sensor_samples and pip_sensor in self.sensor_samples:
                # Check if we have enough matching samples
                mcp_samples = self.sensor_samples[mcp_sensor]
                pip_samples = self.sensor_samples[pip_sensor]
                
                if len(mcp_samples) >= 20 and len(pip_samples) >= 20:
                    # Extract values (simple approach - just use the values)
                    mcp_values = [v for _, v in mcp_samples]
                    pip_values = [v for _, v in pip_samples]
                    
                    # Trim to same length
                    min_len = min(len(mcp_values), len(pip_values))
                    mcp_values = mcp_values[:min_len]
                    pip_values = pip_values[:min_len]
                    
                    # Calculate correlation (simple approach)
                    try:
                        # Simple correlation calculation
                        mcp_mean = sum(mcp_values) / len(mcp_values)
                        pip_mean = sum(pip_values) / len(pip_values)
                        
                        mcp_diff = [v - mcp_mean for v in mcp_values]
                        pip_diff = [v - pip_mean for v in pip_values]
                        
                        cov = sum(m*p for m, p in zip(mcp_diff, pip_diff))
                        mcp_var = sum(m*m for m in mcp_diff)
                        pip_var = sum(p*p for p in pip_diff)
                        
                        if mcp_var > 0 and pip_var > 0:
                            correlation = cov / ((mcp_var * pip_var) ** 0.5)
                            correlations[finger] = correlation
                            
                            # Add findings based on correlation
                            if abs(correlation) > 0.7:
                                report["findings"].append(f"Strong correlation between {mcp_sensor} and {pip_sensor}: {correlation:.2f}")
                    except Exception as e:
                        logger.error(f"Error calculating correlation: {e}")
        
        # Add correlations to report
        report["correlations"] = correlations
        
        # Check for error patterns
        error_counts = {}
        for sensor in self.status:
            if self.status[sensor] == "BAD":
                error_counts[sensor] = error_counts.get(sensor, 0) + 1
        
        # Add findings about errors
        for sensor, count in error_counts.items():
            if count > 5:
                report["findings"].append(f"Sensor {sensor} has high error rate")
        
        return report

# Simple test function
def test():
    """Simple test function for the controller"""
    import signal
    
    # Create controller
    controller = SimplifiedController(
        control_rate=10,
        enable_logging=False,
        use_simulated_motors=True,
    )
    
    # Handle Ctrl+C
    def signal_handler(sig, frame):
        print("Stopping controller...")
        controller.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Start and run controller
    try:
        controller.start()
        controller.run()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        controller.stop()

if __name__ == "__main__":
    test()