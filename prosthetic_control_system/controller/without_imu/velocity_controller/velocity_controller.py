#!/usr/bin/env python3
"""
Velocity-Based Proximity Controller for Prosthetic Hand.

This module implements a controller that uses velocity control instead of position
control to create smoother, more natural finger movements. It maintains the same
state machine logic and I2C communication pattern as SimplifiedController but
differs in how it outputs commands to the motors.

Key differences from SimplifiedController:
1. Commands velocity targets instead of position targets
2. Implements velocity damping for smoother transitions
3. Tracks estimated finger positions internally
4. Performs periodic position recalibration
5. Provides motion inertia during sensor dropouts
"""

import time
import smbus2
import logging
import json
import os
import sys
import signal
import numpy as np
from enum import Enum
from typing import Dict, List, Optional, Tuple, Any
from collections import defaultdict, deque

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("VelocityController")

# Add parent directories to path if necessary
parent_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

# Local imports
from prosthetic_control_system.hand.motor_interface import MotorInterface, ControlMode, SimulatedMotorInterface
from prosthetic_control_system.hand.ability_hand_interface import AbilityHandInterface
from prosthetic_control_system.controller.state_machines import FingerState, HandState, FingerFSM, HandFSM
from prosthetic_control_system.utils.logger import DataLogger
from prosthetic_control_system.controller.without_imu.simplified_controller import SimplifiedController
from prosthetic_control_system.controller.without_imu.simplified_controller import mux_select, write_byte, read_byte, vl_init
from prosthetic_control_system.controller.without_imu.simplified_controller import start_range, clear_int, wait_ready, get_distance
from prosthetic_control_system.controller.without_imu.velocity_controller.config import DEFAULT_CONFIG, FINGER_MAPPING, MCP_SENSORS, FINGER_FALLBACK_MAP
from prosthetic_control_system.controller.without_imu.velocity_controller.enhanced_filtering import get_filtered_sensor_reading

class VelocityController(SimplifiedController):
    """
    Velocity-based proximity controller for prosthetic hand.
    
    This controller uses velocity commands instead of position commands for
    smoother motion. It inherits most of its functionality from SimplifiedController
    but replaces the position-based output with velocity-based output.
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
                 velocity_scale: float = DEFAULT_CONFIG["velocity_scale"],
                 velocity_damping: float = DEFAULT_CONFIG["velocity_damping"],
                 max_closing_velocity: float = DEFAULT_CONFIG["max_closing_velocity"],
                 max_opening_velocity: float = DEFAULT_CONFIG["max_opening_velocity"],
                 idle_return_factor: float = DEFAULT_CONFIG["idle_return_factor"],
                 position_recalibration_interval: float = DEFAULT_CONFIG["position_recalibration_interval"],
                 verbose_logging: bool = DEFAULT_CONFIG["verbose_logging"]):
        """
        Initialize the velocity-based proximity controller.
        
        Args:
            control_rate: Control loop frequency in Hz
            enable_logging: Whether to enable data logging
            log_dir: Directory for log files
            use_simulated_motors: Whether to use simulated motors
            motor_interface_kwargs: Additional kwargs for motor interface
            approach_threshold: Distance threshold for approach phase (mm)
            contact_threshold: Distance threshold for contact phase (mm)
            consecutive_readings_required: Number of consecutive readings needed for state change
            reset_distance: Distance to reset to after contact
            startup_delay: Delay after startup before enabling control
            velocity_scale: Scaling factor for velocity commands
            velocity_damping: Damping factor for velocity changes (0.0-1.0)
            max_closing_velocity: Maximum closing velocity (deg/sec)
            max_opening_velocity: Maximum opening velocity (deg/sec)
            idle_return_factor: Factor for return-to-open velocity when idle
            position_recalibration_interval: Seconds between position recalibrations
            verbose_logging: Whether to enable verbose logging
        """
        # Call the parent constructor with the base parameters
        # We skip setting finger_positions since we'll handle velocity instead
        super().__init__(
            control_rate=control_rate,
            enable_logging=enable_logging,
            log_dir=log_dir,
            use_simulated_motors=use_simulated_motors,
            motor_interface_kwargs=motor_interface_kwargs,
            approach_threshold=approach_threshold,
            contact_threshold=contact_threshold,
            consecutive_readings_required=consecutive_readings_required,
            reset_distance=reset_distance,
            startup_delay=startup_delay,
            verbose_logging=verbose_logging
        )
        
        # Velocity control parameters
        self.velocity_scale = velocity_scale
        self.velocity_damping = velocity_damping
        self.max_closing_velocity = max_closing_velocity
        self.max_opening_velocity = max_opening_velocity
        self.idle_return_factor = idle_return_factor
        self.position_recalibration_interval = position_recalibration_interval
        
        # Initialize velocity state
        self.finger_velocities = {
            "Thumb": 0.0,
            "Index": 0.0,
            "Middle": 0.0,
            "Ring": 0.0,
            "Pinky": 0.0,
        }
        
        # Estimated position tracking
        self.estimated_positions = {
            "Thumb": 0.0,
            "Index": 0.0,
            "Middle": 0.0,
            "Ring": 0.0,
            "Pinky": 0.0,
        }
        
        # Enhanced distance filtering
        self.filtered_distances = {finger: 100.0 for finger in self.finger_fsm}
        self.distance_filter_alpha = DEFAULT_CONFIG["distance_filter_alpha"]
        self.consecutive_readings = {finger: 0 for finger in self.finger_fsm}
        self.consecutive_readings_threshold = DEFAULT_CONFIG["consecutive_readings_threshold"]
        self.last_distance_readings = {finger: [] for finger in self.finger_fsm}
        # Keep history of 3 readings for each finger
        self.distance_history_size = 3
        
        # Finger state velocity maps
        self.finger_state_velocities = {
            FingerState.IDLE: DEFAULT_CONFIG["finger_velocities"]["idle"],
            FingerState.APPROACH: DEFAULT_CONFIG["finger_velocities"]["approach"],
            FingerState.PROPORTIONAL: DEFAULT_CONFIG["finger_velocities"]["proportional_max"],
            FingerState.CONTACT: DEFAULT_CONFIG["finger_velocities"]["contact"],
        }
        
        # Timing tracking
        self.last_position_recalibration = time.time()
        self.in_recalibration = False
        self.recalibration_finger = None
        
        # Feedback tracking
        self.last_read_positions = {finger: 0.0 for finger in self.finger_fsm}

        # Log controller type
        logger.info("Initialized velocity-based controller")
    
    def _normalize_distance(self, distance, min_dist=None, max_dist=None):
        """Normalize distance to 0-1 range for proportional control"""
        if min_dist is None:
            min_dist = self.contact_threshold
        if max_dist is None:
            max_dist = self.approach_threshold
            
        # Clamp distance to valid range
        distance = max(min_dist, min(max_dist, distance))
        
        # Normalize to 0-1 range (0 = contact, 1 = approach threshold)
        range_size = max_dist - min_dist
        if range_size > 0:
            return (distance - min_dist) / range_size
        else:
            # Safety fallback if thresholds are invalid
            return 0.5
    
    def _update_finger_velocities(self):
        """
        Update finger velocities based on state and proximity values.
        
        This replaces the _update_finger_positions method from SimplifiedController
        with a velocity-based approach.
        """
        # Startup protection
        time_since_startup = time.time() - self.startup_time
        startup_protection_active = time_since_startup < self.startup_delay_seconds
        
        # If startup protection is active, keep all velocities at 0
        if startup_protection_active:
            for finger in self.finger_velocities:
                self.finger_velocities[finger] = 0.0
            return
        
        # Time since last update for velocity calculations
        dt = self.last_cycle_time if self.last_cycle_time > 0 else 1.0 / self.control_rate
        
        # Process each finger
        for finger, fsm in self.finger_fsm.items():
            # Current velocity
            current_vel = self.finger_velocities[finger]
            
            # Calculate target velocity based on state
            if fsm.state == FingerState.IDLE:
                # Idle state - gradually return to open position
                # Negative velocity opens the finger
                target_vel = self.finger_state_velocities[FingerState.IDLE] * self.idle_return_factor
                
                # If already near open position, reduce velocity to prevent oscillation
                if self.estimated_positions[finger] < 5.0:
                    target_vel *= self.estimated_positions[finger] / 5.0
                
            elif fsm.state == FingerState.CONTACT:
                # Contact state - slow closing motion
                target_vel = self.finger_state_velocities[FingerState.CONTACT]
                
                # Ensure we don't exceed position limits
                if self.estimated_positions[finger] >= self.max_finger_angles[finger] * 0.95:
                    target_vel = 0.0  # Stop if near maximum position
                
            elif fsm.state == FingerState.PROPORTIONAL:
                # Proportional state - velocity based on proximity
                primary_sensor = f"{finger[0]}1"  # e.g., "T1" for Thumb
                
                # Use enhanced filtering for smoother distance values
                distance, sensor_used = get_filtered_sensor_reading(finger, primary_sensor, self)
                
                # Normalize distance to 0-1
                normalized_dist = self._normalize_distance(distance)
                
                # Inverse relationship: closer = faster closing
                # 0 = contact threshold, 1 = approach threshold
                closing_factor = 1.0 - normalized_dist
                
                # Scale to get velocity value within narrow range
                # Use minimum and maximum values to keep velocities in a tight band
                min_vel = DEFAULT_CONFIG["finger_velocities"]["proportional_min"]
                max_vel = DEFAULT_CONFIG["finger_velocities"]["proportional_max"]
                
                # Map closing factor to velocity range
                target_vel = min_vel + (closing_factor * (max_vel - min_vel))
                
                # Only change velocity if we have enough consecutive similar readings
                # This prevents jitter from noise in the sensor readings
                if self.consecutive_readings[finger] < self.consecutive_readings_threshold:
                    # Not enough consistent readings, use damped current velocity
                    target_vel = current_vel * 0.98  # Slight damping to gradually slow down
                
                # Ensure we don't exceed position limits
                if self.estimated_positions[finger] >= self.max_finger_angles[finger] * 0.95:
                    target_vel = min(target_vel, 0.0)  # Only allow negative (opening) velocity
                
            elif fsm.state == FingerState.APPROACH:
                # Approach state - very gentle constant velocity
                target_vel = self.finger_state_velocities[FingerState.APPROACH]
                
                # Apply extra damping in APPROACH state to make motion extra smooth
                # Particularly important in this intermediate state
                if abs(current_vel - target_vel) > 2.0:
                    # For large velocity changes, use additional smoothing
                    target_vel = current_vel + (0.05 * (target_vel - current_vel))
                
                # Ensure we don't exceed position limits
                if self.estimated_positions[finger] >= self.max_finger_angles[finger] * 0.7:
                    target_vel *= 0.3  # Significantly reduce velocity when approaching limits
            
            else:
                # Unknown state, maintain velocity but apply damping
                target_vel = current_vel * 0.9
            
            # Scale target velocity
            target_vel = target_vel * self.velocity_scale / 100.0
            
            # Apply velocity damping for smooth transitions
            if self.velocity_damping > 0:
                # Calculate damped velocity
                new_vel = (current_vel * self.velocity_damping) + (target_vel * (1.0 - self.velocity_damping))
            else:
                # No damping
                new_vel = target_vel
            
            # Apply velocity limits
            if new_vel > 0:  # Closing velocity
                new_vel = min(new_vel, self.max_closing_velocity)
            else:  # Opening velocity
                new_vel = max(new_vel, -self.max_opening_velocity)
            
            # Update velocity
            self.finger_velocities[finger] = new_vel
            
            # Update estimated position based on velocity
            self.estimated_positions[finger] += new_vel * dt
            
            # Ensure estimated position stays within valid bounds
            self.estimated_positions[finger] = max(0.0, min(self.estimated_positions[finger], 
                                                          self.max_finger_angles[finger]))
    
    def _update_estimated_positions(self):
        """Update estimated positions based on real position feedback"""
        # Get actual position feedback from hand
        for finger in self.finger_fsm:
            # Map finger name to motor name
            motor = FINGER_MAPPING.get(finger, finger)
            
            # Get position feedback
            try:
                position = self.hand.get_position(motor)
                if position is not None:
                    self.last_read_positions[finger] = position
            except Exception as e:
                logger.warning(f"Error getting position feedback for {finger}: {e}")
    
    def _check_position_recalibration(self):
        """Check if we need to recalibrate position"""
        current_time = time.time()
        
        # If in recalibration, check if it's complete
        if self.in_recalibration:
            if self.recalibration_finger is None:
                # Recalibration complete
                self.in_recalibration = False
                logger.debug("Position recalibration complete")
                return
            
            # Continue recalibration for current finger
            self._recalibrate_finger_position(self.recalibration_finger)
            
            # Move to next finger or complete
            fingers = list(self.finger_fsm.keys())
            try:
                idx = fingers.index(self.recalibration_finger)
                if idx < len(fingers) - 1:
                    self.recalibration_finger = fingers[idx + 1]
                else:
                    self.recalibration_finger = None
            except ValueError:
                self.recalibration_finger = None
                
            return
        
        # Check if it's time for recalibration
        if current_time - self.last_position_recalibration >= self.position_recalibration_interval:
            # Start recalibration
            self.in_recalibration = True
            self.recalibration_finger = list(self.finger_fsm.keys())[0]  # Start with first finger
            self.last_position_recalibration = current_time
            logger.debug("Starting position recalibration")
    
    def _recalibrate_finger_position(self, finger):
        """Recalibrate position for a single finger"""
        # Get estimated and actual positions
        estimated = self.estimated_positions[finger]
        actual = self.last_read_positions[finger]
        
        # Calculate error
        error = abs(estimated - actual)
        logger.debug(f"Position error for {finger}: {error:.1f}° (estimated: {estimated:.1f}°, actual: {actual:.1f}°)")
        
        # If error is significant, update estimated position
        if error > DEFAULT_CONFIG["position_error_threshold"]:
            self.estimated_positions[finger] = actual
            logger.debug(f"Recalibrated {finger} position: {estimated:.1f}° -> {actual:.1f}°")
    
    def _send_motor_commands(self):
        """Send velocity commands to the motors"""
        try:
            # Check if we're in recalibration mode
            if self.in_recalibration and self.recalibration_finger is not None:
                # During recalibration, send position commands for stability
                motor = FINGER_MAPPING.get(self.recalibration_finger, self.recalibration_finger)
                position = self.estimated_positions[self.recalibration_finger]
                self.hand.set_position(motor, position)
                logger.debug(f"Sent position command for {self.recalibration_finger}: {position:.1f}°")
                return
            
            # Send velocity commands for each finger
            for finger, velocity in self.finger_velocities.items():
                # Map finger name to motor name
                motor = FINGER_MAPPING.get(finger, finger)
                
                # Send command to motor
                self.hand.set_velocity(motor, velocity)
            
            # Execute commands
            self.hand.update()
            
        except Exception as e:
            logger.error(f"Error sending motor commands: {e}")
            self.fault_conditions["hand_timeout"] = True
    
    def run_one_cycle(self):
        """
        Run a single control cycle with velocity-based approach.
        
        This overrides SimplifiedController's run_one_cycle to add
        velocity-specific logic.
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
            
            # PHASE 3: Update finger velocities instead of positions
            self._update_finger_velocities()
            
            # PHASE 4: Check for position recalibration
            self._update_estimated_positions()
            self._check_position_recalibration()
            
            # PHASE 5: Send motor commands
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
    
    def get_system_status(self):
        """
        Get the current system status.
        
        Extends SimplifiedController's get_system_status with velocity info.
        """
        # Get base status from parent
        status = super().get_system_status()
        
        # Add velocity-specific information
        status["velocities"] = self.finger_velocities.copy()
        status["estimated_positions"] = self.estimated_positions.copy()
        status["actual_positions"] = self.last_read_positions.copy()
        
        # Add calibration status
        status["calibration"] = {
            "in_progress": self.in_recalibration,
            "current_finger": self.recalibration_finger,
            "time_since_last": time.time() - self.last_position_recalibration,
            "interval": self.position_recalibration_interval,
        }
        
        # Add velocity control parameters
        status["debug"]["velocity_control"] = {
            "damping": self.velocity_damping,
            "max_closing_velocity": self.max_closing_velocity,
            "max_opening_velocity": self.max_opening_velocity,
            "idle_return_factor": self.idle_return_factor,
        }
        
        return status
    
    def _log_data(self):
        """Log system data if logging is enabled"""
        if not self.enable_logging or not self.logger:
            return
        
        try:
            # Create log entry (extending parent class log data)
            log_data = {
                "timestamp": time.time(),
                "hand_state": self.hand_fsm.state.name,
                "finger_states": {f: fsm.state.name for f, fsm in self.finger_fsm.items()},
                "finger_velocities": self.finger_velocities.copy(),
                "estimated_positions": self.estimated_positions.copy(),
                "actual_positions": self.last_read_positions.copy(),
                "position_error": {f: abs(self.estimated_positions[f] - self.last_read_positions[f]) 
                                 for f in self.finger_fsm},
                "proximity_values": {s[0]: self.final_values.get(s[0]) for s in self.SENSORS},
                "raw_values": {s[0]: self.raw_values.get(s[0]) for s in self.SENSORS},
                "sensor_status": self.status.copy(),
                "cycle_time": self.last_cycle_time,
                "faults": self.fault_conditions.copy(),
                "substitutions": self.finger_substitutions.copy(),
                "calibration": {
                    "in_progress": self.in_recalibration,
                    "current_finger": self.recalibration_finger,
                },
            }
            
            # Add to log
            self.logger.log_data(log_data)
            
        except Exception as e:
            logger.error(f"Error logging data: {e}")
    
    def safety_reset(self):
        """Reset the hand to a safe open position"""
        try:
            logger.info("Performing safety reset - opening hand")
            
            # Stop all velocity
            for finger in self.finger_velocities:
                self.finger_velocities[finger] = 0.0
            
            # Reset estimated positions
            for finger in self.estimated_positions:
                self.estimated_positions[finger] = 0.0
                
            # Reset state machines
            for finger in self.finger_fsm:
                self.finger_fsm[finger].force_state(FingerState.IDLE)
            
            # Reset hand state
            self.hand_fsm.force_state(HandState.IDLE)
            
            # Force position command to ensure reset
            for finger in self.finger_fsm:
                motor = FINGER_MAPPING.get(finger, finger)
                self.hand.set_position(motor, 0.0)
            
            # Reset awaiting_reset flags
            self.awaiting_reset = {finger: False for finger in self.finger_fsm}
            
            # Reset recalibration
            self.in_recalibration = False
            self.recalibration_finger = None
            
            logger.info("Safety reset completed")
            
        except Exception as e:
            logger.error(f"Error during safety reset: {e}")

# Simple test function
def test():
    """Simple test function for the controller"""
    import signal
    
    # Create controller
    controller = VelocityController(
        control_rate=20,
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