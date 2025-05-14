#!/usr/bin/env python3
"""
Proximity-Only Controller for Prosthetic Hand.

This module implements a simplified controller that uses only proximity sensors
for hand control, without relying on IMU data.
"""

import time
import threading
import logging
import json
import os
import sys
from enum import Enum
from typing import Dict, List, Optional, Tuple, Any

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("ProximityController")

# Add parent directories to path if necessary
parent_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

# Local imports
from prosthetic_control_system.proximity.proximity_manager import ProximityManager
from prosthetic_control_system.hand.motor_interface import MotorInterface, ControlMode, SimulatedMotorInterface
from prosthetic_control_system.hand.ability_hand_interface import AbilityHandInterface
from prosthetic_control_system.controller.state_machines import FingerState, HandState, FingerFSM, HandFSM
from prosthetic_control_system.utils.logger import DataLogger
from prosthetic_control_system.controller.without_imu.config import DEFAULT_CONFIG, FINGER_MAPPING, MCP_SENSORS

# Mock IMU MotionState for compatibility with existing code
class MotionState(Enum):
    """Motion states for IMU-free operation."""
    STATIC = 0
    MOVING = 1
    IMPACT = 2
    UNKNOWN = 3

class ProximityController:
    """
    Proximity-only controller for prosthetic hand.
    
    This controller uses only proximity sensing to control the hand,
    without relying on IMU data.
    """
    
    def __init__(self, 
                 control_rate: int = DEFAULT_CONFIG["control_rate"], 
                 enable_logging: bool = DEFAULT_CONFIG["enable_logging"],
                 log_dir: str = DEFAULT_CONFIG["log_dir"],
                 use_simulated_motors: bool = DEFAULT_CONFIG["use_simulated_motors"],
                 motor_interface_kwargs: dict = None,
                 proximity_sampling_rate: int = None,
                 approach_threshold: int = DEFAULT_CONFIG["approach_threshold"],
                 contact_threshold: int = DEFAULT_CONFIG["contact_threshold"],
                 consecutive_readings_required: int = DEFAULT_CONFIG["consecutive_readings_required"],
                 reset_distance: int = DEFAULT_CONFIG["reset_distance"],
                 startup_delay: float = DEFAULT_CONFIG["startup_delay"],
                 min_time_in_idle: float = DEFAULT_CONFIG["min_time_in_idle"],
                 min_time_in_approach: float = DEFAULT_CONFIG["min_time_in_approach"],
                 min_time_in_proportional: float = DEFAULT_CONFIG["min_time_in_proportional"],
                 min_time_in_contact: float = DEFAULT_CONFIG["min_time_in_contact"],
                 verbose_logging: bool = DEFAULT_CONFIG["verbose_logging"]):
        """
        Initialize the proximity controller.
        
        Args:
            control_rate: Control loop frequency in Hz
            enable_logging: Whether to enable data logging
            log_dir: Directory for log files (default: ~/prosthetic_logs)
            use_simulated_motors: Whether to use simulated motors
            motor_interface_kwargs: Additional kwargs for motor interface
            proximity_sampling_rate: Override for proximity sensor sampling rate
            approach_threshold: Distance threshold for approach phase (mm)
            contact_threshold: Distance threshold for contact phase (mm)
        """
        # Save parameters
        self.control_rate = control_rate
        self.control_interval = 1.0 / control_rate
        self.enable_logging = enable_logging
        self.approach_threshold = approach_threshold
        self.contact_threshold = contact_threshold
        self.consecutive_readings_required = consecutive_readings_required
        self.reset_distance = reset_distance
        self.startup_delay_seconds = startup_delay
        
        # State transition timing constraints
        self.min_time_in_idle = min_time_in_idle
        self.min_time_in_approach = min_time_in_approach
        self.min_time_in_proportional = min_time_in_proportional
        self.min_time_in_contact = min_time_in_contact
        
        # Set up enhanced logging if verbose mode is enabled
        self.verbose_logging = verbose_logging
        if verbose_logging:
            logger.setLevel(logging.DEBUG)
            logger.debug("Verbose logging enabled")
        
        # Set up logging
        if log_dir is None:
            self.log_dir = os.path.expanduser("~/prosthetic_logs")
        else:
            self.log_dir = log_dir
            
        if self.enable_logging:
            os.makedirs(self.log_dir, exist_ok=True)
        
        # Set up proximity sampling rate (fixed at 5Hz by default for reliable readings)
        if proximity_sampling_rate is None:
            # Hard-coded to 5Hz which matches fallback_test.py and is known to work reliably
            self.proximity_sampling_rate = 5
            logger.info(f"Using fixed 5Hz proximity sampling rate for reliable readings")
        else:
            self.proximity_sampling_rate = proximity_sampling_rate
            logger.info(f"Using custom proximity sampling rate: {proximity_sampling_rate}Hz")
        
        # Initialize proximity manager with custom thresholds and correct sensor config
        logger.info("Initializing proximity manager...")
        try:
            # Import sensors from config
            from prosthetic_control_system.controller.without_imu.config import DEFAULT_SENSORS
            
            # Create instance with explicit sensor configuration
            self.proximity = ProximityManager(
                sampling_rate=self.proximity_sampling_rate,
                approach_threshold=self.approach_threshold,
                contact_threshold=self.contact_threshold,
                sensors=DEFAULT_SENSORS  # Explicitly pass the correct sensor configuration
            )
            logger.info(f"Proximity manager initialized with {len(DEFAULT_SENSORS)} sensors")
            
        except Exception as e:
            logger.error(f"Error configuring proximity manager with custom sensors: {e}")
            logger.info("Falling back to default sensor configuration")
            
            # Fallback to default config
            self.proximity = ProximityManager(
                sampling_rate=self.proximity_sampling_rate,
                approach_threshold=self.approach_threshold,
                contact_threshold=self.contact_threshold
            )
        
        # Initialize motor interface
        if motor_interface_kwargs is None:
            motor_interface_kwargs = {}
            
        motor_interface_kwargs['control_rate'] = control_rate
        
        if use_simulated_motors:
            logger.info("Using simulated motors")
            self.motors = SimulatedMotorInterface(**motor_interface_kwargs)
        else:
            try:
                logger.info("Initializing Ability Hand interface...")
                self.motors = AbilityHandInterface(**motor_interface_kwargs)
            except Exception as e:
                logger.error(f"Error initializing Ability Hand interface: {e}")
                logger.info("Falling back to simulated motors")
                self.motors = SimulatedMotorInterface(**motor_interface_kwargs)
        
        # Finger mapping from proximity sensors to motors
        self.finger_mapping = FINGER_MAPPING
        
        # Create finger FSMs
        logger.info("Initializing finger state machines...")
        self.finger_fsms = {}
        self.state_transitions = {}
        self.state_entry_times = {}
        
        for sensor_name, finger_name in self.finger_mapping.items():
            self.finger_fsms[finger_name] = FingerFSM(finger_name)
            # Explicitly set initial state to IDLE
            self.finger_fsms[finger_name].state = FingerState.IDLE
            
            # Initialize state tracking for timing constraints
            self.state_transitions[finger_name] = {}
            self.state_entry_times[finger_name] = {state: 0.0 for state in FingerState}
            self.state_entry_times[finger_name][FingerState.IDLE] = time.time()
        
        # Create hand FSM
        logger.info("Initializing hand state machine...")
        self.hand_fsm = HandFSM()
        # Explicitly set initial state to IDLE
        self.hand_fsm.state = HandState.IDLE
        
        # Initialize data logger
        if self.enable_logging:
            logger.info("Initializing data logger...")
            self.data_logger = DataLogger(
                log_dir=self.log_dir,
                log_rate=control_rate
            )
        else:
            self.data_logger = None
        
        # Thread control
        self.running = False
        self.shutdown_event = threading.Event()  # Event to signal shutdown to all components
        self.thread = None
        
        # Performance monitoring
        self.cycle_times = []
        self.max_cycle_time = 0.0
        self.last_cycle_time = 0.0
        
        # Fault handling
        self.fault_status = {
            "loop_overrun": False,
            "comm_loss": False,
            "sensor_failure": False
        }
        
        # Startup protection - prevent immediate closure on startup
        self.startup_time = None  # Will be set when controller starts
        self.startup_delay_seconds = 2.0  # Wait 2 seconds before allowing fingers to close
        
        # Hysteresis tracking - require objects to move away before allowing new closure
        self.last_distances = {finger: 100.0 for finger in self.finger_mapping.values()}
        self.distance_history = {finger: [100.0] * self.consecutive_readings_required 
                              for finger in self.finger_mapping.values()}
        self.awaiting_reset = {finger: False for finger in self.finger_mapping.values()}
        self.reset_threshold_mm = reset_distance  # Object must move beyond this distance to reset hysteresis
        
        # Shutdown status
        self.is_shutting_down = False
        
        logger.info("Proximity controller initialized")
    
    def _process_finger(self, sensor_name, finger_name):
        """Process control for a single finger"""
        # Get the finger FSM
        fsm = self.finger_fsms[finger_name]
        
        try:
            # Get sensor distance with error handling
            distance, status = self.proximity.get_sensor_value(
                sensor_name, filtered=True, with_status=True
            )
            
            # Safety check - if sensor reading is invalid, default to safe distance
            if distance is None or status == "BAD":
                logger.debug(f"Invalid sensor reading for {sensor_name}, defaulting to safe distance")
                # Default to a safe value that won't cause closure
                distance = 100
                status = "DEFAULT"
                
            # Track last distance for hysteresis
            last_distance = self.last_distances.get(finger_name, 100.0)
            self.last_distances[finger_name] = distance
            
            # Update distance history for filtering
            if finger_name in self.distance_history:
                # Shift history and add new reading
                history = self.distance_history[finger_name]
                history.pop(0)  # Remove oldest reading
                history.append(distance)  # Add newest reading
                
                # Check if we have enough consecutive readings below threshold
                consecutive_below_approach = all(d <= self.approach_threshold for d in history)
                consecutive_below_contact = all(d <= self.contact_threshold for d in history)
            else:
                # Initialize new history if not present
                self.distance_history[finger_name] = [distance] * self.consecutive_readings_required
                consecutive_below_approach = False
                consecutive_below_contact = False
                
            # Log filtered distance values
            logger.debug(f"{finger_name} distance history: {self.distance_history[finger_name]}, "
                      f"below_approach: {consecutive_below_approach}, below_contact: {consecutive_below_contact}")
            
            # Apply startup delay protection - forces IDLE state during startup period
            startup_protection_active = False
            if self.startup_time is not None:
                time_since_startup = time.time() - self.startup_time
                if time_since_startup < self.startup_delay_seconds:
                    startup_protection_active = True
                    logger.debug(f"Startup protection active for {finger_name}: {self.startup_delay_seconds - time_since_startup:.1f}s remaining")
                    
            # Apply hysteresis - if awaiting reset, check if object has moved away far enough
            if self.awaiting_reset.get(finger_name, False):
                # Use consecutive readings for reliable reset detection
                consecutive_readings_above_reset = all(d >= self.reset_threshold_mm 
                                                     for d in self.distance_history[finger_name])
                if consecutive_readings_above_reset:
                    # Object moved away reliably, allow new closure
                    self.awaiting_reset[finger_name] = False
                    logger.info(f"{finger_name} reset - object moved away to {distance:.1f}mm")
                else:
                    # Still waiting for object to move away - force IDLE state
                    logger.debug(f"{finger_name} waiting for reset - object at {distance:.1f}mm, need {self.reset_threshold_mm}mm")
            
            # Override distance if startup protection is active
            original_distance = distance
            if startup_protection_active:
                # Force a safe distance during startup period
                distance = 100
                
            # Override distance if awaiting reset and object is still too close
            if self.awaiting_reset.get(finger_name, False):
                # Force a safe distance while waiting for object to move away
                distance = 100
                
            # Get motor feedback with error handling
            try:
                current = self.motors.get_current(finger_name)
                position = self.motors.get_position(finger_name)
            except Exception as e:
                logger.warning(f"Error getting motor feedback for {finger_name}: {e}")
                # Default to safe values
                current = 0.0
                position = 0.0
            
            # Get current derivative for slip detection
            current_derivative = 0.0
            if hasattr(self.motors, 'get_current_derivative'):
                try:
                    current_derivative = self.motors.get_current_derivative(finger_name)
                except Exception as e:
                    logger.debug(f"Error getting current derivative: {e}")
            
            # Determine filtered distance based on consecutive readings
            # Only consider object detected if we have multiple consistent readings
            filtered_distance = distance
            if not consecutive_below_approach and distance <= self.approach_threshold:
                # If not enough consecutive readings below threshold but current reading is,
                # override to a safe distance to prevent premature finger movement
                filtered_distance = self.approach_threshold + 1
                logger.debug(f"{finger_name} filtered: overriding {distance:.1f}mm to {filtered_distance:.1f}mm (need {self.consecutive_readings_required} consecutive readings)")
            
            # Get the current state and how long we've been in it
            current_state = fsm.state
            current_time = time.time()
            state_entry_time = self.state_entry_times[finger_name].get(current_state, 0.0)
            time_in_state = current_time - state_entry_time
            
            # Check if we need to enforce minimum time in current state
            enforce_timing = False
            min_time_required = 0.0
            
            if current_state == FingerState.IDLE:
                min_time_required = self.min_time_in_idle
                enforce_timing = time_in_state < min_time_required
            elif current_state == FingerState.APPROACH:
                min_time_required = self.min_time_in_approach
                enforce_timing = time_in_state < min_time_required
            elif current_state == FingerState.PROPORTIONAL:
                min_time_required = self.min_time_in_proportional
                enforce_timing = time_in_state < min_time_required
            elif current_state == FingerState.CONTACT:
                min_time_required = self.min_time_in_contact
                enforce_timing = time_in_state < min_time_required
            
            # Log timing enforcement if applicable
            if enforce_timing and self.verbose_logging:
                logger.debug(f"{finger_name} enforcing min time in {current_state.name}: "
                           f"{time_in_state:.3f}s/{min_time_required:.3f}s")
                
            # Temporarily adjust distance if we need to prevent state transitions
            temp_distance = filtered_distance
            if enforce_timing:
                # Modify temp_distance to stay in the current state
                if current_state == FingerState.IDLE:
                    # Keep in IDLE by making object seem far away
                    temp_distance = self.approach_threshold + 10
                elif current_state == FingerState.APPROACH:
                    # Keep in APPROACH by making object seem at approach threshold
                    temp_distance = self.approach_threshold - 1
                elif current_state == FingerState.PROPORTIONAL:
                    # Keep in PROPORTIONAL by making object seem not quite at contact
                    temp_distance = self.contact_threshold + 1
                elif current_state == FingerState.CONTACT:
                    # Keep in CONTACT by making object seem at contact
                    temp_distance = self.contact_threshold - 1
            
            # Update FSM with error handling - use modified distance if enforcing timing
            try:
                state, position_target, torque_target = fsm.update(
                    temp_distance if enforce_timing else filtered_distance, 
                    current, current_derivative, position
                )
                
                # Check if state changed and update timing info
                if state != current_state:
                    # Only allow transitions if not enforcing timing
                    if not enforce_timing:
                        logger.info(f"{finger_name} state transition: {current_state.name} -> {state.name} "
                                   f"after {time_in_state:.3f}s (object at {filtered_distance:.1f}mm)")
                        # Record the transition time
                        self.state_entry_times[finger_name][state] = current_time
                    else:
                        # Stay in current state if enforcing timing
                        logger.debug(f"{finger_name} prevented transition {current_state.name} -> {state.name} "
                                    f"(enforcing min time {min_time_required:.3f}s)")
                        state = current_state
            except Exception as e:
                logger.error(f"Error updating FSM for {finger_name}: {e}")
                # Default to IDLE state for safety
                state = FingerState.IDLE
                position_target = 0.0
                torque_target = 0.0
            
            # Check for transitions to PROPORTIONAL or CONTACT - enable hysteresis tracking
            prev_state = fsm.state  # This is the state before the update
            
            # If transitioning to PROPORTIONAL or CONTACT, mark for future hysteresis
            if (prev_state in [FingerState.IDLE, FingerState.APPROACH] and 
                state in [FingerState.PROPORTIONAL, FingerState.CONTACT]):
                # Only set awaiting_reset if this is a genuine closure (not during startup/reset periods)
                if not startup_protection_active and not self.awaiting_reset.get(finger_name, False):
                    logger.info(f"{finger_name} will need reset - transitioned to {state.name} at {original_distance:.1f}mm")
                    self.awaiting_reset[finger_name] = True
            
            # Apply control based on state with error handling
            try:
                if state == FingerState.IDLE or state == FingerState.APPROACH:
                    # Position control with open hand
                    self.motors.set_position(finger_name, 0.0)
                    
                elif state == FingerState.PROPORTIONAL:
                    # Position control with proportional position
                    self.motors.set_position(finger_name, position_target)
                    
                elif state == FingerState.CONTACT:
                    # Torque control
                    self.motors.set_torque(finger_name, torque_target)
            except Exception as e:
                logger.error(f"Error applying control to {finger_name}: {e}")
                # Try to set to safe position
                try:
                    self.motors.set_position(finger_name, 0.0)
                except:
                    pass  # Already tried our best
                
            # Calculate time in current state
            current_time = time.time()
            state_entry_time = self.state_entry_times[finger_name].get(state, current_time)
            time_in_state = current_time - state_entry_time
            
            # Return state information for logging/monitoring
            return {
                "state": state.name,
                "distance": original_distance,  # Report actual distance from sensor
                "modified_distance": distance,  # Distance after safety overrides
                "filtered_distance": filtered_distance,  # Distance after filtering
                "sensor_status": status,
                "position": position,
                "position_target": position_target,
                "current": current,
                "torque_target": torque_target,
                "current_derivative": current_derivative,
                "awaiting_reset": self.awaiting_reset.get(finger_name, False),
                "startup_protection": startup_protection_active,
                "consecutive_below_approach": consecutive_below_approach,
                "consecutive_below_contact": consecutive_below_contact,
                "time_in_state": time_in_state,
                "enforced_timing": enforce_timing if 'enforce_timing' in locals() else False
            }
            
        except Exception as e:
            # Catch-all error handler
            logger.error(f"Unhandled error in _process_finger for {finger_name}: {e}")
            # Return safe default values
            return {
                "state": "ERROR",
                "distance": 100,  # Far distance
                "sensor_status": "ERROR",
                "position": 0.0,
                "position_target": 0.0,
                "current": 0.0,
                "torque_target": 0.0,
                "current_derivative": 0.0
            }
    
    def _update_hand_state(self, finger_states):
        """Update hand state machine based on finger states"""
        # Count fingers in each state
        state_counts = {state: 0 for state in FingerState}
        for finger, data in finger_states.items():
            state = FingerState[data["state"]]
            state_counts[state] += 1
            
        # Create finger currents dictionary from finger_states data
        finger_currents = {
            finger: data["current"] 
            for finger, data in finger_states.items()
        }
        
        # Create simulated IMU parameters since we don't have IMU
        is_lowering = False        # Without IMU, assume never lowering
        is_impact_detected = False # Without IMU, assume no impacts
        is_stationary = True       # Without IMU, assume always stationary
        time_since_impact = float('inf')  # No impacts, so infinite time
        
        # Update hand FSM with required parameters
        try:
            hand_state = self.hand_fsm.update(
                state_counts,      # Finger states dictionary
                finger_currents,   # Finger currents dictionary
                is_lowering,       # Simulated IMU parameter
                is_impact_detected, # Simulated IMU parameter
                is_stationary,     # Simulated IMU parameter
                time_since_impact  # Simulated IMU parameter
            )
            return hand_state
            
        except Exception as e:
            logger.error(f"Error updating hand state: {e}")
            # Keep current state in case of error
            return self.hand_fsm.state
    
    def _control_loop(self):
        """Main control loop that runs at the specified rate"""
        next_update_time = time.time()
        
        while self.running and not self.shutdown_event.is_set():
            # Check for shutdown at loop start
            if self.is_shutting_down or self.shutdown_event.is_set():
                logger.info("Control loop detected shutdown request, exiting...")
                break
                
            start_time = time.time()
            
            # Check if it's time for the next control update
            if start_time >= next_update_time:
                try:
                    # Check for shutdown again before main processing
                    if self.is_shutting_down or self.shutdown_event.is_set():
                        break
                        
                    # Read all finger sensors and process control
                    finger_data = {}
                    for sensor_name, finger_name in self.finger_mapping.items():
                        # Skip if shutting down
                        if self.is_shutting_down or self.shutdown_event.is_set():
                            break
                        finger_data[finger_name] = self._process_finger(sensor_name, finger_name)
                    
                    # Abort if shutdown detected during processing
                    if self.is_shutting_down or self.shutdown_event.is_set():
                        break
                    
                    # Update hand state
                    hand_state = self._update_hand_state(finger_data)
                    
                    # Detect faults (more gracefully with retries)
                    try:
                        # Count bad sensors instead of just checking two specific ones
                        bad_sensors = 0
                        critical_sensors = ["Thumb1", "Index1", "Middle1", "Ring1", "Pinky1"]
                        
                        for sensor in critical_sensors:
                            try:
                                # Skip if we're shutting down
                                if self.is_shutting_down or self.shutdown_event.is_set():
                                    break
                                    
                                # Get sensor status with retry
                                status = "BAD"  # Default to BAD
                                for retry in range(3):
                                    try:
                                        _, status = self.proximity.get_sensor_value(sensor, with_status=True)
                                        if status != "BAD":
                                            break
                                        time.sleep(0.002)  # Small delay between retries
                                    except Exception:
                                        time.sleep(0.002)
                                
                                if status == "BAD":
                                    bad_sensors += 1
                                    logger.debug(f"Sensor {sensor} is BAD")
                            except Exception as e:
                                logger.debug(f"Error checking sensor {sensor}: {e}")
                                bad_sensors += 1
                        
                        # Set fault if more than 2 critical sensors are bad
                        if bad_sensors >= 2:
                            self.fault_status["sensor_failure"] = True
                            logger.warning(f"Sensor failure detected: {bad_sensors} critical sensors failing")
                        else:
                            self.fault_status["sensor_failure"] = False
                    except Exception as e:
                        logger.warning(f"Error checking sensor status: {e}")
                        # Don't change fault status if we can't check
                    
                    # Log data if not shutting down
                    if self.data_logger and not self.is_shutting_down:
                        try:
                            proximity_values = {}
                            # Only get values if not shutting down
                            if not self.is_shutting_down and not self.shutdown_event.is_set():
                                for name in self.proximity.sensor_names:
                                    try:
                                        proximity_values[name] = self.proximity.get_sensor_value(name)
                                    except Exception:
                                        proximity_values[name] = None
                            
                            # Create log data compatible with logger format
                            log_data = {
                                "timestamp": time.time(),
                                "hand_state": hand_state.name,
                                "fingers": finger_data,
                                "proximity": {
                                    "raw": proximity_values,
                                    "filtered": proximity_values,
                                    "status": {name: "OK" for name in proximity_values}
                                },
                                "imu": {
                                    "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
                                    "acceleration": {"x": 0.0, "y": 0.0, "z": 0.0},
                                    "angular_rate": {"x": 0.0, "y": 0.0, "z": 0.0},
                                    "motion_state": "STATIC"
                                },
                                "faults": self.fault_status
                            }
                            # Use the correct method name (log_data instead of log)
                            self.data_logger.log_data(log_data)
                        except Exception as e:
                            logger.error(f"Error logging data: {e}")
                    
                    # Update timing stats
                    end_time = time.time()
                    cycle_time = end_time - start_time
                    self.last_cycle_time = cycle_time
                    self.cycle_times.append(cycle_time)
                    
                    # Keep only the last 100 cycle times
                    if len(self.cycle_times) > 100:
                        self.cycle_times = self.cycle_times[-100:]
                    
                    # Check for loop overrun
                    if cycle_time > self.control_interval:
                        self.fault_status["loop_overrun"] = True
                        logger.warning(f"Control loop overrun: {cycle_time*1000:.1f}ms > {self.control_interval*1000:.1f}ms")
                    else:
                        self.fault_status["loop_overrun"] = False
                    
                    # Update max cycle time
                    self.max_cycle_time = max(self.max_cycle_time, cycle_time)
                    
                    # Calculate next update time
                    next_update_time = start_time + self.control_interval
                    
                except Exception as e:
                    logger.error(f"Error in control loop: {e}")
                    # Make sure we don't get stuck
                    next_update_time = time.time() + self.control_interval
            
            # Check shutdown before sleeping
            if self.is_shutting_down or self.shutdown_event.is_set():
                break
                
            # Small sleep to prevent CPU hogging, using Event.wait for interruptible sleep
            sleep_time = max(0.001, next_update_time - time.time())
            # This will return True if event is set during the wait
            if self.shutdown_event.wait(sleep_time):
                logger.info("Control loop received shutdown event during sleep")
                break
        
        logger.info("Control loop exited")
    
    def start(self):
        """Start all subsystems and the control loop"""
        # Check if already running
        if self.running:
            logger.warning("Controller already running")
            return
        
        # Reset shutdown flags
        self.is_shutting_down = False
        self.shutdown_event.clear()
        
        # Set startup time for startup delay protection
        self.startup_time = time.time()
        logger.info(f"Controller starting with {self.startup_delay_seconds}s startup protection delay")
        
        # Start proximity manager
        logger.info("Starting proximity manager...")
        self.proximity.start()
        
        # Start motor interface
        logger.info("Starting motor interface...")
        self.motors.start()
        
        # Start data logger if enabled
        if self.data_logger:
            logger.info("Starting data logger...")
            self.data_logger.start()
        
        # Start control loop
        logger.info("Starting control loop...")
        self.running = True
        self.thread = threading.Thread(target=self._control_loop)
        self.thread.daemon = True
        self.thread.start()
        
        logger.info("Proximity controller started")
    
    def stop(self):
        """Stop all subsystems and the control loop"""
        logger.info("Stopping controller...")
        
        # Mark that we're shutting down to prevent new hardware access
        self.is_shutting_down = True
        self.shutdown_event.set()
        
        # Stop control loop first
        self.running = False
        
        # Give the control loop thread time to notice shutdown
        logger.info("Waiting for control thread to exit...")
        if self.thread and self.thread.is_alive():
            try:
                self.thread.join(timeout=2.0)  # Increased timeout for safer shutdown
                if self.thread.is_alive():
                    logger.warning("Control thread did not exit within timeout, continuing with shutdown")
            except Exception as e:
                logger.error(f"Error joining control thread: {e}")
        
        # SAFETY: First set all fingers to flat/open position 
        try:
            if self.motors:
                logger.info("Safety: Opening hand to flat position...")
                # Set all standard fingers to open position
                for finger in self.motors.fingers:
                    try:
                        self.motors.set_position(finger, 0.0)
                    except Exception as e:
                        logger.error(f"Failed to open finger {finger}: {e}")
                
                # Also handle thumb rotation if available
                try:
                    if hasattr(self.motors, 'get_finger_status') and "ThumbRotate" in self.motors.position_targets:
                        self.motors.set_position("ThumbRotate", 0.0)
                except Exception as e:
                    logger.error(f"Failed to reset thumb rotation: {e}")
                
                # Give motors time to move to position
                time.sleep(0.5)
        except Exception as e:
            logger.error(f"Error during safety hand opening: {e}")
        
        # Now stop components in reverse initialization order
        
        # Stop data logger if enabled
        if self.data_logger:
            logger.info("Stopping data logger...")
            try:
                self.data_logger.stop()
            except Exception as e:
                logger.error(f"Error stopping data logger: {e}")
        
        # Stop motor interface
        logger.info("Stopping motors...")
        if self.motors:
            try:
                self.motors.stop()
            except Exception as e:
                logger.error(f"Error stopping motors: {e}")
        
        # Stop proximity manager last (after we've already set motors to safe position)
        logger.info("Stopping proximity manager...")
        if self.proximity:
            try:
                # The shutdown_event ensures the proximity manager knows about the shutdown
                self.proximity.stop()
            except Exception as e:
                logger.error(f"Error stopping proximity manager: {e}")
        
        logger.info("Proximity controller stopped")
    
    def safety_reset(self):
        """
        Reset the hand to a safe state (open position).
        
        This is useful in case of errors or unexpected behaviors.
        """
        logger.info("Performing safety reset...")
        
        try:
            # Set all fingers to open position
            if self.motors:
                for finger in self.motors.fingers:
                    try:
                        self.motors.set_position(finger, 0.0)
                    except Exception as e:
                        logger.error(f"Error resetting {finger}: {e}")
                
                # Also reset thumb rotation
                if hasattr(self.motors, 'get_finger_status') and "ThumbRotate" in self.motors.position_targets:
                    try:
                        self.motors.set_position("ThumbRotate", 0.0)
                    except Exception as e:
                        logger.error(f"Error resetting ThumbRotate: {e}")
                
                # Reset all FSMs to IDLE state
                for finger, fsm in self.finger_fsms.items():
                    fsm.state = FingerState.IDLE
                
                # Reset hand FSM
                self.hand_fsm.state = HandState.IDLE
                
                logger.info("Safety reset completed")
                return True
        except Exception as e:
            logger.error(f"Error during safety reset: {e}")
            return False
    
    def get_system_status(self):
        """
        Get current system status.
        
        Returns:
            Dictionary with current status of all components
        """
        # Get status information
        try:
            # Check if shutting down
            if self.is_shutting_down or self.shutdown_event.is_set():
                return {
                    "status": "SHUTTING_DOWN",
                    "faults": self.fault_status
                }
                
            # Get current hand state
            hand_state = self.hand_fsm.state
            
            # Get finger states
            finger_states = {}
            for finger, fsm in self.finger_fsms.items():
                finger_states[finger] = fsm.state.name
            
            # Get proximity values (with graceful failure)
            proximity_values = {}
            for sensor_name in MCP_SENSORS:
                try:
                    # Skip if we're shutting down
                    if self.is_shutting_down or self.shutdown_event.is_set():
                        proximity_values[sensor_name] = None
                        continue
                        
                    value, status = self.proximity.get_sensor_value(
                        sensor_name, filtered=True, with_status=True
                    )
                    proximity_values[sensor_name] = value
                except Exception as e:
                    logger.debug(f"Error getting sensor value for {sensor_name}: {e}")
                    proximity_values[sensor_name] = None
            
            # Get motor positions (with graceful failure)
            positions = {}
            currents = {}
            for finger_name in self.motors.fingers:
                try:
                    positions[finger_name] = self.motors.get_position(finger_name)
                    currents[finger_name] = self.motors.get_current(finger_name)
                except Exception as e:
                    logger.debug(f"Error getting motor data for {finger_name}: {e}")
                    positions[finger_name] = 0.0
                    currents[finger_name] = 0.0
            
            # Also get thumb rotator
            try:
                if "ThumbRotate" in self.motors.position_targets:
                    positions["ThumbRotate"] = self.motors.get_position("ThumbRotate")
                    currents["ThumbRotate"] = self.motors.get_current("ThumbRotate")
                else:
                    positions["ThumbRotate"] = 0.0
                    currents["ThumbRotate"] = 0.0
            except Exception as e:
                logger.debug(f"Error getting ThumbRotate data: {e}")
                positions["ThumbRotate"] = 0.0
                currents["ThumbRotate"] = 0.0
            
            # Get cycle time statistics
            cycle_stats = {
                "last": self.last_cycle_time,
                "avg": sum(self.cycle_times) / max(1, len(self.cycle_times)),
                "max": self.max_cycle_time
            }
            
            # Build detailed status report
            time_since_startup = 0.0
            startup_protection_active = False
            if self.startup_time is not None:
                time_since_startup = time.time() - self.startup_time
                startup_protection_active = time_since_startup < self.startup_delay_seconds
            
            # Get state transition timing information
            state_timing = {}
            for finger, states in self.state_entry_times.items():
                state_timing[finger] = {}
                current_time = time.time()
                for state, entry_time in states.items():
                    if entry_time > 0.0:  # Only report non-zero times
                        state_timing[finger][state.name] = current_time - entry_time
            
            # Get distance history information
            distance_history = {}
            for finger, history in self.distance_history.items():
                # Find the corresponding sensor for this finger
                sensor = None
                for s, f in self.finger_mapping.items():
                    if f == finger:
                        sensor = s
                        break
                if sensor:
                    distance_history[sensor] = history
            
            # Prepare detailed debug information
            debug_info = {
                "startup": {
                    "time_since_startup": time_since_startup,
                    "startup_protection_active": startup_protection_active,
                    "startup_delay_seconds": self.startup_delay_seconds
                },
                "timing": {
                    "state_entry_times": state_timing,
                    "min_times": {
                        "idle": self.min_time_in_idle,
                        "approach": self.min_time_in_approach,
                        "proportional": self.min_time_in_proportional,
                        "contact": self.min_time_in_contact
                    }
                },
                "hysteresis": {
                    "awaiting_reset": self.awaiting_reset,
                    "reset_threshold": self.reset_threshold_mm,
                },
                "filtering": {
                    "consecutive_required": self.consecutive_readings_required,
                    "distance_history": distance_history
                },
                "thresholds": {
                    "approach": self.approach_threshold,
                    "contact": self.contact_threshold
                }
            }
            
            return {
                "hand": {
                    "hand_state": hand_state.name,
                    "finger_states": finger_states
                },
                "proximity": proximity_values,
                "positions": positions,
                "currents": currents,
                "cycle_time": cycle_stats,
                "faults": self.fault_status,
                "status": "RUNNING",
                "debug": debug_info
            }
            
        except Exception as e:
            logger.error(f"Error getting system status: {e}")
            return {
                "error": str(e),
                "faults": self.fault_status,
                "status": "ERROR"
            }

# Simple test code
if __name__ == "__main__":
    print("Testing proximity controller...")
    controller = ProximityController(control_rate=20, use_simulated_motors=True)
    controller.start()
    
    try:
        print("Running for 10 seconds...")
        for _ in range(10):
            status = controller.get_system_status()
            print(f"Hand state: {status['hand']['hand_state']}")
            print(f"Finger states: {status['hand']['finger_states']}")
            print(f"Proximity values: {status['proximity']}")
            time.sleep(1)
    
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    
    finally:
        controller.stop()
        print("Test complete.")