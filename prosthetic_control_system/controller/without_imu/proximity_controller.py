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
                 contact_threshold: int = DEFAULT_CONFIG["contact_threshold"]):
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
        
        # Set up logging
        if log_dir is None:
            self.log_dir = os.path.expanduser("~/prosthetic_logs")
        else:
            self.log_dir = log_dir
            
        if self.enable_logging:
            os.makedirs(self.log_dir, exist_ok=True)
        
        # Set up proximity sampling rate (2x control rate by default)
        if proximity_sampling_rate is None:
            self.proximity_sampling_rate = control_rate * 2
        else:
            self.proximity_sampling_rate = proximity_sampling_rate
        
        # Initialize proximity manager with custom thresholds
        logger.info("Initializing proximity manager...")
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
        for sensor_name, finger_name in self.finger_mapping.items():
            self.finger_fsms[finger_name] = FingerFSM(finger_name)
        
        # Create hand FSM
        logger.info("Initializing hand state machine...")
        self.hand_fsm = HandFSM()
        
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
        
        logger.info("Proximity controller initialized")
    
    def _process_finger(self, sensor_name, finger_name):
        """Process control for a single finger"""
        # Get the finger FSM
        fsm = self.finger_fsms[finger_name]
        
        # Get sensor distance
        distance, status = self.proximity.get_sensor_value(
            sensor_name, filtered=True, with_status=True
        )
        
        # Get motor feedback
        current = self.motors.get_current(finger_name)
        position = self.motors.get_position(finger_name)
        
        # Get current derivative for slip detection
        if hasattr(self.motors, 'get_current_derivative'):
            current_derivative = self.motors.get_current_derivative(finger_name)
        else:
            # Fall back to zero if not available
            current_derivative = 0.0
        
        # Update FSM
        state, position_target, torque_target = fsm.update(
            distance, current, current_derivative, position
        )
        
        # Apply control based on state
        if state == FingerState.IDLE or state == FingerState.APPROACH:
            # Position control with open hand
            self.motors.set_position(finger_name, 0.0)
            
        elif state == FingerState.PROPORTIONAL:
            # Position control with proportional position
            self.motors.set_position(finger_name, position_target)
            
        elif state == FingerState.CONTACT:
            # Torque control
            self.motors.set_torque(finger_name, torque_target)
            
        # Return state information for logging/monitoring
        return {
            "state": state.name,
            "distance": distance,
            "sensor_status": status,
            "position": position,
            "position_target": position_target,
            "current": current,
            "torque_target": torque_target,
            "current_derivative": current_derivative
        }
    
    def _update_hand_state(self, finger_states):
        """Update hand state machine based on finger states"""
        # Count fingers in each state
        state_counts = {state: 0 for state in FingerState}
        for finger, data in finger_states.items():
            state = FingerState[data["state"]]
            state_counts[state] += 1
        
        # Update hand FSM
        # Since we don't have IMU, always use STATIC for motion state
        motion_state = MotionState.STATIC
        hand_state = self.hand_fsm.update(state_counts, motion_state)
        
        return hand_state
    
    def _control_loop(self):
        """Main control loop that runs at the specified rate"""
        next_update_time = time.time()
        
        while self.running:
            start_time = time.time()
            
            # Check if it's time for the next control update
            if start_time >= next_update_time:
                try:
                    # Read all finger sensors and process control
                    finger_data = {}
                    for sensor_name, finger_name in self.finger_mapping.items():
                        finger_data[finger_name] = self._process_finger(sensor_name, finger_name)
                    
                    # Update hand state
                    hand_state = self._update_hand_state(finger_data)
                    
                    # Detect faults
                    if self.proximity.get_sensor_value("Thumb1", with_status=True)[1] == "BAD" and \
                       self.proximity.get_sensor_value("Index1", with_status=True)[1] == "BAD":
                        # Multiple critical sensors failing
                        self.fault_status["sensor_failure"] = True
                    else:
                        self.fault_status["sensor_failure"] = False
                    
                    # Log data
                    if self.data_logger:
                        log_data = {
                            "timestamp": time.time(),
                            "hand_state": hand_state.name,
                            "fingers": finger_data,
                            "proximity": {
                                name: self.proximity.get_sensor_value(name)
                                for name in self.proximity.sensor_names
                            },
                            "faults": self.fault_status
                        }
                        self.data_logger.log(log_data)
                    
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
            
            # Small sleep to prevent CPU hogging
            sleep_time = max(0.001, next_update_time - time.time())
            time.sleep(sleep_time)
    
    def start(self):
        """Start all subsystems and the control loop"""
        # Check if already running
        if self.running:
            logger.warning("Controller already running")
            return
        
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
        
        # Stop control loop
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        
        # Stop motor interface
        logger.info("Stopping motors...")
        if self.motors:
            self.motors.stop()
        
        # Stop proximity manager
        logger.info("Stopping proximity manager...")
        if self.proximity:
            self.proximity.stop()
        
        # Stop data logger if enabled
        if self.data_logger:
            logger.info("Stopping data logger...")
            self.data_logger.stop()
        
        logger.info("Proximity controller stopped")
    
    def get_system_status(self):
        """
        Get current system status.
        
        Returns:
            Dictionary with current status of all components
        """
        # Get status information
        try:
            # Get current hand state
            hand_state = self.hand_fsm.state
            
            # Get finger states
            finger_states = {}
            for finger, fsm in self.finger_fsms.items():
                finger_states[finger] = fsm.state.name
            
            # Get proximity values
            proximity_values = {}
            for sensor_name in MCP_SENSORS:
                value, status = self.proximity.get_sensor_value(
                    sensor_name, filtered=True, with_status=True
                )
                proximity_values[sensor_name] = value
            
            # Get motor positions
            positions = {}
            currents = {}
            for finger_name in self.motors.fingers:
                positions[finger_name] = self.motors.get_position(finger_name)
                currents[finger_name] = self.motors.get_current(finger_name)
            
            # Also get thumb rotator
            positions["ThumbRotate"] = self.motors.get_position("ThumbRotate")
            currents["ThumbRotate"] = self.motors.get_current("ThumbRotate")
            
            # Get cycle time statistics
            cycle_stats = {
                "last": self.last_cycle_time,
                "avg": sum(self.cycle_times) / max(1, len(self.cycle_times)),
                "max": self.max_cycle_time
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
                "faults": self.fault_status
            }
            
        except Exception as e:
            logger.error(f"Error getting system status: {e}")
            return {
                "error": str(e),
                "faults": self.fault_status
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