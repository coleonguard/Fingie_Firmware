#!/usr/bin/env python3
"""
Unified Controller for Prosthetic Control System.

This module implements the main controller that integrates all components:
- Proximity sensing
- IMU motion detection
- Hand motor control
- Finger and hand state machines
- Data logging
"""

import time
import threading
import logging
import json
from enum import Enum
from typing import Dict, List, Optional, Tuple, Any
import os
import sys

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("UnifiedController")

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Local imports
from proximity.proximity_manager import ProximityManager
from imu.imu_interface import IMUInterface, MotionState
from hand.motor_interface import MotorInterface, ControlMode, SimulatedMotorInterface
from hand.ability_hand_interface import AbilityHandInterface
from controller.state_machines import FingerState, HandState, FingerFSM, HandFSM
from utils.logger import DataLogger

class UnifiedController:
    """
    Unified controller for prosthetic hand.
    
    This class integrates proximity sensing with motor control to provide
    a complete prosthetic control system with state machines for automatic
    grasp and release.
    """
    
    def __init__(self, 
                 control_rate: int = 20, 
                 enable_logging: bool = True,
                 log_dir: str = None,
                 use_simulated_motors: bool = False,
                 motor_interface_kwargs: dict = None,
                 proximity_sampling_rate: int = None,
                 imu_sampling_rate: int = None):
        """
        Initialize the unified controller.
        
        Args:
            control_rate: Control loop frequency in Hz
            enable_logging: Whether to enable data logging
            log_dir: Directory for log files (default: ~/prosthetic_logs)
            use_simulated_motors: Whether to use simulated motors
            motor_interface_kwargs: Additional kwargs for motor interface
            proximity_sampling_rate: Override for proximity sensor sampling rate
            imu_sampling_rate: Override for IMU sampling rate
        """
        # Save parameters
        self.control_rate = control_rate
        self.control_interval = 1.0 / control_rate
        self.enable_logging = enable_logging
        
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
        
        # Set up IMU sampling rate (10x control rate by default)
        if imu_sampling_rate is None:
            self.imu_sampling_rate = control_rate * 10
        else:
            self.imu_sampling_rate = imu_sampling_rate
        
        # Initialize proximity manager
        logger.info("Initializing proximity manager...")
        self.proximity = ProximityManager(sampling_rate=self.proximity_sampling_rate)
        
        # Initialize IMU interface
        logger.info("Initializing IMU interface...")
        self.imu = IMUInterface(sampling_rate=self.imu_sampling_rate)
        
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
        self.finger_mapping = {
            "Thumb1": "Thumb",
            "Index1": "Index",
            "Middle1": "Middle",
            "Ring1": "Ring",
            "Pinky1": "Pinky"
        }
        
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
        
        logger.info("Unified controller initialized")
    
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
    
    def _update_all_fingers(self):
        """Process control for all fingers"""
        # For each MCP sensor, determine control phase and set appropriate control
        for sensor_name, finger_name in self.finger_mapping.items():
            self._process_finger(sensor_name, finger_name)
    
    def _update_hand_state(self):
        """Update the hand state machine"""
        # Get finger states and currents
        finger_states = {
            finger_name: fsm.state 
            for finger_name, fsm in self.finger_fsms.items()
        }
        
        finger_currents = {
            finger_name: self.motors.get_current(finger_name)
            for finger_name in self.finger_fsms
        }
        
        # Get IMU-based movement detection
        imu_data = self.imu.get_data()
        motion_state, duration, elapsed_since_impact = self.imu.get_motion_state()
        
        is_lowering = (motion_state == MotionState.LOWERING)
        is_impact_detected = (motion_state == MotionState.IMPACT or 
                             motion_state == MotionState.STATIONARY)
        is_stationary = (motion_state == MotionState.STATIONARY)
        
        # Update hand state
        hand_state = self.hand_fsm.update(
            finger_states, finger_currents,
            is_lowering, is_impact_detected, is_stationary,
            elapsed_since_impact
        )
        
        # Apply hand state control logic
        if hand_state == HandState.RELEASE:
            # Force all fingers to open
            for finger_name, fsm in self.finger_fsms.items():
                fsm.reset()  # Reset to IDLE state
                self.motors.set_position(finger_name, 0.0)  # Open position
                
        elif hand_state == HandState.RETRACT:
            # Keep fingers opening
            for finger_name, fsm in self.finger_fsms.items():
                if fsm.state != FingerState.IDLE:
                    fsm.reset()  # Reset to IDLE state
                self.motors.set_position(finger_name, 0.0)  # Open position
    
    def _log_data(self):
        """Log current system state"""
        if not self.enable_logging or self.data_logger is None:
            return
            
        # Create log entry
        entry = {
            "timestamp": time.time(),
            
            # Proximity sensors
            "proximity": {
                "raw": self.proximity.get_all_values(filtered=False),
                "filtered": self.proximity.get_all_values(filtered=True),
                "status": self.proximity.status.copy() if hasattr(self.proximity, 'status') else {}
            },
            
            # IMU data
            "imu": {
                "orientation": {
                    "roll": self.imu.current_data.roll,
                    "pitch": self.imu.current_data.pitch,
                    "yaw": self.imu.current_data.yaw
                },
                "acceleration": {
                    "x": self.imu.current_data.accel_x,
                    "y": self.imu.current_data.accel_y,
                    "z": self.imu.current_data.accel_z
                },
                "angular_rate": {
                    "x": self.imu.current_data.gyro_x,
                    "y": self.imu.current_data.gyro_y,
                    "z": self.imu.current_data.gyro_z
                },
                "motion_state": self.imu.motion_state.name
            },
            
            # Finger data
            "fingers": {},
            
            # Hand state
            "hand_state": self.hand_fsm.state.name
        }
        
        # Add finger data
        for finger_name, fsm in self.finger_fsms.items():
            entry["fingers"][finger_name] = {
                "state": fsm.state.name,
                "position": self.motors.get_position(finger_name),
                "velocity": self.motors.get_velocity(finger_name),
                "current": self.motors.get_current(finger_name),
                "target_position": fsm.position_target,
                "target_torque": fsm.torque_target
            }
        
        # Add faults if any
        for fault, status in self.fault_status.items():
            if status:
                entry["fault"] = fault
                break
                
        # Add motor faults if any
        motor_faults = self.motors.get_fault_status()
        for fault, status in motor_faults.items():
            if status:
                entry["motor_fault"] = fault
                break
        
        # Log the entry
        self.data_logger.log_data(entry)
    
    def _control_loop(self):
        """Main control loop"""
        next_update_time = time.time()
        
        while self.running:
            cycle_start_time = time.time()
            
            # Check if we're running late
            if cycle_start_time > next_update_time + 0.1:  # 100ms late
                logger.warning(f"Control loop over-run: {cycle_start_time - next_update_time:.3f}s")
                self.fault_status["loop_overrun"] = True
                
                # Safety measure: set all fingers to idle
                for finger_name, fsm in self.finger_fsms.items():
                    fsm.reset()  # Reset to IDLE state
                    self.motors.set_torque(finger_name, 0.0)  # Zero torque
            
            # Check if it's time for the next control update
            if cycle_start_time >= next_update_time:
                # Process all fingers
                self._update_all_fingers()
                
                # Update hand state
                self._update_hand_state()
                
                # Log data if enabled
                if self.enable_logging:
                    self._log_data()
                
                # Calculate next update time
                next_update_time = cycle_start_time + self.control_interval
                
                # Measure cycle time
                cycle_end_time = time.time()
                cycle_time = cycle_end_time - cycle_start_time
                self.last_cycle_time = cycle_time
                self.max_cycle_time = max(self.max_cycle_time, cycle_time)
                
                # Keep a history of recent cycle times
                self.cycle_times.append(cycle_time)
                if len(self.cycle_times) > 100:
                    self.cycle_times = self.cycle_times[-100:]
            
            # Small sleep to prevent CPU hogging
            time.sleep(0.001)
    
    def start(self):
        """Start the unified controller"""
        if not self.running:
            # Reset fault status
            for fault in self.fault_status:
                self.fault_status[fault] = False
                
            # Reset performance monitoring
            self.cycle_times = []
            self.max_cycle_time = 0.0
            
            # Start subsystems
            logger.info("Starting proximity manager...")
            self.proximity.start()
            
            logger.info("Starting IMU interface...")
            self.imu.start()
            
            logger.info("Starting motor interface...")
            self.motors.start()
            
            # Start data logger if enabled
            if self.enable_logging and self.data_logger is not None:
                logger.info("Starting data logger...")
                self.data_logger.start()
            
            # Start the main control thread
            logger.info("Starting control loop...")
            self.running = True
            self.thread = threading.Thread(target=self._control_loop)
            self.thread.daemon = True
            self.thread.start()
            
            logger.info("Unified controller started")
    
    def stop(self):
        """Stop the unified controller"""
        if not self.running:
            return
            
        logger.info("Stopping unified controller...")
        
        # Stop the main control thread
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        
        # Stop subsystems
        logger.info("Stopping motor interface...")
        self.motors.stop()
        
        logger.info("Stopping IMU interface...")
        self.imu.stop()
        
        logger.info("Stopping proximity manager...")
        self.proximity.stop()
        
        # Stop data logger if enabled
        if self.enable_logging and self.data_logger is not None:
            logger.info("Stopping data logger...")
            self.data_logger.stop()
        
        logger.info("Unified controller stopped")
    
    def get_finger_status(self, finger_name):
        """
        Get detailed status for a finger.
        
        Args:
            finger_name: Name of the finger
            
        Returns:
            Dictionary with comprehensive status
        """
        if finger_name not in self.finger_fsms:
            return {"error": "Invalid finger name"}
        
        # Get motor status
        motor_status = self.motors.get_finger_status(finger_name)
        
        # Get FSM status
        fsm = self.finger_fsms[finger_name]
        
        # Find sensor name for this finger (reverse mapping)
        sensor_name = None
        for s, f in self.finger_mapping.items():
            if f == finger_name:
                sensor_name = s
                break
        
        # Add proximity data if sensor found
        if sensor_name:
            motor_status["distance"] = self.proximity.get_sensor_value(sensor_name, filtered=True)
            motor_status["raw_distance"] = self.proximity.get_sensor_value(sensor_name, filtered=False)
            if hasattr(self.proximity, 'status'):
                motor_status["sensor_status"] = self.proximity.status.get(sensor_name, "UNKNOWN")
            
        # Add FSM state
        motor_status["fsm_state"] = fsm.state.name
        motor_status["target_position"] = fsm.position_target
        motor_status["target_torque"] = fsm.torque_target
            
        return motor_status
    
    def get_hand_status(self):
        """
        Get status of the overall hand.
        
        Returns:
            Dictionary with hand status
        """
        # Get IMU status
        imu_data = self.imu.get_data()
        motion_state, duration, elapsed_since_impact = self.imu.get_motion_state()
        
        status = {
            "hand_state": self.hand_fsm.state.name,
            "state_duration": time.time() - self.hand_fsm.state_entry_time,
            "motion_state": motion_state.name,
            "imu": {
                "accel_z": imu_data.accel_z,
                "gyro_magnitude": imu_data.gyro_magnitude,
                "is_stationary": (motion_state == MotionState.STATIONARY),
                "elapsed_since_impact": elapsed_since_impact
            },
            "fault_status": self.fault_status.copy()
        }
        
        return status
    
    def get_system_status(self):
        """
        Get comprehensive system status.
        
        Returns:
            Dictionary with system status
        """
        status = {
            "running": self.running,
            "control_rate": self.control_rate,
            "cycle_time": {
                "last": self.last_cycle_time,
                "max": self.max_cycle_time,
                "avg": sum(self.cycle_times) / len(self.cycle_times) if self.cycle_times else 0
            },
            "hand": self.get_hand_status(),
            "fingers": {},
            "faults": self.fault_status.copy()
        }
        
        # Add status for each finger
        for finger_name in self.finger_fsms:
            status["fingers"][finger_name] = self.get_finger_status(finger_name)
        
        # Add logger stats if enabled
        if self.enable_logging and self.data_logger is not None:
            status["logger"] = self.data_logger.get_stats()
            
        return status
    
    def reset(self):
        """
        Reset the controller to initial state.
        
        This resets all state machines and clears fault conditions.
        """
        # Reset finger FSMs
        for finger_name, fsm in self.finger_fsms.items():
            fsm.reset()
            
        # Reset hand FSM
        self.hand_fsm.reset()
        
        # Reset fault status
        for fault in self.fault_status:
            self.fault_status[fault] = False
            
        logger.info("Controller reset to initial state")
        
        return {"status": "reset_complete"}

# Simple test code
if __name__ == "__main__":
    print("Testing unified controller...")
    # Use simulated components for testing
    controller = UnifiedController(control_rate=20, use_simulated_motors=True)
    
    try:
        print("Starting controller...")
        controller.start()
        
        # Run for a while, reporting status
        for i in range(30):
            if i % 5 == 0:
                status = controller.get_system_status()
                
                print("\n--- System Status ---")
                print(f"Hand state: {status['hand']['hand_state']}")
                print(f"Motion state: {status['hand']['motion_state']}")
                
                # Print finger states
                for finger, data in status['fingers'].items():
                    print(f"{finger}: {data['fsm_state']}, "
                         f"Pos={data['position']:.1f}°, "
                         f"Cur={data['current']:.2f}A")
                
                print(f"Cycle time: {status['cycle_time']['last']*1000:.1f}ms, "
                     f"Max={status['cycle_time']['max']*1000:.1f}ms")
                
                if any(status['faults'].values()):
                    print(f"FAULTS: {status['faults']}")
            
            time.sleep(0.5)
            
        # Test reset
        print("\nResetting controller...")
        controller.reset()
        
        # Check status after reset
        status = controller.get_system_status()
        print("\n--- Status After Reset ---")
        print(f"Hand state: {status['hand']['hand_state']}")
        for finger, data in status['fingers'].items():
            print(f"{finger}: {data['fsm_state']}")
            
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        
    finally:
        controller.stop()
        print("Test complete.")