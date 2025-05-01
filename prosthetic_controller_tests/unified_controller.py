#!/usr/bin/env python3
# Unified Controller for Prosthetic Control

import time
import threading
import numpy as np
from enum import Enum
from typing import List, Dict, Union, Optional, Tuple
import os
import json

# Import our components
from proximity_manager import ProximityManager, MCP_SENSORS
from motor_interface import MotorInterface, ControlMode, SimulatedMotorInterface
from ability_hand_interface import AbilityHandInterface

class ControlPhase(Enum):
    """Control phases for the prosthetic hand"""
    IDLE = 0        # No control, motors off
    APPROACH = 1    # Beyond threshold, no movement
    PROPORTIONAL = 2  # Between threshold and contact, proportional control
    CONTACT = 3     # Object contact detected, torque control

class UnifiedController:
    """
    Unified controller for prosthetic hand
    
    This class integrates proximity sensing with motor control to provide
    a complete prosthetic control system.
    """
    
    def __init__(self, 
                 control_rate: int = 20, 
                 enable_logging: bool = True,
                 log_dir: str = None,
                 use_simulated_motors: bool = False,
                 motor_interface_kwargs: dict = None,
                 proximity_sampling_rate: int = None):
        """
        Initialize the unified controller
        
        Args:
            control_rate: Control loop frequency in Hz
            enable_logging: Whether to enable data logging
            log_dir: Directory for log files (default: ~/prosthetic_logs)
            use_simulated_motors: Whether to use simulated motors
            motor_interface_kwargs: Additional kwargs for motor interface
            proximity_sampling_rate: Override for proximity sensor sampling rate
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
        
        # Initialize proximity manager
        self.proximity = ProximityManager(sampling_rate=self.proximity_sampling_rate)
        
        # Initialize motor interface
        if motor_interface_kwargs is None:
            motor_interface_kwargs = {}
            
        motor_interface_kwargs['control_rate'] = control_rate
        
        if use_simulated_motors:
            self.motors = SimulatedMotorInterface(**motor_interface_kwargs)
        else:
            try:
                self.motors = AbilityHandInterface(**motor_interface_kwargs)
            except Exception as e:
                print(f"Error initializing Ability Hand interface: {e}")
                print("Falling back to simulated motors")
                self.motors = SimulatedMotorInterface(**motor_interface_kwargs)
        
        # Finger mapping from proximity sensors to motors
        self.finger_mapping = {
            "Thumb1": "Thumb",
            "Index1": "Index",
            "Middle1": "Middle",
            "Ring1": "Ring",
            "Pinky1": "Pinky"
        }
        
        # Control thresholds (in mm)
        self.approach_threshold = 40  # Distance beyond which no action occurs
        self.contact_threshold = 5    # Distance at which contact is detected
        
        # Control parameters
        self.max_position = 100.0     # Maximum position value
        self.max_current = 0.8        # Maximum current during proportional control (A)
        self.contact_current = 0.5    # Current during contact for stable grip (A)
        
        # Control phase for each finger
        self.control_phases = {finger: ControlPhase.IDLE for finger in self.finger_mapping.values()}
        
        # Thread control
        self.running = False
        self.thread = None
        
        # Logging
        self.log_data = {
            "timestamps": [],
            "proximity": {},
            "positions": {},
            "currents": {},
            "control_phases": {}
        }
        
        # Initialize log data structures
        for sensor in MCP_SENSORS:
            self.log_data["proximity"][sensor] = []
            
        for finger in self.motors.fingers:
            self.log_data["positions"][finger] = []
            self.log_data["currents"][finger] = []
            self.log_data["control_phases"][finger] = []
    
    def _determine_control_phase(self, distance):
        """Determine control phase based on current distance
        
        Args:
            distance: Current distance reading (mm)
            
        Returns:
            Appropriate ControlPhase enum value
        """
        if distance >= self.approach_threshold:
            return ControlPhase.APPROACH
        elif distance <= self.contact_threshold:
            return ControlPhase.CONTACT
        else:
            return ControlPhase.PROPORTIONAL
    
    def _calculate_proportional_position(self, distance):
        """Calculate position for proportional control based on distance
        
        Args:
            distance: Current distance reading (mm)
            
        Returns:
            Appropriate position value (degrees or normalized)
        """
        # Linear mapping from distance to position
        # At approach_threshold → 0 position (open)
        # At contact_threshold → max_position (closed)
        
        # Normalize the distance to [0, 1] range
        range_size = self.approach_threshold - self.contact_threshold
        if range_size <= 0:  # Safety check
            return 0.0
            
        normalized_distance = (self.approach_threshold - distance) / range_size
        normalized_distance = max(0.0, min(1.0, normalized_distance))  # Clamp to [0, 1]
        
        # Apply to position range
        position = normalized_distance * self.max_position
        return position
    
    def _calculate_proportional_current(self, distance):
        """Calculate current for proportional control based on distance
        
        Args:
            distance: Current distance reading (mm)
            
        Returns:
            Appropriate current value (A)
        """
        # Linear mapping from distance to current
        # At approach_threshold → 0 current
        # At contact_threshold → max_current
        
        # Normalize the distance to [0, 1] range
        range_size = self.approach_threshold - self.contact_threshold
        if range_size <= 0:  # Safety check
            return 0.0
            
        normalized_distance = (self.approach_threshold - distance) / range_size
        normalized_distance = max(0.0, min(1.0, normalized_distance))  # Clamp to [0, 1]
        
        # Apply to current range
        current = normalized_distance * self.max_current
        return current
    
    def _control_loop(self):
        """Main control loop"""
        next_update_time = time.time()
        
        while self.running:
            current_time = time.time()
            
            # Check if it's time for the next control update
            if current_time >= next_update_time:
                # Process all fingers
                self._update_all_fingers()
                
                # Log data if enabled
                if self.enable_logging:
                    self._log_data(current_time)
                
                # Calculate next update time
                next_update_time = current_time + self.control_interval
            
            # Small sleep to prevent CPU hogging
            time.sleep(0.001)
    
    def _update_all_fingers(self):
        """Process control for all fingers"""
        # For each MCP sensor, determine control phase and set appropriate control
        for sensor_name in MCP_SENSORS:
            # Get the corresponding finger name
            finger_name = self.finger_mapping.get(sensor_name)
            if not finger_name:
                continue
                
            # Get filtered sensor reading
            filtered_distance = self.proximity.get_sensor_value(sensor_name, filtered=True)
            
            # Determine control phase
            control_phase = self._determine_control_phase(filtered_distance)
            self.control_phases[finger_name] = control_phase
            
            # Apply control based on phase
            if control_phase == ControlPhase.APPROACH:
                # No movement in approach phase - open hand
                self.motors.set_position(finger_name, 0.0)
                
            elif control_phase == ControlPhase.PROPORTIONAL:
                # Position control based on distance
                position = self._calculate_proportional_position(filtered_distance)
                self.motors.set_position(finger_name, position)
                
            elif control_phase == ControlPhase.CONTACT:
                # Contact detected, switch to torque control
                self.motors.set_torque(finger_name, self.contact_current)
    
    def _log_data(self, timestamp):
        """Log current state data
        
        Args:
            timestamp: Current time
        """
        # Add timestamp
        self.log_data["timestamps"].append(timestamp)
        
        # Log proximity data
        for sensor in MCP_SENSORS:
            value = self.proximity.get_sensor_value(sensor, filtered=True)
            self.log_data["proximity"][sensor].append(value)
        
        # Log motor data
        for finger in self.motors.fingers:
            self.log_data["positions"][finger].append(self.motors.get_position(finger))
            self.log_data["currents"][finger].append(self.motors.get_current(finger))
            
            # Get control phase as string
            if finger in self.control_phases:
                phase = self.control_phases[finger].name
            else:
                phase = "UNKNOWN"
                
            self.log_data["control_phases"][finger].append(phase)
    
    def start(self):
        """Start the unified controller"""
        if not self.running:
            # Start subsystems
            self.proximity.start()
            self.motors.start()
            
            # Start the main control thread
            self.running = True
            self.thread = threading.Thread(target=self._control_loop)
            self.thread.daemon = True
            self.thread.start()
            
            print("Unified controller started")
    
    def stop(self):
        """Stop the unified controller"""
        # Stop the main control thread
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        
        # Stop subsystems
        self.motors.stop()
        self.proximity.stop()
        
        # Save log data if enabled
        if self.enable_logging and len(self.log_data["timestamps"]) > 0:
            self._save_log_data()
        
        print("Unified controller stopped")
    
    def _save_log_data(self):
        """Save logged data to file"""
        # Create filename with timestamp
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        filename = os.path.join(self.log_dir, f"prosthetic_log_{timestamp}.json")
        
        # Save to file
        with open(filename, 'w') as f:
            json.dump(self.log_data, f)
            
        print(f"Log data saved to {filename}")
    
    def get_finger_status(self, finger):
        """Get comprehensive status for a finger
        
        Args:
            finger: Name of finger
            
        Returns:
            Dictionary with current status
        """
        if finger not in self.motors.fingers:
            return {"error": "Invalid finger name"}
        
        # Get motor status
        motor_status = self.motors.get_finger_status(finger)
        
        # Get sensor name for this finger (reverse mapping)
        sensor_name = None
        for s, f in self.finger_mapping.items():
            if f == finger:
                sensor_name = s
                break
        
        # Add proximity data if sensor found
        if sensor_name:
            motor_status["distance"] = self.proximity.get_sensor_value(sensor_name, filtered=True)
            motor_status["raw_distance"] = self.proximity.get_sensor_value(sensor_name, filtered=False)
            
        # Add control phase
        if finger in self.control_phases:
            motor_status["control_phase"] = self.control_phases[finger].name
            
        return motor_status
    
    def get_system_status(self):
        """Get comprehensive system status
        
        Returns:
            Dictionary with system status
        """
        status = {
            "running": self.running,
            "control_rate": self.control_rate,
            "fingers": {}
        }
        
        # Add status for each finger
        for finger in self.motors.fingers:
            status["fingers"][finger] = self.get_finger_status(finger)
            
        return status

# Simple test code
if __name__ == "__main__":
    print("Testing unified controller...")
    # Use simulated motors for testing
    controller = UnifiedController(control_rate=20, use_simulated_motors=True)
    
    try:
        print("Starting controller...")
        controller.start()
        
        # Run for a while, reporting status
        for i in range(20):
            if i % 5 == 0:
                print("\n--- System Status ---")
                for finger in ["Thumb", "Index", "Middle", "Ring", "Pinky"]:
                    status = controller.get_finger_status(finger)
                    print(f"{finger}: Dist={status.get('distance', 'N/A'):.1f}mm, "
                          f"Pos={status['position']:.1f}°, "
                          f"Current={status['current']:.2f}A, "
                          f"Phase={status.get('control_phase', 'N/A')}")
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        
    finally:
        controller.stop()
        print("Test complete.")