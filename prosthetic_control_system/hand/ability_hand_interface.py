#!/usr/bin/env python3
"""
Ability Hand Motor Interface for Prosthetic Control.

This module implements the interface for the PSYONIC Ability Hand,
using the AHSerialClient from the ability-hand-api package.
"""

import time
import threading
import logging
import numpy as np
from typing import Dict, List, Optional, Union, Tuple
import sys
import os

# Local imports
from .motor_interface import MotorInterface, ControlMode

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("AbilityHandInterface")

class AbilityHandInterface(MotorInterface):
    """
    Interface for the PSYONIC Ability Hand.
    
    This class implements the MotorInterface for the PSYONIC Ability Hand,
    using the AHSerialClient from the ability-hand-api package.
    """
    
    def __init__(self, 
                 control_rate: int = 50, 
                 port: str = None, 
                 baud_rate: int = 460800, 
                 reply_mode: int = 2):
        """
        Initialize the Ability Hand interface.
        
        Args:
            control_rate: Control loop frequency in Hz
            port: Optional serial port override
            baud_rate: Serial baud rate
            reply_mode: Hand reply mode (0: Pos,Cur,Touch, 1: Pos,Vel,Touch, 2: Pos,Cur,Vel)
        """
        super().__init__(control_rate)
        
        # Save initialization parameters
        self.port = port
        self.baud_rate = baud_rate
        self.reply_mode = reply_mode
        
        # Client for Ability Hand
        self.client = None
        self._initialize_client()
        
        # Hand has 6 fingers, but we'll maintain the 5-finger convention
        # by handling the thumb rotator separately
        self.fingers = ["Index", "Middle", "Ring", "Pinky", "Thumb"]
        self.finger_indices = {
            "Index": 0,
            "Middle": 1,
            "Ring": 2,
            "Pinky": 3,
            "Thumb": 4
        }
        
        # Add thumb rotator to our feedback structures
        self.position_feedback["ThumbRotate"] = 0.0
        self.velocity_feedback["ThumbRotate"] = 0.0
        self.torque_feedback["ThumbRotate"] = 0.0
        self.current_feedback["ThumbRotate"] = 0.0
        self.position_targets["ThumbRotate"] = 0.0
        self.velocity_targets["ThumbRotate"] = 0.0
        self.torque_targets["ThumbRotate"] = 0.0
        self.duty_targets["ThumbRotate"] = 0.0
        self.control_modes["ThumbRotate"] = ControlMode.POSITION
        
        # Scale factors for conversions
        self.position_scale = 1.0  # Scale factor for position values (if needed)
        self.velocity_scale = 1.0  # Scale factor for velocity values (if needed)
        self.torque_scale = 1.0    # Scale factor for torque values (if needed)
        
        # Maximum values (specific to Ability Hand)
        self.max_position = 100.0  # Maximum position value (0-100)
        self.max_velocity = 100.0  # Maximum velocity
        self.max_torque = 0.6      # Maximum torque (current) value
        
        # Flag to indicate if we need to reset client
        self.client_error = False
        
        # Current derivatives for slip detection
        self.current_history = {finger: [] for finger in self.fingers}
        self.current_time_history = {finger: [] for finger in self.fingers}
        self.current_derivatives = {finger: 0.0 for finger in self.fingers}
    
    def _initialize_client(self):
        """Initialize the Ability Hand client"""
        try:
            # We'll import here to avoid dependency issues if the package isn't available
            try:
                from ah_wrapper.ah_serial_client import AHSerialClient
                logger.info(f"Connecting to Ability Hand on port {self.port}")
                self.client = AHSerialClient(
                    port=self.port,
                    baud_rate=self.baud_rate,
                    reply_mode=self.reply_mode,
                    rate_hz=self.control_rate
                )
                logger.info("Successfully connected to Ability Hand")
                self.client_error = False
            except ImportError:
                logger.warning("Could not import AHSerialClient. Using simulated mode.")
                logger.warning("To use real hardware, install the ability-hand-api package.")
                self._simulate_client()
        except Exception as e:
            logger.error(f"Could not initialize Ability Hand: {e}")
            self._simulate_client()
    
    def _simulate_client(self):
        """Create a simulated client for testing"""
        self.client_error = True
        logger.info("Creating simulated Ability Hand client")
        
        # Create a minimal simulation of the AHSerialClient and Hand classes
        class SimulatedHand:
            def __init__(self):
                self.positions = [30, 30, 30, 30, 30, -30]  # Last one is thumb rotator
                self.velocities = [0, 0, 0, 0, 0, 0]
                self.currents = [0, 0, 0, 0, 0, 0]
            
            def get_position(self):
                return self.positions
                
            def get_velocity(self):
                return self.velocities
                
            def get_current(self):
                return self.currents
        
        class SimulatedClient:
            def __init__(self):
                self.hand = SimulatedHand()
                
            def set_position(self, positions, reply_mode=None):
                if isinstance(positions, list):
                    self.hand.positions = positions.copy()
                else:
                    self.hand.positions = [positions] * 5 + [-positions]
                    
            def set_velocity(self, velocities, reply_mode=None):
                if isinstance(velocities, list):
                    self.hand.velocities = velocities.copy()
                else:
                    self.hand.velocities = [velocities] * 5 + [-velocities]
                    
            def set_torque(self, currents, reply_mode=None):
                if isinstance(currents, list):
                    self.hand.currents = currents.copy()
                else:
                    self.hand.currents = [currents] * 5 + [-currents]
            
            def set_duty(self, duties, reply_mode=None):
                # Convert duty to current (simplified) and use torque method
                if isinstance(duties, list):
                    # Scale duty cycle (-100 to 100) to current (-1 to 1)
                    scaled_currents = [d/100.0 for d in duties]
                    self.hand.currents = scaled_currents.copy()
                else:
                    # Single value
                    scaled_current = duties/100.0
                    self.hand.currents = [scaled_current] * 5 + [-scaled_current]
                    
            def close(self):
                pass
                
        self.client = SimulatedClient()
        logger.info("Initialized simulated Ability Hand client")
    
    def _apply_control(self):
        """Apply control to the Ability Hand based on current mode"""
        try:
            # We need to collect commands for all fingers into arrays
            # since Ability Hand API expects all fingers to be controlled at once
            
            # First, determine the dominant control mode
            position_mode_count = 0
            velocity_mode_count = 0
            torque_mode_count = 0
            duty_mode_count = 0
            
            for finger in self.fingers:
                mode = self.control_modes[finger]
                if mode == ControlMode.POSITION:
                    position_mode_count += 1
                elif mode == ControlMode.VELOCITY:
                    velocity_mode_count += 1
                elif mode == ControlMode.TORQUE:
                    torque_mode_count += 1
                elif mode == ControlMode.DUTY:
                    duty_mode_count += 1
            
            # Get the mode with the most fingers
            max_count = max(position_mode_count, velocity_mode_count, 
                          torque_mode_count, duty_mode_count)
            
            if max_count == position_mode_count:
                # Position control mode
                self._apply_position_control()
            elif max_count == velocity_mode_count:
                # Velocity control mode
                self._apply_velocity_control()
            elif max_count == torque_mode_count:
                # Torque control mode
                self._apply_torque_control()
            elif max_count == duty_mode_count:
                # Duty cycle control mode
                self._apply_duty_control()
                
            # Update current derivatives for slip detection
            current_time = time.time()
            for finger in self.fingers:
                # Add current to history
                self.current_history[finger].append(self.current_feedback[finger])
                self.current_time_history[finger].append(current_time)
                
                # Keep history limited to 10 samples
                if len(self.current_history[finger]) > 10:
                    self.current_history[finger] = self.current_history[finger][-10:]
                    self.current_time_history[finger] = self.current_time_history[finger][-10:]
                
                # Calculate derivative if we have enough history
                if len(self.current_history[finger]) >= 5:
                    i_values = self.current_history[finger]
                    t_values = self.current_time_history[finger]
                    
                    # Calculate derivative using linear regression
                    n = len(i_values)
                    if t_values[-1] > t_values[0]:  # Avoid division by zero
                        dt = t_values[-1] - t_values[0]
                        di = i_values[-1] - i_values[0]
                        self.current_derivatives[finger] = di / dt
                
        except Exception as e:
            logger.error(f"Error in _apply_control: {e}")
            self.client_error = True
    
    def _apply_position_control(self):
        """Apply position control to all fingers"""
        try:
            # Collect position values for all fingers
            positions = [0.0] * 6  # 6 channels for Ability Hand
            
            # Regular fingers
            for finger, idx in self.finger_indices.items():
                positions[idx] = self.position_targets[finger]
            
            # Thumb rotator (negative value for rotation)
            positions[5] = -self.position_targets["ThumbRotate"]
            
            # Send to hand
            self.client.set_position(positions, reply_mode=self.reply_mode)
            
        except Exception as e:
            logger.error(f"Error in position control: {e}")
            self.client_error = True
    
    def _apply_velocity_control(self):
        """Apply velocity control to all fingers"""
        try:
            # Collect velocity values for all fingers
            velocities = [0.0] * 6  # 6 channels for Ability Hand
            
            # Regular fingers
            for finger, idx in self.finger_indices.items():
                velocities[idx] = self.velocity_targets[finger]
            
            # Thumb rotator (negative value for rotation)
            velocities[5] = -self.velocity_targets["ThumbRotate"]
            
            # Send to hand
            self.client.set_velocity(velocities, reply_mode=self.reply_mode)
            
        except Exception as e:
            logger.error(f"Error in velocity control: {e}")
            self.client_error = True
    
    def _apply_torque_control(self):
        """Apply torque (current) control to all fingers"""
        try:
            # Collect torque values for all fingers
            currents = [0.0] * 6  # 6 channels for Ability Hand
            
            # Regular fingers
            for finger, idx in self.finger_indices.items():
                currents[idx] = self.torque_targets[finger]
            
            # Thumb rotator (negative value for rotation)
            currents[5] = -self.torque_targets["ThumbRotate"]
            
            # Send to hand
            self.client.set_torque(currents, reply_mode=self.reply_mode)
            
        except Exception as e:
            logger.error(f"Error in torque control: {e}")
            self.client_error = True
    
    def _apply_duty_control(self):
        """Apply duty cycle control to all fingers"""
        try:
            # Collect duty values for all fingers
            duties = [0.0] * 6  # 6 channels for Ability Hand
            
            # Regular fingers
            for finger, idx in self.finger_indices.items():
                duties[idx] = self.duty_targets[finger]
            
            # Thumb rotator (negative value for rotation)
            duties[5] = -self.duty_targets["ThumbRotate"]
            
            # Send to hand
            self.client.set_duty(duties, reply_mode=self.reply_mode)
            
        except Exception as e:
            logger.error(f"Error in duty control: {e}")
            self.client_error = True
    
    def _update_feedback(self):
        """Update feedback from Ability Hand"""
        if self.client_error:
            # Try to recover the client connection
            self._initialize_client()
            return
            
        try:
            # Get position feedback
            try:
                positions = self.client.hand.get_position()
                if positions:
                    for finger, idx in self.finger_indices.items():
                        if idx < len(positions):
                            self.position_feedback[finger] = positions[idx]
                    
                    # Handle thumb rotator (value is negative)
                    if len(positions) > 5:
                        self.position_feedback["ThumbRotate"] = -positions[5]
            except:
                logger.debug("Failed to get position feedback")
                
            # Get velocity feedback
            try:
                velocities = self.client.hand.get_velocity()
                if velocities:
                    for finger, idx in self.finger_indices.items():
                        if idx < len(velocities):
                            self.velocity_feedback[finger] = velocities[idx]
                    
                    # Handle thumb rotator (value is negative)
                    if len(velocities) > 5:
                        self.velocity_feedback["ThumbRotate"] = -velocities[5]
            except:
                logger.debug("Failed to get velocity feedback")
                
            # Get current feedback
            try:
                currents = self.client.hand.get_current()
                if currents:
                    for finger, idx in self.finger_indices.items():
                        if idx < len(currents):
                            self.current_feedback[finger] = currents[idx]
                            # Approximate torque from current (simplified)
                            self.torque_feedback[finger] = currents[idx]
                    
                    # Handle thumb rotator (value is negative)
                    if len(currents) > 5:
                        self.current_feedback["ThumbRotate"] = -currents[5]
                        self.torque_feedback["ThumbRotate"] = -currents[5]
            except:
                logger.debug("Failed to get current feedback")
                
        except Exception as e:
            logger.error(f"Error updating feedback: {e}")
            self.client_error = True
    
    def start(self):
        """Start the control loop"""
        # Initialize/check the client before starting
        if self.client is None or self.client_error:
            self._initialize_client()
            
        super().start()
    
    def stop(self):
        """Stop the control loop and close the client"""
        super().stop()
        
        # Close the client
        if self.client:
            try:
                self.client.close()
            except:
                pass
            
        logger.info("Ability Hand interface stopped")
    
    def get_current_derivative(self, finger):
        """
        Get current derivative for slip detection.
        
        Args:
            finger: Name of finger
            
        Returns:
            Current derivative in A/s
        """
        return self.current_derivatives.get(finger, 0.0)
    
    def get_touch_sensors(self):
        """
        Get touch sensor values from the hand.
        
        Returns:
            Dictionary mapping finger to list of sensor values
        """
        try:
            # Touch sensors are in the hand.fsr attribute, if available
            if hasattr(self.client.hand, 'fsr'):
                # Reshape into a dictionary by finger
                touch_values = {}
                fsr_values = self.client.hand.fsr
                
                # Ability Hand has 30 touch sensors: 6 per finger
                if len(fsr_values) == 30:
                    for finger, idx in self.finger_indices.items():
                        start_idx = idx * 6
                        touch_values[finger] = fsr_values[start_idx:start_idx+6]
                        
                return touch_values
            else:
                return {}
        except:
            return {}

# Simple test code
if __name__ == "__main__":
    print("Testing Ability Hand interface...")
    hand = AbilityHandInterface(control_rate=20)
    hand.start()
    
    try:
        # Test position control
        print("\nTesting position control...")
        hand.set_position("Thumb", 50.0)
        hand.set_position("ThumbRotate", 50.0)
        hand.set_position("Index", 70.0)
        
        for i in range(20):
            thumb_status = hand.get_finger_status("Thumb")
            thumb_rotate_status = hand.get_finger_status("ThumbRotate")
            index_status = hand.get_finger_status("Index")
            
            print(f"Thumb: {thumb_status['position']:.1f}°, Rotate: {thumb_rotate_status['position']:.1f}°, "
                  f"Index: {index_status['position']:.1f}°")
            time.sleep(0.2)
        
        # Test torque control
        print("\nTesting torque control...")
        hand.set_torque("Thumb", 0.2)
        hand.set_torque("Index", 0.3)
        
        for i in range(10):
            thumb_status = hand.get_finger_status("Thumb")
            index_status = hand.get_finger_status("Index")
            
            print(f"Thumb: {thumb_status['torque']:.2f}Nm, {thumb_status['current']:.2f}A, "
                  f"Index: {index_status['torque']:.2f}Nm, {index_status['current']:.2f}A")
            time.sleep(0.2)
    
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    
    finally:
        hand.stop()
        print("Test complete.")