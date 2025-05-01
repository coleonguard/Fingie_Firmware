#!/usr/bin/env python3
# Ability Hand Motor Interface

import time
import threading
import numpy as np
from typing import List, Dict, Union, Optional, Tuple
import sys
import os

# Import from motor_interface.py in the same directory
from motor_interface import MotorInterface, ControlMode

class AbilityHandInterface(MotorInterface):
    """
    Interface for the PSYONIC Ability Hand
    
    This class implements the MotorInterface for the PSYONIC Ability Hand,
    using the AHSerialClient from the ability-hand-api package.
    """
    
    def __init__(self, 
                 control_rate: int = 50, 
                 port: str = None, 
                 baud_rate: int = 460800, 
                 reply_mode: int = 2):
        """
        Initialize the Ability Hand interface
        
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
        self.max_torque = 1.0      # Maximum torque (current) value
        
        # Flag to indicate if we need to reset client
        self.client_error = False
    
    def _initialize_client(self):
        """Initialize the Ability Hand client"""
        try:
            # We'll import here to avoid dependency issues if the package isn't available
            try:
                from ah_wrapper.ah_serial_client import AHSerialClient
                self.client = AHSerialClient(
                    port=self.port,
                    baud_rate=self.baud_rate,
                    reply_mode=self.reply_mode,
                    rate_hz=self.control_rate
                )
                print("Successfully connected to Ability Hand")
                self.client_error = False
            except ImportError:
                print("WARNING: Could not import AHSerialClient. Using simulated mode.")
                print("To use real hardware, install the ability-hand-api package.")
                self._simulate_client()
        except Exception as e:
            print(f"ERROR: Could not initialize Ability Hand: {e}")
            self._simulate_client()
    
    def _simulate_client(self):
        """Create a simulated client for testing"""
        self.client_error = True
        
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
                    
            def close(self):
                pass
                
        self.client = SimulatedClient()
        print("Initialized simulated Ability Hand client")
    
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
                
        except Exception as e:
            print(f"Error in _apply_control: {e}")
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
            print(f"Error in position control: {e}")
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
            print(f"Error in velocity control: {e}")
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
            print(f"Error in torque control: {e}")
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
            print(f"Error in duty control: {e}")
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
                pass
                
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
                pass
                
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
                pass
                
        except Exception as e:
            print(f"Error updating feedback: {e}")
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
            
        print("Ability Hand interface stopped")
    
    def get_touch_sensors(self):
        """Get touch sensor values from the hand
        
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