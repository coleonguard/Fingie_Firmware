#!/usr/bin/env python3
"""
Motor Interface for Prosthetic Control.

This module defines the abstract MotorInterface class and a simulation
implementation, providing a common interface for controlling any prosthetic
hand hardware.
"""

import time
import threading
import logging
import numpy as np
from enum import Enum
from typing import Dict, List, Optional, Union, Tuple

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("MotorInterface")

class ControlMode(Enum):
    """Control modes for the prosthetic hand"""
    POSITION = 0    # Position control
    VELOCITY = 1    # Velocity control
    TORQUE = 2      # Torque/current control
    DUTY = 3        # Duty cycle control

class MotorInterface:
    """
    Abstract base class for motor control interfaces.
    
    This class defines the interface that all motor controllers must implement.
    Specific implementations will inherit from this class and provide the
    actual hardware control logic.
    """
    
    def __init__(self, control_rate=50):
        """
        Initialize the motor interface.
        
        Args:
            control_rate: Control loop frequency in Hz
        """
        # Control parameters
        self.control_rate = control_rate
        self.control_interval = 1.0 / control_rate
        
        # Thread control
        self.running = False
        self.thread = None
        self.last_update_time = 0.0
        self.command_watchdog_timeout = 0.1  # 100ms
        
        # Motor data - using standard finger naming
        self.fingers = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
        
        # Target values (what we're commanding)
        self.position_targets = {finger: 0.0 for finger in self.fingers}
        self.velocity_targets = {finger: 0.0 for finger in self.fingers}
        self.torque_targets = {finger: 0.0 for finger in self.fingers}
        self.duty_targets = {finger: 0.0 for finger in self.fingers}
        
        # Previous targets for rate limiting
        self.prev_position_targets = {finger: 0.0 for finger in self.fingers}
        self.prev_torque_targets = {finger: 0.0 for finger in self.fingers}
        
        # Feedback values (what we're reading back)
        self.position_feedback = {finger: 0.0 for finger in self.fingers}
        self.velocity_feedback = {finger: 0.0 for finger in self.fingers}
        self.torque_feedback = {finger: 0.0 for finger in self.fingers}
        self.current_feedback = {finger: 0.0 for finger in self.fingers}
        
        # Current control mode
        self.control_modes = {finger: ControlMode.POSITION for finger in self.fingers}
        
        # Safety limits
        self.max_position = 100.0  # Maximum position value (degrees or normalized)
        self.max_position_rate = 8.0  # Maximum position change per update (degrees)
        self.max_velocity = 100.0  # Maximum velocity value (degrees/s or normalized)
        self.max_torque = 0.6      # Maximum torque value (Nm or normalized)
        self.max_torque_rate = 0.05  # Maximum torque change per update (Nm)
        self.max_current = 0.6     # Maximum current value (A)
        self.max_duty = 100.0      # Maximum duty cycle (%)
        
        # Fault detection
        self.fault_status = {
            "comm_loss": False,
            "over_current": False,
            "over_temp": False,
            "watchdog": False
        }
    
    def _control_loop(self):
        """Main control loop that runs at the specified rate"""
        next_update_time = time.time()
        self.last_update_time = next_update_time
        
        while self.running:
            current_time = time.time()
            
            # Check if it's time for the next control update
            if current_time >= next_update_time:
                # Check for command timeout
                time_since_update = current_time - self.last_update_time
                if time_since_update > self.command_watchdog_timeout:
                    logger.warning(f"Command timeout: {time_since_update:.3f}s > {self.command_watchdog_timeout:.3f}s")
                    # Set all torques to 0 for safety
                    for finger in self.fingers:
                        self.torque_targets[finger] = 0.0
                        self.control_modes[finger] = ControlMode.TORQUE
                    # Set fault flag
                    self.fault_status["watchdog"] = True
                
                # Store previous targets for rate limiting
                for finger in self.fingers:
                    self.prev_position_targets[finger] = self.position_targets[finger]
                    self.prev_torque_targets[finger] = self.torque_targets[finger]
                
                # Apply rate limits to position and torque targets
                self._apply_rate_limits()
                
                # Apply control based on current mode
                self._apply_control()
                
                # Update feedback
                self._update_feedback()
                
                # Update last command time if no watchdog fault
                if not self.fault_status["watchdog"]:
                    self.last_update_time = current_time
                
                # Calculate next update time
                next_update_time = current_time + self.control_interval
            
            # Small sleep to prevent CPU hogging
            time.sleep(0.001)
    
    def _apply_rate_limits(self):
        """Apply rate limits to position and torque targets"""
        for finger in self.fingers:
            # Position rate limiting
            if self.control_modes[finger] == ControlMode.POSITION:
                prev_pos = self.prev_position_targets[finger]
                target_pos = self.position_targets[finger]
                delta_pos = target_pos - prev_pos
                
                # Limit position change rate
                if abs(delta_pos) > self.max_position_rate:
                    # Clamp the change
                    direction = 1 if delta_pos > 0 else -1
                    limited_pos = prev_pos + direction * self.max_position_rate
                    self.position_targets[finger] = limited_pos
                    logger.debug(f"Rate limited {finger} position: {target_pos:.1f} -> {limited_pos:.1f}")
            
            # Torque rate limiting
            elif self.control_modes[finger] == ControlMode.TORQUE:
                prev_torque = self.prev_torque_targets[finger]
                target_torque = self.torque_targets[finger]
                delta_torque = target_torque - prev_torque
                
                # Limit torque change rate
                if abs(delta_torque) > self.max_torque_rate:
                    # Clamp the change
                    direction = 1 if delta_torque > 0 else -1
                    limited_torque = prev_torque + direction * self.max_torque_rate
                    self.torque_targets[finger] = limited_torque
                    logger.debug(f"Rate limited {finger} torque: {target_torque:.3f} -> {limited_torque:.3f}")
    
    def _apply_control(self):
        """
        Apply control actions based on current mode and targets.
        
        This method must be overridden by subclasses to implement the
        actual hardware control logic.
        """
        raise NotImplementedError("Subclasses must implement _apply_control")
    
    def _update_feedback(self):
        """
        Update feedback values from hardware.
        
        This method must be overridden by subclasses to implement the
        actual hardware feedback logic.
        """
        raise NotImplementedError("Subclasses must implement _update_feedback")
    
    def start(self):
        """Start the control loop thread"""
        if not self.running:
            # Reset fault status
            for fault in self.fault_status:
                self.fault_status[fault] = False
                
            self.running = True
            self.thread = threading.Thread(target=self._control_loop)
            self.thread.daemon = True
            self.thread.start()
            logger.info(f"{self.__class__.__name__} started")
    
    def stop(self):
        """Stop the control loop thread and set all targets to zero"""
        # Zero all targets
        for finger in self.fingers:
            self.set_position(finger, 0.0)
            self.set_velocity(finger, 0.0)
            self.set_torque(finger, 0.0)
            self.set_duty(finger, 0.0)
        
        # Apply zero commands
        if hasattr(self, '_apply_control'):
            self._apply_control()
        
        # Stop the thread
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
            
        logger.info(f"{self.__class__.__name__} stopped")
    
    def set_position(self, finger, position):
        """
        Set position target for a finger.
        
        Args:
            finger: Name of finger to control
            position: Target position (degrees or normalized)
        """
        if finger in self.fingers:
            # Clamp to safe range
            safe_position = max(0.0, min(position, self.max_position))
            self.position_targets[finger] = safe_position
            self.control_modes[finger] = ControlMode.POSITION
            
            # Reset watchdog timeout
            self.last_update_time = time.time()
            self.fault_status["watchdog"] = False
    
    def set_velocity(self, finger, velocity):
        """
        Set velocity target for a finger.
        
        Args:
            finger: Name of finger to control
            velocity: Target velocity (degrees/s or normalized)
        """
        if finger in self.fingers:
            # Clamp to safe range
            safe_velocity = max(-self.max_velocity, min(velocity, self.max_velocity))
            self.velocity_targets[finger] = safe_velocity
            self.control_modes[finger] = ControlMode.VELOCITY
            
            # Reset watchdog timeout
            self.last_update_time = time.time()
            self.fault_status["watchdog"] = False
    
    def set_torque(self, finger, torque):
        """
        Set torque target for a finger.
        
        Args:
            finger: Name of finger to control
            torque: Target torque (Nm or normalized)
        """
        if finger in self.fingers:
            # Clamp to safe range
            safe_torque = max(-self.max_torque, min(torque, self.max_torque))
            self.torque_targets[finger] = safe_torque
            self.control_modes[finger] = ControlMode.TORQUE
            
            # Reset watchdog timeout
            self.last_update_time = time.time()
            self.fault_status["watchdog"] = False
    
    def set_duty(self, finger, duty):
        """
        Set duty cycle target for a finger.
        
        Args:
            finger: Name of finger to control
            duty: Target duty cycle (%)
        """
        if finger in self.fingers:
            # Clamp to safe range
            safe_duty = max(-self.max_duty, min(duty, self.max_duty))
            self.duty_targets[finger] = safe_duty
            self.control_modes[finger] = ControlMode.DUTY
            
            # Reset watchdog timeout
            self.last_update_time = time.time()
            self.fault_status["watchdog"] = False
    
    def get_position(self, finger):
        """
        Get current position feedback for a finger.
        
        Args:
            finger: Name of finger
            
        Returns:
            Current position
        """
        return self.position_feedback.get(finger, 0.0)
    
    def get_velocity(self, finger):
        """
        Get current velocity feedback for a finger.
        
        Args:
            finger: Name of finger
            
        Returns:
            Current velocity
        """
        return self.velocity_feedback.get(finger, 0.0)
    
    def get_torque(self, finger):
        """
        Get current torque feedback for a finger.
        
        Args:
            finger: Name of finger
            
        Returns:
            Current torque
        """
        return self.torque_feedback.get(finger, 0.0)
    
    def get_current(self, finger):
        """
        Get current feedback for a finger.
        
        Args:
            finger: Name of finger
            
        Returns:
            Current in Amperes
        """
        return self.current_feedback.get(finger, 0.0)
    
    def get_finger_status(self, finger):
        """
        Get comprehensive status for a finger.
        
        Args:
            finger: Name of finger
            
        Returns:
            Dictionary with current status
        """
        if finger not in self.fingers:
            return {"error": "Invalid finger name"}
            
        return {
            "finger": finger,
            "mode": self.control_modes[finger].name,
            "position": self.position_feedback[finger],
            "velocity": self.velocity_feedback[finger],
            "torque": self.torque_feedback[finger],
            "current": self.current_feedback[finger],
            "position_target": self.position_targets[finger],
            "velocity_target": self.velocity_targets[finger],
            "torque_target": self.torque_targets[finger],
            "duty_target": self.duty_targets[finger]
        }
    
    def get_fault_status(self):
        """
        Get current fault status.
        
        Returns:
            Dictionary with fault conditions
        """
        return self.fault_status.copy()

class SimulatedMotorInterface(MotorInterface):
    """
    Simulated motor interface for testing without hardware.
    
    This class provides a realistic simulation of motor dynamics
    for testing the controller without actual hardware.
    """
    
    def __init__(self, control_rate=50):
        """
        Initialize the simulated motor interface.
        
        Args:
            control_rate: Control loop frequency in Hz
        """
        super().__init__(control_rate)
        
        # Simulation parameters
        self.position_time_constant = 0.2  # Time constant for position changes (s)
        self.velocity_time_constant = 0.1  # Time constant for velocity changes (s)
        self.torque_time_constant = 0.05   # Time constant for torque changes (s)
        
        # Motor dynamics model parameters
        self.motor_damping = 0.8          # Damping coefficient
        self.motor_inertia = 0.5          # Inertia coefficient
        self.motor_efficiency = 0.7       # Motor efficiency for torque/current
        
        # Current derivatives for slip detection
        self.current_history = {finger: [] for finger in self.fingers}
        self.current_time_history = {finger: [] for finger in self.fingers}
        self.current_derivatives = {finger: 0.0 for finger in self.fingers}
    
    def _apply_control(self):
        """Apply simulated control based on current mode"""
        # Apply control based on mode for each finger
        for finger in self.fingers:
            # Get current mode
            mode = self.control_modes[finger]
            
            # Apply based on mode
            if mode == ControlMode.POSITION:
                self._apply_position_control(finger)
            elif mode == ControlMode.VELOCITY:
                self._apply_velocity_control(finger)
            elif mode == ControlMode.TORQUE:
                self._apply_torque_control(finger)
            elif mode == ControlMode.DUTY:
                self._apply_duty_control(finger)
        
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
    
    def _apply_position_control(self, finger):
        """Apply position control for a finger"""
        # Simple first-order response to target
        current = self.position_feedback[finger]
        target = self.position_targets[finger]
        error = target - current
        
        # Update based on time constant
        dt = self.control_interval
        rate = 1.0 / self.position_time_constant
        new_position = current + error * rate * dt
        
        # Update feedback
        self.position_feedback[finger] = new_position
        
        # Calculate implied velocity
        self.velocity_feedback[finger] = error * rate
        
        # Calculate implied torque (proportional to error and velocity)
        torque = error * 0.01 + self.velocity_feedback[finger] * 0.001
        self.torque_feedback[finger] = max(-self.max_torque, min(torque, self.max_torque))
        
        # Calculate implied current (proportional to torque)
        self.current_feedback[finger] = self.torque_feedback[finger] / self.motor_efficiency
    
    def _apply_velocity_control(self, finger):
        """Apply velocity control for a finger"""
        # Simple first-order response to target
        current_vel = self.velocity_feedback[finger]
        target_vel = self.velocity_targets[finger]
        vel_error = target_vel - current_vel
        
        # Update velocity based on time constant
        dt = self.control_interval
        rate = 1.0 / self.velocity_time_constant
        new_velocity = current_vel + vel_error * rate * dt
        
        # Update position based on velocity
        current_pos = self.position_feedback[finger]
        new_position = current_pos + new_velocity * dt
        new_position = max(0.0, min(new_position, self.max_position))
        
        # Update feedback
        self.velocity_feedback[finger] = new_velocity
        self.position_feedback[finger] = new_position
        
        # Calculate implied torque (proportional to velocity)
        torque = new_velocity * 0.01
        self.torque_feedback[finger] = max(-self.max_torque, min(torque, self.max_torque))
        
        # Calculate implied current
        self.current_feedback[finger] = self.torque_feedback[finger] / self.motor_efficiency
    
    def _apply_torque_control(self, finger):
        """Apply torque control for a finger"""
        # Target torque
        target_torque = self.torque_targets[finger]
        
        # Update torque with simple first-order response
        current_torque = self.torque_feedback[finger]
        torque_error = target_torque - current_torque
        
        dt = self.control_interval
        rate = 1.0 / self.torque_time_constant
        new_torque = current_torque + torque_error * rate * dt
        
        # Update velocity based on torque and damping
        current_vel = self.velocity_feedback[finger]
        accel = new_torque / self.motor_inertia - current_vel * self.motor_damping
        new_velocity = current_vel + accel * dt
        
        # Update position based on velocity
        current_pos = self.position_feedback[finger]
        new_position = current_pos + new_velocity * dt
        new_position = max(0.0, min(new_position, self.max_position))
        
        # Update feedback
        self.torque_feedback[finger] = new_torque
        self.velocity_feedback[finger] = new_velocity
        self.position_feedback[finger] = new_position
        
        # Calculate implied current
        self.current_feedback[finger] = new_torque / self.motor_efficiency
    
    def _apply_duty_control(self, finger):
        """Apply duty cycle control for a finger"""
        # Convert duty to torque (simplified)
        duty = self.duty_targets[finger]
        implied_torque = duty / 100.0 * self.max_torque
        
        # Now use torque control
        self.torque_targets[finger] = implied_torque
        self._apply_torque_control(finger)
    
    def _update_feedback(self):
        """Update feedback values (already done in _apply_control for simulation)"""
        # Nothing to do here, since we already updated feedback in _apply_control
        pass
    
    def get_current_derivative(self, finger):
        """
        Get the current derivative for slip detection.
        
        Args:
            finger: Name of finger
            
        Returns:
            Current derivative in A/s
        """
        return self.current_derivatives.get(finger, 0.0)
    
    def simulate_slip(self, finger, slip_amount=0.1):
        """
        Simulate a slip event for testing.
        
        Args:
            finger: Name of finger
            slip_amount: Amount of current drop (A)
        """
        if finger in self.fingers:
            # Reduce current feedback to simulate slip
            self.current_feedback[finger] -= slip_amount
            
            # Update current history to ensure derivative shows the slip
            current_time = time.time()
            self.current_history[finger].append(self.current_feedback[finger])
            self.current_time_history[finger].append(current_time)
            
            # Keep history limited
            if len(self.current_history[finger]) > 10:
                self.current_history[finger] = self.current_history[finger][-10:]
                self.current_time_history[finger] = self.current_time_history[finger][-10:]
            
            logger.info(f"Simulated slip on {finger}, current: {self.current_feedback[finger]:.3f}A")

# Simple test code
if __name__ == "__main__":
    print("Testing simulated motor interface...")
    motors = SimulatedMotorInterface(control_rate=50)
    motors.start()
    
    try:
        # Test position control
        print("\nTesting position control...")
        motors.set_position("Thumb", 50.0)
        motors.set_position("Index", 70.0)
        for _ in range(10):
            thumb_status = motors.get_finger_status("Thumb")
            index_status = motors.get_finger_status("Index")
            print(f"Thumb: {thumb_status['position']:.1f}°, Index: {index_status['position']:.1f}°")
            time.sleep(0.1)
        
        # Test torque control
        print("\nTesting torque control...")
        motors.set_torque("Thumb", 0.2)
        motors.set_torque("Index", 0.3)
        for _ in range(10):
            thumb_status = motors.get_finger_status("Thumb")
            index_status = motors.get_finger_status("Index")
            print(f"Thumb: {thumb_status['torque']:.2f}Nm, {thumb_status['current']:.2f}A, "
                  f"Index: {index_status['torque']:.2f}Nm, {index_status['current']:.2f}A")
            time.sleep(0.1)
        
        # Test slip detection
        print("\nTesting slip detection...")
        motors.set_torque("Thumb", 0.3)
        time.sleep(0.5)  # Let it stabilize
        
        # Now simulate a slip
        motors.simulate_slip("Thumb", 0.15)
        time.sleep(0.1)  # Let derivative calculation happen
        
        derivative = motors.get_current_derivative("Thumb")
        print(f"Thumb current derivative: {derivative:.3f} A/s")
    
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    
    finally:
        motors.stop()
        print("Test complete.")