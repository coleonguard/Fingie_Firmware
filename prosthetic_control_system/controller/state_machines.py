#!/usr/bin/env python3
"""
State Machines for Prosthetic Hand Control.

This module defines the state machines used by the unified controller:
1. Finger FSM: Manages individual finger states (APPROACH/PROPORTIONAL/CONTACT)
2. Hand FSM: Manages overall hand states (REACH/GRASP/LIFT/LOWER/RELEASE)
"""

import time
import logging
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple, Any

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("StateMachines")

class FingerState(Enum):
    """States for individual finger control"""
    IDLE = auto()           # No control, motors idle
    APPROACH = auto()       # Beyond threshold, no movement
    PROPORTIONAL = auto()   # Between threshold and contact, proportional control
    CONTACT = auto()        # Object contact detected, torque control

class HandState(Enum):
    """States for overall hand control"""
    IDLE = auto()           # Hand at rest, no active control
    REACH = auto()          # Hand reaching toward object (positioning)
    GRASP = auto()          # Hand actively grasping object
    LIFT = auto()           # Lifting grasped object
    LOWER = auto()          # Lowering object to surface
    RELEASE = auto()        # Releasing object
    RETRACT = auto()        # Retracting hand after release

class FingerFSM:
    """
    Finite State Machine for individual finger control.
    
    This class manages the state transitions for each finger based on
    proximity sensor data and motor feedback.
    """
    
    def __init__(
        self,
        finger_name: str,
        approach_threshold: float = 40.0,  # mm
        contact_threshold: float = 5.0,    # mm
        max_position: float = 100.0,       # degrees
        initial_torque: float = 0.3,       # A
        max_torque: float = 0.6,           # A
        slip_detection_threshold: float = -0.05,  # A/s
        slip_consecutive_cycles: int = 2,
        high_current_threshold: float = 0.45     # A
    ):
        """
        Initialize the finger state machine.
        
        Args:
            finger_name: Name of this finger
            approach_threshold: Distance threshold for approach phase (mm)
            contact_threshold: Distance threshold for contact phase (mm)
            max_position: Maximum finger position (degrees)
            initial_torque: Initial torque when contact is detected (A)
            max_torque: Maximum allowable torque (A)
            slip_detection_threshold: Current derivative threshold for slip detection (A/s)
            slip_consecutive_cycles: Required consecutive cycles to confirm slip
            high_current_threshold: Threshold for current reduction (A)
        """
        self.finger_name = finger_name
        self.approach_threshold = approach_threshold
        self.contact_threshold = contact_threshold
        self.max_position = max_position
        self.initial_torque = initial_torque
        self.max_torque = max_torque
        self.slip_detection_threshold = slip_detection_threshold
        self.slip_consecutive_cycles = slip_consecutive_cycles
        self.high_current_threshold = high_current_threshold
        
        # Current state
        self.state = FingerState.IDLE
        self.previous_state = FingerState.IDLE
        self.state_entry_time = time.time()
        
        # Slip detection
        self.slip_counter = 0
        self.previous_current = 0.0
        
        # Target values
        self.position_target = 0.0
        self.torque_target = 0.0
        
        # Logging
        logger.info(f"Initialized {finger_name} FSM")
    
    def update(
        self, 
        distance: float, 
        current: float, 
        current_derivative: float,
        position: float = None
    ) -> Tuple[FingerState, float, float]:
        """
        Update the finger state based on sensor data.
        
        Args:
            distance: Current distance reading (mm)
            current: Current motor current (A)
            current_derivative: Rate of change of current (A/s)
            position: Current finger position (degrees), optional
            
        Returns:
            Tuple of (state, position_target, torque_target)
        """
        # Save current state as previous
        self.previous_state = self.state
        
        # State transitions
        if self.state == FingerState.IDLE:
            # From IDLE, transition to APPROACH if we have valid distance data
            if distance is not None:
                self.state = FingerState.APPROACH
                self.state_entry_time = time.time()
                logger.debug(f"{self.finger_name}: IDLE → APPROACH")
        
        elif self.state == FingerState.APPROACH:
            # From APPROACH:
            # - Stay in APPROACH if distance > threshold
            # - Go to PROPORTIONAL if distance < threshold
            if distance is not None and distance < self.approach_threshold:
                self.state = FingerState.PROPORTIONAL
                self.state_entry_time = time.time()
                logger.debug(f"{self.finger_name}: APPROACH → PROPORTIONAL (d={distance:.1f}mm)")
        
        elif self.state == FingerState.PROPORTIONAL:
            # From PROPORTIONAL:
            # - Go back to APPROACH if distance > threshold
            # - Go to CONTACT if distance <= contact_threshold
            # - Go to CONTACT if current derivative indicates contact
            
            # Check for approach transition
            if distance is not None and distance >= self.approach_threshold:
                self.state = FingerState.APPROACH
                self.state_entry_time = time.time()
                logger.debug(f"{self.finger_name}: PROPORTIONAL → APPROACH (d={distance:.1f}mm)")
            
            # Check for contact transition based on distance
            elif distance is not None and distance <= self.contact_threshold:
                self.state = FingerState.CONTACT
                self.torque_target = self.initial_torque
                self.state_entry_time = time.time()
                logger.debug(f"{self.finger_name}: PROPORTIONAL → CONTACT (distance threshold)")
            
            # Check for contact transition based on current derivative
            elif current_derivative < self.slip_detection_threshold:
                # Sharp decrease in current can indicate contact
                self.state = FingerState.CONTACT
                self.torque_target = self.initial_torque
                self.state_entry_time = time.time()
                logger.debug(
                    f"{self.finger_name}: PROPORTIONAL → CONTACT "
                    f"(current derivative: {current_derivative:.3f}A/s)"
                )
        
        elif self.state == FingerState.CONTACT:
            # In CONTACT state:
            # - Stay in CONTACT
            # - Adjust torque based on feedback
            # - Return to PROPORTIONAL only on explicit reset
            
            # High current reduction
            if current > self.high_current_threshold:
                # Reduce torque by 10%
                self.torque_target = 0.9 * self.torque_target
                logger.debug(
                    f"{self.finger_name}: Reducing torque due to high current "
                    f"({current:.3f}A > {self.high_current_threshold:.3f}A)"
                )
            
            # Slip detection
            if current_derivative < self.slip_detection_threshold:
                self.slip_counter += 1
                if self.slip_counter >= self.slip_consecutive_cycles:
                    # Increase torque to counter slip
                    increased_torque = min(self.torque_target + 0.05, self.max_torque)
                    if increased_torque > self.torque_target:
                        logger.debug(
                            f"{self.finger_name}: Increasing torque due to slip "
                            f"({current_derivative:.3f}A/s) "
                            f"from {self.torque_target:.3f}A to {increased_torque:.3f}A"
                        )
                        self.torque_target = increased_torque
                    
                    # Reset slip counter
                    self.slip_counter = 0
            else:
                # Reset slip counter if no slip detected
                self.slip_counter = 0
        
        # Calculate appropriate targets based on state
        if self.state == FingerState.IDLE or self.state == FingerState.APPROACH:
            # Open position in IDLE and APPROACH
            self.position_target = 0.0
            self.torque_target = 0.0
            
        elif self.state == FingerState.PROPORTIONAL:
            # Calculate position based on distance
            if distance is not None:
                # Normalize distance to [0, 1] range
                normalized_distance = max(0.0, min(1.0, 
                    (self.approach_threshold - distance) / 
                    (self.approach_threshold - self.contact_threshold)
                ))
                # Map to position range
                self.position_target = normalized_distance * self.max_position
            
        # Return current state and targets
        return (self.state, self.position_target, self.torque_target)
    
    def force_state(self, new_state: FingerState):
        """
        Force the finger into a specific state.
        
        Args:
            new_state: The state to force
        """
        if new_state != self.state:
            logger.debug(f"{self.finger_name}: Forced {self.state.name} → {new_state.name}")
            
            # Special handling for entering CONTACT state
            if new_state == FingerState.CONTACT:
                self.torque_target = self.initial_torque
            
            # Update state
            self.previous_state = self.state
            self.state = new_state
            self.state_entry_time = time.time()
    
    def reset(self):
        """Reset the finger to IDLE state"""
        self.force_state(FingerState.IDLE)
        self.position_target = 0.0
        self.torque_target = 0.0
        self.slip_counter = 0
        logger.debug(f"{self.finger_name}: Reset to IDLE state")

class HandFSM:
    """
    Finite State Machine for overall hand control.
    
    This class manages the high-level states of the hand during the
    reach → grasp → lift → lower → release sequence.
    """
    
    def __init__(self,
                 release_current_threshold: float = 0.2,  # A
                 required_fingers_grasping: int = 2):
        """
        Initialize the hand state machine.
        
        Args:
            release_current_threshold: Current threshold below which release is allowed
            required_fingers_grasping: Minimum fingers required to be in CONTACT state
                                      for a grasp to be considered active
        """
        self.state = HandState.IDLE
        self.previous_state = HandState.IDLE
        self.state_entry_time = time.time()
        
        # Configuration
        self.release_current_threshold = release_current_threshold
        self.required_fingers_grasping = required_fingers_grasping
        
        # Logging
        logger.info("Initialized Hand FSM")
    
    def update(
        self,
        finger_states: Dict[str, FingerState],
        finger_currents: Dict[str, float],
        is_lowering: bool,
        is_impact_detected: bool,
        is_stationary: bool,
        time_since_impact: float
    ) -> HandState:
        """
        Update the hand state based on sensor data and finger states.
        
        Args:
            finger_states: Dictionary mapping finger names to their states
            finger_currents: Dictionary mapping finger names to their currents
            is_lowering: Whether IMU indicates hand is lowering
            is_impact_detected: Whether IMU indicates impact was detected
            is_stationary: Whether IMU indicates hand is stationary
            time_since_impact: Time elapsed since impact was detected
            
        Returns:
            Current hand state
        """
        # Save current state as previous
        self.previous_state = self.state
        
        # Count fingers in each state
        n_contact = sum(1 for state in finger_states.values() if state == FingerState.CONTACT)
        n_proportional = sum(1 for state in finger_states.values() if state == FingerState.PROPORTIONAL)
        n_approach = sum(1 for state in finger_states.values() if state == FingerState.APPROACH)
        
        # Check if all finger currents are below threshold for release
        all_currents_low = all(current < self.release_current_threshold 
                              for current in finger_currents.values())
        
        # State transitions
        if self.state == HandState.IDLE:
            # From IDLE:
            # - Go to REACH if any finger is in APPROACH or PROPORTIONAL
            if n_approach > 0 or n_proportional > 0:
                self.state = HandState.REACH
                self.state_entry_time = time.time()
                logger.debug(f"Hand: IDLE → REACH")
        
        elif self.state == HandState.REACH:
            # From REACH:
            # - Go to GRASP if enough fingers are in CONTACT
            # - Go back to IDLE if all fingers are in APPROACH/IDLE
            if n_contact >= self.required_fingers_grasping:
                self.state = HandState.GRASP
                self.state_entry_time = time.time()
                logger.debug(f"Hand: REACH → GRASP (fingers in contact: {n_contact})")
            elif n_contact == 0 and n_proportional == 0:
                self.state = HandState.IDLE
                self.state_entry_time = time.time()
                logger.debug(f"Hand: REACH → IDLE (lost object)")
        
        elif self.state == HandState.GRASP:
            # From GRASP:
            # - Go to LIFT if we detect upward movement
            # - Go to LOWER if we detect downward movement
            # - Go back to REACH if not enough fingers are in CONTACT
            if n_contact < self.required_fingers_grasping:
                self.state = HandState.REACH
                self.state_entry_time = time.time()
                logger.debug(f"Hand: GRASP → REACH (fingers in contact: {n_contact})")
            elif is_lowering:
                # Reset impact detection when entering LOWER state
                # This ensures we only detect impacts that occur during lowering
                self.state = HandState.LOWER
                self.state_entry_time = time.time()
                logger.debug(f"Hand: GRASP → LOWER (lowering detected)")
            # Note: We don't have a specific condition for LIFT currently
        
        elif self.state == HandState.LIFT:
            # From LIFT:
            # - Go to LOWER if we detect downward movement
            # - Go back to GRASP if not enough fingers are in CONTACT
            if is_lowering:
                self.state = HandState.LOWER
                self.state_entry_time = time.time()
                logger.debug(f"Hand: LIFT → LOWER (lowering detected)")
            elif n_contact < self.required_fingers_grasping:
                self.state = HandState.GRASP
                self.state_entry_time = time.time()
                logger.debug(f"Hand: LIFT → GRASP (fingers in contact: {n_contact})")
        
        elif self.state == HandState.LOWER:
            # From LOWER:
            # - Go to RELEASE if impact detected + stationary + currents low
            # - Go back to GRASP if not enough fingers are in CONTACT
            # - Go back to GRASP if not lowering anymore
            if is_impact_detected and is_stationary and all_currents_low:
                self.state = HandState.RELEASE
                self.state_entry_time = time.time()
                logger.debug(f"Hand: LOWER → RELEASE (impact + stationary + low current)")
            elif n_contact < self.required_fingers_grasping:
                self.state = HandState.GRASP
                self.state_entry_time = time.time()
                logger.debug(f"Hand: LOWER → GRASP (fingers in contact: {n_contact})")
            elif not is_lowering:
                self.state = HandState.GRASP
                self.state_entry_time = time.time()
                logger.debug(f"Hand: LOWER → GRASP (no longer lowering)")
        
        elif self.state == HandState.RELEASE:
            # From RELEASE:
            # - Always go to RETRACT (one-way transition)
            self.state = HandState.RETRACT
            self.state_entry_time = time.time()
            logger.debug(f"Hand: RELEASE → RETRACT")
        
        elif self.state == HandState.RETRACT:
            # From RETRACT:
            # - Go to IDLE when all fingers fully open
            if n_contact == 0 and n_proportional == 0:
                self.state = HandState.IDLE
                self.state_entry_time = time.time()
                logger.debug(f"Hand: RETRACT → IDLE")
        
        return self.state
    
    def force_state(self, new_state: HandState):
        """
        Force the hand into a specific state.
        
        Args:
            new_state: The state to force
        """
        if new_state != self.state:
            logger.debug(f"Hand: Forced {self.state.name} → {new_state.name}")
            self.previous_state = self.state
            self.state = new_state
            self.state_entry_time = time.time()
    
    def reset(self):
        """Reset the hand to IDLE state"""
        self.force_state(HandState.IDLE)
        logger.debug("Hand: Reset to IDLE state")

# Test code
if __name__ == "__main__":
    print("Testing state machines...")
    
    # Create a finger FSM
    finger = FingerFSM("Index")
    
    # Create a hand FSM
    hand = HandFSM()
    
    # Simulate some state changes
    print("\nSimulating finger movements:")
    states = [
        (50.0, 0.0, 0.0),    # Beyond approach threshold
        (35.0, 0.0, 0.0),    # Within proportional zone
        (20.0, 0.0, 0.0),    # Closer
        (10.0, 0.1, -0.01),  # Getting close
        (4.0, 0.2, -0.02),   # Contact by distance
        (None, 0.3, -0.01),  # Contact maintained
        (None, 0.5, -0.06),  # Slip detected
        (30.0, 0.0, 0.0),    # Return to approach
    ]
    
    finger_states = {"Index": FingerState.IDLE}
    finger_currents = {"Index": 0.0}
    
    for i, (distance, current, derivative) in enumerate(states):
        f_state, pos, torque = finger.update(distance, current, derivative)
        finger_states["Index"] = f_state
        finger_currents["Index"] = current
        
        h_state = hand.update(
            finger_states, finger_currents,
            is_lowering=False,
            is_impact_detected=False,
            is_stationary=False,
            time_since_impact=0.0
        )
        
        print(f"Step {i+1}:")
        print(f"  Inputs: distance={distance}, current={current:.2f}A, derivative={derivative:.3f}A/s")
        print(f"  Finger: {f_state.name}, position={pos:.1f}°, torque={torque:.2f}A")
        print(f"  Hand: {h_state.name}")
        
    print("\nSimulating release sequence:")
    # Now simulate a release sequence
    finger.force_state(FingerState.CONTACT)
    finger_states["Index"] = FingerState.CONTACT
    finger_currents["Index"] = 0.3
    hand.force_state(HandState.GRASP)
    
    steps = [
        # State, is_lowering, is_impact, is_stationary, current
        (HandState.GRASP, False, False, False, 0.3),
        (HandState.GRASP, True, False, False, 0.3),  # Start lowering
        (HandState.LOWER, True, True, False, 0.3),   # Impact detected
        (HandState.LOWER, True, True, True, 0.3),    # Become stationary
        (HandState.LOWER, True, True, True, 0.1),    # Current drops below threshold
    ]
    
    for i, (expected_state, lowering, impact, stationary, current) in enumerate(steps):
        finger_currents["Index"] = current
        h_state = hand.update(
            finger_states, finger_currents,
            is_lowering=lowering,
            is_impact_detected=impact,
            is_stationary=stationary,
            time_since_impact=0.5 if impact else 0.0
        )
        
        print(f"Step {i+1}:")
        print(f"  Inputs: lowering={lowering}, impact={impact}, stationary={stationary}, current={current:.2f}A")
        print(f"  Hand: {h_state.name} (expected: {expected_state.name})")
        
    print("\nTest complete.")