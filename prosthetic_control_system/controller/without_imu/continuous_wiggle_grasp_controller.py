#!/usr/bin/env python3
"""
Continuous Wiggle Grasp Controller for Prosthetic Hand.

This controller extends the idle_wiggle_grasp_controller by maintaining
continuous random wiggling across ALL states, not just IDLE. Key features:

1. Fingers wiggle randomly in ALL states (IDLE, THUMB_OPPOSING, GRASP_FORMING)
2. Thumb opposition and grasping movements are superimposed on wiggling
3. When object is detected, thumb moves to opposition but wiggling continues
4. When thumb detects object, fingers begin grasping while still wiggling
5. Wiggling amplitude scales based on state (full in IDLE, reduced during GRASP)
6. Clear separation between sensor reading and movement phases
7. Enhanced visualization of finger positions and sensor readings
"""

import os
import sys
import time
import signal
import argparse
import logging
import threading
import random

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("ContinuousWiggleGrasp")

# Set logger level explicitly
logger.setLevel(logging.INFO)

# Add all necessary directories to path
# First get the repository root directory (Fingie_Firmware)
repo_root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../"))

# Ensure repo root is in the path
if repo_root not in sys.path:
    sys.path.insert(0, repo_root)

# Import main components
from prosthetic_control_system.proximity.proximity_manager import ProximityManager
from prosthetic_control_system.hand.motor_interface import MotorInterface, ControlMode, SimulatedMotorInterface
from prosthetic_control_system.hand.ability_hand_interface import AbilityHandInterface
from prosthetic_control_system.controller.without_imu.config import DEFAULT_SENSORS, MCP_SENSORS, FINGER_MAPPING

# Global controller instance
controller = None
running = True

class ContinuousWiggleGraspController:
    """
    Controller that implements continuous wiggling during ALL states and
    proximity-triggered grasping.
    
    This controller implements a state machine with three states:
    1. IDLE: Fingers randomly wiggling, waiting for object detection
    2. THUMB_OPPOSING: Thumb in opposition, other fingers wiggling
    3. GRASP_FORMING: All fingers closing to form a grip while still wiggling
    
    Each state alternates between sensor reading and movement phases.
    The key difference from idle_wiggle_grasp_controller is that wiggling
    continues in ALL states, not just IDLE.
    """
    
    def __init__(
        self,
        control_rate=20,
        proximity_rate=None,
        use_simulated_motors=False,
        motor_interface_kwargs=None,
        min_phase_duration=0.5,
        max_phase_duration=2.0,
        max_angle=10.0,
        wiggle_speed=5.0,
        min_fingers=1,
        max_fingers=4,
        finger_threshold=100.0,
        thumb_threshold=50.0,
        finger_close_angle=30.0,
        thumb_opposition_angle=100.0,
        hysteresis=20.0,
        # New parameters specific to continuous wiggling
        idle_wiggle_scale=1.0,     # Full wiggle amplitude in IDLE
        opposing_wiggle_scale=0.6, # 60% wiggle amplitude in THUMB_OPPOSING
        grasp_wiggle_scale=0.4     # 40% wiggle amplitude in GRASP_FORMING
    ):
        """
        Initialize the continuous wiggle grasp controller.
        
        Args:
            control_rate: Control loop frequency in Hz
            proximity_rate: Proximity sampling rate in Hz (defaults to 2x control_rate)
            use_simulated_motors: Whether to use simulated motors
            motor_interface_kwargs: Additional kwargs for motor interface
            min_phase_duration: Minimum phase duration in seconds
            max_phase_duration: Maximum phase duration in seconds
            max_angle: Maximum finger angle for wiggling
            wiggle_speed: Maximum wiggle speed in degrees per control cycle
            min_fingers: Minimum number of fingers to wiggle
            max_fingers: Maximum number of fingers to wiggle
            finger_threshold: Distance threshold for any finger detection (mm)
            thumb_threshold: Distance threshold for thumb detection (mm)
            finger_close_angle: Angle for closing fingers (degrees)
            thumb_opposition_angle: Angle for thumb opposition (degrees)
            hysteresis: Extra distance for release detection (mm)
            idle_wiggle_scale: Wiggle amplitude scaling factor in IDLE state
            opposing_wiggle_scale: Wiggle amplitude scaling factor in THUMB_OPPOSING state
            grasp_wiggle_scale: Wiggle amplitude scaling factor in GRASP_FORMING state
        """
        # Save parameters
        self.control_rate = control_rate
        self.control_interval = 1.0 / control_rate
        self.min_phase_duration = min_phase_duration
        self.max_phase_duration = max_phase_duration
        self.max_angle = max_angle
        self.wiggle_speed = wiggle_speed
        self.min_fingers = min_fingers
        self.max_fingers = max_fingers
        
        # Grasp parameters
        self.finger_threshold = finger_threshold
        self.thumb_threshold = thumb_threshold
        self.finger_close_angle = finger_close_angle
        self.thumb_opposition_angle = thumb_opposition_angle
        self.hysteresis = hysteresis
        
        # Wiggle scaling factors for each state
        self.wiggle_scales = {
            "IDLE": idle_wiggle_scale,
            "THUMB_OPPOSING": opposing_wiggle_scale,
            "GRASP_FORMING": grasp_wiggle_scale
        }
        
        # Set up proximity sampling rate (2x control rate by default for smoother readings)
        if proximity_rate is None:
            self.proximity_rate = control_rate * 2
        else:
            self.proximity_rate = proximity_rate
        
        # Initialize proximity manager
        logger.info("Initializing proximity manager...")
        try:
            self.proximity = ProximityManager(
                sampling_rate=self.proximity_rate,
                approach_threshold=self.finger_threshold,
                contact_threshold=self.thumb_threshold,
                sensors=DEFAULT_SENSORS
            )
        except Exception as e:
            logger.error(f"Error creating proximity manager: {e}")
            raise
        
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
        
        # Finger mapping (from sensors to motors)
        self.finger_mapping = FINGER_MAPPING
        
        # State machine
        self.state = "IDLE"
        self.state_entry_time = time.time()
        
        # Phase tracking
        self.current_phase = "SENSOR_PHASE"  # Start with sensor phase
        self.phase_start_time = time.time()
        self.next_phase_time = time.time() + self._generate_phase_duration()
        self.sensor_readings = {}
        
        # Finger positions
        self.current_positions = {finger: 0.0 for finger in self.motors.fingers}
        self.target_positions = {finger: 0.0 for finger in self.motors.fingers}
        
        # Base positions (without wiggle)
        self.base_positions = {finger: 0.0 for finger in self.motors.fingers}
        
        # Finger movement directions (1 = increasing, -1 = decreasing)
        self.movement_directions = {finger: random.choice([-1, 1]) for finger in self.motors.fingers}
        
        # Wiggle information
        self.wiggling_fingers = []
        self.wiggle_count = 0
        self.phase_count = 0
        
        # Detection tracking
        self.finger_detection_count = 0
        self.thumb_detection_count = 0
        self.release_detection_count = 0
        self.readings_required = 3  # Consecutive readings needed
        
        # Previous readings for debouncing
        self.prev_finger_distance = 100.0
        self.prev_thumb_distance = 100.0
        
        # Thread control
        self.running = False
        self.shutdown_event = threading.Event()
        self.thread = None
        
        # Status reporting
        self.last_status_update = time.time()
        self.status_cache = {}
        
        logger.info(f"Continuous wiggle grasp controller initialized with phase_duration={min_phase_duration}-{max_phase_duration}s (uniform random)")
        
        # Select initial fingers to wiggle
        self._select_random_fingers()
    
    def _generate_phase_duration(self):
        """Generate random phase duration using uniform distribution"""
        # Generate random duration using uniform distribution
        duration = random.uniform(self.min_phase_duration, self.max_phase_duration)
        return duration
    
    def _control_loop(self):
        """Main control loop that alternates between phases with random timing"""
        next_update_time = time.time()
        
        while self.running and not self.shutdown_event.is_set():
            current_time = time.time()
            
            # Check if it's time for the next control update
            if current_time >= next_update_time:
                try:
                    # Check if it's time to switch phases
                    if current_time >= self.next_phase_time:
                        self._toggle_phase()
                    
                    # Execute current phase
                    if self.current_phase == "SENSOR_PHASE":
                        self._execute_sensor_phase()
                    else:  # MOVEMENT_PHASE
                        self._execute_movement_phase()
                    
                    # Calculate next update time
                    next_update_time = current_time + self.control_interval
                    
                except Exception as e:
                    logger.error(f"Error in control loop: {e}")
                    # Make sure we don't get stuck
                    next_update_time = time.time() + self.control_interval
            
            # Small sleep to prevent CPU hogging
            sleep_time = max(0.001, next_update_time - time.time())
            # This will return True if event is set during the wait
            if self.shutdown_event.wait(sleep_time):
                logger.info("Control loop received shutdown event during sleep")
                break
        
        logger.info("Control loop exited")
    
    def _toggle_phase(self):
        """Toggle between sensor reading and movement phases with random durations"""
        self.phase_count += 1
        
        if self.current_phase == "SENSOR_PHASE":
            # Switch to movement phase
            self.current_phase = "MOVEMENT_PHASE"
            
            # Generate random phase duration
            phase_duration = self._generate_phase_duration()
            self.next_phase_time = time.time() + phase_duration
            
            logger.info(f"🔄 SWITCHING TO MOVEMENT PHASE - duration: {phase_duration:.2f}s")
            print(f"\n🔄 SWITCHING TO MOVEMENT PHASE - duration: {phase_duration:.2f}s")
            
            # Select new random fingers to wiggle for this movement phase
            # We do this in EVERY state now, not just IDLE
            self._select_random_fingers()
            
        else:
            # Switch to sensor phase
            self.current_phase = "SENSOR_PHASE"
            
            # Generate random phase duration
            phase_duration = self._generate_phase_duration()
            self.next_phase_time = time.time() + phase_duration
            
            logger.info(f"🔄 SWITCHING TO SENSOR PHASE - duration: {phase_duration:.2f}s")
            print(f"\n🔄 SWITCHING TO SENSOR PHASE - duration: {phase_duration:.2f}s")
        
        # Reset phase timer
        self.phase_start_time = time.time()
    
    def _execute_sensor_phase(self):
        """Execute sensor reading phase - read from sensors and update state machine"""
        # Read from thumb sensor
        thumb_distance, thumb_status = self.proximity.get_sensor_value(
            "Thumb1", filtered=True, with_status=True
        )
        
        # Safety check - if sensor reading is invalid, default to safe distance
        if thumb_distance is None or thumb_status == "BAD":
            thumb_distance = 100.0
        
        # Read from all finger sensors, use the minimum distance
        finger_distances = {}
        all_fingers_distance = 100.0  # Default safe value
        
        for sensor_name in ["Index1", "Middle1", "Ring1", "Pinky1"]:
            distance, status = self.proximity.get_sensor_value(
                sensor_name, filtered=True, with_status=True
            )
            
            if distance is not None and status != "BAD":
                finger_distances[sensor_name] = distance
                
                # Store in sensor_readings for display
                self.sensor_readings[sensor_name] = {
                    "value": distance,
                    "status": status
                }
            else:
                self.sensor_readings[sensor_name] = {
                    "value": None,
                    "status": "BAD"
                }
        
        # Store thumb reading
        self.sensor_readings["Thumb1"] = {
            "value": thumb_distance,
            "status": thumb_status
        }
        
        # Find minimum finger distance if any valid readings
        if finger_distances:
            all_fingers_distance = min(finger_distances.values())
        
        # Update state based on sensor readings
        self._update_state(all_fingers_distance, thumb_distance)
    
    def _execute_movement_phase(self):
        """Execute movement phase - move fingers according to current state"""
        if self.state == "IDLE":
            self._execute_idle_movement()
        elif self.state == "THUMB_OPPOSING":
            self._execute_opposing_movement()
        elif self.state == "GRASP_FORMING":
            self._execute_grasp_movement()
    
    def _update_state(self, finger_distance, thumb_distance):
        """Update controller state based on sensor readings"""
        current_time = time.time()
        time_in_state = current_time - self.state_entry_time
        
        # Debounce non-100mm readings - require two consecutive non-100mm readings that aren't identical
        # For finger distance
        valid_finger_reading = True
        if finger_distance < 100:
            if self.prev_finger_distance >= 100:
                # First non-100 reading, don't consider valid yet
                valid_finger_reading = False
                logger.debug(f"First non-100 finger reading: {finger_distance:.1f}mm - waiting for confirmation")
            elif finger_distance == self.prev_finger_distance:
                # Two identical non-100 readings in a row - likely a glitch
                valid_finger_reading = False
                logger.debug(f"Identical consecutive finger readings: {finger_distance:.1f}mm - likely a glitch")
        
        # For thumb distance
        valid_thumb_reading = True
        if thumb_distance < 100:
            if self.prev_thumb_distance >= 100:
                # First non-100 reading, don't consider valid yet
                valid_thumb_reading = False
                logger.debug(f"First non-100 thumb reading: {thumb_distance:.1f}mm - waiting for confirmation")
            elif thumb_distance == self.prev_thumb_distance:
                # Two identical non-100 readings in a row - likely a glitch
                valid_thumb_reading = False
                logger.debug(f"Identical consecutive thumb readings: {thumb_distance:.1f}mm - likely a glitch")
        
        # Save current readings as previous for next iteration
        self.prev_finger_distance = finger_distance
        self.prev_thumb_distance = thumb_distance
        
        # If readings aren't valid, don't proceed with state update
        if not valid_finger_reading or not valid_thumb_reading:
            return
        
        # State transitions with debouncing
        if self.state == "IDLE":
            # Check if any finger is detecting an object
            if finger_distance < self.finger_threshold:
                self.finger_detection_count += 1
                if self.finger_detection_count >= self.readings_required:
                    logger.info(f"Transitioning to THUMB_OPPOSING (finger distance: {finger_distance:.1f}mm)")
                    print(f"\n🖐 OBJECT DETECTED - Transitioning to THUMB_OPPOSING (distance: {finger_distance:.1f}mm)")
                    self.state = "THUMB_OPPOSING"
                    self.state_entry_time = current_time
                    self.finger_detection_count = 0
            else:
                self.finger_detection_count = 0
                
        elif self.state == "THUMB_OPPOSING":
            # Check if thumb is detecting an object
            if thumb_distance < self.thumb_threshold:
                self.thumb_detection_count += 1
                if self.thumb_detection_count >= self.readings_required:
                    logger.info(f"Transitioning to GRASP_FORMING (thumb distance: {thumb_distance:.1f}mm)")
                    print(f"\n👍 THUMB CONTACT - Transitioning to GRASP_FORMING (distance: {thumb_distance:.1f}mm)")
                    self.state = "GRASP_FORMING"
                    self.state_entry_time = current_time
                    self.thumb_detection_count = 0
            else:
                self.thumb_detection_count = 0
                
            # Check if fingers are no longer detecting (with hysteresis)
            if finger_distance > self.finger_threshold + self.hysteresis:
                self.release_detection_count += 1
                if self.release_detection_count >= self.readings_required:
                    logger.info(f"Returning to IDLE from THUMB_OPPOSING (finger distance: {finger_distance:.1f}mm)")
                    print(f"\n👋 OBJECT REMOVED - Returning to IDLE (distance: {finger_distance:.1f}mm)")
                    self.state = "IDLE" 
                    self.state_entry_time = current_time
                    self.release_detection_count = 0
            else:
                self.release_detection_count = 0
                    
        elif self.state == "GRASP_FORMING":
            # Check if both thumb and fingers are no longer detecting (with hysteresis)
            if (thumb_distance > self.thumb_threshold + self.hysteresis or 
                finger_distance > self.finger_threshold + self.hysteresis):
                self.release_detection_count += 1
                if self.release_detection_count >= self.readings_required:
                    logger.info(f"Returning to IDLE from GRASP_FORMING (thumb: {thumb_distance:.1f}mm, finger: {finger_distance:.1f}mm)")
                    print(f"\n👋 OBJECT REMOVED - Returning to IDLE (thumb: {thumb_distance:.1f}mm, finger: {finger_distance:.1f}mm)")
                    self.state = "IDLE"
                    self.state_entry_time = current_time
                    self.release_detection_count = 0
            else:
                self.release_detection_count = 0
    
    def _execute_idle_movement(self):
        """Execute wiggle behavior during IDLE state"""
        # Set base positions for all fingers to 0 (open position)
        for finger in self.motors.fingers:
            self.base_positions[finger] = 0.0
        
        # Set base position for thumb rotate to 0
        self.base_positions["ThumbRotate"] = 0.0
        
        # Apply wiggling on top of base positions
        self._apply_wiggle_movement()
    
    def _execute_opposing_movement(self):
        """Execute movement during THUMB_OPPOSING state with wiggling"""
        # Set base positions - thumb in opposition, others open
        for finger in self.motors.fingers:
            if finger != "Thumb":
                # Non-thumb fingers are at 0 base position
                self.base_positions[finger] = 0.0
            else:
                # Thumb has minimal flexion
                self.base_positions[finger] = 5.0
        
        # Set thumb rotate to opposition position
        self.base_positions["ThumbRotate"] = self.thumb_opposition_angle
        
        # Apply wiggling on top of base positions
        self._apply_wiggle_movement()
    
    def _execute_grasp_movement(self):
        """Execute grasp movement during GRASP_FORMING state with wiggling"""
        # Set base positions - thumb in opposition, others in grasp position
        for finger in self.motors.fingers:
            if finger != "Thumb":
                # Non-thumb fingers are in grasping position
                self.base_positions[finger] = self.finger_close_angle
            else:
                # Thumb has minimal flexion
                self.base_positions[finger] = 5.0
        
        # Set thumb rotate to opposition position
        self.base_positions["ThumbRotate"] = self.thumb_opposition_angle
        
        # Apply wiggling on top of base positions
        self._apply_wiggle_movement()
    
    def _apply_wiggle_movement(self):
        """Apply wiggle movement on top of base positions with state-dependent scaling"""
        # Get current wiggle scale factor based on state
        wiggle_scale = self.wiggle_scales[self.state]
        
        # Update wiggle targets with uniform random movement
        self._update_wiggle_targets()
        
        # Move fingers with base position + scaled wiggle
        for finger in self.motors.fingers:
            try:
                # Start with base position for the current state
                base_position = self.base_positions[finger]
                
                # Apply scaled wiggle movement for wiggling fingers
                wiggle_amount = 0.0
                if finger in self.wiggling_fingers and finger != "Thumb":
                    wiggle_offset = self.target_positions[finger] - base_position
                    wiggle_amount = wiggle_offset * wiggle_scale
                
                # Calculate final position: base + scaled wiggle
                final_position = base_position + wiggle_amount
                
                # Apply safety bounds
                final_position = max(0.0, min(final_position, self.max_angle))
                
                # Set position on motor
                self.motors.set_position(finger, final_position)
                self.current_positions[finger] = final_position
                
            except Exception as e:
                logger.error(f"Error setting position for {finger}: {e}")
        
        # Handle thumb rotate separately
        try:
            self.motors.set_position("ThumbRotate", self.base_positions["ThumbRotate"])
            self.current_positions["ThumbRotate"] = self.base_positions["ThumbRotate"]
        except Exception as e:
            logger.error(f"Error setting thumb rotation: {e}")
    
    def _select_random_fingers(self):
        """Select random fingers to wiggle"""
        # Get all non-thumb fingers
        non_thumb_fingers = [f for f in self.motors.fingers if f != "Thumb"]
        
        # Determine how many fingers to wiggle (between min and max)
        num_fingers = random.randint(self.min_fingers, min(self.max_fingers, len(non_thumb_fingers)))
        
        # Randomly select fingers
        self.wiggling_fingers = random.sample(non_thumb_fingers, num_fingers)
        self.wiggle_count += 1
        
        finger_list = ", ".join(self.wiggling_fingers)
        logger.info(f"Selected {num_fingers} fingers to wiggle: {finger_list}")
        print(f"Selected {num_fingers} fingers to wiggle: {finger_list}")
    
    def _update_wiggle_targets(self):
        """Update target positions for wiggling fingers with smart direction changes"""
        # Generate a shared random component for partial correlation
        master_random = random.gauss(0, 1)
        correlation = 0.6  # 60% correlation between fingers
        
        for finger in self.wiggling_fingers:
            # Get current position relative to base position
            current_pos = self.current_positions.get(finger, 0.0) - self.base_positions.get(finger, 0.0)
            current_dir = self.movement_directions.get(finger, 1)
            
            # Calculate distance to limits
            distance_to_max = self.max_angle - current_pos
            distance_to_min = current_pos - 0
            
            # Determine if we need to change direction
            if (current_dir > 0 and distance_to_max < self.wiggle_speed * 2) or \
               (current_dir < 0 and distance_to_min < self.wiggle_speed * 2):
                # Approaching a limit, reverse direction with high probability
                if random.random() < 0.8:
                    current_dir = -current_dir
                    self.movement_directions[finger] = current_dir
            elif random.random() < 0.1:
                # Randomly change direction sometimes (10% chance)
                current_dir = -current_dir
                self.movement_directions[finger] = current_dir
            
            # Blend shared randomness with finger-specific randomness
            finger_random = (correlation * master_random + 
                           (1-correlation) * random.gauss(0, 1))
            
            # Use uniform random movement between 0-10 degrees in the current direction
            max_movement = 10.0  # Fixed 10 degrees maximum movement
            movement = random.uniform(0, max_movement) * current_dir
            
            # Calculate new target position (relative to base)
            new_target = current_pos + movement
            
            # Enforce bounds for the wiggle component
            new_target = max(0, min(new_target, self.max_angle))
            
            # If we hit a bound, reverse direction for next time
            if new_target <= 0 or new_target >= self.max_angle:
                self.movement_directions[finger] = -current_dir
            
            # Set the new target (relative to base)
            self.target_positions[finger] = new_target + self.base_positions[finger]
    
    def start(self):
        """Start the controller"""
        if self.running:
            logger.warning("Controller already running")
            return
        
        # Reset shutdown event
        self.shutdown_event.clear()
        
        # Start proximity manager
        logger.info("Starting proximity manager...")
        self.proximity.start()
        
        # Start motor interface
        logger.info("Starting motor interface...")
        self.motors.start()
        
        # Start control loop
        logger.info("Starting control loop...")
        self.running = True
        self.thread = threading.Thread(target=self._control_loop)
        self.thread.daemon = True
        self.thread.start()
        
        logger.info("Continuous wiggle grasp controller started")
    
    def stop(self):
        """Stop the controller"""
        logger.info("Stopping controller...")
        
        # Mark that we're shutting down
        self.running = False
        self.shutdown_event.set()
        
        # Wait for control loop to exit
        if self.thread and self.thread.is_alive():
            try:
                self.thread.join(timeout=2.0)
            except Exception as e:
                logger.error(f"Error joining thread: {e}")
        
        # Ensure all fingers are opened safely (non-thumb fingers first)
        try:
            if self.motors:
                # First open all non-thumb fingers
                for finger in self.motors.fingers:
                    if finger != "Thumb":
                        try:
                            self.motors.set_position(finger, 0.0)
                        except Exception as e:
                            logger.error(f"Error opening finger {finger}: {e}")
                
                # Give motors time to move
                time.sleep(0.5)
                
                # Then open thumb last to prevent collision (both flexor and rotator)
                try:
                    # First reduce the opposition (rotator)
                    self.motors.set_position("ThumbRotate", 0.0)
                    time.sleep(0.3)  # Give time for rotation to complete
                    
                    # Then open the flexor
                    self.motors.set_position("Thumb", 0.0)
                except Exception as e:
                    logger.error(f"Error opening thumb: {e}")
                
                # Give thumb time to move
                time.sleep(0.5)
        except Exception as e:
            logger.error(f"Error opening fingers: {e}")
        
        # Stop components in reverse order
        if self.motors:
            logger.info("Stopping motors...")
            try:
                self.motors.stop()
            except Exception as e:
                logger.error(f"Error stopping motors: {e}")
        
        if self.proximity:
            logger.info("Stopping proximity manager...")
            try:
                self.proximity.stop()
            except Exception as e:
                logger.error(f"Error stopping proximity manager: {e}")
        
        logger.info("Continuous wiggle grasp controller stopped")
    
    def get_status(self):
        """Get current system status"""
        current_time = time.time()
        
        # Only calculate these every 0.1s to reduce processing load
        if current_time - self.last_status_update > 0.1:
            status = {
                "state": self.state,
                "time_in_state": current_time - self.state_entry_time,
                "phase": self.current_phase,
                "phase_count": self.phase_count,
                "time_in_phase": current_time - self.phase_start_time,
                "time_to_next_phase": max(0, self.next_phase_time - current_time),
                "fingers": {},
                "proximity": self.sensor_readings.copy(),
                "wiggle_info": {
                    "count": self.wiggle_count,
                    "wiggling_fingers": self.wiggling_fingers.copy(),
                    "max_angle": self.max_angle,
                    "speed": self.wiggle_speed,
                    "current_scale": self.wiggle_scales[self.state]
                },
                "thresholds": {
                    "finger_threshold": self.finger_threshold,
                    "thumb_threshold": self.thumb_threshold,
                    "finger_close_angle": self.finger_close_angle,
                    "thumb_opposition_angle": self.thumb_opposition_angle
                }
            }
            
            # Get finger status
            for finger_name in self.motors.fingers:
                try:
                    position = self.motors.get_position(finger_name)
                    current = self.motors.get_current(finger_name)
                    target = self.target_positions.get(finger_name, 0.0)
                    base = self.base_positions.get(finger_name, 0.0)
                    direction = self.movement_directions.get(finger_name, 0)
                    is_wiggling = finger_name in self.wiggling_fingers
                    
                    # Calculate wiggle component
                    wiggle_component = position - base
                    
                    # Determine direction symbol
                    direction_symbol = "→" if direction > 0 else "←" if direction < 0 else "-"
                    
                    status["fingers"][finger_name] = {
                        "position": position,
                        "current": current,
                        "target": target,
                        "base": base,
                        "wiggle": wiggle_component,
                        "direction": direction,
                        "direction_symbol": direction_symbol,
                        "wiggling": is_wiggling
                    }
                except Exception as e:
                    logger.debug(f"Error getting status for {finger_name}: {e}")
                    status["fingers"][finger_name] = {
                        "position": 0.0,
                        "current": 0.0,
                        "target": 0.0,
                        "base": 0.0,
                        "wiggle": 0.0,
                        "direction": 0,
                        "direction_symbol": "-",
                        "wiggling": False
                    }
            
            # Add ThumbRotate status if available
            try:
                thumb_rotate_position = self.motors.get_position("ThumbRotate")
                thumb_rotate_target = self.target_positions.get("ThumbRotate", 0.0)
                thumb_rotate_base = self.base_positions.get("ThumbRotate", 0.0)
                
                status["fingers"]["ThumbRotate"] = {
                    "position": thumb_rotate_position,
                    "target": thumb_rotate_target,
                    "base": thumb_rotate_base,
                    "wiggle": 0.0,  # Thumb rotate doesn't wiggle
                    "wiggling": False
                }
            except:
                pass
            
            # Save for next time
            self.status_cache = status
            self.last_status_update = current_time
            return status
        else:
            # Return cached status to reduce processing load
            return self.status_cache


def signal_handler(sig, frame):
    """Handle keyboard interrupt gracefully"""
    global running
    running = False
    if controller:
        logger.info("Stopping controller due to signal...")
        controller.stop()
    sys.exit(0)


def main():
    """Main entry point"""
    global controller, running
    
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Continuous Wiggle Grasp Controller')
    
    parser.add_argument('--simulate', action='store_true',
                      help='Use simulated hardware')
    
    parser.add_argument('--rate', type=int, default=20,
                      help='Control loop rate in Hz (default: 20)')
    
    parser.add_argument('--port', type=str, default=None,
                      help='Serial port for Ability Hand (default: auto-detect)')
    
    parser.add_argument('--proximity-rate', type=int, default=5,
                      help='Proximity sensor sampling rate in Hz (default: 5)')
    
    parser.add_argument('--min-phase', type=float, default=0.5,
                      help='Minimum phase duration in seconds (default: 0.5)')
    
    parser.add_argument('--max-phase', type=float, default=2.0,
                      help='Maximum phase duration in seconds (default: 2.0)')
    
    parser.add_argument('--max-angle', type=float, default=10.0,
                      help='Maximum wiggle angle in degrees (default: 10.0)')
    
    parser.add_argument('--wiggle-speed', type=float, default=5.0,
                      help='Maximum wiggle speed in degrees per control cycle (default: 5.0)')
    
    parser.add_argument('--min-fingers', type=int, default=1,
                      help='Minimum number of fingers to wiggle (default: 1)')
    
    parser.add_argument('--max-fingers', type=int, default=4,
                      help='Maximum number of fingers to wiggle (default: 4)')
    
    parser.add_argument('--finger-threshold', type=float, default=100.0,
                      help='Finger detection distance threshold in mm (default: 100.0)')
    
    parser.add_argument('--thumb-threshold', type=float, default=50.0,
                      help='Thumb detection distance threshold in mm (default: 50.0)')
    
    parser.add_argument('--finger-close', type=float, default=30.0,
                      help='Finger closure angle in degrees (default: 30.0)')
    
    parser.add_argument('--thumb-oppose', type=float, default=100.0,
                      help='Thumb opposition angle in degrees (default: 100.0, max rotation)')
    
    parser.add_argument('--hysteresis', type=float, default=20.0,
                      help='Hysteresis for release detection in mm (default: 20.0)')
    
    parser.add_argument('--idle-scale', type=float, default=1.0,
                      help='Wiggle amplitude scale factor in IDLE state (default: 1.0)')
    
    parser.add_argument('--opposing-scale', type=float, default=0.6,
                      help='Wiggle amplitude scale factor in THUMB_OPPOSING state (default: 0.6)')
    
    parser.add_argument('--grasp-scale', type=float, default=0.4,
                      help='Wiggle amplitude scale factor in GRASP_FORMING state (default: 0.4)')
    
    parser.add_argument('--debug', action='store_true',
                      help='Enable debug logging')
    
    args = parser.parse_args()
    
    # Set debug logging if requested
    if args.debug:
        logger.setLevel(logging.DEBUG)
        logger.debug("Debug logging enabled")
    
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # Create motor kwargs if needed
        motor_kwargs = {}
        if args.port:
            motor_kwargs['port'] = args.port
        
        # Create controller
        logger.info(f"Creating controller (simulated: {args.simulate}, rate: {args.rate}Hz)")
        controller = ContinuousWiggleGraspController(
            control_rate=args.rate,
            proximity_rate=args.proximity_rate,
            use_simulated_motors=args.simulate,
            motor_interface_kwargs=motor_kwargs,
            min_phase_duration=args.min_phase,
            max_phase_duration=args.max_phase,
            max_angle=args.max_angle,
            wiggle_speed=args.wiggle_speed,
            min_fingers=args.min_fingers,
            max_fingers=args.max_fingers,
            finger_threshold=args.finger_threshold,
            thumb_threshold=args.thumb_threshold,
            finger_close_angle=args.finger_close,
            thumb_opposition_angle=args.thumb_oppose,
            hysteresis=args.hysteresis,
            idle_wiggle_scale=args.idle_scale,
            opposing_wiggle_scale=args.opposing_scale,
            grasp_wiggle_scale=args.grasp_scale
        )
        
        # Start controller
        logger.info("Starting controller...")
        controller.start()
        
        # Monitor loop
        print("\nContinuous Wiggle Grasp Controller Active - Press Ctrl+C to stop")
        print("----------------------------------------------------")
        print(f"This controller implements continuous wiggling during ALL states:")
        print(f"1. IDLE: Fingers randomly wiggling ({args.idle_scale*100:.0f}% amplitude), waiting for object detection")
        print(f"2. THUMB_OPPOSING: Thumb positioned at {args.thumb_oppose}° opposition, other fingers wiggling ({args.opposing_scale*100:.0f}% amplitude)")
        print(f"3. GRASP_FORMING: All fingers closing to {args.finger_close}° while still wiggling ({args.grasp_scale*100:.0f}% amplitude)")
        print(f"\nPhases alternate with random durations ({args.min_phase}-{args.max_phase}s)")
        print(f"Will select {args.min_fingers}-{args.max_fingers} fingers to wiggle in all states")
        print(f"Maximum wiggle angle: {args.max_angle}°, Maximum wiggle speed: {args.wiggle_speed}°/cycle")
        print(f"\nProximity Thresholds:")
        print(f"- Any finger < {args.finger_threshold}mm: Position thumb in opposition")
        print(f"- Thumb < {args.thumb_threshold}mm: Form grip with all fingers")
        print(f"- Release hysteresis: +{args.hysteresis}mm above detection thresholds")
        
        while running:
            try:
                # Get current status
                status = controller.get_status()
                
                # Clear screen
                os.system('clear' if os.name == 'posix' else 'cls')
                
                # Print status header
                print("\nContinuous Wiggle Grasp Controller")
                print("================================")
                print(f"State: {status['state']} for {status['time_in_state']:.1f}s")
                print(f"Phase: {status['phase']} - Next phase in: {status['time_to_next_phase']:.1f}s")
                
                # Print wiggle info
                wiggle_info = status["wiggle_info"]
                if wiggle_info["wiggling_fingers"]:
                    finger_list = ", ".join(wiggle_info["wiggling_fingers"])
                    print(f"\nWiggling {len(wiggle_info['wiggling_fingers'])} fingers: {finger_list}")
                    print(f"Wiggle count: {wiggle_info['count']}, Max angle: {wiggle_info['max_angle']}°")
                    print(f"Current wiggle scale: {wiggle_info['current_scale']:.1f} ({wiggle_info['current_scale']*100:.0f}%)")
                
                # Print finger states with improved visualization
                print("\nFinger Status:")
                print("-------------")
                for finger, data in status["fingers"].items():
                    # Skip ThumbRotate in the regular display
                    if finger == "ThumbRotate":
                        continue
                        
                    # Prepare indicators
                    wiggle_indicator = "✓" if data.get("wiggling", False) else " "
                    dir_symbol = data.get("direction_symbol", "-")
                    
                    # Get base and wiggle components
                    base_pos = data.get("base", 0.0)
                    wiggle_comp = data.get("wiggle", 0.0)
                    
                    # Create a visual bar to show position relative to max
                    if finger != "Thumb" and status["state"] in ["IDLE", "THUMB_OPPOSING", "GRASP_FORMING"]:
                        # During wiggling, show relative to finger close angle
                        max_value = status["thresholds"]["finger_close_angle"]
                    else:
                        # Default case
                        max_value = status["thresholds"]["finger_close_angle"]
                        
                    # Ensure we don't divide by zero
                    if max_value > 0:
                        position_percent = int(data["position"] / max_value * 20)
                        base_percent = int(base_pos / max_value * 20)
                    else:
                        position_percent = 0
                        base_percent = 0
                        
                    # Create base position marker and position bar
                    position_bar = ""
                    for i in range(20):
                        if i == base_percent:
                            position_bar += "│"  # Base position marker
                        elif i < position_percent:
                            position_bar += "▓"  # Filled bar
                        else:
                            position_bar += "░"  # Empty bar
                    
                    # Different display for thumb vs other fingers
                    if finger == "Thumb":
                        # For thumb, also show rotation position
                        thumb_rotate = status["fingers"].get("ThumbRotate", {}).get("position", 0.0)
                        thumb_rotate_base = status["fingers"].get("ThumbRotate", {}).get("base", 0.0)
                        
                        # Simplify the rotation display
                        rotation_percent = int(thumb_rotate / status["thresholds"]["thumb_opposition_angle"] * 10)
                        rotation_base = int(thumb_rotate_base / status["thresholds"]["thumb_opposition_angle"] * 10)
                        
                        # Create rotation bar with base marker
                        rotation_bar = ""
                        for i in range(10):
                            if i == rotation_base:
                                rotation_bar += "│"  # Base position marker
                            elif i < rotation_percent:
                                rotation_bar += "▓"  # Filled bar
                            else:
                                rotation_bar += "░"  # Empty bar
                        
                        print(f"{finger:10s}: Pos: {data['position']:5.1f}° | "
                              f"Rotation: {rotation_bar} {thumb_rotate:.1f}° | "
                              f"Current: {data['current']:.3f}A")
                    else:
                        # Regular fingers
                        print(f"{finger:10s}: [{wiggle_indicator}] {dir_symbol} {position_bar} {data['position']:5.1f}° | "
                              f"Base: {base_pos:5.1f}° + Wiggle: {wiggle_comp:5.1f}° | "
                              f"Current: {data['current']:.3f}A")
                
                # Only show detailed proximity values in sensor phase
                if status["phase"] == "SENSOR_PHASE":
                    # Print proximity sensor readings with improved visualization
                    print("\nProximity Sensors:")
                    print("-----------------")
                    
                    # Get the maximum range for visualization
                    max_range = 100  # Default max range in mm
                    
                    for sensor, data in status["proximity"].items():
                        if data["value"] is None:
                            value_str = "N/A"
                            distance_bar = "?" * 20  # Unknown
                        else:
                            value_str = f"{data['value']:5.1f}mm"
                            
                            # Calculate bar length (inverse of distance - closer objects have longer bars)
                            # Clamp value between 0 and max_range
                            clamped_value = max(0, min(data['value'], max_range))
                            bar_length = 20 - int((clamped_value / max_range) * 20)
                            
                            # Add visual indicators for threshold detection
                            if sensor == "Thumb1" and data["value"] < status["thresholds"]["thumb_threshold"]:
                                distance_bar = "█" * bar_length + "▁" * (20 - bar_length) + " 👍"
                                value_str = f"{value_str} ⚠️"
                            elif sensor != "Thumb1" and data["value"] < status["thresholds"]["finger_threshold"]:
                                distance_bar = "█" * bar_length + "▁" * (20 - bar_length) + " 🖐"
                                value_str = f"{value_str} ⚠️"
                            else:
                                distance_bar = "█" * bar_length + "▁" * (20 - bar_length)
                        
                        # Get corresponding finger name
                        finger_name = FINGER_MAPPING.get(sensor, "?")
                        
                        print(f"{sensor:10s} ({finger_name:5s}): {distance_bar} {value_str:15s} | Status: {data['status']}")
                
                # Instructions
                print("\nPress Ctrl+C to stop")
                
                # Sleep
                time.sleep(0.2)
                
            except KeyboardInterrupt:
                running = False
                break
            except Exception as e:
                logger.error(f"Error in monitor loop: {e}")
                time.sleep(1)
    
    except Exception as e:
        logger.error(f"Error in main: {e}")
    
    finally:
        # Ensure controller is stopped
        if controller:
            logger.info("Stopping controller...")
            controller.stop()
        
        logger.info("Exiting")


if __name__ == "__main__":
    main()