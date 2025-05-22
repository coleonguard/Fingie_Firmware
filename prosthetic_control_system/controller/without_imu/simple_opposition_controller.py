#!/usr/bin/env python3
"""
Simple Opposition Controller for Prosthetic Hand.

This controller implements a basic, robust control approach that:
1. Rotates the thumb to opposition position when ANY finger detects an object
2. Closes the other fingers when the thumb detects an object
3. Handles I2C bus contention by separating sensor reading and motor control phases

It's designed to be reliable even with imperfect sensor positioning and occasional glitches.
"""

import os
import sys
import time
import signal
import argparse
import logging
import threading

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("SimpleOppositionController")

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

class SimpleOppositionController:
    """
    Simple opposition controller for the prosthetic hand.
    
    This controller uses a straightforward approach:
    1. IDLE: All fingers open, waiting for initial detection
    2. THUMB_OPPOSING: Thumb positioned in opposition, waiting for thumb proximity
    3. GRIP_CLOSING: All other fingers close to make contact
    
    It handles I2C bus contention by separating sensor reading and motor control phases.
    """
    
    def __init__(
        self,
        control_rate=20,
        proximity_rate=None,
        use_simulated_motors=False,
        motor_interface_kwargs=None,
        finger_close_angle=30.0,
        thumb_opposition_angle=30.0,
        finger_threshold=100.0,
        thumb_threshold=50.0,
    ):
        """
        Initialize the simple opposition controller.
        
        Args:
            control_rate: Control loop frequency in Hz
            proximity_rate: Proximity sampling rate in Hz (defaults to 2x control_rate)
            use_simulated_motors: Whether to use simulated motors
            motor_interface_kwargs: Additional kwargs for motor interface
            finger_close_angle: Angle for closing fingers (degrees)
            thumb_opposition_angle: Angle for thumb opposition (degrees)
            finger_threshold: Distance threshold for any finger detection (mm)
            thumb_threshold: Distance threshold for thumb detection (mm)
        """
        # Save parameters
        self.control_rate = control_rate
        self.control_interval = 1.0 / control_rate
        
        # Grip parameters
        self.finger_close_angle = finger_close_angle
        self.thumb_opposition_angle = thumb_opposition_angle
        self.finger_threshold = finger_threshold
        self.thumb_threshold = thumb_threshold
        self.hysteresis = 20.0  # mm
        self.min_dwell_time = 0.3  # seconds
        self.motion_phase_duration = 1.0  # seconds (increased to accommodate sequential movements)
        
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
        
        # State tracking
        self.state = "IDLE"
        self.state_entry_time = time.time()
        self.in_motion_phase = False
        self.motion_start_time = 0
        
        # Finger positions
        self.current_positions = {finger: 0.0 for finger in self.motors.fingers}
        self.target_positions = {finger: 0.0 for finger in self.motors.fingers}
        
        # Consecutive readings for debouncing
        self.finger_detection_count = 0
        self.thumb_detection_count = 0
        self.release_detection_count = 0
        self.readings_required = 3  # Consecutive readings needed
        
        # Thread control
        self.running = False
        self.shutdown_event = threading.Event()
        self.thread = None
    
    def _control_loop(self):
        """Main control loop - separates sensor reading and motor control phases"""
        next_update_time = time.time()
        
        while self.running and not self.shutdown_event.is_set():
            start_time = time.time()
            
            # Check if it's time for the next control update
            if start_time >= next_update_time:
                try:
                    # If we're in motion phase, skip sensor reading
                    if self.in_motion_phase:
                        if time.time() - self.motion_start_time > self.motion_phase_duration:
                            self.in_motion_phase = False
                            logger.debug("Motion phase complete, resuming sensor reading")
                    else:
                        # Full I2C sensing phase - read all sensors first
                        self._read_sensors_and_update_state()
                        
                        # If state requires motion, enter motion phase and move fingers
                        if self._should_move():
                            self._enter_motion_phase()
                            self._move_fingers()
                    
                    # Calculate next update time
                    next_update_time = start_time + self.control_interval
                    
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
    
    def _read_sensors_and_update_state(self):
        """Read all sensors and update controller state"""
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
        
        # Find minimum finger distance if any valid readings
        if finger_distances:
            all_fingers_distance = min(finger_distances.values())
        
        # Update state based on sensor readings
        self._update_state(all_fingers_distance, thumb_distance)
    
    def _update_state(self, finger_distance, thumb_distance):
        """Update controller state based on sensor readings"""
        current_time = time.time()
        time_in_state = current_time - self.state_entry_time
        
        # Only allow state transitions after minimum dwell time
        if time_in_state < self.min_dwell_time:
            return
        
        # State transitions with debouncing
        if self.state == "IDLE":
            # Check if any finger is detecting an object
            if finger_distance < self.finger_threshold:
                self.finger_detection_count += 1
                if self.finger_detection_count >= self.readings_required:
                    logger.info(f"Transitioning to THUMB_OPPOSING (finger distance: {finger_distance:.1f}mm)")
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
                    logger.info(f"Transitioning to GRIP_CLOSING (thumb distance: {thumb_distance:.1f}mm)")
                    self.state = "GRIP_CLOSING"
                    self.state_entry_time = current_time
                    self.thumb_detection_count = 0
            else:
                self.thumb_detection_count = 0
                
            # Check if fingers are no longer detecting (with hysteresis)
            if finger_distance > self.finger_threshold + self.hysteresis:
                self.release_detection_count += 1
                if self.release_detection_count >= self.readings_required:
                    logger.info(f"Returning to IDLE from THUMB_OPPOSING (finger distance: {finger_distance:.1f}mm)")
                    self.state = "IDLE" 
                    self.state_entry_time = current_time
                    self.release_detection_count = 0
            else:
                self.release_detection_count = 0
                    
        elif self.state == "GRIP_CLOSING":
            # Check if both thumb and fingers are no longer detecting (with hysteresis)
            if (thumb_distance > self.thumb_threshold + self.hysteresis or 
                finger_distance > self.finger_threshold + self.hysteresis):
                self.release_detection_count += 1
                if self.release_detection_count >= self.readings_required:
                    logger.info(f"Returning to IDLE from GRIP_CLOSING (thumb: {thumb_distance:.1f}mm, finger: {finger_distance:.1f}mm)")
                    self.state = "IDLE"
                    self.state_entry_time = current_time
                    self.release_detection_count = 0
            else:
                self.release_detection_count = 0
    
    def _should_move(self):
        """Check if we need to move fingers based on state changes"""
        if self.state == "IDLE" and any(self.current_positions.values()):
            return True
        elif self.state == "THUMB_OPPOSING" and self.current_positions["Thumb"] != self.thumb_opposition_angle:
            return True
        elif self.state == "GRIP_CLOSING" and any(pos != self.finger_close_angle 
                for finger, pos in self.current_positions.items() if finger != "Thumb"):
            return True
        return False
    
    def _enter_motion_phase(self):
        """Enter motion phase where sensors are ignored"""
        logger.debug(f"Entering motion phase from state {self.state}")
        self.in_motion_phase = True
        self.motion_start_time = time.time()
    
    def _move_fingers(self):
        """Execute finger movements based on current state"""
        if self.state == "IDLE":
            # When returning to IDLE, first open non-thumb fingers, then thumb
            # This prevents the thumb from colliding with other fingers during return
            for finger in self.motors.fingers:
                if finger != "Thumb":
                    self.target_positions[finger] = 0.0
            
            # Apply non-thumb finger movements first
            for finger, position in self.target_positions.items():
                if finger != "Thumb":
                    try:
                        self.motors.set_position(finger, position)
                        self.current_positions[finger] = position
                    except Exception as e:
                        logger.error(f"Error setting position for {finger}: {e}")
            
            # Small delay to let fingers move first
            time.sleep(0.2)
            
            # Now move thumb
            self.target_positions["Thumb"] = 0.0
            try:
                self.motors.set_position("Thumb", 0.0)
                self.current_positions["Thumb"] = 0.0
            except Exception as e:
                logger.error(f"Error setting position for Thumb: {e}")
                
        elif self.state == "THUMB_OPPOSING":
            # Position thumb in opposition first, before other fingers move
            self.target_positions["Thumb"] = self.thumb_opposition_angle
            try:
                self.motors.set_position("Thumb", self.thumb_opposition_angle)
                self.current_positions["Thumb"] = self.thumb_opposition_angle
            except Exception as e:
                logger.error(f"Error setting position for Thumb: {e}")
            
            # Small delay to let thumb move to opposition first
            time.sleep(0.2)
            
            # Now set other fingers
            for finger in self.motors.fingers:
                if finger != "Thumb":
                    self.target_positions[finger] = 0.0
                    try:
                        self.motors.set_position(finger, 0.0)
                        self.current_positions[finger] = 0.0
                    except Exception as e:
                        logger.error(f"Error setting position for {finger}: {e}")
                    
        elif self.state == "GRIP_CLOSING":
            # Keep thumb in opposition position
            self.target_positions["Thumb"] = self.thumb_opposition_angle
            
            # Verify thumb is in opposition before closing other fingers
            if self.current_positions["Thumb"] < self.thumb_opposition_angle * 0.8:
                # Ensure thumb is positioned first
                try:
                    self.motors.set_position("Thumb", self.thumb_opposition_angle)
                    self.current_positions["Thumb"] = self.thumb_opposition_angle
                    # Wait for thumb to reach position
                    time.sleep(0.2)
                except Exception as e:
                    logger.error(f"Error setting position for Thumb: {e}")
            
            # Now close other fingers
            for finger in self.motors.fingers:
                if finger != "Thumb":
                    self.target_positions[finger] = self.finger_close_angle
                    try:
                        self.motors.set_position(finger, self.finger_close_angle)
                        self.current_positions[finger] = self.finger_close_angle
                    except Exception as e:
                        logger.error(f"Error setting position for {finger}: {e}")
    
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
        
        logger.info("Simple opposition controller started")
    
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
                
                # Then open thumb last to prevent collision
                try:
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
        
        logger.info("Simple opposition controller stopped")
    
    def get_status(self):
        """Get current system status"""
        status = {
            "state": self.state,
            "time_in_state": time.time() - self.state_entry_time,
            "in_motion_phase": self.in_motion_phase,
            "fingers": {},
            "proximity": {}
        }
        
        # Get proximity values for MCP sensors
        for sensor_name in MCP_SENSORS:
            try:
                value, sensor_status = self.proximity.get_sensor_value(
                    sensor_name, filtered=True, with_status=True
                )
                status["proximity"][sensor_name] = {
                    "value": value,
                    "status": sensor_status
                }
            except Exception as e:
                logger.debug(f"Error getting sensor value for {sensor_name}: {e}")
                status["proximity"][sensor_name] = {
                    "value": None,
                    "status": "ERROR"
                }
        
        # Get finger status
        for finger_name in self.motors.fingers:
            try:
                position = self.motors.get_position(finger_name)
                current = self.motors.get_current(finger_name)
                target = self.target_positions.get(finger_name, 0.0)
                
                status["fingers"][finger_name] = {
                    "position": position,
                    "current": current,
                    "target": target
                }
            except Exception as e:
                logger.debug(f"Error getting status for {finger_name}: {e}")
                status["fingers"][finger_name] = {
                    "position": 0.0,
                    "current": 0.0,
                    "target": 0.0
                }
        
        return status


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
    parser = argparse.ArgumentParser(description='Simple Opposition Controller')
    
    parser.add_argument('--simulate', action='store_true',
                      help='Use simulated hardware')
    
    parser.add_argument('--rate', type=int, default=20,
                      help='Control loop rate in Hz (default: 20)')
    
    parser.add_argument('--port', type=str, default=None,
                      help='Serial port for Ability Hand (default: auto-detect)')
    
    parser.add_argument('--proximity-rate', type=int, default=5,
                      help='Proximity sensor sampling rate in Hz (default: 5)')
    
    parser.add_argument('--finger-close', type=float, default=30.0,
                      help='Finger closure angle in degrees (default: 30.0)')
    
    parser.add_argument('--thumb-oppose', type=float, default=30.0,
                      help='Thumb opposition angle in degrees (default: 30.0)')
    
    parser.add_argument('--finger-threshold', type=float, default=100.0,
                      help='Finger detection distance threshold in mm (default: 100.0)')
    
    parser.add_argument('--thumb-threshold', type=float, default=50.0,
                      help='Thumb detection distance threshold in mm (default: 50.0)')
    
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
        controller = SimpleOppositionController(
            control_rate=args.rate,
            proximity_rate=args.proximity_rate,
            use_simulated_motors=args.simulate,
            motor_interface_kwargs=motor_kwargs,
            finger_close_angle=args.finger_close,
            thumb_opposition_angle=args.thumb_oppose,
            finger_threshold=args.finger_threshold,
            thumb_threshold=args.thumb_threshold
        )
        
        # Start controller
        logger.info("Starting controller...")
        controller.start()
        
        # Monitor loop
        print("\nSimple Opposition Controller Active - Press Ctrl+C to stop")
        print("----------------------------------------------------------")
        print(f"This controller positions the thumb at {args.thumb_oppose}° opposition")
        print(f"when any finger detects an object within {args.finger_threshold}mm.")
        print(f"Once the thumb detects an object within {args.thumb_threshold}mm,")
        print(f"the other fingers close to {args.finger_close}°.")
        print("\nI2C bus contention is handled by separating sensor reading and motor control phases.")
        print("Note: VL6180X sensors report distance readings up to 100mm range")
        
        while running:
            try:
                # Get current status
                status = controller.get_status()
                
                # Clear screen
                os.system('clear' if os.name == 'posix' else 'cls')
                
                # Print status header
                print("\nSimple Opposition Controller")
                print("===========================")
                print(f"State: {status['state']} for {status['time_in_state']:.1f}s")
                print(f"Motion Phase: {'ACTIVE' if status['in_motion_phase'] else 'INACTIVE'}")
                
                # Print finger states
                print("\nFinger Status:")
                print("-------------")
                for finger, data in status["fingers"].items():
                    print(f"{finger:10s}: Pos: {data['position']:5.1f}° | "
                          f"Target: {data['target']:5.1f}° | "
                          f"Current: {data['current']:.3f}A")
                
                # Print proximity values
                print("\nProximity Sensors (MCP joints):")
                print("-----------------------------")
                for sensor, data in status["proximity"].items():
                    if data["value"] is None:
                        value_str = "N/A"
                    else:
                        value_str = f"{data['value']:5.1f}mm"
                    
                    # Highlight sensors in threshold range
                    if sensor == "Thumb1" and data["value"] is not None:
                        if data["value"] <= args.thumb_threshold:
                            value_str = f"{value_str} ⚠️"
                    elif data["value"] is not None and data["value"] <= args.finger_threshold:
                        value_str = f"{value_str} ⚠️"
                    
                    print(f"{sensor:10s}: {value_str:15s} | Status: {data['status']}")
                
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