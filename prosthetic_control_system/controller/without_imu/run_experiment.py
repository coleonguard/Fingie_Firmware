#!/usr/bin/env python3
"""
Experimental controller that makes fingers twitch instead of fully closing.

This script runs a modified version of the proximity controller where
fingers only twitch (close 5° then reopen) when an object is detected
within 10mm, rather than forming a full grasp.
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
    level=logging.INFO,  # Can change to logging.DEBUG for more detailed output
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("ExperimentController")

# Set logger level explicitly
logger.setLevel(logging.INFO)  # Can change to logging.DEBUG for more detailed output

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

class ExperimentController:
    """
    Experimental controller that makes fingers twitch instead of fully closing.
    This simplified controller makes fingers twitch (close 5° then reopen) when
    proximity sensors detect objects within 10mm.
    """
    
    def __init__(
        self,
        control_rate=20,
        proximity_rate=None,
        use_simulated_motors=False,
        motor_interface_kwargs=None,
    ):
        """
        Initialize the experimental controller.
        
        Args:
            control_rate: Control loop frequency in Hz
            proximity_rate: Proximity sampling rate in Hz (defaults to 2x control_rate)
            use_simulated_motors: Whether to use simulated motors
            motor_interface_kwargs: Additional kwargs for motor interface
        """
        # Save parameters
        self.control_rate = control_rate
        self.control_interval = 1.0 / control_rate
        
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
                approach_threshold=40,  # Not really used in this experiment
                contact_threshold=10,   # We'll use this as our trigger threshold
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
        
        # Twitch state for each finger (used to track the twitch cycle)
        # 0 = idle (no twitch), 1 = closing, 2 = opening, 3 = reset (waiting for object to move away)
        self.twitch_state = {finger: 0 for finger in self.motors.fingers}
        self.twitch_position = {finger: 0.0 for finger in self.motors.fingers}
        
        # Track previous distance readings to detect when objects move away
        self.prev_distances = {}
        # Track whether we're waiting for the object to move away before allowing another twitch
        self.awaiting_reset = {finger: False for finger in self.motors.fingers}
        
        # Thread control
        self.running = False
        self.shutdown_event = threading.Event()
        self.thread = None
    
    def _process_finger(self, sensor_name, finger_name):
        """
        Process control for a single finger based on proximity readings.
        Instead of full closure, this will make fingers twitch when objects
        are detected within 10mm.
        
        Args:
            sensor_name: Name of the proximity sensor
            finger_name: Name of the corresponding finger
            
        Returns:
            Dictionary with finger state information
        """
        # Get sensor distance with error handling
        try:
            distance, status = self.proximity.get_sensor_value(
                sensor_name, filtered=True, with_status=True
            )
            
            # Safety check - if sensor reading is invalid, default to safe distance
            if distance is None or status == "BAD":
                logger.debug(f"Invalid sensor reading for {sensor_name}, defaulting to safe distance")
                distance = 100
                status = "DEFAULT"
            
            # Get current position
            try:
                position = self.motors.get_position(finger_name)
            except Exception as e:
                logger.warning(f"Error getting position for {finger_name}: {e}")
                position = 0.0  # Default to fully open
            
            # Store distance for tracking object movements
            sensor_key = f"{finger_name}_sensor"
            prev_distance = self.prev_distances.get(sensor_key, 100)
            self.prev_distances[sensor_key] = distance
            
            # Debug output
            logger.debug(f"{finger_name}: distance={distance}mm, state={self.twitch_state[finger_name]}, "
                        f"awaiting_reset={self.awaiting_reset[finger_name]}, position={position}°")

            # Implement twitching behavior - only for MCP sensors (prox1)
            if sensor_name.endswith("1"):
                # Check if this is the first reading for this finger
                if sensor_key not in self.prev_distances:
                    self.prev_distances[sensor_key] = distance
                
                # CASE 1: Object is within twitching range (25mm) and we're ready for a new twitch
                if distance <= 25 and self.twitch_state[finger_name] == 0 and not self.awaiting_reset[finger_name]:
                    # Start closing (twitch phase 1)
                    target_position = 5.0  # Close 5 degrees
                    self.twitch_state[finger_name] = 1
                    self.twitch_position[finger_name] = target_position
                    logger.debug(f"{finger_name} starting twitch (closing)")
                    
                    # Set position
                    try:
                        self.motors.set_position(finger_name, target_position)
                    except Exception as e:
                        logger.error(f"Error setting position for {finger_name}: {e}")
                
                # CASE 2: Finger is closing and has reached the target position
                elif self.twitch_state[finger_name] == 1 and position >= 4.0:
                    # Once we've reached close to 5 degrees, start opening (twitch phase 2)
                    # Using 4.0 as threshold to account for motor lag
                    target_position = 0.0  # Open fully
                    self.twitch_state[finger_name] = 2
                    self.twitch_position[finger_name] = target_position
                    logger.debug(f"{finger_name} continuing twitch (opening)")
                    
                    # Set position
                    try:
                        self.motors.set_position(finger_name, target_position)
                    except Exception as e:
                        logger.error(f"Error setting position for {finger_name}: {e}")
                
                # CASE 3: Finger is opening and has reached open position
                elif self.twitch_state[finger_name] == 2 and position <= 2.5:  # Increased threshold to 2.5 degrees
                    # Once we've opened up, enter reset waiting state
                    # The finger won't twitch again until object moves away beyond 40mm
                    self.twitch_state[finger_name] = 0
                    self.awaiting_reset[finger_name] = True
                    logger.debug(f"{finger_name} completed twitch cycle, waiting for object to move away")
                
                # CASE 4: Check if object has moved away enough for a new twitch
                elif self.awaiting_reset[finger_name] and distance >= 40:
                    # Object has moved far enough away (>=40mm), allow new twitches
                    self.awaiting_reset[finger_name] = False
                    # Ensure twitch state is properly reset to idle
                    self.twitch_state[finger_name] = 0
                    # Make sure position is zeroed
                    if self.motors and not self.shutdown_event.is_set():
                        try:
                            self.motors.set_position(finger_name, 0.0)
                        except Exception as e:
                            logger.error(f"Error setting position during reset for {finger_name}: {e}")
                    logger.info(f"{finger_name} RESET - object moved away, ready for new twitch")
                
                # CASE 5: If we're in the middle of a twitch, keep the current target
                elif self.twitch_state[finger_name] in (1, 2):
                    target_position = self.twitch_position[finger_name]
                    try:
                        self.motors.set_position(finger_name, target_position)
                    except Exception as e:
                        logger.error(f"Error setting position for {finger_name}: {e}")
            
            # If not an MCP sensor or object is too far away
            else:
                # If not within 25mm or not an MCP sensor, ensure finger is open
                # Only reset if we're not in the middle of opening already
                if self.twitch_state[finger_name] != 2:
                    target_position = 0.0
                    
                    # Keep twitch state at 0, but don't reset awaiting_reset flag
                    # (that only gets reset when object moves far enough away)
                    self.twitch_state[finger_name] = 0
                    
                    try:
                        self.motors.set_position(finger_name, target_position)
                    except Exception as e:
                        logger.error(f"Error setting position for {finger_name}: {e}")
            
            # Return current state
            return {
                "distance": distance,
                "status": status,
                "position": position,
                "twitch_state": self.twitch_state[finger_name],
                "awaiting_reset": self.awaiting_reset[finger_name],
                "target_position": self.twitch_position.get(finger_name, 0.0)
            }
            
        except Exception as e:
            logger.error(f"Error processing finger {finger_name}: {e}")
            return {
                "distance": 100,
                "status": "ERROR",
                "position": 0.0,
                "twitch_state": 0,
                "target_position": 0.0
            }
    
    def _control_loop(self):
        """Main control loop"""
        next_update_time = time.time()
        
        while self.running and not self.shutdown_event.is_set():
            start_time = time.time()
            
            # Check if it's time for the next control update
            if start_time >= next_update_time:
                try:
                    # Read all finger sensors and process control
                    finger_data = {}
                    for sensor_name, finger_name in self.finger_mapping.items():
                        # Skip if shutting down
                        if not self.running or self.shutdown_event.is_set():
                            break
                        finger_data[finger_name] = self._process_finger(sensor_name, finger_name)
                    
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
        
        logger.info("Experiment controller started")
    
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
        
        # Ensure all fingers are in open position
        try:
            if self.motors:
                for finger in self.motors.fingers:
                    try:
                        self.motors.set_position(finger, 0.0)
                    except Exception as e:
                        logger.error(f"Error opening finger {finger}: {e}")
                
                # Give motors time to move
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
        
        logger.info("Experiment controller stopped")
    
    def get_status(self):
        """Get current system status"""
        status = {
            "fingers": {},
            "proximity": {}
        }
        
        # Get proximity values
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
                twitch_state = self.twitch_state.get(finger_name, 0)
                awaiting_reset = self.awaiting_reset.get(finger_name, False)
                
                # Convert twitch state to descriptive string
                if twitch_state == 0:
                    if awaiting_reset:
                        state_str = "WAITING"
                    else:
                        state_str = "IDLE"
                elif twitch_state == 1:
                    state_str = "CLOSING"
                elif twitch_state == 2:
                    state_str = "OPENING"
                else:
                    state_str = f"UNKNOWN ({twitch_state})"
                
                status["fingers"][finger_name] = {
                    "position": position,
                    "current": current,
                    "twitch_state": state_str,
                    "awaiting_reset": awaiting_reset,
                    "target": self.twitch_position.get(finger_name, 0.0)
                }
            except Exception as e:
                logger.debug(f"Error getting status for {finger_name}: {e}")
                status["fingers"][finger_name] = {
                    "position": 0.0,
                    "current": 0.0,
                    "twitch_state": "ERROR",
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
    parser = argparse.ArgumentParser(description='Experimental Twitch Controller')
    
    parser.add_argument('--simulate', action='store_true',
                      help='Use simulated hardware')
    
    parser.add_argument('--rate', type=int, default=20,
                      help='Control loop rate in Hz (default: 20)')
    
    parser.add_argument('--port', type=str, default=None,
                      help='Serial port for Ability Hand (default: auto-detect)')
    
    parser.add_argument('--proximity-rate', type=int, default=5,
                      help='Proximity sensor sampling rate in Hz (default: 5)')
    
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
        logger.info(f"Creating controller (simulated: {args.simulate}, rate: {args.rate}Hz, proximity_rate: {args.proximity_rate}Hz)")
        controller = ExperimentController(
            control_rate=args.rate,
            proximity_rate=args.proximity_rate,
            use_simulated_motors=args.simulate,
            motor_interface_kwargs=motor_kwargs
        )
        
        # Start controller
        logger.info("Starting controller...")
        controller.start()
        
        # Monitor loop
        print("\nExperimental Twitch Controller Active - Press Ctrl+C to stop")
        print("----------------------------------------------------------")
        print("This controller makes fingers twitch (close 5° then reopen)")
        print("when proximity sensors detect objects within 25mm")
        print("\nAfter each twitch, the finger enters a RESET state and waits until")
        print("the object has moved away to at least 40mm before allowing a new twitch.")
        print("\nNote: VL6180X sensors report distance readings up to 100mm range")
        print("When no object is detected, the sensor reports the maximum distance value.")
        print("Status will be OK for valid readings, SUB for substituted values, and BAD for failures.")
        
        while running:
            try:
                # Get current status
                status = controller.get_status()
                
                # Clear screen
                os.system('clear' if os.name == 'posix' else 'cls')
                
                # Print status header
                print("\nExperimental Twitch Controller")
                print("=============================")
                
                # Print finger states
                print("\nFinger Status:")
                print("-------------")
                for finger, data in status["fingers"].items():
                    # Add reset indicator if awaiting_reset is True
                    reset_indicator = "(RESET)" if data.get('awaiting_reset', False) else ""
                    
                    print(f"{finger:10s}: {data['twitch_state']:10s} {reset_indicator:8s} | "
                          f"Pos: {data['position']:5.1f}° | "
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
                    
                    # Highlight sensors within twitch threshold
                    if data["value"] is not None and data["value"] <= 10:
                        value_str = f"{value_str} ⚠️"
                    
                    print(f"{sensor:10s}: {value_str:10s} | Status: {data['status']}")
                
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