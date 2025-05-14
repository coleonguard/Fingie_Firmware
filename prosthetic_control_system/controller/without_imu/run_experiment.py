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
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("ExperimentController")

# Add parent directory to path if needed
parent_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

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
        # 0 = idle (no twitch), 1 = closing, 2 = opening
        self.twitch_state = {finger: 0 for finger in self.motors.fingers}
        self.twitch_position = {finger: 0.0 for finger in self.motors.fingers}
        
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
            
            # Implement twitching behavior - only for MCP sensors (prox1)
            if sensor_name.endswith("1") and distance <= 10:  # Within 10mm
                # Check current twitch state
                if self.twitch_state[finger_name] == 0:
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
                
                elif self.twitch_state[finger_name] == 2 and position <= 1.0:
                    # Once we've opened up, reset twitch state for next cycle
                    # Using 1.0 as threshold to account for motor lag
                    self.twitch_state[finger_name] = 0
                    logger.debug(f"{finger_name} completed twitch cycle")
                
                # If we're in the middle of a twitch, keep the current target
                elif self.twitch_state[finger_name] in (1, 2):
                    target_position = self.twitch_position[finger_name]
                    try:
                        self.motors.set_position(finger_name, target_position)
                    except Exception as e:
                        logger.error(f"Error setting position for {finger_name}: {e}")
            
            else:
                # If not within 10mm or not an MCP sensor, ensure finger is open
                # Only reset if we're not in the middle of opening already
                if self.twitch_state[finger_name] != 2:
                    target_position = 0.0
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
                
                # Convert twitch state to descriptive string
                if twitch_state == 0:
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
    
    args = parser.parse_args()
    
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
        print("when proximity sensors detect objects within 10mm")
        
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
                    print(f"{finger:10s}: {data['twitch_state']:10s} | "
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