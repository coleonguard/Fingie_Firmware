#!/usr/bin/env python3
"""
Simple Phase Toggle Test for Prosthetic Hand.

This script implements a simplified test that clearly toggles between two phases:
1. Sensor Reading Phase - Only reads from proximity sensors
2. Movement Phase - Only moves fingers to random positions

Each phase lasts for a fixed amount of time (1 second by default).
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
logger = logging.getLogger("PhaseToggleTest")

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

class PhaseToggleController:
    """
    Simple controller that clearly toggles between sensor reading and movement phases.
    
    This controller alternates between:
    1. SENSOR_PHASE: Only reading from proximity sensors (1 second)
    2. MOVEMENT_PHASE: Only moving fingers to random positions (1 second)
    """
    
    def __init__(
        self,
        control_rate=20,
        proximity_rate=None,
        use_simulated_motors=False,
        motor_interface_kwargs=None,
        phase_duration=1.0,
        movement_amplitude=15.0
    ):
        """
        Initialize the phase toggle controller.
        
        Args:
            control_rate: Control loop frequency in Hz
            proximity_rate: Proximity sampling rate in Hz (defaults to 2x control_rate)
            use_simulated_motors: Whether to use simulated motors
            motor_interface_kwargs: Additional kwargs for motor interface
            phase_duration: Duration of each phase in seconds
            movement_amplitude: Maximum movement amplitude in degrees
        """
        # Save parameters
        self.control_rate = control_rate
        self.control_interval = 1.0 / control_rate
        self.phase_duration = phase_duration
        self.movement_amplitude = movement_amplitude
        
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
                approach_threshold=40,
                contact_threshold=10,
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
        
        # Phase tracking
        self.current_phase = "SENSOR_PHASE"  # Start with sensor phase
        self.phase_start_time = time.time()
        self.sensor_readings = {}
        
        # Finger positions
        self.current_positions = {finger: 0.0 for finger in self.motors.fingers}
        self.target_positions = {finger: 0.0 for finger in self.motors.fingers}
        
        # Thread control
        self.running = False
        self.shutdown_event = threading.Event()
        self.thread = None
        
        logger.info(f"Phase toggle controller initialized with phase_duration={phase_duration}s")
    
    def _control_loop(self):
        """Main control loop that clearly toggles between phases"""
        next_update_time = time.time()
        
        while self.running and not self.shutdown_event.is_set():
            current_time = time.time()
            
            # Check if it's time for the next control update
            if current_time >= next_update_time:
                try:
                    # Check if it's time to switch phases
                    time_in_phase = current_time - self.phase_start_time
                    if time_in_phase >= self.phase_duration:
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
        """Toggle between sensor reading and movement phases"""
        if self.current_phase == "SENSOR_PHASE":
            # Switch to movement phase
            self.current_phase = "MOVEMENT_PHASE"
            logger.info("🔄 SWITCHING TO MOVEMENT PHASE - only finger movement for 1 second")
            print("\n🔄 SWITCHING TO MOVEMENT PHASE - only finger movement for 1 second")
            
            # Generate random positions for movement
            self._generate_random_positions()
        else:
            # Switch to sensor phase
            self.current_phase = "SENSOR_PHASE"
            logger.info("🔄 SWITCHING TO SENSOR PHASE - only sensor reading for 1 second")
            print("\n🔄 SWITCHING TO SENSOR PHASE - only sensor reading for 1 second")
            
            # Reset finger positions
            for finger in self.motors.fingers:
                try:
                    self.motors.set_position(finger, 0.0)
                    self.current_positions[finger] = 0.0
                except Exception as e:
                    logger.error(f"Error resetting position for {finger}: {e}")
        
        # Reset phase timer
        self.phase_start_time = time.time()
    
    def _execute_sensor_phase(self):
        """Execute sensor reading phase - ONLY read from sensors, NO movement"""
        # Read all proximity sensors
        for sensor_name in MCP_SENSORS:
            try:
                value, status = self.proximity.get_sensor_value(
                    sensor_name, filtered=True, with_status=True
                )
                self.sensor_readings[sensor_name] = {
                    "value": value,
                    "status": status
                }
            except Exception as e:
                logger.error(f"Error reading sensor {sensor_name}: {e}")
                self.sensor_readings[sensor_name] = {
                    "value": None,
                    "status": "ERROR"
                }
    
    def _execute_movement_phase(self):
        """Execute movement phase - ONLY move fingers, NO sensor reading"""
        # Move fingers to target positions
        for finger in self.motors.fingers:
            try:
                target = self.target_positions[finger]
                self.motors.set_position(finger, target)
                self.current_positions[finger] = target
            except Exception as e:
                logger.error(f"Error setting position for {finger}: {e}")
    
    def _generate_random_positions(self):
        """Generate random positions for all fingers"""
        for finger in self.motors.fingers:
            # Random position between 0 and movement_amplitude
            self.target_positions[finger] = random.uniform(0, self.movement_amplitude)
            logger.info(f"Generated random position for {finger}: {self.target_positions[finger]:.1f}°")
    
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
        
        logger.info("Phase toggle controller started")
    
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
        
        logger.info("Phase toggle controller stopped")
    
    def get_status(self):
        """Get current system status"""
        status = {
            "phase": self.current_phase,
            "time_in_phase": time.time() - self.phase_start_time,
            "time_to_next_phase": max(0, self.phase_duration - (time.time() - self.phase_start_time)),
            "fingers": {},
            "proximity": self.sensor_readings.copy()
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
    parser = argparse.ArgumentParser(description='Phase Toggle Test')
    
    parser.add_argument('--simulate', action='store_true',
                      help='Use simulated hardware')
    
    parser.add_argument('--rate', type=int, default=20,
                      help='Control loop rate in Hz (default: 20)')
    
    parser.add_argument('--port', type=str, default=None,
                      help='Serial port for Ability Hand (default: auto-detect)')
    
    parser.add_argument('--proximity-rate', type=int, default=5,
                      help='Proximity sensor sampling rate in Hz (default: 5)')
    
    parser.add_argument('--phase-duration', type=float, default=1.0,
                      help='Duration of each phase in seconds (default: 1.0)')
    
    parser.add_argument('--amplitude', type=float, default=15.0,
                      help='Maximum movement amplitude in degrees (default: 15.0)')
    
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
        controller = PhaseToggleController(
            control_rate=args.rate,
            proximity_rate=args.proximity_rate,
            use_simulated_motors=args.simulate,
            motor_interface_kwargs=motor_kwargs,
            phase_duration=args.phase_duration,
            movement_amplitude=args.amplitude
        )
        
        # Start controller
        logger.info("Starting controller...")
        controller.start()
        
        # Monitor loop
        print("\nPhase Toggle Test Active - Press Ctrl+C to stop")
        print("----------------------------------------------")
        print(f"This controller alternates between two phases, each lasting {args.phase_duration} seconds:")
        print("1. SENSOR_PHASE: Only reading from proximity sensors")
        print("2. MOVEMENT_PHASE: Only moving fingers to random positions")
        print(f"Maximum movement amplitude: {args.amplitude}°")
        
        while running:
            try:
                # Get current status
                status = controller.get_status()
                
                # Clear screen
                os.system('clear' if os.name == 'posix' else 'cls')
                
                # Print status header
                print("\nPhase Toggle Test")
                print("================")
                print(f"Current Phase: {status['phase']}")
                print(f"Time in phase: {status['time_in_phase']:.1f}s")
                print(f"Next phase in: {status['time_to_next_phase']:.1f}s")
                
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