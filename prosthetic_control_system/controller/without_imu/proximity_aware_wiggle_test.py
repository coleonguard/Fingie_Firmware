#!/usr/bin/env python3
"""
Proximity-Aware Random Wiggle Test for Prosthetic Hand.

This script combines proximity sensing with continuous random finger wiggling.
Key features:

1. Clear separation between sensor reading and movement phases
2. Randomized phase timing with configurable mean and standard deviation
3. Continuous wiggling without returning to neutral positions
4. Smart direction reversal to keep fingers within angle constraints
5. Random finger selection and correlated movements
6. Enhanced visualization of finger positions and movement directions
7. Proximity sensor readings displayed when not wiggling
"""

import os
import sys
import time
import signal
import argparse
import logging
import threading
import random
import math

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("ProximityAwareWiggle")

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

class ProximityAwareWiggleController:
    """
    Controller that combines proximity sensing with continuous random wiggling.
    
    This controller alternates between:
    1. SENSOR_PHASE: Only reading from proximity sensors (random duration)
    2. WIGGLE_PHASE: Only moving fingers with continuous random motion (random duration)
    
    Finger angles are kept below MAX_ANGLE with smart direction changes.
    Proximity readings are captured during the sensor phase and displayed.
    """
    
    def __init__(
        self,
        control_rate=20,
        proximity_rate=None,
        use_simulated_motors=False,
        motor_interface_kwargs=None,
        phase_mean_duration=1.0,
        phase_std_dev=0.2,
        min_phase_duration=0.5,
        max_angle=30.0,
        wiggle_speed=5.0,
        min_fingers=1,
        max_fingers=4,
        proximity_threshold=30.0
    ):
        """
        Initialize the proximity-aware wiggle controller.
        
        Args:
            control_rate: Control loop frequency in Hz
            proximity_rate: Proximity sampling rate in Hz (defaults to 2x control_rate)
            use_simulated_motors: Whether to use simulated motors
            motor_interface_kwargs: Additional kwargs for motor interface
            phase_mean_duration: Mean duration of each phase in seconds
            phase_std_dev: Standard deviation of phase duration in seconds
            min_phase_duration: Minimum phase duration in seconds
            max_angle: Maximum finger angle (percentage of full range, 0-100)
            wiggle_speed: Maximum movement speed in degrees per control cycle
            min_fingers: Minimum number of fingers to wiggle
            max_fingers: Maximum number of fingers to wiggle
            proximity_threshold: Distance threshold for proximity highlighting (mm)
        """
        # Save parameters
        self.control_rate = control_rate
        self.control_interval = 1.0 / control_rate
        self.phase_mean_duration = phase_mean_duration
        self.phase_std_dev = phase_std_dev
        self.min_phase_duration = min_phase_duration
        self.max_angle = max_angle
        self.wiggle_speed = wiggle_speed
        self.min_fingers = min_fingers
        self.max_fingers = max_fingers
        self.proximity_threshold = proximity_threshold
        
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
        self.next_phase_time = time.time() + self._generate_phase_duration()
        self.sensor_readings = {}
        
        # Finger positions
        self.current_positions = {finger: 0.0 for finger in self.motors.fingers}
        self.target_positions = {finger: 0.0 for finger in self.motors.fingers}
        
        # Finger movement directions (1 = increasing, -1 = decreasing)
        self.movement_directions = {finger: random.choice([-1, 1]) for finger in self.motors.fingers}
        
        # Wiggle information
        self.wiggling_fingers = []
        self.wiggle_count = 0
        self.phase_count = 0
        
        # Proximity detection flags
        self.proximity_detected = {sensor: False for sensor in MCP_SENSORS}
        self.proximity_detection_count = {sensor: 0 for sensor in MCP_SENSORS}
        
        # Thread control
        self.running = False
        self.shutdown_event = threading.Event()
        self.thread = None
        
        logger.info(f"Proximity-aware wiggle controller initialized with phase_mean_duration={phase_mean_duration}s, std_dev={phase_std_dev}s")
    
    def _generate_phase_duration(self):
        """Generate random phase duration using Gaussian distribution"""
        # Generate random duration using Gaussian distribution
        duration = random.gauss(self.phase_mean_duration, self.phase_std_dev)
        
        # Apply bounds
        duration = max(self.min_phase_duration, duration)
        
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
                    else:  # WIGGLE_PHASE
                        self._execute_wiggle_phase()
                    
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
        """Toggle between sensor reading and wiggle phases with random durations"""
        self.phase_count += 1
        
        if self.current_phase == "SENSOR_PHASE":
            # Switch to wiggle phase
            self.current_phase = "WIGGLE_PHASE"
            self.wiggle_count += 1
            
            # Generate random phase duration
            phase_duration = self._generate_phase_duration()
            self.next_phase_time = time.time() + phase_duration
            
            logger.info(f"🔄 SWITCHING TO WIGGLE PHASE #{self.wiggle_count} - duration: {phase_duration:.2f}s")
            print(f"\n🔄 SWITCHING TO WIGGLE PHASE #{self.wiggle_count} - duration: {phase_duration:.2f}s")
            
            # Select random fingers to wiggle
            self._select_random_fingers()
            
            # Initialize target positions near current positions
            for finger in self.wiggling_fingers:
                current_pos = self.current_positions.get(finger, 0.0)
                # Random initial movement (small)
                initial_delta = random.uniform(-self.wiggle_speed/2, self.wiggle_speed/2)
                self.target_positions[finger] = current_pos + initial_delta
        else:
            # Switch to sensor phase
            self.current_phase = "SENSOR_PHASE"
            
            # Generate random phase duration
            phase_duration = self._generate_phase_duration()
            self.next_phase_time = time.time() + phase_duration
            
            logger.info(f"🔄 SWITCHING TO SENSOR PHASE - duration: {phase_duration:.2f}s")
            print(f"\n🔄 SWITCHING TO SENSOR PHASE - duration: {phase_duration:.2f}s")
            
            # Keep the current finger positions (no reset to neutral)
            # Just clear the wiggling fingers list
            self.wiggling_fingers = []
        
        # Reset phase timer
        self.phase_start_time = time.time()
    
    def _select_random_fingers(self):
        """Select random fingers to wiggle"""
        # Get all non-thumb fingers
        non_thumb_fingers = [f for f in self.motors.fingers if f != "Thumb"]
        
        # Determine how many fingers to wiggle (between min and max)
        num_fingers = random.randint(self.min_fingers, min(self.max_fingers, len(non_thumb_fingers)))
        
        # Randomly select fingers
        self.wiggling_fingers = random.sample(non_thumb_fingers, num_fingers)
        
        finger_list = ", ".join(self.wiggling_fingers)
        logger.info(f"Selected {num_fingers} fingers to wiggle: {finger_list}")
        print(f"Selected {num_fingers} fingers to wiggle: {finger_list}")
    
    def _execute_sensor_phase(self):
        """Execute sensor reading phase - ONLY read from sensors, NO movement"""
        # Read all proximity sensors
        for sensor_name in MCP_SENSORS:
            try:
                value, status = self.proximity.get_sensor_value(
                    sensor_name, filtered=True, with_status=True
                )
                
                # Store the reading
                self.sensor_readings[sensor_name] = {
                    "value": value,
                    "status": status
                }
                
                # Check for proximity detection (with debouncing)
                if value is not None and status != "BAD" and value < self.proximity_threshold:
                    # Increment detection counter
                    self.proximity_detection_count[sensor_name] += 1
                    if self.proximity_detection_count[sensor_name] >= 3:
                        # Mark as detected after 3 consecutive readings
                        if not self.proximity_detected[sensor_name]:
                            logger.info(f"Proximity detected at {sensor_name}: {value:.1f}mm")
                        self.proximity_detected[sensor_name] = True
                else:
                    # Reset counter and detection flag if reading is above threshold
                    self.proximity_detection_count[sensor_name] = 0
                    if self.proximity_detected[sensor_name]:
                        logger.info(f"Proximity cleared at {sensor_name}")
                    self.proximity_detected[sensor_name] = False
                    
            except Exception as e:
                logger.error(f"Error reading sensor {sensor_name}: {e}")
                self.sensor_readings[sensor_name] = {
                    "value": None,
                    "status": "ERROR"
                }
    
    def _execute_wiggle_phase(self):
        """Execute wiggle phase - ONLY move fingers with continuous random motion"""
        # Generate new random target positions
        self._update_wiggle_targets()
        
        # Move selected fingers to target positions
        for finger in self.wiggling_fingers:
            try:
                target = self.target_positions[finger]
                self.motors.set_position(finger, target)
                self.current_positions[finger] = target
            except Exception as e:
                logger.error(f"Error setting position for {finger}: {e}")
    
    def _update_wiggle_targets(self):
        """Update target positions for wiggling fingers with smart direction changes"""
        # Generate a shared random component for partial correlation
        master_random = random.gauss(0, 1)
        correlation = 0.6  # 60% correlation between fingers
        
        for finger in self.wiggling_fingers:
            current_pos = self.current_positions.get(finger, 0.0)
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
            
            # Calculate movement amount (base movement + random component)
            # Scale random component to be smaller than base movement
            base_movement = self.wiggle_speed * current_dir
            random_component = finger_random * (self.wiggle_speed * 0.5)
            movement = base_movement + random_component
            
            # Calculate new target position
            new_target = current_pos + movement
            
            # Enforce bounds
            new_target = max(0, min(new_target, self.max_angle))
            
            # If we hit a bound, reverse direction for next time
            if new_target <= 0 or new_target >= self.max_angle:
                self.movement_directions[finger] = -current_dir
            
            # Set the new target
            self.target_positions[finger] = new_target
    
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
        
        logger.info("Proximity-aware wiggle controller started")
    
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
        
        logger.info("Proximity-aware wiggle controller stopped")
    
    def get_status(self):
        """Get current system status"""
        current_time = time.time()
        status = {
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
                "speed": self.wiggle_speed
            },
            "proximity_detection": self.proximity_detected.copy()
        }
        
        # Get finger status
        for finger_name in self.motors.fingers:
            try:
                position = self.motors.get_position(finger_name)
                current = self.motors.get_current(finger_name)
                target = self.target_positions.get(finger_name, 0.0)
                direction = self.movement_directions.get(finger_name, 0)
                is_wiggling = finger_name in self.wiggling_fingers
                
                # Determine direction symbol
                direction_symbol = "→" if direction > 0 else "←" if direction < 0 else "-"
                
                status["fingers"][finger_name] = {
                    "position": position,
                    "current": current,
                    "target": target,
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
                    "direction": 0,
                    "direction_symbol": "-",
                    "wiggling": False
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
    parser = argparse.ArgumentParser(description='Proximity-Aware Random Wiggle Test')
    
    parser.add_argument('--simulate', action='store_true',
                      help='Use simulated hardware')
    
    parser.add_argument('--rate', type=int, default=20,
                      help='Control loop rate in Hz (default: 20)')
    
    parser.add_argument('--port', type=str, default=None,
                      help='Serial port for Ability Hand (default: auto-detect)')
    
    parser.add_argument('--proximity-rate', type=int, default=5,
                      help='Proximity sensor sampling rate in Hz (default: 5)')
    
    parser.add_argument('--phase-mean', type=float, default=1.0,
                      help='Mean duration of each phase in seconds (default: 1.0)')
    
    parser.add_argument('--phase-std', type=float, default=0.2,
                      help='Standard deviation of phase duration in seconds (default: 0.2)')
    
    parser.add_argument('--min-phase', type=float, default=0.5,
                      help='Minimum phase duration in seconds (default: 0.5)')
    
    parser.add_argument('--max-angle', type=float, default=30.0,
                      help='Maximum finger angle in degrees (default: 30.0)')
    
    parser.add_argument('--wiggle-speed', type=float, default=5.0,
                      help='Maximum wiggle speed in degrees per control cycle (default: 5.0)')
    
    parser.add_argument('--min-fingers', type=int, default=1,
                      help='Minimum number of fingers to wiggle (default: 1)')
    
    parser.add_argument('--max-fingers', type=int, default=4,
                      help='Maximum number of fingers to wiggle (default: 4)')
    
    parser.add_argument('--proximity-threshold', type=float, default=30.0,
                      help='Distance threshold for proximity highlighting in mm (default: 30.0)')
    
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
        controller = ProximityAwareWiggleController(
            control_rate=args.rate,
            proximity_rate=args.proximity_rate,
            use_simulated_motors=args.simulate,
            motor_interface_kwargs=motor_kwargs,
            phase_mean_duration=args.phase_mean,
            phase_std_dev=args.phase_std,
            min_phase_duration=args.min_phase,
            max_angle=args.max_angle,
            wiggle_speed=args.wiggle_speed,
            min_fingers=args.min_fingers,
            max_fingers=args.max_fingers,
            proximity_threshold=args.proximity_threshold
        )
        
        # Start controller
        logger.info("Starting controller...")
        controller.start()
        
        # Monitor loop
        print("\nProximity-Aware Random Wiggle Test Active - Press Ctrl+C to stop")
        print("-----------------------------------------------------------")
        print("This controller alternates between two phases with random durations:")
        print(f"1. SENSOR_PHASE: Only reading from proximity sensors (μ={args.phase_mean}s, σ={args.phase_std}s)")
        print(f"2. WIGGLE_PHASE: Only moving randomly selected fingers (μ={args.phase_mean}s, σ={args.phase_std}s)")
        print(f"Will select {args.min_fingers}-{args.max_fingers} fingers to wiggle each time")
        print(f"Maximum finger angle: {args.max_angle}°, Maximum wiggle speed: {args.wiggle_speed}°/cycle")
        print(f"Proximity threshold: {args.proximity_threshold}mm")
        
        while running:
            try:
                # Get current status
                status = controller.get_status()
                
                # Clear screen
                os.system('clear' if os.name == 'posix' else 'cls')
                
                # Print status header
                print("\nProximity-Aware Random Wiggle Test")
                print("=================================")
                print(f"Current Phase: {status['phase']} (#{status['phase_count']})")
                print(f"Time in phase: {status['time_in_phase']:.1f}s")
                print(f"Next phase in: {status['time_to_next_phase']:.1f}s")
                
                # Print wiggle info
                wiggle_info = status["wiggle_info"]
                if status["phase"] == "WIGGLE_PHASE" and wiggle_info["wiggling_fingers"]:
                    finger_list = ", ".join(wiggle_info["wiggling_fingers"])
                    print(f"\nWiggle count: {wiggle_info['count']}")
                    print(f"Wiggling fingers: {finger_list}")
                    print(f"Max angle: {wiggle_info['max_angle']}°, Speed: {wiggle_info['speed']}°/cycle")
                
                # Print finger states with improved visualization
                print("\nFinger Status:")
                print("-------------")
                for finger, data in status["fingers"].items():
                    wiggle_indicator = "✓" if data["wiggling"] else " "
                    dir_symbol = data["direction_symbol"]
                    
                    # Create a visual bar to show position relative to max
                    position_percent = int(data["position"] / wiggle_info["max_angle"] * 20)
                    position_bar = "▓" * position_percent + "░" * (20 - position_percent)
                    
                    print(f"{finger:10s}: [{wiggle_indicator}] {dir_symbol} {position_bar} {data['position']:5.1f}° | "
                          f"Target: {data['target']:5.1f}° | "
                          f"Current: {data['current']:.3f}A")
                
                # Only show detailed proximity values in sensor phase
                if status["phase"] == "SENSOR_PHASE":
                    # Print proximity sensor readings with improved visualization
                    print("\nProximity Sensors (MCP joints):")
                    print("-----------------------------")
                    
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
                            
                            # Add visual indicator for detected proximities
                            if status["proximity_detection"].get(sensor, False):
                                distance_bar = "█" * bar_length + "▁" * (20 - bar_length) + " 📡"
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