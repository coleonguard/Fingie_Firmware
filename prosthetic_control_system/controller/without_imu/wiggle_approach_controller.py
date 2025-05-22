#!/usr/bin/env python3
"""
Wiggle Approach Controller for Prosthetic Hand.

This controller extends the SafeThumbOppositionController by adding a
wiggle motion during the approach phase to provide better tactile feedback
and help with object detection.

Key features:
1. Safe thumb opposition ensures thumb never collides with other fingers
2. Finger wiggling during approach phase for enhanced tactile feedback
3. I2C bus contention handling with separate sensor and motor phases
"""

import os
import sys
import time
import signal
import argparse
import logging
import threading
import math
import random

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("WiggleApproachController")

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
from prosthetic_control_system.controller.without_imu.safe_thumb_opposition_controller import SimpleOppositionController

# Global controller instance
controller = None
running = True

class WiggleApproachController(SimpleOppositionController):
    """
    Extended controller that adds wiggle motion during approach.
    
    This controller builds on the safe thumb opposition controller and adds:
    1. Small, randomized finger movements during the THUMB_OPPOSING phase
    2. Coordinated wiggle patterns to enhance object detection
    3. All while maintaining safe thumb positioning
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
        wiggle_amplitude=5.0,
        wiggle_frequency=0.5
    ):
        """
        Initialize the wiggle approach controller.
        
        Args:
            control_rate: Control loop frequency in Hz
            proximity_rate: Proximity sampling rate in Hz (defaults to 2x control_rate)
            use_simulated_motors: Whether to use simulated motors
            motor_interface_kwargs: Additional kwargs for motor interface
            finger_close_angle: Angle for closing fingers (degrees)
            thumb_opposition_angle: Angle for thumb opposition (degrees)
            finger_threshold: Distance threshold for any finger detection (mm)
            thumb_threshold: Distance threshold for thumb detection (mm)
            wiggle_amplitude: Maximum amplitude of wiggle motion (degrees)
            wiggle_frequency: Frequency of wiggle motion (Hz)
        """
        # Initialize base controller
        super().__init__(
            control_rate=control_rate,
            proximity_rate=proximity_rate,
            use_simulated_motors=use_simulated_motors,
            motor_interface_kwargs=motor_interface_kwargs,
            finger_close_angle=finger_close_angle,
            thumb_opposition_angle=thumb_opposition_angle,
            finger_threshold=finger_threshold,
            thumb_threshold=thumb_threshold
        )
        
        # Wiggle parameters
        self.wiggle_amplitude = wiggle_amplitude
        self.wiggle_frequency = wiggle_frequency
        self.wiggle_start_time = time.time()
        self.last_wiggle_time = 0
        self.next_wiggle_time = 0
        
        # Wiggle timing parameters
        self.wiggle_mean_interval = 0.7  # Mean time between wiggles (seconds)
        self.wiggle_std_dev = 0.2        # Standard deviation for timing (seconds)
        self.wiggle_min_interval = 0.2   # Minimum time between wiggles (seconds)
        
        # Motion timing
        self.thumb_move_time = 0.2  # seconds for thumb to move to position
        self.finger_move_time = 0.2  # seconds for fingers to move to position
        self.motion_phase_duration = 0.4  # Optimized for minimum necessary time
        
        # Previous readings for non-100mm debouncing
        self.prev_finger_distance = 100
        self.prev_thumb_distance = 100
        
        logger.info(f"Wiggle approach controller initialized with amplitude={wiggle_amplitude}°, frequency={wiggle_frequency}Hz")
    
    def _move_fingers(self):
        """Execute finger movements based on current state, with wiggle during approach"""
        current_time = time.time()
        
        if self.state == "IDLE":
            # When returning to IDLE, use the safe sequence from parent class
            super()._move_fingers()
                
        elif self.state == "THUMB_OPPOSING":
            # First ensure thumb is in full opposition position using ThumbRotate joint
            # Note: ThumbRotate is the thumb rotator which controls opposition
            max_rotation = 100.0  # Maximum rotation value for Ability Hand
            self.target_positions["ThumbRotate"] = max_rotation
            try:
                # Set the thumb rotator to maximum opposition position
                self.motors.set_position("ThumbRotate", max_rotation)
                self.current_positions["ThumbRotate"] = max_rotation
                
                # Set minimal flexion for the thumb flexor
                self.motors.set_position("Thumb", 5.0)  # Minimal flexion
                self.current_positions["Thumb"] = 5.0
            except Exception as e:
                logger.error(f"Error setting position for thumb: {e}")
            
            # Small delay to let thumb move to opposition first
            time.sleep(0.1)
            
            # Generate random time for next wiggle using Gaussian distribution
            # centered around 0.7 seconds
            if self.last_wiggle_time == 0:
                # First wiggle after entering this state
                next_wiggle_time = current_time
            elif not hasattr(self, 'next_wiggle_time'):
                # Initialize next wiggle time if not set
                random_interval = max(self.wiggle_min_interval, 
                                     random.gauss(self.wiggle_mean_interval, self.wiggle_std_dev))
                self.next_wiggle_time = self.last_wiggle_time + random_interval
                logger.debug(f"Next wiggle in {random_interval:.2f}s (Gaussian: μ={self.wiggle_mean_interval}s, σ={self.wiggle_std_dev}s)")
                
            # Check if it's time for the next wiggle
            if current_time >= getattr(self, 'next_wiggle_time', current_time):
                self.last_wiggle_time = current_time
                
                # Schedule next wiggle with Gaussian distribution
                random_interval = max(self.wiggle_min_interval, 
                                     random.gauss(self.wiggle_mean_interval, self.wiggle_std_dev))
                self.next_wiggle_time = current_time + random_interval
                
                logger.debug(f"Wiggle now, next in {random_interval:.2f}s (Gaussian: μ={self.wiggle_mean_interval}s, σ={self.wiggle_std_dev}s)")
                
                # Get list of non-thumb fingers
                non_thumb_fingers = [f for f in self.motors.fingers if f != "Thumb"]
                
                # Randomly select a subset of fingers to move this time
                # Number of fingers to move is random between 1 and the total number
                num_fingers_to_move = random.randint(1, len(non_thumb_fingers))
                fingers_to_move = random.sample(non_thumb_fingers, num_fingers_to_move)
                
                logger.debug(f"Moving {num_fingers_to_move} fingers: {fingers_to_move}")
                
                # Calculate wiggle angles for selected fingers using Gaussian distribution
                for finger in fingers_to_move:
                    # Use a Gaussian distribution for more natural movement
                    base_position = 0.0  # Base position (open)
                    std_dev = self.wiggle_amplitude / 3.0  # 3-sigma rule
                    
                    # Generate random wiggle with partial correlation between fingers
                    master_random = random.gauss(0, 1)  # Shared randomness component
                    correlation = 0.6  # Correlation factor
                    
                    # Blend master random with finger-specific random
                    finger_random = (correlation * master_random + 
                                    (1-correlation) * random.gauss(0, 1))
                    
                    # Scale to desired amplitude and shift to ensure positive values
                    wiggle_angle = base_position + (std_dev * finger_random)
                    wiggle_angle = max(0, min(wiggle_angle, self.wiggle_amplitude * 2))
                    
                    self.target_positions[finger] = wiggle_angle
                    
                    try:
                        self.motors.set_position(finger, self.target_positions[finger])
                        self.current_positions[finger] = self.target_positions[finger]
                    except Exception as e:
                        logger.error(f"Error setting position for {finger}: {e}")
                            
        elif self.state == "GRIP_CLOSING":
            # Use the safe grip closing from parent class
            super()._move_fingers()
    
    def get_status(self):
        """Get current system status with wiggle information"""
        # Get base status
        status = super().get_status()
        
        # Add wiggle information if in THUMB_OPPOSING state
        if self.state == "THUMB_OPPOSING":
            current_time = time.time()
            status["wiggle"] = {
                "amplitude": self.wiggle_amplitude,
                "mean_interval": self.wiggle_mean_interval,
                "std_dev": self.wiggle_std_dev,
                "elapsed": current_time - self.wiggle_start_time,
                "next_wiggle_in": max(0, self.next_wiggle_time - current_time) if hasattr(self, 'next_wiggle_time') else 0
            }
        
        return status
    
    def _update_state(self, finger_distance, thumb_distance):
        """Update controller state based on sensor readings"""
        previous_state = self.state
        
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
            
        # Update state using parent method, but use super's parent method to avoid calling our modified version
        # that already has the non-100mm debouncing
        SimpleOppositionController._update_state(self, finger_distance, thumb_distance)
        
        # If we just entered THUMB_OPPOSING state, reset wiggle parameters
        if previous_state != "THUMB_OPPOSING" and self.state == "THUMB_OPPOSING":
            self.wiggle_start_time = time.time()
            self.last_wiggle_time = 0
            self.next_wiggle_time = time.time()  # Start wiggling immediately after entering state


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
    parser = argparse.ArgumentParser(description='Wiggle Approach Controller')
    
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
    
    parser.add_argument('--thumb-oppose', type=float, default=100.0,
                      help='Thumb opposition angle in degrees (default: 100.0, max rotation)')
    
    parser.add_argument('--finger-threshold', type=float, default=100.0,
                      help='Finger detection distance threshold in mm (default: 100.0)')
    
    parser.add_argument('--thumb-threshold', type=float, default=50.0,
                      help='Thumb detection distance threshold in mm (default: 50.0)')
    
    parser.add_argument('--wiggle-amplitude', type=float, default=5.0,
                      help='Maximum amplitude of wiggle motion in degrees (default: 5.0)')
    
    parser.add_argument('--wiggle-frequency', type=float, default=0.5,
                      help='Frequency of wiggle motion in Hz (default: 0.5)')
                      
    parser.add_argument('--wiggle-interval', type=float, default=0.7,
                      help='Mean time between wiggles in seconds (default: 0.7)')
                      
    parser.add_argument('--wiggle-stddev', type=float, default=0.2,
                      help='Standard deviation for wiggle timing in seconds (default: 0.2)')
    
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
        controller = WiggleApproachController(
            control_rate=args.rate,
            proximity_rate=args.proximity_rate,
            use_simulated_motors=args.simulate,
            motor_interface_kwargs=motor_kwargs,
            finger_close_angle=args.finger_close,
            thumb_opposition_angle=args.thumb_oppose,
            finger_threshold=args.finger_threshold,
            thumb_threshold=args.thumb_threshold,
            wiggle_amplitude=args.wiggle_amplitude,
            wiggle_frequency=args.wiggle_frequency
        )
        
        # Start controller
        logger.info("Starting controller...")
        controller.start()
        
        # Monitor loop
        print("\nWiggle Approach Controller Active - Press Ctrl+C to stop")
        print("----------------------------------------------------------")
        print(f"This controller positions the thumb at {args.thumb_oppose}° opposition")
        print(f"when any finger detects an object within {args.finger_threshold}mm.")
        print(f"During approach phase, fingers wiggle with amplitude {args.wiggle_amplitude}° at {args.wiggle_frequency}Hz")
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
                print("\nWiggle Approach Controller")
                print("=========================")
                print(f"State: {status['state']} for {status['time_in_state']:.1f}s")
                print(f"Motion Phase: {'ACTIVE' if status['in_motion_phase'] else 'INACTIVE'}")
                
                # Print wiggle info if in approach state
                if status['state'] == "THUMB_OPPOSING" and "wiggle" in status:
                    wiggle = status["wiggle"]
                    print(f"Wiggle: Amplitude={wiggle['amplitude']}°, Frequency={wiggle['frequency']}Hz, Time={wiggle['elapsed']:.1f}s")
                
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