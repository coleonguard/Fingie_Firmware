#!/usr/bin/env python3
"""
Simple test script for the Ability Hand interface.

This script demonstrates how to use the AbilityHandInterface to control
the PSYONIC Ability Hand prosthetic. It will run a simple test sequence
that exercises all fingers.
"""

import sys
import os
import time
import logging
import argparse

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("AbilityHandTest")

# Ensure the module's directory is in the path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

# Import the hand interface
from prosthetic_control_system.hand.ability_hand_interface import AbilityHandInterface

def test_position_control(hand, duration=10):
    """Test position control for all fingers"""
    logger.info("Testing position control...")
    fingers = ["Thumb", "Index", "Middle", "Ring", "Pinky", "ThumbRotate"]
    
    # First close all fingers to 50%
    logger.info("Closing all fingers to 50%...")
    for finger in fingers:
        hand.set_position(finger, 50.0)
    
    time.sleep(2.0)
    
    # Then open and close each finger in sequence
    for finger in fingers:
        logger.info(f"Testing {finger}...")
        
        # Open the finger
        logger.info(f"  Opening {finger}...")
        hand.set_position(finger, 0.0)
        time.sleep(1.0)
        
        # Close the finger
        logger.info(f"  Closing {finger}...")
        hand.set_position(finger, 80.0)
        time.sleep(1.0)
        
        # Return to 50%
        logger.info(f"  Setting {finger} to 50%...")
        hand.set_position(finger, 50.0)
        time.sleep(0.5)
    
    # Open all fingers
    logger.info("Opening all fingers...")
    for finger in fingers:
        hand.set_position(finger, 0.0)
    
    time.sleep(2.0)

def test_torque_control(hand, duration=10):
    """Test torque control for all fingers"""
    logger.info("Testing torque control...")
    fingers = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
    
    # First open all fingers
    logger.info("Opening all fingers...")
    for finger in hand.fingers + ["ThumbRotate"]:
        hand.set_position(finger, 0.0)
    
    time.sleep(2.0)
    
    # Set thumb rotation
    logger.info("Setting thumb rotation to 50°...")
    hand.set_position("ThumbRotate", 50.0)
    time.sleep(1.0)
    
    # Test torque control on each finger
    for finger in fingers:
        logger.info(f"Testing torque on {finger}...")
        
        # Set small torque
        torque = 0.2
        logger.info(f"  Setting {finger} torque to {torque:.1f}A...")
        hand.set_torque(finger, torque)
        time.sleep(2.0)
        
        # Set medium torque
        torque = 0.4
        logger.info(f"  Setting {finger} torque to {torque:.1f}A...")
        hand.set_torque(finger, torque)
        time.sleep(2.0)
        
        # Reset to position control (0°)
        logger.info(f"  Resetting {finger} to position control...")
        hand.set_position(finger, 0.0)
        time.sleep(1.0)
    
    # Reset thumb rotation
    logger.info("Resetting thumb rotation to 0°...")
    hand.set_position("ThumbRotate", 0.0)
    time.sleep(1.0)

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description='Ability Hand Test')
    
    parser.add_argument('--port', type=str, default=None,
                    help='Serial port of Ability Hand (default: auto-detect)')
    
    parser.add_argument('--baud', type=int, default=460800,
                    help='Baud rate (default: 460800)')
    
    parser.add_argument('--duration', type=int, default=10,
                      help='Test duration in seconds (default: 10)')
    
    parser.add_argument('--test', type=str, choices=['all', 'position', 'torque'],
                      default='all', help='Which test to run (default: all)')
    
    args = parser.parse_args()
    
    try:
        # Create hand interface
        logger.info(f"Creating Ability Hand interface (port: {args.port or 'auto-detect'}, baud: {args.baud})")
        hand = AbilityHandInterface(
            port=args.port,
            baud_rate=args.baud,
            control_rate=50,
            reply_mode=2  # Position, Current, Velocity
        )
        
        # Start the interface
        logger.info("Starting interface...")
        hand.start()
        
        # Wait a moment for initialization
        time.sleep(1.0)
        
        # Show initial status
        logger.info("Initial status:")
        for finger in hand.fingers + ["ThumbRotate"]:
            status = hand.get_finger_status(finger)
            logger.info(f"  {finger}: Pos={status['position']:.1f}°, Curr={status['current']:.3f}A, Vel={status['velocity']:.1f}°/s")
        
        # Run the requested tests
        if args.test in ['all', 'position']:
            test_position_control(hand, args.duration)
        
        if args.test in ['all', 'torque']:
            test_torque_control(hand, args.duration)
        
        # Final status
        logger.info("Final status:")
        for finger in hand.fingers + ["ThumbRotate"]:
            status = hand.get_finger_status(finger)
            logger.info(f"  {finger}: Pos={status['position']:.1f}°, Curr={status['current']:.3f}A, Vel={status['velocity']:.1f}°/s")
        
    except KeyboardInterrupt:
        logger.info("Test interrupted by user")
    except Exception as e:
        logger.error(f"Error: {e}")
    finally:
        # Make sure to stop the interface
        if 'hand' in locals() and hand:
            logger.info("Stopping interface...")
            hand.stop()
        
        logger.info("Test complete")

if __name__ == "__main__":
    main()