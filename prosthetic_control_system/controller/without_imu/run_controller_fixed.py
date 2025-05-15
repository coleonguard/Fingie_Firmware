#!/usr/bin/env python3
"""
Fixed launch script for proximity-only control (no IMU).

This version adds additional error handling to avoid 'string indices must be integers' errors.
"""

import os
import sys
import time
import signal
import argparse
import logging
import traceback

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("ProximityControl")

# Add all necessary directories to path
# First get the repository root directory (Fingie_Firmware)
repo_root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../"))

# Ensure repo root is in the path
if repo_root not in sys.path:
    sys.path.insert(0, repo_root)

# Global controller instance
controller = None

def signal_handler(sig, frame):
    """Handle keyboard interrupt gracefully"""
    if controller:
        logger.info("Stopping controller due to signal...")
        controller.stop()
    sys.exit(0)

def _display_minimal_status(status, args):
    """Display minimal status information with error handling"""
    try:
        # Check for required keys with error handling
        if 'hand' not in status:
            print("\n⚠️ Limited status information available")
            return
            
        # Get hand state with error checking
        if 'hand_state' in status['hand']:
            hand_state = status['hand']['hand_state']
            # Print hand state with emoji
            emoji_map = {
                'IDLE': '🚫',
                'ACTIVE': '✅',
                'HOLDING': '👊',
                'RELEASING': '👐',
                'ERROR': '❌'
            }
            emoji = emoji_map.get(hand_state, '⚠️')
            print(f"\n{emoji} Hand state: {hand_state}")
        else:
            print("\n⚠️ Hand state information unavailable")
            
        # Print finger information with error checking
        if 'finger_states' in status['hand']:
            finger_states = status['hand']['finger_states']
            
            # Print minimal finger information
            active_fingers = []
            for finger, state in finger_states.items():
                if state != 'IDLE':
                    active_fingers.append(f"{finger}: {state}")
            
            if active_fingers:
                print(f"Active fingers: {', '.join(active_fingers)}")
            else:
                print("All fingers idle")
        else:
            print("Finger state information unavailable")
            
    except Exception as e:
        print(f"\n⚠️ Error displaying status: {e}")

def main():
    """Main entry point with enhanced error handling"""
    global controller
    
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Proximity-Only Control System (Fixed)')
    
    parser.add_argument('--simulate', action='store_true',
                        help='Use simulated hardware')
    
    parser.add_argument('--rate', type=int, default=20,
                        help='Control loop rate in Hz (default: 20)')
    
    parser.add_argument('--port', type=str, default=None,
                        help='Serial port for Ability Hand (default: auto-detect)')
    
    parser.add_argument('--simplified', action='store_true',
                        help='Use simplified controller instead of threaded implementation')
    
    args = parser.parse_args()
    
    # Set up signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # Import controllers with error handling
        try:
            # First try to import the controllers
            logger.info("Importing controller modules...")
            
            if args.simplified:
                from prosthetic_control_system.controller.without_imu.simplified_controller import SimplifiedController as ControllerClass
                controller_name = "SimplifiedController"
            else:
                # The error is likely occurring here when importing the proximity controller
                try:
                    from prosthetic_control_system.controller.without_imu.proximity_controller import ProximityController as ControllerClass
                    controller_name = "ProximityController"
                except Exception as e:
                    logger.error(f"Error importing ProximityController: {e}")
                    logger.info("Falling back to SimplifiedController due to import error")
                    from prosthetic_control_system.controller.without_imu.simplified_controller import SimplifiedController as ControllerClass
                    controller_name = "SimplifiedController (fallback)"
                    args.simplified = True
            
            from prosthetic_control_system.controller.without_imu.config import DEFAULT_CONFIG
        except Exception as e:
            logger.error(f"Critical error importing controller modules: {e}")
            logger.error(traceback.format_exc())
            sys.exit(1)
        
        # Create motor interface kwargs
        motor_kwargs = {}
        if args.port:
            motor_kwargs['port'] = args.port
        
        # Create controller with error handling
        logger.info(f"Creating {controller_name} (simulated: {args.simulate}, rate: {args.rate}Hz)")
        
        try:
            # Try to create the controller
            controller = ControllerClass(
                control_rate=args.rate,
                enable_logging=False,  # Disable logging to reduce complexity
                use_simulated_motors=args.simulate,
                motor_interface_kwargs=motor_kwargs,
                approach_threshold=DEFAULT_CONFIG["approach_threshold"],
                contact_threshold=DEFAULT_CONFIG["contact_threshold"],
                verbose_logging=False
            )
            
            # Start controller
            logger.info("Starting controller...")
            controller.start()
            
            # Get initial status with error handling
            try:
                status = controller.get_system_status()
                if 'hand' in status and 'hand_state' in status['hand']:
                    logger.info(f"Controller running. Hand state: {status['hand']['hand_state']}")
                else:
                    logger.info("Controller running (status information incomplete)")
            except Exception as e:
                logger.error(f"Error getting initial status: {e}")
                logger.info("Controller started but status info unavailable")
            
            # Monitor loop with robust error handling
            print("\nProximity Controller Active - Press Ctrl+C to stop")
            print("---------------------------------------------------")
            
            while True:
                try:
                    # Get current status with error handling
                    try:
                        status = controller.get_system_status()
                    except Exception as e:
                        logger.error(f"Error getting status: {e}")
                        status = {"hand": {"hand_state": "ERROR", "finger_states": {}}}
                    
                    # Check for faults if available in status
                    if 'faults' in status:
                        faults = status['faults']
                        if any(faults.values()):
                            logger.warning(f"FAULTS DETECTED: {faults}")
                    
                    # Clear screen for better display
                    os.system('clear' if os.name == 'posix' else 'cls')
                    
                    # Use minimal display mode for maximum compatibility
                    _display_minimal_status(status, args)
                    
                    # Show fault information if available
                    if 'faults' in status and any(status['faults'].values()):
                        print("\n⚠️  FAULTS DETECTED:")
                        for fault, active in status['faults'].items():
                            if active:
                                print(f" - {fault.upper()}")
                    
                    print("\nControls:")
                    print("Press 'r' + Enter to reset hand to safe position")
                    print("Press Ctrl+C to stop")
                    
                    # Check for keyboard input (non-blocking) with error handling
                    try:
                        import select
                        import termios
                        import tty
                        
                        # Set stdin to non-blocking mode
                        old_settings = termios.tcgetattr(sys.stdin)
                        try:
                            tty.setcbreak(sys.stdin.fileno())
                            if select.select([sys.stdin], [], [], 0)[0]:
                                key = sys.stdin.read(1)
                                # 'r' key for reset
                                if key == 'r':
                                    print("\nSAFETY RESET REQUESTED - Opening hand...")
                                    try:
                                        controller.safety_reset()
                                    except Exception as e:
                                        logger.error(f"Error during safety reset: {e}")
                        except Exception as e:
                            # Ignore input errors
                            pass
                        finally:
                            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                    except Exception as e:
                        # Terminal may not support these operations, that's okay
                        pass
                    
                    # Sleep with error handling
                    time.sleep(0.5)
                    
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    logger.error(f"Unhandled error in main loop: {e}")
                    time.sleep(1)  # Wait before trying again
        
        except Exception as e:
            logger.error(f"Error creating or starting controller: {e}")
            logger.error(traceback.format_exc())
            sys.exit(1)
    
    except Exception as e:
        logger.error(f"Critical error in main: {e}")
        logger.error(traceback.format_exc())
    
    finally:
        # Ensure controller is stopped safely
        if controller:
            try:
                logger.info("Stopping controller...")
                controller.stop()
            except Exception as e:
                logger.error(f"Error stopping controller: {e}")
        
        logger.info("Exiting")

if __name__ == "__main__":
    main()