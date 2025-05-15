#!/usr/bin/env python3
"""
Enhanced Controller (with velocity control option) for Prosthetic Hand.

This script provides a robust implementation with options for:
- Position control (default, most stable)
- Velocity control (potentially smoother but less stable)
- Simplified controller mode
- Simulation mode

All with enhanced error handling to prevent crashes.
"""

import os
import sys
import time
import signal
import argparse
import logging
import traceback
from enum import Enum

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("EnhancedController")

# Add all necessary directories to path
# First get the repository root directory (Fingie_Firmware)
repo_root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../"))

# Ensure repo root is in the path
if repo_root not in sys.path:
    sys.path.insert(0, repo_root)

# Define control modes
class ControlMode(Enum):
    POSITION = "position"
    VELOCITY = "velocity"
    
    def __str__(self):
        return self.value

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
            
            # Show control mode in the title
            control_mode = "VELOCITY" if args.velocity else "POSITION"
            print(f"\n{emoji} Hand state: {hand_state} ({control_mode} MODE)")
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
        
        # Print velocity info if in velocity mode
        if args.velocity and 'velocities' in status:
            print("\nFinger Velocities:")
            for finger, vel in status['velocities'].items():
                print(f"{finger}: {vel:+.1f} deg/s")
            
    except Exception as e:
        print(f"\n⚠️ Error displaying status: {e}")

def _display_standard_status(status, args):
    """Display standard status information with error handling"""
    try:
        # First show minimal status
        _display_minimal_status(status, args)
        
        # Show proximity sensors if available
        if 'proximity' in status:
            print("\nProximity Sensors:")
            print("-" * 50)
            for sensor, value in status['proximity'].items():
                print(f"{sensor}: {value:.1f}mm")
        
        # Show positions/velocities based on mode
        if args.velocity:
            # In velocity mode, show positions and velocities
            if 'velocities' in status and 'estimated_positions' in status:
                print("\nFinger Status:")
                print("-" * 50)
                for finger in status['velocities']:
                    vel = status['velocities'].get(finger, 0.0)
                    est_pos = status['estimated_positions'].get(finger, 0.0)
                    act_pos = status.get('actual_positions', {}).get(finger, "N/A")
                    print(f"{finger}: {vel:+.1f} deg/s | Est. Pos: {est_pos:.1f}° | Act. Pos: {act_pos}")
        else:
            # In position mode show positions
            if 'positions' in status:
                print("\nFinger Positions:")
                print("-" * 50)
                for finger, pos in status['positions'].items():
                    print(f"{finger}: {pos:.1f}°")
        
        # Show cycle time info if available
        if 'cycle_time' in status:
            print(f"\nSystem: {status['cycle_time']['last']*1000:.1f}ms/cycle (avg: {status['cycle_time']['avg']*1000:.1f}ms)")
        
    except Exception as e:
        print(f"\n⚠️ Error displaying standard status: {e}")

def create_controller(args):
    """Create and return the appropriate controller based on arguments"""
    try:
        # Import necessary modules based on control mode
        if args.velocity:
            # Velocity controller import
            try:
                logger.info("Importing velocity controller module...")
                from prosthetic_control_system.controller.without_imu.velocity_controller.velocity_controller import VelocityController
                from prosthetic_control_system.controller.without_imu.velocity_controller.config import DEFAULT_CONFIG
                
                # Create controller parameters
                controller_params = {
                    'control_rate': args.rate,
                    'enable_logging': not args.no_logging,
                    'log_dir': args.log_dir,
                    'use_simulated_motors': args.simulate,
                    'motor_interface_kwargs': {'port': args.port} if args.port else {},
                    'approach_threshold': args.approach,
                    'contact_threshold': args.contact,
                    'velocity_scale': args.velocity_scale,
                    'velocity_damping': args.velocity_damping,
                    'verbose_logging': args.debug
                }
                
                # Create velocity controller
                logger.info(f"Creating velocity controller with params: rate={args.rate}Hz, "
                           f"damping={args.velocity_damping}, scale={args.velocity_scale}")
                
                return VelocityController(**controller_params)
                
            except Exception as e:
                logger.error(f"Error creating velocity controller: {e}")
                logger.error(traceback.format_exc())
                raise
                
        elif args.simplified:
            # Simplified controller import
            try:
                logger.info("Importing simplified controller module...")
                from prosthetic_control_system.controller.without_imu.simplified_controller import SimplifiedController
                from prosthetic_control_system.controller.without_imu.config import DEFAULT_CONFIG
                
                # Create controller parameters
                controller_params = {
                    'control_rate': args.rate,
                    'enable_logging': not args.no_logging,
                    'log_dir': args.log_dir,
                    'use_simulated_motors': args.simulate,
                    'motor_interface_kwargs': {'port': args.port} if args.port else {},
                    'approach_threshold': args.approach,
                    'contact_threshold': args.contact,
                    'verbose_logging': args.debug
                }
                
                # Create simplified controller
                logger.info(f"Creating simplified controller with params: rate={args.rate}Hz")
                
                return SimplifiedController(**controller_params)
                
            except Exception as e:
                logger.error(f"Error creating simplified controller: {e}")
                logger.error(traceback.format_exc())
                raise
                
        else:
            # Standard proximity controller (position-based)
            try:
                logger.info("Importing proximity controller module...")
                from prosthetic_control_system.controller.without_imu.proximity_controller import ProximityController
                from prosthetic_control_system.controller.without_imu.config import DEFAULT_CONFIG
                
                # Create controller parameters
                controller_params = {
                    'control_rate': args.rate,
                    'enable_logging': not args.no_logging,
                    'log_dir': args.log_dir,
                    'use_simulated_motors': args.simulate,
                    'motor_interface_kwargs': {'port': args.port} if args.port else {},
                    'approach_threshold': args.approach,
                    'contact_threshold': args.contact,
                    'verbose_logging': args.debug,
                    # Set low sampling rate to avoid I2C issues
                    'proximity_sampling_rate': 5  # Hard-coded to 5Hz for reliability
                }
                
                # Create proximity controller
                logger.info(f"Creating proximity controller with params: rate={args.rate}Hz")
                
                return ProximityController(**controller_params)
                
            except Exception as e:
                logger.error(f"Error creating proximity controller: {e}")
                logger.error(traceback.format_exc())
                # Fall back to simplified controller
                logger.info("Falling back to simplified controller...")
                return create_controller({**args, 'simplified': True})
                
    except Exception as e:
        logger.error(f"Unhandled error creating controller: {e}")
        logger.error(traceback.format_exc())
        raise

def main():
    """Main entry point with enhanced error handling"""
    global controller
    
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Enhanced Controller for Prosthetic Hand')
    
    # Core configuration
    parser.add_argument('--simulate', action='store_true',
                        help='Use simulated hardware')
    
    parser.add_argument('--rate', type=int, default=20,
                        help='Control loop rate in Hz (default: 20)')
    
    parser.add_argument('--port', type=str, default=None,
                        help='Serial port for Ability Hand (default: auto-detect)')
    
    # Controller type selection
    parser.add_argument('--simplified', action='store_true',
                        help='Use simplified controller (more reliable)')
    
    parser.add_argument('--velocity', action='store_true',
                        help='Use velocity-based control (experimental, may be smoother)')
    
    # Velocity controller parameters
    parser.add_argument('--velocity-scale', type=float, default=25.0,
                        help='Velocity scaling factor (default: 25.0)')
    
    parser.add_argument('--velocity-damping', type=float, default=0.95,
                        help='Velocity damping factor (default: 0.95, higher = smoother)')
    
    # Sensor thresholds
    parser.add_argument('--approach', type=int, default=40,
                        help='Approach threshold in mm (default: 40)')
    
    parser.add_argument('--contact', type=int, default=5,
                        help='Contact threshold in mm (default: 5)')
    
    # Display options
    parser.add_argument('--minimal', action='store_true',
                        help='Use minimal status display')
    
    # Logging options
    parser.add_argument('--debug', action='store_true',
                        help='Enable verbose debugging output')
    
    parser.add_argument('--no-logging', action='store_true',
                        help='Disable data logging')
    
    parser.add_argument('--log-dir', type=str, default=None,
                        help='Directory for log files (default: ~/prosthetic_logs)')
    
    args = parser.parse_args()
    
    # Set up signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # Create appropriate controller
        controller = create_controller(args)
        
        # Start controller with error handling
        try:
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
            print("\nEnhanced Controller Active - Press Ctrl+C to stop")
            if args.velocity:
                print("VELOCITY CONTROL MODE (EXPERIMENTAL)")
            else:
                print("POSITION CONTROL MODE (STANDARD)")
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
                    
                    # Choose display style based on args
                    if args.minimal:
                        _display_minimal_status(status, args)
                    else:
                        _display_standard_status(status, args)
                    
                    # Show fault information if available
                    if 'faults' in status and any(status['faults'].values()):
                        print("\n⚠️  FAULTS DETECTED:")
                        for fault, active in status['faults'].items():
                            if active:
                                print(f" - {fault.upper()}")
                    
                    print("\nControls:")
                    print("Press 'r' + Enter to reset hand to safe position")
                    if args.velocity:
                        print("Press 'f' + Enter to freeze all velocities")
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
                                # 'f' key to freeze velocities in velocity mode
                                elif key == 'f' and args.velocity:
                                    print("\nFREEZE REQUESTED - Stopping all finger velocities...")
                                    try:
                                        if hasattr(controller, 'finger_velocities'):
                                            for finger in controller.finger_velocities:
                                                controller.finger_velocities[finger] = 0.0
                                            print("All velocities set to zero.")
                                    except Exception as e:
                                        logger.error(f"Error freezing velocities: {e}")
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
            logger.error(f"Error starting controller: {e}")
            logger.error(traceback.format_exc())
            sys.exit(1)
    
    except Exception as e:
        logger.error(f"Critical error creating controller: {e}")
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