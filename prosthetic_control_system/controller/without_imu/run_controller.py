#!/usr/bin/env python3
"""
Launch script for proximity-only control (no IMU).

This script runs the controller with IMU disabled, suitable for
testing with only proximity sensors and the Ability Hand.
"""

import os
import sys
import time
import signal
import argparse
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("ProximityControl")

# Add parent directory to path if needed
parent_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

# Import from this package
from prosthetic_control_system.controller.without_imu.proximity_controller import ProximityController

# Global controller instance
controller = None

def signal_handler(sig, frame):
    """Handle keyboard interrupt gracefully"""
    if controller:
        logger.info("Stopping controller due to signal...")
        controller.stop()
    sys.exit(0)

def main():
    """Main entry point"""
    global controller
    
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Proximity-Only Control System')
    
    parser.add_argument('--simulate', action='store_true',
                        help='Use simulated hardware')
    
    parser.add_argument('--rate', type=int, default=20,
                        help='Control loop rate in Hz (default: 20)')
    
    parser.add_argument('--no-logging', action='store_true',
                        help='Disable data logging')
    
    parser.add_argument('--log-dir', type=str, default=None,
                        help='Directory for log files (default: ~/prosthetic_logs)')
    
    parser.add_argument('--port', type=str, default=None,
                        help='Serial port for Ability Hand (default: auto-detect)')
    
    parser.add_argument('--approach', type=int, default=40,
                      help='Approach threshold in mm (default: 40)')
    
    parser.add_argument('--contact', type=int, default=5,
                      help='Contact threshold in mm (default: 5)')
    
    args = parser.parse_args()
    
    # Set up signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # Create motor interface kwargs if needed
        motor_kwargs = {}
        if args.port:
            motor_kwargs['port'] = args.port
        
        # Create controller with IMU disabled
        logger.info(f"Creating controller (simulated: {args.simulate}, rate: {args.rate}Hz)")
        controller = ProximityController(
            control_rate=args.rate,
            enable_logging=not args.no_logging,
            log_dir=args.log_dir,
            use_simulated_motors=args.simulate,
            motor_interface_kwargs=motor_kwargs,
            approach_threshold=args.approach,
            contact_threshold=args.contact
        )
        
        # Start controller
        logger.info("Starting controller...")
        controller.start()
        
        # Print initial status
        status = controller.get_system_status()
        logger.info(f"Controller running. Hand state: {status['hand']['hand_state']}")
        
        # Monitor loop
        print("\nProximity Controller Active - Press Ctrl+C to stop")
        print("---------------------------------------------------")
        
        while True:
            try:
                # Get current status
                status = controller.get_system_status()
                
                # Check for faults
                faults = status['faults']
                if any(faults.values()):
                    logger.warning(f"FAULTS DETECTED: {faults}")
                
                # Clear screen for better display
                os.system('clear' if os.name == 'posix' else 'cls')
                
                # Print nice status display
                print("\nProximity Controller Status")
                print("==========================")
                print(f"Hand State: {status['hand']['hand_state']}")
                
                # Finger states and positions
                print("\nFinger Status:")
                print("-------------")
                for finger, state in status['hand']['finger_states'].items():
                    position = status['positions'].get(finger, 0)
                    current = status['currents'].get(finger, 0)
                    print(f"{finger:10s}: {state:13s} | Pos: {position:5.1f}° | Current: {current:.3f}A")
                
                # Proximity values
                print("\nProximity Sensors:")
                print("----------------")
                for sensor, value in status['proximity'].items():
                    sensor_status = "OK"
                    if value is None:
                        value = "N/A"
                        sensor_status = "FAIL"
                    else:
                        value = f"{value:.1f}mm"
                    print(f"{sensor:10s}: {value:8s} | Status: {sensor_status}")
                
                # System stats
                print("\nSystem Performance:")
                print("-----------------")
                cycle_time = status['cycle_time']
                print(f"Loop Time: {cycle_time['last']*1000:.1f}ms | Avg: {cycle_time['avg']*1000:.1f}ms | Max: {cycle_time['max']*1000:.1f}ms")
                
                if any(faults.values()):
                    print("\n⚠️  FAULTS DETECTED:")
                    for fault, active in faults.items():
                        if active:
                            print(f" - {fault.upper()}")
                
                print("\nControls:")
                print("Press 'r' + Enter to reset hand to safe position")
                print("Press Ctrl+C to stop")
                
                # Check for keyboard input (non-blocking)
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
                            controller.safety_reset()
                except Exception as e:
                    # Ignore input errors
                    pass
                finally:
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                
                # Sleep for a while
                time.sleep(0.5)
                
            except KeyboardInterrupt:
                break
    
    except Exception as e:
        logger.error(f"Error running controller: {e}")
        if controller:
            controller.stop()
    
    finally:
        # Make sure controller is stopped
        if controller:
            logger.info("Stopping controller...")
            controller.stop()
        
        logger.info("Exiting")

if __name__ == "__main__":
    main()