#!/usr/bin/env python3
"""
Launch script for velocity-based proximity control.

This script runs the velocity-based controller to explore using
velocity control instead of position control for smoother finger motions.
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
logger = logging.getLogger("VelocityControl")

# Add all necessary directories to path
# First get the repository root directory (Fingie_Firmware)
repo_root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../../"))

# Ensure repo root is in the path
if repo_root not in sys.path:
    sys.path.insert(0, repo_root)

# Import from this package
from prosthetic_control_system.controller.without_imu.velocity_controller.velocity_controller import VelocityController
from prosthetic_control_system.controller.without_imu.velocity_controller.config import DEFAULT_CONFIG

# Global controller instance
controller = None

def signal_handler(sig, frame):
    """Handle keyboard interrupt gracefully"""
    if controller:
        logger.info("Stopping controller due to signal...")
        controller.stop()
    sys.exit(0)

def _display_minimal_status(status, args):
    """Display minimal status information"""
    # Get hand state and finger states
    hand_state = status['hand']['hand_state']
    finger_states = status['hand']['finger_states']
    
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
    
    # Print minimal finger information
    active_fingers = []
    for finger, state in finger_states.items():
        if state != 'IDLE':
            active_fingers.append(f"{finger}: {state}")
    
    if active_fingers:
        print(f"Active fingers: {', '.join(active_fingers)}")
    else:
        print("All fingers idle")
    
    # Add velocity information
    print("\nVelocities:")
    for finger, vel in status.get('velocities', {}).items():
        print(f"{finger}: {vel:.1f} deg/s")

def _display_basic_status(status, args, controller=None):
    """Display standard status with velocity information"""
    # Get status data
    hand_state = status['hand']['hand_state']
    finger_states = status['hand']['finger_states']
    proximity = status.get('proximity', {})
    velocities = status.get('velocities', {})
    estimated_pos = status.get('estimated_positions', {})
    actual_pos = status.get('actual_positions', {})
    
    # Header with hand state
    emoji_map = {
        'IDLE': '🚫',
        'ACTIVE': '✅',
        'HOLDING': '👊',
        'RELEASING': '👐',
        'ERROR': '❌'
    }
    emoji = emoji_map.get(hand_state, '⚠️')
    
    print(f"\n{emoji} Hand State: {hand_state} (VELOCITY MODE)\n")
    
    # Print finger information in a table format
    print("Finger Status:")
    print("-" * 100)
    print(f"{'Finger':<10}{'State':<15}{'Distance (mm)':<15}{'Velocity (°/s)':<15}{'Est. Pos (°)':<15}{'Actual Pos (°)':<15}")
    print("-" * 100)
    
    # Order fingers for consistent display
    finger_order = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
    
    # Try to access finger status info
    finger_status = {}
    if controller and hasattr(controller, 'proximity') and hasattr(controller.proximity, 'status'):
        # Get sensor status if available
        for sensor, status in controller.proximity.status.items():
            for finger in finger_order:
                if sensor.startswith(finger[0]) and sensor.endswith('1'):  # MCP sensors
                    finger_status[finger] = status
    
    # Try to access finger substitution info
    finger_fallbacks = {}
    if controller and hasattr(controller, 'finger_substitutions'):
        for finger, sub_from in controller.finger_substitutions.items():
            if sub_from is not None:
                finger_fallbacks[finger] = f"From {sub_from}"
    
    for finger in finger_order:
        if finger in finger_states:
            state = finger_states[finger]
            
            # Find the corresponding distance for this finger
            filtered_distance = "N/A"
            sensor_name = None
            
            # Get MCP sensor name for this finger
            for sensor in proximity.keys():
                if sensor.startswith(finger[0]) and sensor.endswith('1'):  # MCP sensors
                    sensor_name = sensor
                    break
            
            # Get proximity value if available
            if sensor_name and sensor_name in proximity:
                value = proximity[sensor_name]
                filtered_distance = f"{value:.1f}" if value is not None else "N/A"
            
            # Get velocity, estimated and actual positions
            velocity = velocities.get(finger, 0.0)
            est_position = estimated_pos.get(finger, 0.0)
            act_position = actual_pos.get(finger, 0.0)
            
            # Add emoji based on state
            state_emoji = {
                'IDLE': '⚪',
                'APPROACH': '🟡',
                'PROPORTIONAL': '🟠',
                'CONTACT': '🔴'
            }.get(state, '⚪')
            
            print(f"{finger:<10}{state_emoji} {state:<13}{filtered_distance:<15}{velocity:+.1f}{' '*10}{est_position:.1f}{' '*10}{act_position:.1f}")
    
    print("-" * 100)
    
    # Display cycle time information
    cycle_time = status['cycle_time']
    print(f"\nSystem: {cycle_time['last']*1000:.1f}ms/cycle (avg: {cycle_time['avg']*1000:.1f}ms)")
    
    # Display calibration information if available
    if 'calibration' in status:
        calib = status['calibration']
        if calib.get('in_progress', False):
            print(f"\n⚙️  Position recalibration in progress: {calib.get('current_finger', 'unknown')}")
        next_calib = calib.get('interval', 10.0) - calib.get('time_since_last', 0.0)
        if next_calib > 0:
            print(f"Next position recalibration in {next_calib:.1f}s")
    
    # Display special modes if active
    if args.calibrate:
        print("\n⚙️  CALIBRATION MODE ACTIVE - Motor movements disabled")
    
    if args.analyze_sensors and 'debug' in status and 'sensor_analysis' in status['debug']:
        analysis_data = status['debug']['sensor_analysis']
        duration = analysis_data.get('duration', 0)
        active = analysis_data.get('active', False)
        if active:
            print(f"\n📊 Sensor analysis running: {duration:.1f}s elapsed")

def _display_detailed_status(status, args, controller=None):
    """Display detailed status with debugging information"""
    # First show the basic status
    _display_basic_status(status, args, controller)
    
    # Get debug info if available
    if 'debug' not in status:
        print("\nDebug information not available.")
        return
    
    debug = status['debug']
    
    # Show velocity control parameters if available
    if 'velocity_control' in debug:
        vel_control = debug['velocity_control']
        print("\nVelocity Control Parameters:")
        print("-" * 52)
        print(f"Velocity Damping:      {vel_control.get('damping', 'N/A'):.2f}")
        print(f"Max Closing Velocity:  {vel_control.get('max_closing_velocity', 'N/A'):.1f} deg/s")
        print(f"Max Opening Velocity:  {vel_control.get('max_opening_velocity', 'N/A'):.1f} deg/s")
        print(f"Idle Return Factor:    {vel_control.get('idle_return_factor', 'N/A'):.2f}")
    
    # Show position estimation errors
    print("\nPosition Estimation Errors:")
    print("-" * 52)
    if 'estimated_positions' in status and 'actual_positions' in status:
        est_pos = status['estimated_positions']
        act_pos = status['actual_positions']
        for finger in est_pos:
            if finger in act_pos:
                error = est_pos[finger] - act_pos[finger]
                print(f"{finger:<10}: {error:+.1f}° (Est: {est_pos[finger]:.1f}°, Act: {act_pos[finger]:.1f}°)")
    
    # Show startup protection status if applicable
    if 'startup' in debug:
        startup = debug['startup']
        if startup.get('time_since_startup', 0) < 5.0:  # Only show during first 5 seconds
            print("\nStartup Protection:")
            active = startup.get('startup_protection_active', False)
            emoji = '🔒' if active else '🔓'
            print(f"{emoji} {'Active' if active else 'Inactive'} - {startup.get('time_since_startup', 0):.1f}s since startup")
    
    # Show hysteresis information
    print("\nHysteresis Status:")
    print("-" * 52)
    
    # Default values in case data is missing
    awaiting_reset = {}
    reset_threshold = 40
    
    if 'hysteresis' in debug:
        awaiting_reset = debug['hysteresis'].get('awaiting_reset', {})
        reset_threshold = debug['hysteresis'].get('reset_threshold', 40)
    
    if awaiting_reset:
        for finger, waiting in awaiting_reset.items():
            status_text = f"🔒 Waiting for reset ({reset_threshold}mm)" if waiting else "🔓 Ready for new triggers"
            print(f"{finger:<10}: {status_text}")
    else:
        print("No hysteresis information available")

def main():
    """Main entry point"""
    global controller
    
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Velocity-Based Proximity Control System')
    
    parser.add_argument('--simulate', action='store_true',
                        help='Use simulated hardware')
    
    parser.add_argument('--rate', type=int, default=DEFAULT_CONFIG["control_rate"],
                        help=f'Control loop rate in Hz (default: {DEFAULT_CONFIG["control_rate"]})')
    
    parser.add_argument('--no-logging', action='store_true',
                        help='Disable data logging')
    
    parser.add_argument('--log-dir', type=str, default=None,
                        help='Directory for log files (default: ~/prosthetic_logs)')
    
    parser.add_argument('--port', type=str, default=None,
                        help='Serial port for Ability Hand (default: auto-detect)')
    
    parser.add_argument('--approach', type=int, default=DEFAULT_CONFIG["approach_threshold"],
                        help=f'Approach threshold in mm (default: {DEFAULT_CONFIG["approach_threshold"]})')
    
    parser.add_argument('--contact', type=int, default=DEFAULT_CONFIG["contact_threshold"],
                        help=f'Contact threshold in mm (default: {DEFAULT_CONFIG["contact_threshold"]})')
    
    parser.add_argument('--velocity-scale', type=float, default=DEFAULT_CONFIG["velocity_scale"],
                        help=f'Velocity scaling factor (default: {DEFAULT_CONFIG["velocity_scale"]})')
    
    parser.add_argument('--velocity-damping', type=float, default=DEFAULT_CONFIG["velocity_damping"],
                        help=f'Velocity damping factor (default: {DEFAULT_CONFIG["velocity_damping"]})')
    
    parser.add_argument('--recalibration-interval', type=float, 
                        default=DEFAULT_CONFIG["position_recalibration_interval"],
                        help=f'Position recalibration interval in seconds (default: {DEFAULT_CONFIG["position_recalibration_interval"]})')
    
    parser.add_argument('--calibrate', action='store_true',
                        help='Run in calibration mode')
    
    parser.add_argument('--analyze-sensors', action='store_true',
                        help='Analyze MCP vs PIP sensor behavior')
    
    parser.add_argument('--visualization', choices=['basic', 'detailed', 'minimal'],
                        default='basic', help='Display style for visualization')
    
    parser.add_argument('--debug', action='store_true',
                        help='Enable verbose debugging output')
    
    args = parser.parse_args()
    
    # Set up signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # Create motor interface kwargs if needed
        motor_kwargs = {}
        if args.port:
            motor_kwargs['port'] = args.port
        
        # Create velocity controller
        logger.info(f"Creating velocity controller (simulated: {args.simulate}, rate: {args.rate}Hz)")
        
        controller = VelocityController(
            control_rate=args.rate,
            enable_logging=not args.no_logging,
            log_dir=args.log_dir,
            use_simulated_motors=args.simulate,
            motor_interface_kwargs=motor_kwargs,
            approach_threshold=args.approach,
            contact_threshold=args.contact,
            velocity_scale=args.velocity_scale,
            velocity_damping=args.velocity_damping,
            position_recalibration_interval=args.recalibration_interval,
            verbose_logging=args.debug
        )
        
        # Start controller
        logger.info("Starting velocity controller...")
        controller.start()
        
        # Start sensor analysis if requested
        if args.analyze_sensors:
            logger.info("Starting MCP vs PIP sensor analysis...")
            controller.start_sensor_analysis()
        
        # Print initial status
        status = controller.get_system_status()
        logger.info(f"Controller running. Hand state: {status['hand']['hand_state']}")
        
        # Print mode information
        if args.calibrate:
            logger.info("Running in CALIBRATION mode - motor movements disabled")
        if args.analyze_sensors:
            logger.info("Sensor analysis active - collecting MCP/PIP comparison data")
        
        # Monitor loop
        print("\nVelocity-Based Proximity Controller Active - Press Ctrl+C to stop")
        print("------------------------------------------------------------")
        
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
                
                # Choose visualization style
                if args.visualization == 'minimal':
                    _display_minimal_status(status, args)
                elif args.visualization == 'detailed':
                    _display_detailed_status(status, args, controller)
                else:  # 'basic' (default)
                    _display_basic_status(status, args, controller)
                
                if any(faults.values()):
                    print("\n⚠️  FAULTS DETECTED:")
                    for fault, active in faults.items():
                        if active:
                            print(f" - {fault.upper()}")
                
                print("\nControls:")
                print("Press 'r' + Enter to reset hand to safe position")
                if 'velocities' in status:
                    print("Press 'f' + Enter to zero all velocities")
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
                        # 'f' key to freeze (stop all velocities)
                        elif key == 'f':
                            print("\nFREEZE REQUESTED - Stopping all finger velocities...")
                            if hasattr(controller, 'finger_velocities'):
                                for finger in controller.finger_velocities:
                                    controller.finger_velocities[finger] = 0.0
                                print("All velocities set to zero.")
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