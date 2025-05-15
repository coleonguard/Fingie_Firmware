#!/usr/bin/env python3
"""
Launch script for proximity-only control (no IMU).

This script runs the controller with IMU disabled, suitable for
testing with only proximity sensors and the Ability Hand.

Three controller implementations are available:
1. ProximityController: Original threaded implementation
2. SimplifiedController: Based on fallback_test.py architecture, more reliable
3. SmoothController: Based on fallback_test.py with physics-based motion smoothing
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

# Add all necessary directories to path
# First get the repository root directory (Fingie_Firmware)
repo_root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../../"))

# Ensure repo root is in the path
if repo_root not in sys.path:
    sys.path.insert(0, repo_root)

# Import from this package
from prosthetic_control_system.controller.without_imu.proximity_controller import ProximityController
from prosthetic_control_system.controller.without_imu.simplified_controller import SimplifiedController
from prosthetic_control_system.controller.without_imu.smooth_controller import run_smooth_controller
from prosthetic_control_system.controller.without_imu.config import DEFAULT_CONFIG

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

def _display_basic_status(status, args, controller=None):
    """Display standard status information"""
    # Get status data
    hand_state = status['hand']['hand_state']
    finger_states = status['hand']['finger_states']
    proximity = status.get('proximity', {})
    positions = status.get('positions', {})
    
    # Header with hand state
    emoji_map = {
        'IDLE': '🚫',
        'ACTIVE': '✅',
        'HOLDING': '👊',
        'RELEASING': '👐',
        'ERROR': '❌'
    }
    emoji = emoji_map.get(hand_state, '⚠️')
    
    print(f"\n{emoji} Hand State: {hand_state}\n")
    
    # Print finger information in a table format
    print("Finger Status:")
    print("-" * 85)
    print(f"{'Finger':<10}{'State':<15}{'Distance (mm)':<15}{'Position (°)':<12}{'Raw Value':<15}{'Fallback':<15}")
    print("-" * 85)
    
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
            raw_proximity = "N/A"
            sensor_name = None
            
            # Get MCP sensor name for this finger
            for sensor in proximity.keys():
                if sensor.startswith(finger[0]) and sensor.endswith('1'):  # MCP sensors
                    sensor_name = sensor
                    break
            
            # Get raw proximity value if available
            if sensor_name and sensor_name in proximity:
                value = proximity[sensor_name]
                raw_proximity = f"{value:.1f}" if value is not None else "N/A"
                filtered_distance = raw_proximity  # For now, same as raw
            
            position = positions.get(finger, 0.0)
            
            # Add emoji based on state
            state_emoji = {
                'IDLE': '⚪',
                'APPROACH': '🟡',
                'PROPORTIONAL': '🟠',
                'CONTACT': '🔴'
            }.get(state, '⚪')
            
            # Add fallback information with status
            fallback_info = ""
            if finger in finger_fallbacks:
                fallback_info = finger_fallbacks[finger]
            elif finger in finger_status:
                status = finger_status[finger]
                if status == "SUB":
                    fallback_info = "Substituted"
                elif status == "BAD":
                    fallback_info = "Sensor Fail"
                elif status == "OK":
                    fallback_info = "OK"
            
            print(f"{finger:<10}{state_emoji} {state:<13}{filtered_distance:<15}{position:.1f}°{raw_proximity:<15}{fallback_info:<15}")
    
    print("-" * 85)
    
    # Display cycle time information
    cycle_time = status['cycle_time']
    print(f"\nSystem: {cycle_time['last']*1000:.1f}ms/cycle (avg: {cycle_time['avg']*1000:.1f}ms)")
    
    # Display special modes if active
    if args.calibrate:
        print("\n⚙️  CALIBRATION MODE ACTIVE - Motor movements disabled")
    
    if args.analyze_sensors and 'debug' in status and 'sensor_analysis' in status['debug']:
        analysis_data = status['debug']['sensor_analysis']
        duration = analysis_data.get('duration', 0)
        active = analysis_data.get('active', False)
        if active:
            print(f"\n📊 Sensor analysis running: {duration:.1f}s elapsed")
            
            # Show sample counts if available
            samples = analysis_data.get('sample_counts', {})
            if samples:
                print(f"   Samples collected: {', '.join([f'{f}:{c}' for f, c in samples.items()])}")

def _display_detailed_status(status, args, controller=None):
    """Display detailed status with debugging information"""
    # First show the basic status
    _display_basic_status(status, args)
    
    # Get debug info if available
    if 'debug' not in status:
        print("\nDebug information not available.")
        return
    
    debug = status['debug']
    
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
    
    # Show sensor fallback/substitution if being reported in debug info
    try:
        finger_substitutions = {}
        
        # Check if the controller provides finger substitution info
        if hasattr(controller, 'finger_substitutions'):
            # Get substitution data directly from controller
            for finger, sub_from in controller.finger_substitutions.items():
                if sub_from is not None:
                    finger_substitutions[finger] = sub_from
        
        if finger_substitutions:
            print("\nSensor Fallbacks Active:")
            for finger, sub_from in finger_substitutions.items():
                print(f"⚠️  {finger} using data from {sub_from}")
    except Exception:
        # Skip if we can't get the substitution info
        pass
    
    # Show timing constraints
    print("\nTiming Constraints:")
    if 'timing' in debug and 'min_times' in debug['timing']:
        min_times = debug['timing']['min_times']
        print(f"Min time in states: IDLE={min_times.get('idle', 0)}s, APPROACH={min_times.get('approach', 0)}s, " +
              f"PROPORTIONAL={min_times.get('proportional', 0)}s, CONTACT={min_times.get('contact', 0)}s")
    else:
        print("Timing constraints information not available")
    
    # Show motion constraints
    print("\nMotion Constraints:")
    finger_angles = status['positions']
    
    # Get max finger angles
    max_angles = {}
    if controller and hasattr(controller, 'max_finger_angles'):
        max_angles = controller.max_finger_angles
    elif 'max_finger_angles' in DEFAULT_CONFIG:
        # Import config if not already available
        from prosthetic_control_system.controller.without_imu.config import DEFAULT_CONFIG
        max_angles = DEFAULT_CONFIG['max_finger_angles']
    
    for finger, angle in finger_angles.items():
        if finger in max_angles:
            max_angle = max_angles.get(finger, 'Unknown')
            print(f"{finger:<10}: Currently {angle:.1f}° / Max {max_angle}°")
    
    # Show sensor analysis data if active
    if args.analyze_sensors and 'sensor_analysis' in debug:
        analysis_data = debug['sensor_analysis']
        duration = analysis_data.get('duration', 0)
        active = analysis_data.get('active', False)
        
        print("\n📊 Sensor Analysis:")
        print("-" * 52)
        if active:
            print(f"Collection active for {duration:.1f}s")
            # Try to get the analysis report if available
            try:
                if controller and hasattr(controller, 'get_sensor_analysis_report'):
                    report = controller.get_sensor_analysis_report()
                    
                    # Print sample counts
                    print("\nSamples collected:")
                    for finger, count in report.get('sample_counts', {}).items():
                        print(f"  {finger:<10}: {count} samples")
                    
                    # Print correlation findings
                    if 'correlations' in report and report['correlations']:
                        print("\nMCP-PIP Correlations:")
                        for finger, corr in report['correlations'].items():
                            strength = "Strong" if abs(corr) > 0.7 else "Moderate" if abs(corr) > 0.4 else "Weak"
                            print(f"  {finger:<10}: {corr:.2f} ({strength})")
                    
                    # Print key findings
                    if 'findings' in report and report['findings']:
                        print("\nKey Findings:")
                        for i, finding in enumerate(report['findings']):
                            print(f"  {i+1}. {finding}")
                else:
                    print("\nNo sensor analysis report available")
            except Exception as e:
                print(f"Error getting analysis report: {e}")

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
                      
    parser.add_argument('--calibrate', action='store_true',
                      help='Run in calibration mode')
                      
    parser.add_argument('--analyze-sensors', action='store_true',
                      help='Analyze MCP vs PIP sensor behavior')
                      
    parser.add_argument('--visualization', choices=['basic', 'detailed', 'minimal'],
                      default='basic', help='Display style for visualization')
                      
    parser.add_argument('--debug', action='store_true',
                      help='Enable verbose debugging output')
                      
    parser.add_argument('--simplified', action='store_true',
                      help='Use simplified controller based on fallback_test.py architecture')
                      
    parser.add_argument('--smooth', action='store_true',
                      help='Use smooth controller with physics-based motion')
    
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
        logger.info(f"Creating controller (simulated: {args.simulate}, rate: {args.rate}Hz, simplified: {args.simplified}, smooth: {args.smooth})")
        
        # Choose the controller implementation based on args
        if args.smooth:
            logger.info("Using smooth controller with physics-based motion")
            # Run the smooth controller directly and exit
            return run_smooth_controller(visual_mode=args.visualization)
        elif args.simplified:
            logger.info("Using simplified controller based on fallback_test.py architecture")
            controller = SimplifiedController(
                control_rate=args.rate,
                enable_logging=not args.no_logging,
                log_dir=args.log_dir,
                use_simulated_motors=args.simulate,
                motor_interface_kwargs=motor_kwargs,
                approach_threshold=args.approach,
                contact_threshold=args.contact,
                verbose_logging=args.debug
            )
        else:
            logger.info("Using original threaded controller implementation")
            controller = ProximityController(
                control_rate=args.rate,
                enable_logging=not args.no_logging,
                log_dir=args.log_dir,
                use_simulated_motors=args.simulate,
                motor_interface_kwargs=motor_kwargs,
                approach_threshold=args.approach,
                contact_threshold=args.contact,
                verbose_logging=args.debug
            )
        
        # Start controller
        logger.info("Starting controller...")
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