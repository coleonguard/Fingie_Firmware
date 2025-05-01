#!/usr/bin/env python3
# Test Runner for Prosthetic Control System

import os
import sys
import time
import argparse

# Import our modules
from proximity_manager import ProximityManager
from motor_interface import SimulatedMotorInterface
from ability_hand_interface import AbilityHandInterface
from unified_controller import UnifiedController
import config

def test_proximity_sensors(duration=10, sample_rate=None):
    """Run a standalone test of the proximity sensors
    
    Args:
        duration: Test duration in seconds
        sample_rate: Optional sampling rate override
    """
    print("=== Proximity Sensor Test ===")
    
    # Use default rate if not specified
    if sample_rate is None:
        sample_rate = config.DEFAULT_PROXIMITY_RATE
        
    # Create and start manager
    manager = ProximityManager(sampling_rate=sample_rate)
    
    try:
        print(f"Starting proximity test (duration: {duration}s, rate: {sample_rate}Hz)...")
        manager.start()
        
        # Run test
        start_time = time.time()
        while time.time() - start_time < duration:
            print("\n---- Sensor Readings ----")
            for sensor_name in ["Thumb1", "Index1", "Middle1", "Ring1", "Pinky1"]:
                raw = manager.get_sensor_value(sensor_name, filtered=False)
                filtered = manager.get_sensor_value(sensor_name, filtered=True)
                print(f"{sensor_name}: Raw={raw}mm, Filtered={filtered:.1f}mm")
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        
    finally:
        manager.stop()
        print("Proximity test complete")

def test_motor_interface(duration=10, simulated=True, control_rate=None):
    """Run a standalone test of the motor interface
    
    Args:
        duration: Test duration in seconds
        simulated: Whether to use simulated motors
        control_rate: Optional control rate override
    """
    print("=== Motor Interface Test ===")
    
    # Use default rate if not specified
    if control_rate is None:
        control_rate = config.DEFAULT_CONTROL_RATE
    
    # Create interface
    if simulated:
        print("Using simulated motors")
        motors = SimulatedMotorInterface(control_rate=control_rate)
    else:
        try:
            print("Connecting to Ability Hand...")
            motors = AbilityHandInterface(
                control_rate=control_rate,
                port=config.ABILITY_HAND_PORT,
                baud_rate=config.ABILITY_HAND_BAUD_RATE,
                reply_mode=config.ABILITY_HAND_REPLY_MODE
            )
        except Exception as e:
            print(f"Error connecting to Ability Hand: {e}")
            print("Falling back to simulated motors")
            motors = SimulatedMotorInterface(control_rate=control_rate)
    
    try:
        print(f"Starting motor test (duration: {duration}s, rate: {control_rate}Hz)...")
        motors.start()
        
        # Test sequence
        print("\n-- Testing position control --")
        
        # Set different positions for each finger
        motors.set_position("Thumb", 30.0)
        motors.set_position("Index", 50.0)
        motors.set_position("Middle", 70.0)
        motors.set_position("Ring", 40.0)
        motors.set_position("Pinky", 60.0)
        
        # Report for a few seconds
        for i in range(5):
            print("\nPosition feedback:")
            for finger in motors.fingers:
                pos = motors.get_position(finger)
                print(f"{finger}: {pos:.1f}°")
            time.sleep(1.0)
        
        # Test torque control
        print("\n-- Testing torque control --")
        
        # Set different torques for each finger
        motors.set_torque("Thumb", 0.2)
        motors.set_torque("Index", 0.3)
        motors.set_torque("Middle", 0.1)
        motors.set_torque("Ring", 0.15)
        motors.set_torque("Pinky", 0.25)
        
        # Report for a few seconds
        for i in range(5):
            print("\nTorque/current feedback:")
            for finger in motors.fingers:
                torque = motors.get_torque(finger)
                current = motors.get_current(finger)
                print(f"{finger}: {torque:.2f}Nm, {current:.2f}A")
            time.sleep(1.0)
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        
    finally:
        motors.stop()
        print("Motor test complete")

def run_unified_controller(duration=30, simulated=None, control_rate=None, log_dir=None):
    """Run the unified controller
    
    Args:
        duration: Run duration in seconds (0 for indefinite)
        simulated: Whether to use simulated motors (None to auto-determine)
        control_rate: Optional control rate override
        log_dir: Optional log directory override
    """
    print("=== Unified Controller ===")
    
    # Use defaults if not specified
    if control_rate is None:
        control_rate = config.DEFAULT_CONTROL_RATE
        
    if log_dir is None:
        log_dir = config.DEFAULT_LOG_DIR
    
    # Determine whether to use simulation
    if simulated is None:
        simulated = config.SIMULATION_MODE
    
    # Create controller
    controller = UnifiedController(
        control_rate=control_rate,
        enable_logging=True,
        log_dir=log_dir,
        use_simulated_motors=simulated
    )
    
    try:
        print("Starting unified controller...")
        controller.start()
        
        start_time = time.time()
        interval_count = 0
        
        # Run for specified duration or indefinitely
        while duration == 0 or (time.time() - start_time < duration):
            # Print status every second
            if interval_count % control_rate == 0:
                print("\n--- System Status ---")
                for finger in ["Thumb", "Index", "Middle", "Ring", "Pinky"]:
                    status = controller.get_finger_status(finger)
                    print(f"{finger}: Dist={status.get('distance', 'N/A'):.1f}mm, "
                          f"Pos={status['position']:.1f}°, "
                          f"Current={status['current']:.2f}A, "
                          f"Phase={status.get('control_phase', 'N/A')}")
            
            # Sleep for one control cycle
            time.sleep(1.0 / control_rate)
            interval_count += 1
            
    except KeyboardInterrupt:
        print("\nController stopped by user")
        
    finally:
        controller.stop()
        print("Controller session complete")

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Prosthetic Control Test Runner")
    
    # Main command
    subparsers = parser.add_subparsers(dest="command", help="Command to run")
    
    # Proximity test command
    prox_parser = subparsers.add_parser("proximity", help="Test proximity sensors")
    prox_parser.add_argument("--duration", type=int, default=10, help="Test duration in seconds")
    prox_parser.add_argument("--rate", type=int, default=None, help="Sampling rate in Hz")
    
    # Motor test command
    motor_parser = subparsers.add_parser("motors", help="Test motor interface")
    motor_parser.add_argument("--duration", type=int, default=10, help="Test duration in seconds")
    motor_parser.add_argument("--simulated", action="store_true", help="Use simulated motors")
    motor_parser.add_argument("--rate", type=int, default=None, help="Control rate in Hz")
    
    # Unified controller command
    unified_parser = subparsers.add_parser("run", help="Run unified controller")
    unified_parser.add_argument("--duration", type=int, default=30, help="Run duration in seconds (0 for indefinite)")
    unified_parser.add_argument("--simulated", action="store_true", help="Use simulated motors")
    unified_parser.add_argument("--real", action="store_true", help="Use real motors (overrides --simulated)")
    unified_parser.add_argument("--rate", type=int, default=None, help="Control rate in Hz")
    unified_parser.add_argument("--log-dir", type=str, default=None, help="Log directory")
    
    # Parse arguments
    args = parser.parse_args()
    
    # Execute command
    if args.command == "proximity":
        test_proximity_sensors(duration=args.duration, sample_rate=args.rate)
    elif args.command == "motors":
        test_motor_interface(duration=args.duration, simulated=args.simulated, control_rate=args.rate)
    elif args.command == "run":
        # Handle real/simulated flag
        simulated = None  # Default to config
        if args.real:
            simulated = False
        elif args.simulated:
            simulated = True
            
        run_unified_controller(
            duration=args.duration,
            simulated=simulated,
            control_rate=args.rate,
            log_dir=args.log_dir
        )
    else:
        parser.print_help()

if __name__ == "__main__":
    main()