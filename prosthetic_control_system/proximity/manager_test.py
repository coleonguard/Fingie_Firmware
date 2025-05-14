#!/usr/bin/env python3
"""
Proximity Manager Test Script

This script tests the ProximityManager in isolation using the same
interface patterns as the run_controller.py script to help identify
any issues with the proximity sensor functionality.
"""

import os
import sys
import time
import signal
import logging
import argparse
from typing import Dict, List, Any, Optional

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("ProximityManagerTest")

# Add parent directory to path if needed
parent_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

# Import the proximity manager
from prosthetic_control_system.proximity.proximity_manager import ProximityManager
from prosthetic_control_system.controller.without_imu.config import DEFAULT_SENSORS, MCP_SENSORS

# Global manager instance for signal handling
manager = None

def signal_handler(sig, frame):
    """Handle keyboard interrupt gracefully"""
    if manager:
        logger.info("Stopping manager due to signal...")
        manager.stop()
    sys.exit(0)

def test_single_read(prox_manager: ProximityManager):
    """Test reading all sensors once and display results"""
    print("\n=== Single Sensor Reading Test ===")
    for sensor in prox_manager.sensor_names:
        try:
            value, status = prox_manager.get_sensor_value(sensor, filtered=True, with_status=True)
            if value is None:
                print(f"{sensor:<7} N/A  (Status: {status})")
            else:
                print(f"{sensor:<7} {value:3.1f} mm  (Status: {status})")
        except Exception as e:
            print(f"{sensor:<7} ERROR: {e}")
    print()

def test_continuous_read(prox_manager: ProximityManager, duration: int = 5, interval: float = 0.2):
    """Test reading all sensors continuously for a specified duration"""
    print(f"\n=== Continuous Reading Test ({duration}s) ===")
    
    # Record test results
    read_count = 0
    errors = 0
    values_history = {name: [] for name in prox_manager.sensor_names}
    status_counts = {name: {"OK": 0, "SUB": 0, "BAD": 0} for name in prox_manager.sensor_names}
    
    end_time = time.time() + duration
    while time.time() < end_time:
        read_count += 1
        
        # Test each sensor
        raw, ok, subs, bad = {}, [], [], []
        
        # Clear screen for better display
        os.system('clear' if os.name == 'posix' else 'cls')
        print(f"\n=== Reading {read_count} ===")
        
        # 1. Get raw readings like in fallback_test.py
        for name in prox_manager.sensor_names:
            try:
                # Get both value and status
                value, status = prox_manager.get_sensor_value(name, filtered=True, with_status=True)
                raw[name] = value
                
                # Track status
                status_counts[name][status] += 1
                
                # Store values
                if value is not None:
                    values_history[name].append(value)
                
                # Track status
                if status == "OK":
                    ok.append(name)
                elif status == "SUB":
                    subs.append(name)
                else:
                    bad.append(name)
                    
                # Print current value
                if value is None:
                    print(f"{name:<7} N/A  (Status: {status})")
                else:
                    print(f"{name:<7} {value:3.1f} mm  (Status: {status})")
            
            except Exception as e:
                print(f"{name:<7} ERROR: {e}")
                errors += 1
                bad.append(name)
        
        print(f"\nSummary: OK={ok}  SUB={subs}  BAD={bad}")
        print(f"Errors: {errors}")
        
        # Sleep until next interval
        time.sleep(interval)
    
    # Print summary statistics
    print("\n=== Test Summary ===")
    print(f"Total Readings: {read_count}")
    print(f"Total Errors: {errors}")
    
    print("\nSensor Status Counts:")
    for name in prox_manager.sensor_names:
        counts = status_counts[name]
        print(f"{name}: OK={counts['OK']} SUB={counts['SUB']} BAD={counts['BAD']}")
    
    print("\nAverage Values:")
    for name in prox_manager.sensor_names:
        if values_history[name]:
            avg = sum(values_history[name]) / len(values_history[name])
            print(f"{name}: {avg:.1f} mm (from {len(values_history[name])} readings)")
        else:
            print(f"{name}: No valid readings")

def fallback_style_read(prox_manager: ProximityManager, duration: int = 30, interval: float = 0.2):
    """
    Test reading all sensors in a style similar to fallback_test.py
    This simulates how the fallback_test.py script reads sensors directly
    but using the ProximityManager API instead of direct I2C communication.
    """
    print(f"\n=== Fallback-Style Reading Test ({duration}s) ===")
    
    end_time = time.time() + duration
    while time.time() < end_time:
        # Clear screen for better display
        os.system('clear' if os.name == 'posix' else 'cls')
        print("\n=== Proximity Readings ===")
        
        raw, ok, subs, bad = {}, [], [], []
        
        # Get the raw values
        for name in prox_manager.sensor_names:
            value, status = prox_manager.get_sensor_value(name, filtered=True, with_status=True)
            raw[name] = value
            
            # Track status the same way as fallback_test.py
            if status == "OK":
                ok.append(name)
            elif status == "SUB":
                subs.append(name)
            else:
                bad.append(name)
        
        # Print values the same way as fallback_test.py
        for name in prox_manager.sensor_names:
            value = raw[name]
            status = prox_manager.status.get(name, "BAD")
            
            if value is None:
                print(f"{name:<7} N/A")
            elif status == "SUB":
                print(f"{name:<7} ~{value:3.1f} mm")
            else:
                print(f"{name:<7}  {value:3.1f} mm")
        
        print(f"\nSummary: OK={ok}  SUB={subs}  BAD={bad}\n")
        
        # Sleep until next interval
        time.sleep(interval)

def controller_style_read(prox_manager: ProximityManager, duration: int = 30, interval: float = 0.2):
    """
    Test reading sensors in the style used by ProximityController
    This simulates how the controller accesses proximity data
    """
    print(f"\n=== Controller-Style Reading Test ({duration}s) ===")
    
    end_time = time.time() + duration
    while time.time() < end_time:
        # Clear screen for better display
        os.system('clear' if os.name == 'posix' else 'cls')
        print("\n=== Proximity Controller Status ===")
        
        # Get current proximity values (filtered)
        proximity_values = {}
        sensor_status = {}
        for name in prox_manager.sensor_names:
            try:
                value, status = prox_manager.get_sensor_value(name, filtered=True, with_status=True)
                proximity_values[name] = value
                sensor_status[name] = status
            except Exception as e:
                logger.error(f"Error getting sensor value for {name}: {e}")
                proximity_values[name] = None
                sensor_status[name] = "ERROR"
        
        # Print proximity data like the controller status display
        print("\nProximity Sensors:")
        print("----------------")
        for sensor, value in proximity_values.items():
            if value is None:
                print(f"{sensor:<10s}: N/A      | Status: {sensor_status[sensor]}")
            else:
                print(f"{sensor:<10s}: {value:5.1f}mm | Status: {sensor_status[sensor]}")
        
        # Detect faults (the way the controller does)
        bad_sensors = 0
        critical_sensors = MCP_SENSORS
        
        for sensor in critical_sensors:
            status = sensor_status.get(sensor, "BAD")
            if status == "BAD":
                bad_sensors += 1
        
        # Show fault status
        sensor_fault = bad_sensors >= 2
        if sensor_fault:
            print(f"\n⚠️  SENSOR FAULT DETECTED: {bad_sensors} critical sensors failing")
            print("Critical sensors with BAD status:")
            for sensor in critical_sensors:
                if sensor_status.get(sensor, "BAD") == "BAD":
                    print(f" - {sensor}")
        
        # Sleep until next interval
        time.sleep(interval)

def main():
    """Main entry point"""
    global manager
    
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Proximity Manager Test')
    
    parser.add_argument('--rate', type=int, default=20,
                      help='Sampling rate in Hz (default: 20)')
    
    parser.add_argument('--approach', type=int, default=40,
                      help='Approach threshold in mm (default: 40)')
    
    parser.add_argument('--contact', type=int, default=5,
                      help='Contact threshold in mm (default: 5)')
    
    parser.add_argument('--duration', type=int, default=30,
                      help='Test duration in seconds (default: 30)')
    
    parser.add_argument('--mode', type=str, choices=['all', 'single', 'continuous', 'fallback', 'controller'],
                      default='all', help='Test mode (default: all)')
    
    args = parser.parse_args()
    
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # Import sensors from config
        from prosthetic_control_system.controller.without_imu.config import DEFAULT_SENSORS
        
        # First, print out our test configuration
        print("\n=== Proximity Manager Test ===")
        print(f"Sampling Rate: {args.rate} Hz")
        print(f"Approach Threshold: {args.approach} mm")
        print(f"Contact Threshold: {args.contact} mm")
        print(f"Test Duration: {args.duration} seconds")
        print(f"Test Mode: {args.mode}")
        
        # Create ProximityManager with explicit sensor configuration
        logger.info("Initializing proximity manager...")
        manager = ProximityManager(
            sampling_rate=args.rate,
            approach_threshold=args.approach,
            contact_threshold=args.contact,
            sensors=DEFAULT_SENSORS  # Use same sensor config as controller
        )
        
        # Start the manager
        logger.info("Starting proximity manager...")
        manager.start()
        
        # Wait for a brief moment to allow initial readings
        time.sleep(1.0)
        
        # Run the requested test mode
        try:
            if args.mode == 'all' or args.mode == 'single':
                test_single_read(manager)
                if args.mode == 'single':
                    time.sleep(1)  # Give time to see results
                
            if args.mode == 'all' or args.mode == 'continuous':
                test_duration = min(5, args.duration)
                test_continuous_read(manager, duration=test_duration)
                
            if args.mode == 'all' or args.mode == 'fallback':
                fallback_style_read(manager, duration=args.duration)
                
            if args.mode == 'all' or args.mode == 'controller':
                controller_style_read(manager, duration=args.duration)
        
        except KeyboardInterrupt:
            print("\nTest interrupted by user")
        
    except Exception as e:
        logger.error(f"Error running test: {e}")
    
    finally:
        # Clean up resources
        if manager:
            logger.info("Stopping proximity manager...")
            manager.stop()
        
        logger.info("Test complete")

if __name__ == "__main__":
    main()