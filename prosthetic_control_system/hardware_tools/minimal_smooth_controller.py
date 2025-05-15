#!/usr/bin/env python3
"""
Minimal Smooth Controller for Proximal Hand Control

This script provides a minimal implementation that works with the Ability Hand
and uses the fallback_test.py pattern for sensor reading, combined with
physics-based motion smoothing for natural movement.
"""

import sys
import os
import time
import signal
import logging
import threading
import collections
import argparse
import numpy as np

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("MinimalSmoothController")

# Ensure the module's directory is in the path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

# Import the hand interface
from prosthetic_control_system.hand.ability_hand_interface import AbilityHandInterface

# Import I2C modules
import smbus2

# Basic constants
MAX_READY_MS = 20
N_RETRIES = 3
SWEEP_HZ = 5  # How often to read sensors
CONTROL_HZ = 50  # How often to update motors

# I2C bus
bus = smbus2.SMBus(1)
VL6180X_ADDR = 0x29

# Finger definitions
FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]

# Finger positions
FINGER_OPEN_POS = 0.0
FINGER_CLOSE_POS = 50.0

# Proximity thresholds
CLOSE_THRESHOLD = 30  # Start closing when object is within 30mm
OPEN_THRESHOLD = 60   # Start opening when object is beyond 60mm

# Sensor definitions from fallback_test.py
SENSORS = [  # (name, mux-addr, channel)
    ("Thumb1",  0x77, 0), ("Thumb2",  0x77, 1),
    ("Index1",  0x77, 2), ("Index2",  0x77, 3),
    ("Middle1", 0x77, 4), ("Middle2", 0x73, 0),
    ("Ring1",   0x73, 1), ("Ring2",   0x73, 2),
    ("Pinky1",  0x73, 3), ("Pinky2",  0x73, 4),
]

# Sensor-to-finger mapping (MCP joints)
SENSOR_TO_FINGER = {
    "Thumb1": "Thumb",
    "Index1": "Index",
    "Middle1": "Middle",
    "Ring1": "Ring",
    "Pinky1": "Pinky"
}

# Fallback mappings for sensors
FALLBACK = {
    "Thumb1":["Thumb2","Index1","Index2","Middle1"],
    "Thumb2":["Thumb1","Index2","Index1","Middle2"],
    "Index1":["Index2","Thumb2","Middle1","Middle2"],
    "Index2":["Index1","Middle2","Thumb2","Middle1"],
    "Middle1":["Middle2","Index1","Ring1","Ring2"],
    "Middle2":["Middle1","Index2","Ring2","Ring1"],
    "Ring1"  :["Ring2","Middle1","Pinky1","Pinky2"],
    "Ring2"  :["Ring1","Middle2","Pinky2","Pinky1"],
    "Pinky1" :["Pinky2","Ring1","Ring2","Middle1"],
    "Pinky2" :["Pinky1","Ring2","Ring1","Middle2"],
}

# Global state for controller
running = True
hand = None
sensor_readings = {}
finger_positions = {f: FINGER_OPEN_POS for f in FINGER_NAMES}
finger_targets = {f: FINGER_OPEN_POS for f in FINGER_NAMES}
finger_velocities = {f: 0.0 for f in FINGER_NAMES}

# Physics parameters for smooth motion
VELOCITY_LIMIT = 0.5   # Max position units per iteration
ACCELERATION_LIMIT = 0.1  # Max velocity change per iteration

# VL6180X sensor functions from fallback_test.py
def mux_select(addr, ch):
    """Select the specified channel on the I2C multiplexer."""
    try:
        bus.write_byte(addr, 1 << ch)
        time.sleep(0.0005)
    except Exception as e:
        logger.error(f"Error selecting multiplexer {addr} channel {ch}: {e}")
        raise

def wb(reg, val):
    """Write byte to VL6180X register."""
    hi, lo = reg >> 8 & 0xFF, reg & 0xFF
    bus.write_i2c_block_data(VL6180X_ADDR, hi, [lo, val])

def rb(reg):
    """Read byte from VL6180X register."""
    hi, lo = reg >> 8 & 0xFF, reg & 0xFF
    bus.write_i2c_block_data(VL6180X_ADDR, hi, [lo])
    time.sleep(0.0003)
    return bus.read_byte(VL6180X_ADDR)

def vl_init():
    """Initialize the VL6180X sensor."""
    if rb(0x016) != 1:  # "fresh-out-of-reset"
        return
    for r, v in [
        (0x0207,1),(0x0208,1),(0x0096,0),(0x0097,0xfd),
        (0x00e3,0),(0x00e4,4),(0x00e5,2),(0x00e6,1),(0x00e7,3),
        (0x00f5,2),(0x00d9,5),(0x00db,0xce),(0x00dc,3),(0x00dd,0xf8),
        (0x009f,0),(0x00a3,0x3c),(0x00b7,0),(0x00bb,0x3c),(0x00b2,9),
        (0x00ca,9),(0x0198,1),(0x01b0,0x17),(0x01ad,0),(0x00ff,5),
        (0x0100,5),(0x0199,5),(0x01a6,0x1b),(0x01ac,0x3e),(0x01a7,0x1f),
        (0x0030,0),(0x0011,0x10),(0x010a,0x30),(0x003f,0x46),(0x0031,0xFF),
        (0x0041,0x63),(0x002e,1),(0x001b,9),(0x003e,0x31),(0x0014,0x24)
    ]:
        wb(r, v)
    wb(0x016, 0)

def start_range(): wb(0x018, 1)
def clear_int(): wb(0x015, 7)

def wait_ready():
    t0 = time.time()
    while (rb(0x04F) & 0x07) != 0x04:
        if (time.time() - t0) * 1000 > MAX_READY_MS:
            raise TimeoutError
        time.sleep(0.0003)

def get_distance():
    for _ in range(N_RETRIES):
        try:
            start_range()
            wait_ready()
            d = rb(0x062)
            clear_int()
            return d
        except TimeoutError:
            time.sleep(0.002)  # Let the sensor settle before retry
    raise TimeoutError("range-ready timeout")

def calculate_smooth_positions(targets, current, velocities):
    """
    Calculate smooth transitions between current positions and targets
    using velocity and acceleration limiting physics model.
    """
    new_positions = {}
    new_velocities = {}
    
    for finger, target in targets.items():
        # Current state
        pos = current[finger]
        vel = velocities[finger]
        
        # Position error
        error = target - pos
        
        # Desired velocity to get to target position
        desired_vel = np.sign(error) * min(abs(error), VELOCITY_LIMIT)
        
        # Apply acceleration limit to change in velocity
        vel_change = desired_vel - vel
        limited_vel_change = np.sign(vel_change) * min(abs(vel_change), ACCELERATION_LIMIT)
        new_vel = vel + limited_vel_change
        
        # Calculate new position
        new_pos = pos + new_vel
        
        # Store results
        new_positions[finger] = new_pos
        new_velocities[finger] = new_vel
        
        # If we're very close to target, snap to it and stop
        if abs(new_pos - target) < 0.01:
            new_positions[finger] = target
            new_velocities[finger] = 0.0
            
    return new_positions, new_velocities

def read_sensors():
    """Read all sensors using the fallback_test.py pattern."""
    global sensor_readings
    
    init_done = collections.defaultdict(bool)
    
    while running:
        start_time = time.time()
        
        # Initialize result structures
        raw = {}
        ok = []
        subs = []
        bad = []
        
        # 1. Grab raw readings
        for name, mux, ch in SENSORS:
            try:
                mux_select(mux, ch)
                if not init_done[(mux, ch)]:
                    vl_init()
                    init_done[(mux, ch)] = True
                raw[name] = get_distance()
                ok.append(name)
            except Exception as e:
                logger.debug(f"Error reading sensor {name}: {e}")
                raw[name] = None
        
        # 2. Substitute where needed
        final = {}
        for name, *_ in SENSORS:
            if raw[name] is not None:
                final[name] = raw[name]
            else:
                for nb in FALLBACK[name]:
                    if raw.get(nb) is not None:
                        final[name] = raw[nb]
                        subs.append(name)
                        break
                else:
                    final[name] = None
                    bad.append(name)
        
        # Update global sensor readings
        sensor_readings = final
        
        # Calculate time to sleep
        elapsed = time.time() - start_time
        sleep_time = max(0, 1.0 / SWEEP_HZ - elapsed)
        time.sleep(sleep_time)

def control_motors():
    """Control motors with physics-based smoothing."""
    global finger_positions, finger_velocities, finger_targets
    
    while running:
        start_time = time.time()
        
        try:
            # Update target positions based on sensor readings
            for sensor_name, finger_name in SENSOR_TO_FINGER.items():
                distance = sensor_readings.get(sensor_name)
                
                if distance is not None:
                    # Calculate target position based on distance
                    if distance <= CLOSE_THRESHOLD:
                        # Object is close - close the finger proportionally
                        # Linear interpolation based on distance
                        progress = min(1.0, max(0.0, (CLOSE_THRESHOLD - distance) / CLOSE_THRESHOLD))
                        finger_targets[finger_name] = FINGER_OPEN_POS + progress * (FINGER_CLOSE_POS - FINGER_OPEN_POS)
                    elif distance >= OPEN_THRESHOLD:
                        # No object detected - fully open
                        finger_targets[finger_name] = FINGER_OPEN_POS
            
            # Apply physics-based smoothing
            new_positions, new_velocities = calculate_smooth_positions(
                finger_targets, finger_positions, finger_velocities
            )
            
            # Update state
            finger_positions = new_positions
            finger_velocities = new_velocities
            
            # Send commands to hand if available
            if hand:
                for finger, position in new_positions.items():
                    hand.set_position(finger, position)
        
        except Exception as e:
            logger.error(f"Error in motor control: {e}")
        
        # Sleep to maintain consistent frequency
        elapsed = time.time() - start_time
        sleep_time = max(0, 1.0 / CONTROL_HZ - elapsed)
        time.sleep(sleep_time)

def display_status():
    """Display system status."""
    while running:
        try:
            # Clear screen
            os.system('clear' if os.name == 'posix' else 'cls')
            
            # Print header
            print("\nMinimal Smooth Controller")
            print("=======================")
            
            # Print finger positions
            print("\nFinger Status:")
            print("-" * 60)
            for finger in FINGER_NAMES:
                # Get actual positions from hand if available
                if hand:
                    try:
                        actual_pos = hand.get_position(finger)
                    except:
                        actual_pos = finger_positions[finger]
                else:
                    actual_pos = finger_positions[finger]
                
                print(f"{finger:10s}: Pos={actual_pos:6.2f}° | "
                      f"Target={finger_targets[finger]:6.2f}° | "
                      f"Vel={finger_velocities[finger]:6.2f}°/s")
            
            # Print sensor readings
            print("\nProximity Sensors (MCP joints):")
            print("-" * 60)
            for sensor in ["Thumb1", "Index1", "Middle1", "Ring1", "Pinky1"]:
                value = sensor_readings.get(sensor)
                value_str = f"{value:3d}mm" if value is not None else "N/A"
                
                # Highlight sensors in range
                if value is not None and value <= CLOSE_THRESHOLD:
                    value_str = f"{value_str} ⚠️"
                
                print(f"{sensor:10s}: {value_str}")
            
            # Sleep briefly to avoid refresh flicker
            time.sleep(0.2)
        
        except Exception as e:
            logger.error(f"Error in display: {e}")
            time.sleep(1)

def signal_handler(sig, frame):
    """Handle keyboard interrupt gracefully."""
    global running
    logger.info("Stopping controller...")
    running = False
    return

def shutdown():
    """Shutdown procedure."""
    global hand
    
    logger.info("Shutting down...")
    
    # Set all fingers to open position
    if hand:
        logger.info("Opening fingers...")
        try:
            for finger in FINGER_NAMES:
                hand.set_position(finger, FINGER_OPEN_POS)
            time.sleep(1.0)
        except Exception as e:
            logger.error(f"Error opening fingers: {e}")
        
        # Stop hand interface
        logger.info("Stopping hand interface...")
        hand.stop()
    
    # Shutdown I2C bus
    try:
        for mux in {m for _, m, _ in SENSORS}:
            bus.write_byte(mux, 0x00)
        bus.close()
    except Exception as e:
        logger.error(f"Error shutting down I2C: {e}")
    
    logger.info("Controller stopped")

def main():
    """Main entry point."""
    global hand, running
    
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Minimal Smooth Controller')
    
    parser.add_argument('--port', type=str, default=None,
                      help='Serial port for Ability Hand (default: auto-detect)')
    
    parser.add_argument('--simulate', action='store_true',
                      help='Run without connecting to hardware')
    
    args = parser.parse_args()
    
    # Set up signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Initialize hand interface if not simulating
    if not args.simulate:
        try:
            logger.info(f"Initializing Ability Hand (port: {args.port or 'auto-detect'})...")
            hand = AbilityHandInterface(
                port=args.port,
                control_rate=CONTROL_HZ,
                reply_mode=2  # Position, Current, Velocity
            )
            
            # Start the interface
            logger.info("Starting hand interface...")
            hand.start()
            
            # Wait a moment for initialization
            time.sleep(1.0)
            
            # Check if we're connected
            if not hasattr(hand, 'client') or hand.client is None:
                logger.error("Failed to connect to Ability Hand")
                if not args.simulate:
                    logger.info("If you intended to run without hardware, use --simulate")
                    return
            
            # Show initial status
            logger.info("Hand connected successfully")
            
        except Exception as e:
            logger.error(f"Error initializing hand interface: {e}")
            if not args.simulate:
                logger.info("If you intended to run without hardware, use --simulate")
                return
    else:
        logger.info("Running in simulation mode (no hardware)")
    
    try:
        # Create threads
        sensor_thread = threading.Thread(target=read_sensors)
        sensor_thread.daemon = True
        
        motor_thread = threading.Thread(target=control_motors)
        motor_thread.daemon = True
        
        display_thread = threading.Thread(target=display_status)
        display_thread.daemon = True
        
        # Start threads
        logger.info("Starting sensor thread...")
        sensor_thread.start()
        
        logger.info("Starting motor control thread...")
        motor_thread.start()
        
        logger.info("Starting display thread...")
        display_thread.start()
        
        print("\nMinimal Smooth Controller Active - Press Ctrl+C to stop")
        print("-------------------------------------------------------")
        print("This controller directly implements sensor reading from fallback_test.py")
        print("combined with physics-based motion smoothing for natural movement.")
        
        # Main thread just waits
        while running:
            time.sleep(0.1)
        
    except Exception as e:
        logger.error(f"Error in main: {e}")
    
    finally:
        # Clean shutdown
        running = False
        shutdown()
        
        # Wait for threads to exit (timeout after 2 seconds)
        if 'sensor_thread' in locals() and sensor_thread.is_alive():
            sensor_thread.join(timeout=2.0)
        
        if 'motor_thread' in locals() and motor_thread.is_alive():
            motor_thread.join(timeout=2.0)
        
        if 'display_thread' in locals() and display_thread.is_alive():
            display_thread.join(timeout=1.0)

if __name__ == "__main__":
    main()