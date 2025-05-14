#!/usr/bin/env python3
"""
simple_controller.py - Simplified controller based directly on fallback_test.py

This script combines the successful sensor reading approach from fallback_test.py
with basic motor control in a single thread. No complex threading, no complex
phase management, just a simple, reliable loop.

Wiring
------
MULT1 (0x77): Thumb1 Thumb2 Index1 Index2 Middle1
MULT2 (0x73): Middle2 Ring1  Ring2  Pinky1  Pinky2
"""

import smbus2
import time
import signal
import sys
import collections
import logging
import os
from enum import Enum

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("SimpleController")

# Add paths for imports
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

# Import hand control components
from prosthetic_control_system.hand.ability_hand_interface import AbilityHandInterface

# --------------------------------------------------------------------------- #
#  User-tunable constants
MAX_READY_MS = 20      # Timeout for sensor ready (20ms)
N_RETRIES    = 3       # Retry attempts
SWEEP_HZ     = 5       # Update rate (controls sleep at end)

# Sensor thresholds
APPROACH_THRESHOLD = 40  # mm
CONTACT_THRESHOLD = 5    # mm

# VL6180X constants
VL6180X_ADDR = 0x29
bus = smbus2.SMBus(1)

# Sensor mapping
SENSORS = [  # (name, mux-addr, channel)
    ("Thumb1",  0x77, 0), ("Thumb2",  0x77, 1),
    ("Index1",  0x77, 2), ("Index2",  0x77, 3),
    ("Middle1", 0x77, 4), ("Middle2", 0x73, 0),
    ("Ring1",   0x73, 1), ("Ring2",   0x73, 2),
    ("Pinky1",  0x73, 3), ("Pinky2",  0x73, 4),
]

# Fallback rules
FALLBACK = {
    "Thumb1": ["Thumb2", "Index1", "Index2", "Middle1"],
    "Thumb2": ["Thumb1", "Index2", "Index1", "Middle2"],
    "Index1": ["Index2", "Thumb2", "Middle1", "Middle2"],
    "Index2": ["Index1", "Middle2", "Thumb2", "Middle1"],
    "Middle1": ["Middle2", "Index1", "Ring1", "Ring2"],
    "Middle2": ["Middle1", "Index2", "Ring2", "Ring1"],
    "Ring1": ["Ring2", "Middle1", "Pinky1", "Pinky2"],
    "Ring2": ["Ring1", "Middle2", "Pinky2", "Pinky1"],
    "Pinky1": ["Pinky2", "Ring1", "Ring2", "Middle1"],
    "Pinky2": ["Pinky1", "Ring2", "Ring1", "Middle2"],
}

# Mapping from sensors to finger names
FINGER_MAPPING = {
    "Thumb1": "Thumb",
    "Index1": "Index",
    "Middle1": "Middle",
    "Ring1": "Ring",
    "Pinky1": "Pinky"
}

# Max finger angles (degrees)
MAX_FINGER_ANGLES = {
    "Thumb": 45.0,
    "Index": 80.0,
    "Middle": 80.0,
    "Ring": 80.0,
    "Pinky": 80.0
}

# Simple finger state enum
class FingerState(Enum):
    IDLE = 0           # No object detected
    APPROACH = 1       # Object beyond threshold
    PROPORTIONAL = 2   # Object between approach and contact
    CONTACT = 3        # Object at contact distance

# Simple hand state enum
class HandState(Enum):
    IDLE = 0           # All fingers idle
    ACTIVE = 1         # Hand in use
    HOLDING = 2        # Hand holding object

# --------------------------------------------------------------------------- #
# I2C Helper Functions
def mux_select(addr, ch):
    """Select multiplexer channel"""
    bus.write_byte(addr, 1 << ch)
    time.sleep(0.0005)  # Keep this exact timing

def wb(reg, val):
    """Write byte to VL6180X register"""
    hi, lo = reg >> 8 & 0xFF, reg & 0xFF
    bus.write_i2c_block_data(VL6180X_ADDR, hi, [lo, val])

def rb(reg):
    """Read byte from VL6180X register"""
    hi, lo = reg >> 8 & 0xFF, reg & 0xFF
    bus.write_i2c_block_data(VL6180X_ADDR, hi, [lo])
    time.sleep(0.0003)  # Keep this exact timing
    return bus.read_byte(VL6180X_ADDR)

def vl_init():
    """Initialize VL6180X sensor"""
    if rb(0x016) != 1:   # "fresh-out-of-reset"
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

def start_range(): 
    """Start range measurement"""
    wb(0x018, 1)

def clear_int():
    """Clear interrupts"""
    wb(0x015, 7)

def wait_ready():
    """Wait for range measurement to be ready"""
    t0 = time.time()
    while (rb(0x04F) & 0x07) != 0x04:
        if (time.time() - t0) * 1000 > MAX_READY_MS:
            raise TimeoutError
        time.sleep(0.0003)  # Keep this exact timing

def get_distance():
    """Get distance measurement with retries"""
    for _ in range(N_RETRIES):
        try:
            start_range()
            wait_ready()
            d = rb(0x062)
            clear_int()
            return d
        except TimeoutError:
            time.sleep(0.002)  # Let the sensor settle before retry
    return None  # All retries failed

# --------------------------------------------------------------------------- #
# Clean shutdown
def shutdown(*_):
    """Handle shutdown gracefully"""
    # Disable all multiplexers
    for mux in {m for _, m, _ in SENSORS}:
        bus.write_byte(mux, 0x00)
    
    # Stop the hand if it exists
    if 'hand' in globals() and hand is not None:
        try:
            # Set all positions to 0 (open)
            for finger in FINGER_MAPPING.values():
                hand.set_position(finger, 0.0)
            hand.update()
            time.sleep(0.2)  # Wait for hand to open
            hand.stop()
        except Exception as e:
            logger.error(f"Error stopping hand: {e}")
    
    # Close the I2C bus
    bus.close()
    logger.info("Controller shutdown complete")
    sys.exit(0)

# Register signal handlers
signal.signal(signal.SIGINT, shutdown)
signal.signal(signal.SIGTERM, shutdown)

# --------------------------------------------------------------------------- #
# Main controller
def main():
    """Main controller loop"""
    global hand  # Make hand accessible to shutdown handler
    
    # Initialize hand interface
    logger.info("Initializing Ability Hand interface")
    hand = AbilityHandInterface(control_rate=SWEEP_HZ)
    hand.start()  # Start motor control thread
    
    # Initialize state tracking
    finger_states = {finger: FingerState.IDLE for finger in FINGER_MAPPING.values()}
    hand_state = HandState.IDLE
    finger_positions = {finger: 0.0 for finger in FINGER_MAPPING.values()}
    
    # Initialize sensor state tracking
    init_done = collections.defaultdict(bool)
    
    # Display info
    print("Simple Controller Starting...")
    print(f"Approach threshold: {APPROACH_THRESHOLD}mm")
    print(f"Contact threshold: {CONTACT_THRESHOLD}mm")
    print(f"Control rate: {SWEEP_HZ}Hz")
    print("Press Ctrl+C to exit")
    print("-" * 50)
    
    # Main loop - directly based on fallback_test.py
    while True:
        loop_start = time.time()
        raw, ok, subs, bad = {}, [], [], []
        
        # PHASE 1: RESET MULTIPLEXERS - Add a clean hardware reset between cycles
        for mux in {m for _, m, _ in SENSORS}:
            try:
                # Disable all channels
                bus.write_byte(mux, 0x00)
                time.sleep(0.001)  # Short delay
            except Exception as e:
                logger.error(f"Error resetting mux {hex(mux)}: {e}")
        
        # PHASE 2: READ ALL SENSORS - Directly from fallback_test.py
        # This is the key to reliable I2C operation
        for name, mux, ch in SENSORS:
            try:
                # Select multiplexer channel
                mux_select(mux, ch)
                
                # Initialize sensor if needed
                if not init_done[(mux, ch)]:
                    vl_init()
                    init_done[(mux, ch)] = True
                    
                # Get distance reading
                raw[name] = get_distance()
                if raw[name] is not None:
                    ok.append(name)
            except Exception as e:
                logger.error(f"Error reading sensor {name}: {e}")
                raw[name] = None
        
        # Apply fallback substitution where needed
        final = {}
        for name, *_ in SENSORS:
            if raw[name] is not None:
                final[name] = raw[name]
            else:
                # Try fallback sensors
                for nb in FALLBACK[name]:
                    if raw.get(nb) is not None:
                        final[name] = raw[nb]
                        subs.append(name)
                        break
                else:
                    final[name] = 100  # Default to max distance (100mm)
                    bad.append(name)
        
        # PHASE 3: UPDATE FINGER STATES
        # Process MCP sensors (primary control sensors) and map to fingers
        for sensor_name, finger_name in FINGER_MAPPING.items():
            # Get distance reading (with fallback)
            distance = final.get(sensor_name, 100)
            
            # Determine finger state based on distance
            if distance <= CONTACT_THRESHOLD:
                new_state = FingerState.CONTACT
            elif distance <= APPROACH_THRESHOLD:
                new_state = FingerState.PROPORTIONAL
            else:
                new_state = FingerState.IDLE
            
            # Update finger state
            finger_states[finger_name] = new_state
            
            # Calculate finger position based on state
            if new_state == FingerState.IDLE:
                target_pos = 0.0  # Fully open
            elif new_state == FingerState.CONTACT:
                target_pos = MAX_FINGER_ANGLES[finger_name]  # Fully closed
            elif new_state == FingerState.PROPORTIONAL:
                # Scale position proportionally between contact and approach thresholds
                range_size = APPROACH_THRESHOLD - CONTACT_THRESHOLD
                if range_size > 0:
                    # Normalize distance within range (0-1)
                    normalized = (distance - CONTACT_THRESHOLD) / range_size
                    # Invert and scale (closer = more closed)
                    normalized = 1.0 - max(0.0, min(1.0, normalized))
                    target_pos = normalized * MAX_FINGER_ANGLES[finger_name]
                else:
                    # Safety fallback
                    target_pos = 0.5 * MAX_FINGER_ANGLES[finger_name]
            
            # Simple rate limiting - move at most 10% of range per cycle
            current_pos = finger_positions[finger_name]
            max_change = 0.1 * MAX_FINGER_ANGLES[finger_name]
            
            if abs(target_pos - current_pos) <= max_change:
                finger_positions[finger_name] = target_pos
            else:
                # Limit change rate
                if target_pos > current_pos:
                    finger_positions[finger_name] = current_pos + max_change
                else:
                    finger_positions[finger_name] = current_pos - max_change
        
        # PHASE 4: UPDATE HAND STATE
        # Determine overall hand state based on finger states
        contact_count = sum(1 for state in finger_states.values() if state == FingerState.CONTACT)
        active_count = sum(1 for state in finger_states.values() if state in [FingerState.APPROACH, FingerState.PROPORTIONAL])
        
        if contact_count > 0:
            hand_state = HandState.HOLDING
        elif active_count > 0:
            hand_state = HandState.ACTIVE
        else:
            hand_state = HandState.IDLE
        
        # PHASE 5: CONTROL MOTORS
        # Send position commands to the hand
        for finger, position in finger_positions.items():
            try:
                hand.set_position(finger, position)
            except Exception as e:
                logger.error(f"Error setting position for {finger}: {e}")
        
        # Execute commands
        try:
            hand.update()
        except Exception as e:
            logger.error(f"Error updating hand: {e}")
        
        # Display status
        print("\n" + "-" * 50)
        print(f"Hand State: {hand_state.name}")
        print("-" * 50)
        print(f"{'Finger':<10}{'State':<15}{'Distance (mm)':<15}{'Position (°)':<15}")
        print("-" * 50)
        
        for finger in FINGER_MAPPING.values():
            sensor = next((k for k, v in FINGER_MAPPING.items() if v == finger), None)
            distance = final.get(sensor, "N/A")
            state = finger_states[finger].name
            position = finger_positions[finger]
            
            print(f"{finger:<10}{state:<15}{distance:<15}{position:.1f}°")
        
        print("-" * 50)
        print(f"Sensors: OK={len(ok)} SUB={len(subs)} BAD={len(bad)}")
        if subs:
            print(f"Substituted: {', '.join(subs)}")
        if bad:
            print(f"Failed: {', '.join(bad)}")
        
        # Wait for next cycle - exactly like fallback_test.py
        elapsed = time.time() - loop_start
        sleep_time = max(0, (1 / SWEEP_HZ) - elapsed)
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            logger.warning(f"Loop took {elapsed*1000:.1f}ms (exceeds cycle time of {1000/SWEEP_HZ:.1f}ms)")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        logger.error(f"Fatal error: {e}")
        shutdown()