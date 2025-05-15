#!/usr/bin/env python3
"""
Smooth Controller for Prosthetic Hand System

This controller maintains reliable I2C communication using the pattern from fallback_test.py
while implementing physics-based smoothing for motor movement. It uses separate threads for
sensor reading and motor control to prevent I2C bus contention.
"""

import os
import sys
import time
import threading
import logging
import numpy as np
from typing import Dict, List, Tuple, Set, Optional, Any

# Add parent directory to path for imports
parent_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(parent_dir)

from prosthetic_control_system.proximity.proximity_manager import ProximityManager
from prosthetic_control_system.hand.ability_hand_interface import AbilityHandInterface
from prosthetic_control_system.controller.state_machines import FingerFSM, HandFSM

# Define configuration constants directly from fallback_test.py
SWEEP_HZ = 5  # Sensor sweep frequency (Hz)

SENSORS = [  # (name, mux-addr, channel)
    ("Thumb1",  0x77, 0), ("Thumb2",  0x77, 1),
    ("Index1",  0x77, 2), ("Index2",  0x77, 3),
    ("Middle1", 0x77, 4), ("Middle2", 0x73, 0),
    ("Ring1",   0x73, 1), ("Ring2",   0x73, 2),
    ("Pinky1",  0x73, 3), ("Pinky2",  0x73, 4),
]

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

# Motor control constants
MAX_DISTANCE = 100  # Maximum distance in mm
PROX_THRESH = 30    # Proximity threshold for finger closure

# Finger positions (0-100 scale)
FINGER_OPEN_POS = {
    "Thumb": 0.0,
    "Index": 0.0,
    "Middle": 0.0,
    "Ring": 0.0,
    "Pinky": 0.0
}

FINGER_CLOSE_POS = {
    "Thumb": 50.0,
    "Index": 50.0,
    "Middle": 50.0,
    "Ring": 50.0,
    "Pinky": 50.0
}

# Standard finger names
FINGER_NAMES = ["Thumb", "Index", "Middle", "Ring", "Pinky"]

# Mapping from finger names to motor IDs
FINGER_TO_MOTOR_MAP = {
    "Thumb": 4,   # Motor ID for thumb
    "Index": 0,   # Motor ID for index
    "Middle": 1,  # Motor ID for middle
    "Ring": 2,    # Motor ID for ring
    "Pinky": 3    # Motor ID for pinky
}

# Physics parameters for smooth motion
VELOCITY_LIMIT = 0.2  # Max position units per iteration
ACCELERATION_LIMIT = 0.05  # Max velocity change per iteration
CONTROL_HZ = 50  # Motor control update frequency
SENSOR_HZ = 5  # Sensor reading frequency

# I2C multiplexer functions
def mux_select(mux, channel):
    """Select the specified channel on the I2C multiplexer."""
    from smbus2 import SMBus
    bus = SMBus(1)
    if 0 <= channel <= 7:
        try:
            bus.write_byte(mux, 1 << channel)
            time.sleep(0.001)  # Small delay for multiplexer to switch channels
        except Exception as e:
            logging.error(f"Failed to select channel {channel} on multiplexer {mux}: {e}")
            raise
        finally:
            bus.close()
    else:
        bus.close()
        raise ValueError(f"Invalid channel {channel}. Must be 0-7.")

def vl_init():
    """Initialize the VL6180X sensor."""
    import VL6180X
    sensor = VL6180X.VL6180X()
    try:
        sensor.get_identification()
        sensor.default_settings()
        return sensor
    except Exception as e:
        logging.error(f"Failed to initialize VL6180X sensor: {e}")
        raise

def get_distance(sensor=None):
    """Get distance reading from the VL6180X sensor."""
    if sensor is None:
        import VL6180X
        sensor = VL6180X.VL6180X()
    
    try:
        return sensor.get_distance()
    except Exception as e:
        logging.error(f"Failed to get distance: {e}")
        raise

class SharedState:
    """Thread-safe shared state between sensor and motor control threads."""
    
    def __init__(self):
        self._lock = threading.RLock()
        self._raw_readings = {}
        self._final_readings = {}
        self._target_positions = {name: FINGER_OPEN_POS[name] for name in FINGER_NAMES}
        self._current_positions = {name: FINGER_OPEN_POS[name] for name in FINGER_NAMES}
        self._velocities = {name: 0.0 for name in FINGER_NAMES}
        self._ok_sensors = []
        self._substituted_sensors = []
        self._bad_sensors = []
        self._running = True
        self._sensor_errors = 0
        self._motor_errors = 0
        self._last_sensor_update = 0
        
    def update_sensor_readings(self, raw, final, ok, subs, bad):
        """Update sensor readings in a thread-safe manner."""
        with self._lock:
            self._raw_readings = raw.copy()
            self._final_readings = final.copy()
            self._ok_sensors = ok.copy()
            self._substituted_sensors = subs.copy()
            self._bad_sensors = bad.copy()
            self._last_sensor_update = time.time()
            
    def get_sensor_readings(self):
        """Get the current sensor readings."""
        with self._lock:
            return {
                'raw': self._raw_readings.copy(),
                'final': self._final_readings.copy(),
                'ok': self._ok_sensors.copy(),
                'substituted': self._substituted_sensors.copy(),
                'bad': self._bad_sensors.copy(),
                'last_update': self._last_sensor_update
            }
            
    def update_targets(self, targets):
        """Update target positions in a thread-safe manner."""
        with self._lock:
            self._target_positions.update(targets)
            
    def get_targets(self):
        """Get the current target positions."""
        with self._lock:
            return self._target_positions.copy()
            
    def update_positions(self, positions, velocities):
        """Update current positions and velocities in a thread-safe manner."""
        with self._lock:
            self._current_positions.update(positions)
            self._velocities.update(velocities)
            
    def get_positions_and_velocities(self):
        """Get the current positions and velocities."""
        with self._lock:
            return self._current_positions.copy(), self._velocities.copy()
            
    def increment_sensor_error(self):
        """Increment the sensor error counter."""
        with self._lock:
            self._sensor_errors += 1
            
    def increment_motor_error(self):
        """Increment the motor error counter."""
        with self._lock:
            self._motor_errors += 1
            
    def get_error_counts(self):
        """Get the current error counts."""
        with self._lock:
            return self._sensor_errors, self._motor_errors
            
    def should_stop(self):
        """Check if the controller should stop."""
        with self._lock:
            return not self._running
            
    def stop(self):
        """Signal the controller to stop."""
        with self._lock:
            self._running = False

def sensor_thread(shared_state):
    """
    Thread for reading proximity sensors using the fallback_test.py pattern.
    Runs at a lower frequency (5Hz) to allow reliable I2C communication.
    """
    logging.info("Starting sensor thread")
    
    # Initialize structures for tracking sensor initialization
    init_done = {(mux, ch): False for name, mux, ch in SENSORS}
    
    # Main sensor reading loop
    while not shared_state.should_stop():
        loop_start = time.time()
        raw, ok, subs, bad = {}, [], [], []
        
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
                logging.debug(f"Error reading sensor {name}: {e}")
                raw[name] = None
                shared_state.increment_sensor_error()
        
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
        
        # Update shared state
        shared_state.update_sensor_readings(raw, final, ok, subs, bad)
        
        # Calculate required sleep time for consistent loop frequency
        elapsed = time.time() - loop_start
        sleep_time = max(0, 1.0 / SENSOR_HZ - elapsed)
        time.sleep(sleep_time)

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

def motor_thread(shared_state, hand):
    """
    Thread for controlling motors with physics-based smoothing.
    Runs at a higher frequency (50Hz) for smooth motion.
    """
    logging.info("Starting motor control thread")
    
    # Create finger state machines
    finger_fsms = {name: FingerFSM(name) for name in FINGER_NAMES}
    hand_fsm = HandFSM(finger_fsms)
    
    # Set initial command offsets to fully open
    command_offsets = {name: FINGER_OPEN_POS[name] for name in FINGER_NAMES}
    
    # Main motor control loop
    while not shared_state.should_stop():
        loop_start = time.time()
        
        try:
            # Get sensor readings and current positions
            readings = shared_state.get_sensor_readings()
            current_positions, current_velocities = shared_state.get_positions_and_velocities()
            
            # Map MCP sensors to fingers for control
            sensor_to_finger = {
                "Thumb1": "Thumb",
                "Index1": "Index",
                "Middle1": "Middle",
                "Ring1": "Ring",
                "Pinky1": "Pinky"
            }
            
            # Update FSMs with proximity data from MCP sensors
            for sensor, finger in sensor_to_finger.items():
                distance = readings['final'].get(sensor)
                if distance is not None:
                    # 1. Update finger FSM
                    finger_fsms[finger].update(distance)
                    
                    # 2. Calculate command offset based on FSM state and proximity
                    if finger_fsms[finger].state == 'CLOSING':
                        # Linear interpolation between open and close positions
                        progress = min(1.0, max(0.0, (MAX_DISTANCE - distance) / (MAX_DISTANCE - PROX_THRESH)))
                        target = FINGER_OPEN_POS[finger] + progress * (FINGER_CLOSE_POS[finger] - FINGER_OPEN_POS[finger])
                        command_offsets[finger] = target
                    elif finger_fsms[finger].state == 'OPENING':
                        command_offsets[finger] = FINGER_OPEN_POS[finger]
                    elif finger_fsms[finger].state == 'CLOSED':
                        command_offsets[finger] = FINGER_CLOSE_POS[finger]
            
            # Update hand FSM
            hand_fsm.update()
            
            # Apply physics-based smoothing
            smooth_positions, smooth_velocities = calculate_smooth_positions(
                command_offsets, current_positions, current_velocities
            )
            
            # Send commands to motors
            motor_commands = {}
            for finger, position in smooth_positions.items():
                motor_id = FINGER_TO_MOTOR_MAP[finger]
                motor_commands[motor_id] = position
            
            # Send command to hand
            if motor_commands:
                hand.set_positions(motor_commands)
            
            # Update shared state with current positions and velocities
            shared_state.update_positions(smooth_positions, smooth_velocities)
            
        except Exception as e:
            logging.error(f"Error in motor control loop: {e}")
            shared_state.increment_motor_error()
        
        # Calculate required sleep time for consistent loop frequency
        elapsed = time.time() - loop_start
        sleep_time = max(0, 1.0 / CONTROL_HZ - elapsed)
        time.sleep(sleep_time)

def run_smooth_controller(visual_mode="detailed"):
    """
    Main function to run the smooth controller with separate threads
    for sensor reading and motor control.
    """
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Initialize shared state
    shared_state = SharedState()
    
    # Initialize hand interface
    hand = AbilityHandInterface()
    hand.clear_errors()
    
    try:
        # Start sensor thread
        sensor_thread_obj = threading.Thread(
            target=sensor_thread,
            args=(shared_state,),
            daemon=True
        )
        sensor_thread_obj.start()
        
        # Start motor control thread
        motor_thread_obj = threading.Thread(
            target=motor_thread,
            args=(shared_state, hand),
            daemon=True
        )
        motor_thread_obj.start()
        
        # Main display loop
        while True:
            try:
                # Get latest data from shared state
                readings = shared_state.get_sensor_readings()
                positions, velocities = shared_state.get_positions_and_velocities()
                sensor_errors, motor_errors = shared_state.get_error_counts()
                
                # Clear screen
                os.system('clear')
                
                # Mapping from MCP sensors to finger names for display
                sensor_to_finger = {
                    "Thumb1": "Thumb",
                    "Index1": "Index",
                    "Middle1": "Middle",
                    "Ring1": "Ring",
                    "Pinky1": "Pinky"
                }
                
                # Display mode-specific information
                if visual_mode == "minimal":
                    print(f"Sensor Errors: {sensor_errors} | Motor Errors: {motor_errors}")
                    for finger in FINGER_NAMES:
                        sensor = next((s for s, f in sensor_to_finger.items() if f == finger), None)
                        dist = readings['final'].get(sensor) if sensor else None
                        dist_str = f"{dist:3d}" if dist is not None else "---"
                        pos = positions.get(finger, 0)
                        print(f"{finger:10s}: Dist={dist_str} Pos={pos:.2f}")
                
                elif visual_mode == "basic":
                    print(f"Sensor Errors: {sensor_errors} | Motor Errors: {motor_errors}")
                    print(f"OK Sensors: {len(readings['ok'])}/{len(SENSORS)}")
                    print(f"Bad Sensors: {readings['bad']}")
                    
                    for finger in FINGER_NAMES:
                        sensor = next((s for s, f in sensor_to_finger.items() if f == finger), None)
                        if sensor:
                            dist = readings['final'].get(sensor)
                            dist_str = f"{dist:3d}" if dist is not None else "---"
                            raw_dist = readings['raw'].get(sensor)
                            raw_str = f"{raw_dist:3d}" if raw_dist is not None else "---"
                            
                            # Show substitution indicator
                            if sensor in readings['substituted']:
                                flag = "*"
                            elif sensor in readings['bad']:
                                flag = "!"
                            else:
                                flag = " "
                        else:
                            dist_str = "---"
                            raw_str = "---"
                            flag = "?"
                            
                        pos = positions.get(finger, 0)
                        vel = velocities.get(finger, 0)
                            
                        print(f"{finger:10s}{flag}: Raw={raw_str} Dist={dist_str} Pos={pos:.2f} Vel={vel:.2f}")
                
                else:  # detailed
                    print(f"---- Smooth Controller Status ----")
                    print(f"Sensor Thread: {'Active' if sensor_thread_obj.is_alive() else 'Dead'}")
                    print(f"Motor Thread: {'Active' if motor_thread_obj.is_alive() else 'Dead'}")
                    print(f"Sensor Errors: {sensor_errors} | Motor Errors: {motor_errors}")
                    print(f"Last Sensor Update: {time.time() - readings['last_update']:.1f}s ago")
                    print(f"OK Sensors: {len(readings['ok'])}/{len(SENSORS)}")
                    print(f"Substituted: {readings['substituted']}")
                    print(f"Bad Sensors: {readings['bad']}")
                    
                    # First show all sensors
                    print("\n---- Sensor Readings ----")
                    for name, *_ in SENSORS:
                        raw_dist = readings['raw'].get(name)
                        raw_str = f"{raw_dist:3d}" if raw_dist is not None else "---"
                        final_dist = readings['final'].get(name)
                        final_str = f"{final_dist:3d}" if final_dist is not None else "---"
                        
                        # Show substitution indicator
                        if name in readings['substituted']:
                            flag = "*"
                            # Get fallback sources
                            fallback_src = ""
                            for nb in FALLBACK[name]:
                                if readings['raw'].get(nb) is not None:
                                    fallback_src = f"(using {nb})"
                                    break
                        elif name in readings['bad']:
                            flag = "!"
                            fallback_src = ""
                        else:
                            flag = " "
                            fallback_src = ""
                        
                        print(f"{name:10s}{flag}: Raw={raw_str} Final={final_str} {fallback_src}")
                    
                    # Then show finger information
                    print("\n---- Finger Status ----")
                    for finger in FINGER_NAMES:
                        sensor = next((s for s, f in sensor_to_finger.items() if f == finger), None)
                        if sensor:
                            dist = readings['final'].get(sensor)
                            dist_str = f"{dist:3d}" if dist is not None else "---"
                            
                            # Show substitution indicator
                            if sensor in readings['substituted']:
                                flag = "*"
                                fallback_src = ""
                                for nb in FALLBACK[sensor]:
                                    if readings['raw'].get(nb) is not None:
                                        fallback_src = f"(using {nb})"
                                        break
                            elif sensor in readings['bad']:
                                flag = "!"
                                fallback_src = ""
                            else:
                                flag = " "
                                fallback_src = ""
                        else:
                            dist_str = "---"
                            flag = "?"
                            fallback_src = ""
                        
                        pos = positions.get(finger, 0)
                        vel = velocities.get(finger, 0)
                        
                        print(f"{finger:10s}{flag}: Dist={dist_str} Pos={pos:.2f} Vel={vel:.2f} {fallback_src}")
                
                # Wait before updating display again
                time.sleep(0.1)
                
            except KeyboardInterrupt:
                print("\nController stopped by user.")
                break
            except Exception as e:
                logging.error(f"Error in display loop: {e}")
                time.sleep(1)  # Wait before retrying
    
    finally:
        # Signal threads to stop
        shared_state.stop()
        
        # Wait for threads to finish
        if 'sensor_thread_obj' in locals():
            sensor_thread_obj.join(timeout=2.0)
        if 'motor_thread_obj' in locals():
            motor_thread_obj.join(timeout=2.0)
            
        # Cleanup hand
        try:
            # Move fingers to open position
            hand.set_positions({motor: FINGER_OPEN_POS[finger] for finger, motor in FINGER_TO_MOTOR_MAP.items()})
            time.sleep(1.0)
        except Exception as e:
            logging.error(f"Error during cleanup: {e}")
        finally:
            hand.clear_errors()

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Smooth Controller for Prosthetic Hand System")
    parser.add_argument('--display', type=str, choices=['minimal', 'basic', 'detailed'], 
                        default='detailed', help='Display mode')
    
    args = parser.parse_args()
    
    try:
        run_smooth_controller(visual_mode=args.display)
    except KeyboardInterrupt:
        print("\nController stopped by user.")
    except Exception as e:
        logging.error(f"Controller crashed: {e}")
        raise