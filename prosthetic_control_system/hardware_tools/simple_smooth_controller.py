#!/usr/bin/env python3
"""
Simple Smooth Controller for Prosthetic Hand System

This controller combines reliable I2C communication using the fallback_test.py pattern
with physics-based smoothing for motor movement. The implementation follows the proven
approach from run_experiment.py but adds smooth motion control.
"""

import os
import sys
import time
import signal
import argparse
import logging
import threading
import smbus2
import collections
import numpy as np

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("SmoothController")

# Add path for imports
repo_root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../"))
if repo_root not in sys.path:
    sys.path.insert(0, repo_root)

# Import hand interfaces
from prosthetic_control_system.hand.ability_hand_interface import AbilityHandInterface
from prosthetic_control_system.hand.motor_interface import SimulatedMotorInterface

# Constants from fallback_test.py
MAX_READY_MS = 20
N_RETRIES = 3
SWEEP_HZ = 5

VL6180X_ADDR = 0x29
bus = smbus2.SMBus(1)

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

# Sensor to finger mapping
SENSOR_TO_FINGER = {
    "Thumb1": "Thumb",
    "Index1": "Index",
    "Middle1": "Middle",
    "Ring1": "Ring",
    "Pinky1": "Pinky"
}

# Physics parameters for smooth motion
VELOCITY_LIMIT = 0.2  # Max position units per iteration
ACCELERATION_LIMIT = 0.05  # Max velocity change per iteration
CONTROL_HZ = 50  # Motor control update frequency

# Global controller instance
controller = None
running = True

# I2C multiplexer functions
def mux_select(addr, ch):
    """Select the specified channel on the I2C multiplexer."""
    bus.write_byte(addr, 1 << ch)
    time.sleep(0.0005)

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

class SmoothController:
    """
    Smooth controller with physics-based motion smoothing.
    Combines reliable I2C communication with smooth motor control.
    """
    
    def __init__(
        self,
        control_rate=20,
        use_simulated_motors=False,
        motor_interface_kwargs=None,
        sensor_debug=False
    ):
        """
        Initialize the smooth controller.
        
        Args:
            control_rate: Control loop frequency in Hz
            use_simulated_motors: Whether to use simulated motors
            motor_interface_kwargs: Additional kwargs for motor interface
            sensor_debug: Enable debug logging for sensor operations
        """
        # Save parameters
        self.control_rate = control_rate
        self.control_interval = 1.0 / control_rate
        self.sensor_debug = sensor_debug
        
        # Initialize motor interface
        if motor_interface_kwargs is None:
            motor_interface_kwargs = {}
            
        motor_interface_kwargs['control_rate'] = control_rate
        
        if use_simulated_motors:
            logger.info("Using simulated motors")
            self.motors = SimulatedMotorInterface(**motor_interface_kwargs)
        else:
            try:
                logger.info("Initializing Ability Hand interface...")
                self.motors = AbilityHandInterface(**motor_interface_kwargs)
            except Exception as e:
                logger.error(f"Error initializing Ability Hand interface: {e}")
                logger.info("Falling back to simulated motors")
                self.motors = SimulatedMotorInterface(**motor_interface_kwargs)
        
        # Sensor and motor state
        self.sensor_readings = {name: None for name, _, _ in SENSORS}
        self.sensor_status = {name: "INIT" for name, _, _ in SENSORS}
        self.final_readings = {name: None for name, _, _ in SENSORS}
        
        # Motor state
        self.target_positions = {name: FINGER_OPEN_POS[name] for name in FINGER_NAMES}
        self.current_positions = {name: FINGER_OPEN_POS[name] for name in FINGER_NAMES}
        self.current_velocities = {name: 0.0 for name in FINGER_NAMES}
        
        # Error counters
        self.sensor_errors = 0
        self.motor_errors = 0
        
        # Sensor thread status
        self.sensor_thread_alive = False
        self.last_sensor_update = 0
        
        # Thread control
        self.running = False
        self.shutdown_event = threading.Event()
        self.sensor_thread = None
        self.motor_thread = None
        
        # Mutual exclusion for sensor data
        self.sensor_lock = threading.RLock()
        self.motor_lock = threading.RLock()
        
        # VL6180X initialization tracking
        self.init_done = collections.defaultdict(bool)
    
    def _sensor_thread_fn(self):
        """
        Thread function for reading sensor data.
        Uses the exact pattern from fallback_test.py for reliable I2C.
        """
        logger.info("Starting sensor thread")
        self.sensor_thread_alive = True
        
        try:
            while self.running and not self.shutdown_event.is_set():
                loop_start = time.time()
                raw, ok, subs, bad = {}, [], [], []
                
                # 1. Grab raw readings using direct I2C
                for name, mux, ch in SENSORS:
                    try:
                        mux_select(mux, ch)
                        if not self.init_done[(mux, ch)]:
                            vl_init()
                            self.init_done[(mux, ch)] = True
                        raw[name] = get_distance()
                        ok.append(name)
                    except Exception as e:
                        if self.sensor_debug:
                            logger.debug(f"Error reading sensor {name}: {e}")
                        raw[name] = None
                        self.sensor_errors += 1
                
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
                
                # 3. Update shared state with thread safety
                with self.sensor_lock:
                    self.sensor_readings = raw.copy()
                    self.final_readings = final.copy()
                    
                    # Update sensor status
                    for name in [name for name, _, _ in SENSORS]:
                        if name in ok:
                            self.sensor_status[name] = "OK"
                        elif name in subs:
                            self.sensor_status[name] = "SUB"
                        elif name in bad:
                            self.sensor_status[name] = "BAD"
                        else:
                            self.sensor_status[name] = "UNKNOWN"
                    
                    self.last_sensor_update = time.time()
                
                # Calculate sleep time to maintain consistent frequency
                elapsed = time.time() - loop_start
                sleep_time = max(0, 1.0 / SWEEP_HZ - elapsed)
                
                # This will return True if event is set during the wait
                if self.shutdown_event.wait(sleep_time):
                    break
                
        except Exception as e:
            logger.error(f"Sensor thread error: {e}")
        finally:
            self.sensor_thread_alive = False
            logger.info("Sensor thread exited")
    
    def _motor_thread_fn(self):
        """
        Thread function for controlling motors with physics-based smoothing.
        Runs at a higher frequency for smooth motion.
        """
        logger.info("Starting motor control thread")
        
        # Use finger state as a simple state machine
        # 0 = idle, 1 = approaching, 2 = in contact
        finger_state = {name: 0 for name in FINGER_NAMES}
        
        try:
            while self.running and not self.shutdown_event.is_set():
                loop_start = time.time()
                
                try:
                    # Get current sensor readings with thread safety
                    with self.sensor_lock:
                        final_readings = self.final_readings.copy()
                        sensor_status = self.sensor_status.copy()
                    
                    # Calculate target positions based on sensor readings
                    command_targets = {}
                    
                    # Process each finger using its MCP joint sensor
                    for sensor_name, finger_name in SENSOR_TO_FINGER.items():
                        distance = final_readings.get(sensor_name)
                        
                        if distance is not None:
                            # Get finger state
                            state = finger_state.get(finger_name, 0)
                            
                            # State transitions with hysteresis
                            if state == 0 and distance <= PROX_THRESH:  # Idle -> Approaching
                                finger_state[finger_name] = 1
                            elif state == 1 and distance <= 10:  # Approaching -> Contact
                                finger_state[finger_name] = 2
                            elif state in (1, 2) and distance >= 50:  # Return to idle
                                finger_state[finger_name] = 0
                            
                            # Calculate position based on state
                            if state == 0:  # Idle
                                target = FINGER_OPEN_POS.get(finger_name, 0.0)
                            elif state == 1:  # Approaching - use proportional control
                                # Linear interpolation between open and close positions
                                progress = min(1.0, max(0.0, (MAX_DISTANCE - distance) / (MAX_DISTANCE - PROX_THRESH)))
                                target = FINGER_OPEN_POS.get(finger_name, 0.0) + progress * (
                                    FINGER_CLOSE_POS.get(finger_name, 50.0) - FINGER_OPEN_POS.get(finger_name, 0.0)
                                )
                            else:  # Contact
                                target = FINGER_CLOSE_POS.get(finger_name, 50.0)
                        else:
                            # Default to open if no reading
                            target = FINGER_OPEN_POS.get(finger_name, 0.0)
                            finger_state[finger_name] = 0
                        
                        command_targets[finger_name] = target
                    
                    # Apply physics-based smoothing
                    with self.motor_lock:
                        new_positions, new_velocities = calculate_smooth_positions(
                            command_targets, self.current_positions, self.current_velocities
                        )
                        
                        # Update state
                        self.current_positions = new_positions
                        self.current_velocities = new_velocities
                        self.target_positions = command_targets
                    
                    # Send commands to hand
                    motor_commands = {}
                    for finger, position in new_positions.items():
                        motor_id = FINGER_TO_MOTOR_MAP.get(finger)
                        if motor_id is not None:
                            motor_commands[motor_id] = position
                    
                    # Set positions on hand
                    if motor_commands and self.motors:
                        self.motors.set_positions(motor_commands)
                
                except Exception as e:
                    logger.error(f"Error in motor control: {e}")
                    self.motor_errors += 1
                
                # Calculate sleep time to maintain consistent frequency
                elapsed = time.time() - loop_start
                sleep_time = max(0, 1.0 / CONTROL_HZ - elapsed)
                
                # This will return True if event is set during the wait
                if self.shutdown_event.wait(sleep_time):
                    break
                
        except Exception as e:
            logger.error(f"Motor thread error: {e}")
        finally:
            logger.info("Motor thread exited")
    
    def start(self):
        """Start the controller"""
        if self.running:
            logger.warning("Controller already running")
            return
        
        # Reset shutdown event
        self.shutdown_event.clear()
        self.running = True
        
        # Start motor interface
        logger.info("Starting motor interface...")
        if self.motors:
            self.motors.start()
        
        # Start sensor thread
        logger.info("Starting sensor thread...")
        self.sensor_thread = threading.Thread(target=self._sensor_thread_fn)
        self.sensor_thread.daemon = True
        self.sensor_thread.start()
        
        # Start motor thread
        logger.info("Starting motor thread...")
        self.motor_thread = threading.Thread(target=self._motor_thread_fn)
        self.motor_thread.daemon = True
        self.motor_thread.start()
        
        logger.info("Smooth controller started")
    
    def stop(self):
        """Stop the controller"""
        logger.info("Stopping controller...")
        
        # Signal threads to stop
        self.running = False
        self.shutdown_event.set()
        
        # Wait for threads to finish
        if self.sensor_thread and self.sensor_thread.is_alive():
            self.sensor_thread.join(timeout=2.0)
        
        if self.motor_thread and self.motor_thread.is_alive():
            self.motor_thread.join(timeout=2.0)
        
        # Ensure all fingers are in open position
        try:
            if self.motors:
                for finger in FINGER_NAMES:
                    motor_id = FINGER_TO_MOTOR_MAP.get(finger)
                    if motor_id is not None:
                        self.motors.set_position(finger, 0.0)
                
                # Give motors time to move
                time.sleep(0.5)
        except Exception as e:
            logger.error(f"Error opening fingers: {e}")
        
        # Stop motor interface
        if self.motors:
            logger.info("Stopping motors...")
            try:
                self.motors.stop()
            except Exception as e:
                logger.error(f"Error stopping motors: {e}")
        
        logger.info("Smooth controller stopped")
    
    def get_status(self):
        """Get the current system status"""
        status = {
            "fingers": {},
            "sensors": {},
            "system": {
                "sensor_thread_alive": self.sensor_thread_alive,
                "sensor_errors": self.sensor_errors,
                "motor_errors": self.motor_errors,
                "last_sensor_update": time.time() - self.last_sensor_update
            }
        }
        
        # Get sensor data
        with self.sensor_lock:
            for name, _, _ in SENSORS:
                raw_value = self.sensor_readings.get(name)
                final_value = self.final_readings.get(name)
                status_value = self.sensor_status.get(name, "UNKNOWN")
                
                status["sensors"][name] = {
                    "raw": raw_value,
                    "final": final_value,
                    "status": status_value
                }
        
        # Get finger data
        with self.motor_lock:
            for name in FINGER_NAMES:
                position = self.current_positions.get(name, 0.0)
                velocity = self.current_velocities.get(name, 0.0)
                target = self.target_positions.get(name, 0.0)
                
                # Get current from motors if available
                try:
                    current = self.motors.get_current(name) if self.motors else 0.0
                except:
                    current = 0.0
                
                status["fingers"][name] = {
                    "position": position,
                    "velocity": velocity,
                    "target": target,
                    "current": current
                }
        
        return status

def signal_handler(sig, frame):
    """Handle keyboard interrupt gracefully"""
    global running
    running = False
    if controller:
        logger.info("Stopping controller due to signal...")
        controller.stop()
    sys.exit(0)

def main():
    """Main entry point"""
    global controller, running
    
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Smooth Physics-Based Controller')
    
    parser.add_argument('--simulate', action='store_true',
                      help='Use simulated hardware')
    
    parser.add_argument('--rate', type=int, default=20,
                      help='Control loop rate in Hz (default: 20)')
    
    parser.add_argument('--port', type=str, default=None,
                      help='Serial port for Ability Hand (default: auto-detect)')
    
    parser.add_argument('--debug', action='store_true',
                      help='Enable debug logging')
    
    parser.add_argument('--sensor-debug', action='store_true',
                      help='Enable debug logging for sensor operations')
    
    args = parser.parse_args()
    
    # Set debug logging if requested
    if args.debug:
        logger.setLevel(logging.DEBUG)
        logger.debug("Debug logging enabled")
    
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # Create motor kwargs if needed
        motor_kwargs = {}
        if args.port:
            motor_kwargs['port'] = args.port
        
        # Create controller
        logger.info(f"Creating controller (simulated: {args.simulate}, rate: {args.rate}Hz)")
        controller = SmoothController(
            control_rate=args.rate,
            use_simulated_motors=args.simulate,
            motor_interface_kwargs=motor_kwargs,
            sensor_debug=args.sensor_debug
        )
        
        # Start controller
        logger.info("Starting controller...")
        controller.start()
        
        # Monitor loop
        print("\nSmooth Physics-Based Controller Active - Press Ctrl+C to stop")
        print("-----------------------------------------------------------")
        print("This controller uses the fallback_test.py pattern for reliable I2C")
        print("combined with physics-based motion smoothing for natural movements.")
        
        while running:
            try:
                # Get current status
                status = controller.get_status()
                
                # Clear screen
                os.system('clear' if os.name == 'posix' else 'cls')
                
                # Print status header
                print("\nSmooth Physics-Based Controller")
                print("============================")
                
                # System status
                system = status["system"]
                print(f"\nSystem Status:")
                print(f"  Sensor Thread: {'Active' if system['sensor_thread_alive'] else 'Dead'}")
                print(f"  Sensor Errors: {system['sensor_errors']}")
                print(f"  Motor Errors: {system['motor_errors']}")
                print(f"  Last Sensor Update: {system['last_sensor_update']:.2f}s ago")
                
                # Print finger states
                print("\nFinger Status:")
                print("-" * 60)
                for finger, data in status["fingers"].items():
                    print(f"{finger:10s}: Pos={data['position']:6.2f}° | "
                          f"Vel={data['velocity']:6.2f}° | "
                          f"Target={data['target']:6.2f}° | "
                          f"Current={data['current']:.3f}A")
                
                # Print proximity values
                print("\nProximity Sensors:")
                print("-" * 60)
                print(f"{'Sensor':10s} | {'Raw':5s} | {'Final':5s} | {'Status':8s}")
                print("-" * 60)
                for sensor, data in status["sensors"].items():
                    raw_str = f"{data['raw']}mm" if data['raw'] is not None else "N/A"
                    final_str = f"{data['final']}mm" if data['final'] is not None else "N/A"
                    
                    # Highlight sensors in close range
                    if data['final'] is not None and data['final'] <= 25:
                        highlight = "⚠️"
                    else:
                        highlight = ""
                    
                    print(f"{sensor:10s} | {raw_str:5s} | {final_str:5s} | {data['status']:8s} {highlight}")
                
                # Instructions
                print("\nPress Ctrl+C to stop")
                
                # Sleep
                time.sleep(0.2)
                
            except KeyboardInterrupt:
                running = False
                break
            except Exception as e:
                logger.error(f"Error in monitor loop: {e}")
                time.sleep(1)
    
    except Exception as e:
        logger.error(f"Error in main: {e}")
    
    finally:
        # Ensure controller is stopped
        if controller:
            logger.info("Stopping controller...")
            controller.stop()
        
        logger.info("Exiting")

if __name__ == "__main__":
    main()