#!/usr/bin/env python3
"""
Sensor Monitoring Tool.

This tool provides real-time monitoring of all sensors in the prosthetic
control system, with options for data visualization and logging.
"""

import os
import sys
import time
import argparse
import logging
import threading
import json
import signal
from datetime import datetime
import subprocess
from collections import deque

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("SensorMonitor")

# Add parent directory to path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Try to import necessary modules
try:
    from proximity.proximity_manager import ProximityManager
    from imu.imu_interface import IMUInterface
    from hand.ability_hand_interface import AbilityHandInterface
    from utils.thread_safe_buffer import TimestampedBuffer
except ImportError as e:
    logger.error(f"Failed to import required modules: {e}")
    logger.error("Make sure you are running from the project root directory")
    sys.exit(1)

# Check for plotting support
try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    import matplotlib.gridspec as gridspec
    import numpy as np
    PLOTTING_AVAILABLE = True
except ImportError:
    PLOTTING_AVAILABLE = False
    logger.warning("Matplotlib not available. Install with 'pip install matplotlib' for plotting support.")

# Constants
DEFAULT_INTERVAL = 0.05  # 50ms sampling interval
DEFAULT_BUFFER_SIZE = 500  # Number of samples to keep in buffer
DEFAULT_LOG_FILE = "sensor_data_{timestamp}.json"
IMU_PORT = "/dev/ttyACM0"  # Default IMU port
HAND_PORT = None  # Let AbilityHandInterface detect automatically

class SensorMonitor:
    """
    Comprehensive sensor monitoring tool for the prosthetic control system.
    
    This class provides unified monitoring for:
    - Proximity sensors
    - IMU (orientation and acceleration)
    - Hand sensors (position, current, velocity)
    
    It supports real-time visualization and data logging.
    """
    
    def __init__(
        self,
        sensors_to_monitor=None,
        interval=DEFAULT_INTERVAL,
        buffer_size=DEFAULT_BUFFER_SIZE,
        imu_port=IMU_PORT,
        hand_port=HAND_PORT,
        log_data=False,
        log_file=None,
        plot=False
    ):
        """
        Initialize the sensor monitor.
        
        Args:
            sensors_to_monitor: List of sensor types to monitor ("proximity", "imu", "hand")
            interval: Sampling interval in seconds
            buffer_size: Number of samples to keep in memory
            imu_port: Serial port for IMU
            hand_port: Serial port for Ability Hand
            log_data: Whether to log data to file
            log_file: Log file path (uses default pattern if None)
            plot: Whether to show real-time plot
        """
        # Configuration
        self.interval = interval
        self.buffer_size = buffer_size
        self.imu_port = imu_port
        self.hand_port = hand_port
        self.log_data = log_data
        
        # Generate log filename if needed
        if log_data and not log_file:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.log_file = DEFAULT_LOG_FILE.format(timestamp=timestamp)
        else:
            self.log_file = log_file
        
        self.plot = plot and PLOTTING_AVAILABLE
        
        # Determine which sensors to monitor
        if sensors_to_monitor:
            self.monitor_proximity = "proximity" in sensors_to_monitor
            self.monitor_imu = "imu" in sensors_to_monitor
            self.monitor_hand = "hand" in sensors_to_monitor
        else:
            # Default: monitor all available sensors
            self.monitor_proximity = True
            self.monitor_imu = True
            self.monitor_hand = True
        
        # Data buffers
        self.timestamps = deque(maxlen=buffer_size)
        
        # Proximity sensor data
        self.proximity_data = None
        self.proximity_sensors = []
        self.proximity_values = {}
        
        # IMU data
        self.imu_data = None
        self.imu_orientation = {
            "roll": deque(maxlen=buffer_size),
            "pitch": deque(maxlen=buffer_size),
            "yaw": deque(maxlen=buffer_size)
        }
        self.imu_acceleration = {
            "x": deque(maxlen=buffer_size),
            "y": deque(maxlen=buffer_size),
            "z": deque(maxlen=buffer_size)
        }
        
        # Hand data
        self.hand_data = None
        self.hand_fingers = []
        self.hand_positions = {}
        self.hand_currents = {}
        self.hand_velocities = {}
        
        # Threading
        self.running = False
        self.thread = None
        self.log_file_handle = None
        
        # Initialize sensors
        self._initialize_sensors()
        
        # Setup plotting
        self.fig = None
        self.axes = {}
        self.lines = {}
        if self.plot:
            self._setup_plot()
    
    def _initialize_sensors(self):
        """Initialize all requested sensor interfaces"""
        # Initialize proximity sensors
        if self.monitor_proximity:
            try:
                logger.info("Initializing proximity sensors...")
                self.proximity_data = ProximityManager(
                    sampling_rate=1.0/self.interval
                )
                self.proximity_sensors = self.proximity_data.sensor_names
                self.proximity_values = {
                    sensor: deque(maxlen=self.buffer_size) 
                    for sensor in self.proximity_sensors
                }
                logger.info(f"Found {len(self.proximity_sensors)} proximity sensors")
            except Exception as e:
                logger.error(f"Failed to initialize proximity sensors: {e}")
                self.monitor_proximity = False
        
        # Initialize IMU
        if self.monitor_imu:
            try:
                logger.info("Initializing IMU...")
                self.imu_data = IMUInterface(
                    sampling_rate=1.0/self.interval,
                    primary_port=self.imu_port,
                    check_orientation=False
                )
                logger.info("IMU initialized")
            except Exception as e:
                logger.error(f"Failed to initialize IMU: {e}")
                self.monitor_imu = False
        
        # Initialize hand interface
        if self.monitor_hand:
            try:
                logger.info("Initializing Ability Hand...")
                self.hand_data = AbilityHandInterface(
                    control_rate=1.0/self.interval,
                    port=self.hand_port
                )
                self.hand_fingers = self.hand_data.fingers + ["ThumbRotate"]
                self.hand_positions = {
                    finger: deque(maxlen=self.buffer_size) 
                    for finger in self.hand_fingers
                }
                self.hand_currents = {
                    finger: deque(maxlen=self.buffer_size) 
                    for finger in self.hand_fingers
                }
                self.hand_velocities = {
                    finger: deque(maxlen=self.buffer_size) 
                    for finger in self.hand_fingers
                }
                logger.info("Ability Hand initialized")
            except Exception as e:
                logger.error(f"Failed to initialize Ability Hand: {e}")
                self.monitor_hand = False
    
    def _setup_plot(self):
        """Setup real-time plotting for monitored sensors"""
        if not PLOTTING_AVAILABLE:
            return
        
        # Count how many sensor types we're monitoring
        sensor_count = sum([self.monitor_proximity, self.monitor_imu, self.monitor_hand])
        if sensor_count == 0:
            logger.error("No sensors to plot")
            self.plot = False
            return
        
        # Create figure with appropriate layout
        self.fig = plt.figure(figsize=(12, 8))
        gs = gridspec.GridSpec(sensor_count, 1, height_ratios=[1] * sensor_count)
        
        # Setup subplots based on which sensors are monitored
        row = 0
        
        # Proximity sensors
        if self.monitor_proximity:
            ax = self.fig.add_subplot(gs[row, 0])
            ax.set_title("Proximity Sensors")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Distance (mm)")
            ax.grid(True)
            
            # Create a line for each proximity sensor
            self.lines["proximity"] = {}
            for i, sensor in enumerate(self.proximity_sensors):
                line, = ax.plot([], [], label=sensor)
                self.lines["proximity"][sensor] = line
            
            ax.legend(loc='upper left')
            self.axes["proximity"] = ax
            row += 1
        
        # IMU
        if self.monitor_imu:
            # Create two subplots for IMU: orientation and acceleration
            ax_orient = self.fig.add_subplot(gs[row, 0])
            ax_orient.set_title("IMU Orientation")
            ax_orient.set_xlabel("Time (s)")
            ax_orient.set_ylabel("Angle (degrees)")
            ax_orient.grid(True)
            
            # Lines for orientation
            self.lines["imu_orientation"] = {}
            line_roll, = ax_orient.plot([], [], 'r-', label='Roll')
            line_pitch, = ax_orient.plot([], [], 'g-', label='Pitch')
            line_yaw, = ax_orient.plot([], [], 'b-', label='Yaw')
            
            self.lines["imu_orientation"]["roll"] = line_roll
            self.lines["imu_orientation"]["pitch"] = line_pitch
            self.lines["imu_orientation"]["yaw"] = line_yaw
            
            ax_orient.legend(loc='upper left')
            self.axes["imu_orientation"] = ax_orient
            
            # Add acceleration info in a text box
            ax_orient.text(
                0.01, 0.02, "X: -- m/s²  Y: -- m/s²  Z: -- m/s²", 
                transform=ax_orient.transAxes,
                bbox=dict(facecolor='white', alpha=0.8)
            )
            
            row += 1
        
        # Hand
        if self.monitor_hand:
            ax = self.fig.add_subplot(gs[row, 0])
            ax.set_title("Hand Positions")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Position (degrees)")
            ax.grid(True)
            
            # Create a line for each finger
            self.lines["hand_position"] = {}
            for i, finger in enumerate(self.hand_fingers):
                line, = ax.plot([], [], label=finger)
                self.lines["hand_position"][finger] = line
            
            ax.legend(loc='upper left')
            self.axes["hand_position"] = ax
            
            # Add current info in a text box
            ax.text(
                0.01, 0.02, "Currents: [waiting for data]", 
                transform=ax.transAxes,
                bbox=dict(facecolor='white', alpha=0.8)
            )
            
            row += 1
        
        plt.tight_layout()
        
        # Setup animation
        self.anim = animation.FuncAnimation(
            self.fig, self._update_plot, interval=self.interval * 1000
        )
    
    def _update_plot(self, frame):
        """Update the plot with latest sensor data"""
        if not self.running:
            return
        
        # Convert timestamps to relative time (starting at 0)
        if not self.timestamps:
            return
        
        t0 = self.timestamps[0]
        times = [t - t0 for t in self.timestamps]
        
        # Update proximity sensor plot
        if self.monitor_proximity and "proximity" in self.axes:
            for sensor, line in self.lines["proximity"].items():
                if sensor in self.proximity_values and self.proximity_values[sensor]:
                    line.set_data(times, list(self.proximity_values[sensor]))
            
            ax = self.axes["proximity"]
            ax.relim()
            ax.autoscale_view()
            
            # Add fixed y-axis limits for proximity sensors
            ax.set_ylim(0, 100)
            ax.set_xlim(0, max(times) if times else 10)
        
        # Update IMU orientation plot
        if self.monitor_imu and "imu_orientation" in self.axes:
            # Update orientation lines
            for axis, line in self.lines["imu_orientation"].items():
                if self.imu_orientation[axis]:
                    line.set_data(times, list(self.imu_orientation[axis]))
            
            ax = self.axes["imu_orientation"]
            ax.relim()
            ax.autoscale_view()
            ax.set_xlim(0, max(times) if times else 10)
            
            # Update acceleration text
            if self.imu_acceleration["x"] and self.imu_acceleration["y"] and self.imu_acceleration["z"]:
                accel_x = self.imu_acceleration["x"][-1]
                accel_y = self.imu_acceleration["y"][-1]
                accel_z = self.imu_acceleration["z"][-1]
                
                ax.texts[0].set_text(
                    f"X: {accel_x:.2f} m/s²  Y: {accel_y:.2f} m/s²  Z: {accel_z:.2f} m/s²"
                )
        
        # Update hand position plot
        if self.monitor_hand and "hand_position" in self.axes:
            # Update position lines
            for finger, line in self.lines["hand_position"].items():
                if finger in self.hand_positions and self.hand_positions[finger]:
                    line.set_data(times, list(self.hand_positions[finger]))
            
            ax = self.axes["hand_position"]
            ax.relim()
            ax.autoscale_view()
            ax.set_xlim(0, max(times) if times else 10)
            
            # Update current text
            current_text = "Currents: "
            if self.hand_currents and all(finger in self.hand_currents for finger in self.hand_fingers):
                currents = []
                for finger in self.hand_fingers:
                    if self.hand_currents[finger]:
                        current = self.hand_currents[finger][-1]
                        currents.append(f"{finger[0]}: {current:.2f}A")
                
                current_text += " ".join(currents)
                ax.texts[0].set_text(current_text)
        
        return []
    
    def start(self):
        """Start the sensor monitor"""
        if self.running:
            return
        
        # Start data logging if requested
        if self.log_data:
            try:
                self.log_file_handle = open(self.log_file, 'w')
                # Write header
                log_header = {
                    "timestamp": datetime.now().isoformat(),
                    "monitored_sensors": {
                        "proximity": self.monitor_proximity,
                        "imu": self.monitor_imu,
                        "hand": self.monitor_hand
                    },
                    "sampling_interval": self.interval,
                    "data": []
                }
                json.dump(log_header, self.log_file_handle)
                # Replace closing bracket with comma to prepare for data entries
                self.log_file_handle.seek(self.log_file_handle.tell() - 1)
                self.log_file_handle.write(',"data":[\n')
                logger.info(f"Logging data to {self.log_file}")
            except Exception as e:
                logger.error(f"Failed to open log file: {e}")
                self.log_data = False
        
        # Start sensor interfaces
        if self.monitor_proximity and self.proximity_data:
            self.proximity_data.start()
        
        if self.monitor_imu and self.imu_data:
            self.imu_data.start()
        
        if self.monitor_hand and self.hand_data:
            self.hand_data.start()
        
        # Start monitoring thread
        self.running = True
        self.thread = threading.Thread(target=self._monitor_thread)
        self.thread.daemon = True
        self.thread.start()
        
        logger.info("Sensor monitor started")
        
        # Start plotting if requested
        if self.plot:
            plt.show()
    
    def stop(self):
        """Stop the sensor monitor"""
        self.running = False
        
        if self.thread:
            self.thread.join(timeout=1.0)
        
        # Stop sensor interfaces
        if self.monitor_proximity and self.proximity_data:
            self.proximity_data.stop()
        
        if self.monitor_imu and self.imu_data:
            self.imu_data.stop()
        
        if self.monitor_hand and self.hand_data:
            self.hand_data.stop()
        
        # Close log file if open
        if self.log_data and self.log_file_handle:
            try:
                # Write closing bracket for data array and document
                self.log_file_handle.write(']}')
                self.log_file_handle.close()
                logger.info(f"Data log saved to {self.log_file}")
            except Exception as e:
                logger.error(f"Error closing log file: {e}")
        
        logger.info("Sensor monitor stopped")
    
    def _monitor_thread(self):
        """Main monitoring thread"""
        try:
            first_frame = True
            
            while self.running:
                start_time = time.time()
                data_frame = {"timestamp": start_time}
                
                # Add timestamp
                self.timestamps.append(start_time)
                
                # Read proximity sensors
                if self.monitor_proximity and self.proximity_data:
                    try:
                        # Get filtered values from all sensors
                        proximity_values = self.proximity_data.get_all_values(filtered=True)
                        
                        # Add to buffers
                        for sensor, value in proximity_values.items():
                            if sensor in self.proximity_values:
                                self.proximity_values[sensor].append(value)
                        
                        # Add to data frame
                        data_frame["proximity"] = proximity_values
                    except Exception as e:
                        logger.error(f"Error reading proximity sensors: {e}")
                
                # Read IMU data
                if self.monitor_imu and self.imu_data:
                    try:
                        # Get latest IMU data
                        imu_data = self.imu_data.get_data()
                        
                        # Add to orientation buffers
                        self.imu_orientation["roll"].append(imu_data.roll)
                        self.imu_orientation["pitch"].append(imu_data.pitch)
                        self.imu_orientation["yaw"].append(imu_data.yaw)
                        
                        # Add to acceleration buffers
                        self.imu_acceleration["x"].append(imu_data.accel_x)
                        self.imu_acceleration["y"].append(imu_data.accel_y)
                        self.imu_acceleration["z"].append(imu_data.accel_z)
                        
                        # Add to data frame
                        data_frame["imu"] = {
                            "orientation": {
                                "roll": imu_data.roll,
                                "pitch": imu_data.pitch,
                                "yaw": imu_data.yaw
                            },
                            "acceleration": {
                                "x": imu_data.accel_x,
                                "y": imu_data.accel_y,
                                "z": imu_data.accel_z
                            },
                            "gyro": {
                                "x": imu_data.gyro_x,
                                "y": imu_data.gyro_y,
                                "z": imu_data.gyro_z
                            }
                        }
                    except Exception as e:
                        logger.error(f"Error reading IMU data: {e}")
                
                # Read hand data
                if self.monitor_hand and self.hand_data:
                    try:
                        # Collect data for all fingers
                        hand_positions = {}
                        hand_currents = {}
                        hand_velocities = {}
                        
                        for finger in self.hand_fingers:
                            status = self.hand_data.get_finger_status(finger)
                            
                            # Add to buffers
                            self.hand_positions[finger].append(status["position"])
                            self.hand_currents[finger].append(status["current"])
                            self.hand_velocities[finger].append(status["velocity"])
                            
                            # Collect for data frame
                            hand_positions[finger] = status["position"]
                            hand_currents[finger] = status["current"]
                            hand_velocities[finger] = status["velocity"]
                        
                        # Add to data frame
                        data_frame["hand"] = {
                            "position": hand_positions,
                            "current": hand_currents,
                            "velocity": hand_velocities
                        }
                    except Exception as e:
                        logger.error(f"Error reading hand data: {e}")
                
                # Write data frame to log file if logging
                if self.log_data and self.log_file_handle:
                    try:
                        # Add comma between entries (except first one)
                        if not first_frame:
                            self.log_file_handle.write(',\n')
                        first_frame = False
                        
                        # Write data frame
                        json.dump(data_frame, self.log_file_handle)
                        self.log_file_handle.flush()
                    except Exception as e:
                        logger.error(f"Error writing to log file: {e}")
                
                # Display status on console
                if not self.plot:
                    self._display_console_status()
                
                # Sleep to maintain sample rate
                time_taken = time.time() - start_time
                sleep_time = max(0, self.interval - time_taken)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    logger.debug(f"Sample processing took {time_taken*1000:.1f}ms (exceeds interval of {self.interval*1000:.1f}ms)")
        
        except Exception as e:
            logger.error(f"Error in monitoring thread: {e}")
            self.running = False
    
    def _display_console_status(self):
        """Display current sensor status on the console"""
        # Clear screen
        os.system('clear' if os.name == 'posix' else 'cls')
        
        print("\n===== Sensor Monitor =====")
        print(f"Monitoring: " + ", ".join(
            filter(None, [
                "Proximity" if self.monitor_proximity else None,
                "IMU" if self.monitor_imu else None,
                "Hand" if self.monitor_hand else None
            ])
        ))
        print(f"Sampling interval: {self.interval*1000:.1f}ms")
        print(f"Log file: {self.log_file if self.log_data else 'Not logging'}")
        
        # Display proximity sensor data
        if self.monitor_proximity:
            print("\n----- Proximity Sensors -----")
            for sensor in self.proximity_sensors:
                value = self.proximity_values[sensor][-1] if self.proximity_values[sensor] else "N/A"
                print(f"  {sensor}: {value} mm")
        
        # Display IMU data
        if self.monitor_imu:
            print("\n----- IMU Data -----")
            if self.imu_orientation["roll"]:
                print(f"  Orientation: Roll={self.imu_orientation['roll'][-1]:.1f}°, "
                      f"Pitch={self.imu_orientation['pitch'][-1]:.1f}°, "
                      f"Yaw={self.imu_orientation['yaw'][-1]:.1f}°")
            
            if self.imu_acceleration["x"]:
                print(f"  Acceleration: X={self.imu_acceleration['x'][-1]:.2f}, "
                      f"Y={self.imu_acceleration['y'][-1]:.2f}, "
                      f"Z={self.imu_acceleration['z'][-1]:.2f} m/s²")
        
        # Display hand data
        if self.monitor_hand:
            print("\n----- Hand Data -----")
            print("  Position:")
            for finger in self.hand_fingers:
                value = self.hand_positions[finger][-1] if self.hand_positions[finger] else "N/A"
                print(f"    {finger}: {value:.1f}°")
            
            print("  Current:")
            for finger in self.hand_fingers:
                value = self.hand_currents[finger][-1] if self.hand_currents[finger] else "N/A"
                print(f"    {finger}: {value:.3f}A")
        
        print("\nPress Ctrl+C to exit")

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description="Sensor Monitoring Tool")
    
    # Sensor selection
    parser.add_argument('--sensors', type=str, default=None,
                       help="Comma-separated list of sensors to monitor (proximity,imu,hand)")
    
    # Hardware options
    parser.add_argument('--imu-port', type=str, default=IMU_PORT,
                       help=f"Serial port for IMU (default: {IMU_PORT})")
    parser.add_argument('--hand-port', type=str, default=None,
                       help="Serial port for Ability Hand (default: auto-detect)")
    
    # Monitoring options
    parser.add_argument('--interval', type=float, default=DEFAULT_INTERVAL,
                       help=f"Sampling interval in seconds (default: {DEFAULT_INTERVAL})")
    parser.add_argument('--buffer', type=int, default=DEFAULT_BUFFER_SIZE,
                       help=f"Sample buffer size (default: {DEFAULT_BUFFER_SIZE})")
    
    # Output options
    parser.add_argument('--log', action='store_true',
                       help="Log data to file")
    parser.add_argument('--log-file', type=str, default=None,
                       help="Log file path (default: auto-generated)")
    parser.add_argument('--plot', action='store_true',
                       help="Show real-time plot")
    
    args = parser.parse_args()
    
    # Parse sensor list
    sensors_to_monitor = None
    if args.sensors:
        sensors_to_monitor = [s.strip().lower() for s in args.sensors.split(',')]
    
    # Create monitor
    monitor = SensorMonitor(
        sensors_to_monitor=sensors_to_monitor,
        interval=args.interval,
        buffer_size=args.buffer,
        imu_port=args.imu_port,
        hand_port=args.hand_port,
        log_data=args.log,
        log_file=args.log_file,
        plot=args.plot
    )
    
    # Handle interrupts gracefully
    def signal_handler(sig, frame):
        print("\nExiting...")
        monitor.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # Start monitoring
        monitor.start()
        
        # Keep main thread alive if not plotting
        if not args.plot:
            while monitor.running:
                time.sleep(0.1)
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    finally:
        monitor.stop()

if __name__ == "__main__":
    main()