#!/usr/bin/env python3
"""
Sensor Data Logger Tool.

This tool provides a dedicated sensor data logging system with configurable
formats, sensors, and durations.
"""

import os
import sys
import time
import argparse
import logging
import json
import csv
import signal
import threading
from datetime import datetime
from pathlib import Path

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("LoggerTool")

# Add parent directory to path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Try to import necessary modules
try:
    from proximity.proximity_manager import ProximityManager
    from imu.imu_interface import IMUInterface
    from hand.ability_hand_interface import AbilityHandInterface
except ImportError as e:
    logger.error(f"Failed to import required modules: {e}")
    logger.error("Make sure you are running from the project root directory")
    sys.exit(1)

# Constants
DEFAULT_DURATION = 60  # seconds
DEFAULT_INTERVAL = 0.1  # seconds
DEFAULT_OUTPUT_DIR = "logs"
DEFAULT_FORMAT = "json"
IMU_PORT = "/dev/ttyACM0"
HAND_PORT = None  # Let AbilityHandInterface detect automatically

class SensorLogger:
    """
    Dedicated tool for logging sensor data to files.
    
    This class provides high-performance logging of sensor data with:
    - Support for JSON and CSV output formats
    - Selectable sensor types
    - Configurable log duration and sampling rate
    - Session metadata recording
    """
    
    def __init__(
        self,
        sensors_to_log=None,
        duration=DEFAULT_DURATION,
        interval=DEFAULT_INTERVAL,
        output_file=None,
        output_format=DEFAULT_FORMAT,
        imu_port=IMU_PORT,
        hand_port=HAND_PORT
    ):
        """
        Initialize the sensor logger.
        
        Args:
            sensors_to_log: List of sensor types to log ("proximity", "imu", "hand")
            duration: Recording duration in seconds
            interval: Sampling interval in seconds
            output_file: Path to output file (auto-generated if None)
            output_format: Output format ("json" or "csv")
            imu_port: Serial port for IMU
            hand_port: Serial port for Ability Hand
        """
        # Configuration
        self.duration = duration
        self.interval = interval
        self.output_format = output_format.lower()
        self.imu_port = imu_port
        self.hand_port = hand_port
        
        # Determine which sensors to log
        if sensors_to_log:
            self.log_proximity = "proximity" in sensors_to_log
            self.log_imu = "imu" in sensors_to_log
            self.log_hand = "hand" in sensors_to_log
        else:
            # Default: log all available sensors
            self.log_proximity = True
            self.log_imu = True
            self.log_hand = True
        
        # Validate output format
        if self.output_format not in ["json", "csv"]:
            logger.warning(f"Unsupported output format: {self.output_format}, using JSON")
            self.output_format = "json"
        
        # Generate output file if not provided
        if not output_file:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            sensor_str = "_".join(filter(None, [
                "proximity" if self.log_proximity else None,
                "imu" if self.log_imu else None,
                "hand" if self.log_hand else None
            ]))
            
            # Create directory if it doesn't exist
            os.makedirs(DEFAULT_OUTPUT_DIR, exist_ok=True)
            
            # Generate filename
            self.output_file = os.path.join(
                DEFAULT_OUTPUT_DIR,
                f"sensor_log_{sensor_str}_{timestamp}.{self.output_format}"
            )
        else:
            self.output_file = output_file
        
        # Sensor interfaces
        self.proximity_manager = None
        self.imu_interface = None
        self.hand_interface = None
        
        # Threading
        self.running = False
        self.thread = None
        self.file_handle = None
        self.csv_writer = None
        
        # Sample counter
        self.sample_count = 0
        
        # Statistics
        self.start_time = None
        self.end_time = None
        self.planned_end_time = None
        
        # Initialize sensors
        self._initialize_sensors()
    
    def _initialize_sensors(self):
        """Initialize all requested sensor interfaces"""
        # Initialize proximity sensors
        if self.log_proximity:
            try:
                logger.info("Initializing proximity sensors...")
                self.proximity_manager = ProximityManager(
                    sampling_rate=1.0/self.interval
                )
                logger.info(f"Found {len(self.proximity_manager.sensor_names)} proximity sensors")
            except Exception as e:
                logger.error(f"Failed to initialize proximity sensors: {e}")
                self.log_proximity = False
        
        # Initialize IMU
        if self.log_imu:
            try:
                logger.info("Initializing IMU...")
                self.imu_interface = IMUInterface(
                    sampling_rate=1.0/self.interval,
                    primary_port=self.imu_port,
                    check_orientation=False
                )
                logger.info("IMU initialized")
            except Exception as e:
                logger.error(f"Failed to initialize IMU: {e}")
                self.log_imu = False
        
        # Initialize hand interface
        if self.log_hand:
            try:
                logger.info("Initializing Ability Hand...")
                self.hand_interface = AbilityHandInterface(
                    control_rate=1.0/self.interval,
                    port=self.hand_port
                )
                logger.info("Ability Hand initialized")
            except Exception as e:
                logger.error(f"Failed to initialize Ability Hand: {e}")
                self.log_hand = False
    
    def _prepare_output_file(self):
        """Prepare the output file based on the selected format"""
        try:
            # Create directory if needed
            output_dir = os.path.dirname(self.output_file)
            if output_dir:
                os.makedirs(output_dir, exist_ok=True)
            
            # Open file for writing
            self.file_handle = open(self.output_file, 'w', newline='' if self.output_format == 'csv' else None)
            
            # Write header based on format
            if self.output_format == 'json':
                # Write JSON header with metadata
                header = {
                    "metadata": {
                        "timestamp": datetime.now().isoformat(),
                        "duration": self.duration,
                        "interval": self.interval,
                        "sensors": {
                            "proximity": self.log_proximity,
                            "imu": self.log_imu,
                            "hand": self.log_hand
                        },
                        "proximity_sensor_count": len(self.proximity_manager.sensor_names) if self.proximity_manager else 0,
                        "hand_finger_count": len(self.hand_interface.fingers) if self.hand_interface else 0
                    },
                    "samples": []
                }
                
                # Write header and open samples array
                self.file_handle.write(json.dumps(header, indent=2).rstrip("]}") + ',\n  "samples": [\n')
                
            elif self.output_format == 'csv':
                # Determine all field names for CSV header
                fields = ["timestamp"]
                
                # Add proximity sensor fields
                if self.log_proximity and self.proximity_manager:
                    for sensor in self.proximity_manager.sensor_names:
                        fields.append(f"proximity_{sensor}")
                
                # Add IMU fields
                if self.log_imu:
                    fields.extend([
                        "imu_roll", "imu_pitch", "imu_yaw",
                        "imu_accel_x", "imu_accel_y", "imu_accel_z",
                        "imu_gyro_x", "imu_gyro_y", "imu_gyro_z"
                    ])
                
                # Add hand fields
                if self.log_hand and self.hand_interface:
                    for finger in self.hand_interface.fingers + ["ThumbRotate"]:
                        fields.extend([
                            f"hand_{finger}_position",
                            f"hand_{finger}_current",
                            f"hand_{finger}_velocity"
                        ])
                
                # Create CSV writer
                self.csv_writer = csv.DictWriter(self.file_handle, fieldnames=fields)
                self.csv_writer.writeheader()
            
            logger.info(f"Output file prepared: {self.output_file}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to prepare output file: {e}")
            return False
    
    def _finalize_output_file(self):
        """Finalize the output file based on the selected format"""
        if not self.file_handle:
            return
        
        try:
            if self.output_format == 'json':
                # Close the samples array and add final statistics
                stats = {
                    "statistics": {
                        "start_time": self.start_time.isoformat() if self.start_time else None,
                        "end_time": self.end_time.isoformat() if self.end_time else None,
                        "planned_duration": self.duration,
                        "actual_duration": (self.end_time - self.start_time).total_seconds() if self.start_time and self.end_time else None,
                        "sample_count": self.sample_count,
                        "actual_interval": ((self.end_time - self.start_time).total_seconds() / max(1, self.sample_count - 1)) if self.start_time and self.end_time and self.sample_count > 1 else None
                    }
                }
                
                # Write closing brackets and statistics
                self.file_handle.write("\n  ],\n")
                self.file_handle.write(f"  \"statistics\": {json.dumps(stats['statistics'], indent=2)}\n")
                self.file_handle.write("}")
            
            # Close file
            self.file_handle.close()
            self.file_handle = None
            
            logger.info(f"Output file finalized: {self.output_file}")
            
        except Exception as e:
            logger.error(f"Error finalizing output file: {e}")
    
    def _write_sample(self, sample_data):
        """Write a sample to the output file"""
        if not self.file_handle:
            return
        
        try:
            if self.output_format == 'json':
                # Write JSON sample
                if self.sample_count > 0:
                    self.file_handle.write(",\n")
                self.file_handle.write("    " + json.dumps(sample_data))
                
            elif self.output_format == 'csv':
                # Flatten the sample data for CSV
                flat_data = {"timestamp": sample_data["timestamp"]}
                
                # Add proximity data
                if "proximity" in sample_data:
                    for sensor, value in sample_data["proximity"].items():
                        flat_data[f"proximity_{sensor}"] = value
                
                # Add IMU data
                if "imu" in sample_data:
                    if "orientation" in sample_data["imu"]:
                        flat_data["imu_roll"] = sample_data["imu"]["orientation"]["roll"]
                        flat_data["imu_pitch"] = sample_data["imu"]["orientation"]["pitch"]
                        flat_data["imu_yaw"] = sample_data["imu"]["orientation"]["yaw"]
                    
                    if "acceleration" in sample_data["imu"]:
                        flat_data["imu_accel_x"] = sample_data["imu"]["acceleration"]["x"]
                        flat_data["imu_accel_y"] = sample_data["imu"]["acceleration"]["y"]
                        flat_data["imu_accel_z"] = sample_data["imu"]["acceleration"]["z"]
                    
                    if "gyro" in sample_data["imu"]:
                        flat_data["imu_gyro_x"] = sample_data["imu"]["gyro"]["x"]
                        flat_data["imu_gyro_y"] = sample_data["imu"]["gyro"]["y"]
                        flat_data["imu_gyro_z"] = sample_data["imu"]["gyro"]["z"]
                
                # Add hand data
                if "hand" in sample_data:
                    for data_type in ["position", "current", "velocity"]:
                        if data_type in sample_data["hand"]:
                            for finger, value in sample_data["hand"][data_type].items():
                                flat_data[f"hand_{finger}_{data_type}"] = value
                
                # Write CSV row
                self.csv_writer.writerow(flat_data)
            
            # Increment sample counter
            self.sample_count += 1
            
        except Exception as e:
            logger.error(f"Error writing sample: {e}")
    
    def _logging_thread(self):
        """Main logging thread"""
        try:
            # Record start time
            self.start_time = datetime.now()
            self.planned_end_time = self.start_time + datetime.timedelta(seconds=self.duration)
            
            logger.info(f"Starting logging: {self.duration}s duration, {self.interval}s interval")
            
            # Start all sensor interfaces
            if self.log_proximity and self.proximity_manager:
                self.proximity_manager.start()
            
            if self.log_imu and self.imu_interface:
                self.imu_interface.start()
            
            if self.log_hand and self.hand_interface:
                self.hand_interface.start()
            
            # Wait for interfaces to initialize
            time.sleep(0.5)
            
            # Main logging loop
            while self.running and datetime.now() < self.planned_end_time:
                sample_start_time = time.time()
                
                # Create sample data structure
                sample_data = {
                    "timestamp": sample_start_time
                }
                
                # Read proximity sensors
                if self.log_proximity and self.proximity_manager:
                    try:
                        proximity_values = self.proximity_manager.get_all_values(filtered=True)
                        sample_data["proximity"] = proximity_values
                    except Exception as e:
                        logger.error(f"Error reading proximity sensors: {e}")
                
                # Read IMU data
                if self.log_imu and self.imu_interface:
                    try:
                        imu_data = self.imu_interface.get_data()
                        sample_data["imu"] = {
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
                if self.log_hand and self.hand_interface:
                    try:
                        hand_positions = {}
                        hand_currents = {}
                        hand_velocities = {}
                        
                        fingers = self.hand_interface.fingers + ["ThumbRotate"]
                        for finger in fingers:
                            status = self.hand_interface.get_finger_status(finger)
                            hand_positions[finger] = status["position"]
                            hand_currents[finger] = status["current"]
                            hand_velocities[finger] = status["velocity"]
                        
                        sample_data["hand"] = {
                            "position": hand_positions,
                            "current": hand_currents,
                            "velocity": hand_velocities
                        }
                    except Exception as e:
                        logger.error(f"Error reading hand data: {e}")
                
                # Write sample to file
                self._write_sample(sample_data)
                
                # Update progress periodically
                if self.sample_count % 10 == 0:
                    elapsed = (datetime.now() - self.start_time).total_seconds()
                    progress = min(100, (elapsed / self.duration) * 100)
                    logger.info(f"Logging progress: {progress:.1f}% ({self.sample_count} samples)")
                
                # Calculate sleep time to maintain sample rate
                time_taken = time.time() - sample_start_time
                sleep_time = max(0, self.interval - time_taken)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    logger.debug(f"Sampling took {time_taken*1000:.1f}ms (exceeds interval of {self.interval*1000:.1f}ms)")
            
            # Record end time
            self.end_time = datetime.now()
            actual_duration = (self.end_time - self.start_time).total_seconds()
            
            logger.info(f"Logging complete: {self.sample_count} samples in {actual_duration:.1f}s")
            
        except Exception as e:
            logger.error(f"Error in logging thread: {e}")
        
        finally:
            # Stop all sensor interfaces
            if self.log_proximity and self.proximity_manager:
                self.proximity_manager.stop()
            
            if self.log_imu and self.imu_interface:
                self.imu_interface.stop()
            
            if self.log_hand and self.hand_interface:
                self.hand_interface.stop()
            
            # Record end time if not set
            if not self.end_time:
                self.end_time = datetime.now()
            
            # Signal completion
            self.running = False
    
    def start(self):
        """Start logging sensor data"""
        if self.running:
            return
        
        # Prepare output file
        if not self._prepare_output_file():
            logger.error("Failed to prepare output file, aborting")
            return
        
        # Start logging thread
        self.running = True
        self.thread = threading.Thread(target=self._logging_thread)
        self.thread.daemon = True
        self.thread.start()
        
        logger.info("Sensor logging started")
    
    def stop(self):
        """Stop logging sensor data"""
        if not self.running:
            return
        
        self.running = False
        
        if self.thread:
            self.thread.join(timeout=1.0)
        
        # Finalize output file
        self._finalize_output_file()
        
        logger.info("Sensor logging stopped")
    
    def wait_for_completion(self):
        """Wait for logging to complete"""
        if self.thread:
            self.thread.join()

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description="Sensor Data Logger Tool")
    
    # Sensor selection
    parser.add_argument('--sensors', type=str, default=None,
                       help="Comma-separated list of sensors to log (proximity,imu,hand)")
    
    # Logging parameters
    parser.add_argument('--duration', type=int, default=DEFAULT_DURATION,
                       help=f"Recording duration in seconds (default: {DEFAULT_DURATION})")
    parser.add_argument('--interval', type=float, default=DEFAULT_INTERVAL,
                       help=f"Sampling interval in seconds (default: {DEFAULT_INTERVAL})")
    
    # Output options
    parser.add_argument('--output', type=str, default=None,
                       help="Output file path (default: auto-generated)")
    parser.add_argument('--format', type=str, choices=['json', 'csv'], default=DEFAULT_FORMAT,
                       help=f"Output format (json, csv) (default: {DEFAULT_FORMAT})")
    
    # Hardware options
    parser.add_argument('--imu-port', type=str, default=IMU_PORT,
                       help=f"Serial port for IMU (default: {IMU_PORT})")
    parser.add_argument('--hand-port', type=str, default=None,
                       help="Serial port for Ability Hand (default: auto-detect)")
    
    args = parser.parse_args()
    
    # Parse sensor list
    sensors_to_log = None
    if args.sensors:
        sensors_to_log = [s.strip().lower() for s in args.sensors.split(',')]
    
    # Create logger
    logger_tool = SensorLogger(
        sensors_to_log=sensors_to_log,
        duration=args.duration,
        interval=args.interval,
        output_file=args.output,
        output_format=args.format,
        imu_port=args.imu_port,
        hand_port=args.hand_port
    )
    
    # Handle interrupts gracefully
    def signal_handler(sig, frame):
        print("\nLogging interrupted, finalizing...")
        logger_tool.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # Start logging
        logger_tool.start()
        
        # Display progress indicator
        start_time = time.time()
        end_time = start_time + args.duration
        
        while logger_tool.running and time.time() < end_time:
            # Calculate progress
            elapsed = time.time() - start_time
            progress = min(100, (elapsed / args.duration) * 100)
            remaining = max(0, args.duration - elapsed)
            
            # Update progress bar
            bar_width = 40
            filled_width = int(bar_width * progress / 100)
            bar = '█' * filled_width + '░' * (bar_width - filled_width)
            
            # Clear line and print progress
            sys.stdout.write(f"\r[{bar}] {progress:.1f}% | {elapsed:.1f}s elapsed | {remaining:.1f}s remaining | {logger_tool.sample_count} samples")
            sys.stdout.flush()
            
            # Sleep briefly
            time.sleep(0.2)
        
        # Wait for logging to complete
        logger_tool.wait_for_completion()
        
        # Final status
        print("\nLogging complete!")
        print(f"Output file: {logger_tool.output_file}")
        print(f"Total samples: {logger_tool.sample_count}")
        if logger_tool.start_time and logger_tool.end_time:
            actual_duration = (logger_tool.end_time - logger_tool.start_time).total_seconds()
            print(f"Duration: {actual_duration:.1f}s")
            if logger_tool.sample_count > 1:
                actual_interval = actual_duration / (logger_tool.sample_count - 1)
                print(f"Average interval: {actual_interval*1000:.2f}ms (target: {args.interval*1000:.2f}ms)")
        
    except KeyboardInterrupt:
        print("\nLogging interrupted by user")
        logger_tool.stop()
    
    except Exception as e:
        logger.error(f"Error in main function: {e}")
        logger_tool.stop()

if __name__ == "__main__":
    main()