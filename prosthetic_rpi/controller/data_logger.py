# Data Logger: Records sensor data, motor currents, and control phases

import os
import time
import json
import csv
import threading
import numpy as np
from datetime import datetime
from pathlib import Path

class DataLogger:
    """Records and saves all prosthetic hand data for analysis and replay
    
    This module handles:
    1. Timestamped recording of all sensor readings
    2. Logging of motor currents and control states
    3. Saving data to disk in multiple formats (CSV and JSON)
    4. Creating dataset directories with metadata
    """
    
    def __init__(self, base_dir="/home/pi/hand_data", log_rate=10):
        """Initialize the data logger
        
        Args:
            base_dir: Directory where log data will be stored
            log_rate: Logging frequency in Hz
        """
        self.base_dir = Path(base_dir)
        self.log_rate = log_rate
        self.log_interval = 1.0 / log_rate
        
        # Ensure base directory exists
        self.base_dir.mkdir(exist_ok=True, parents=True)
        
        # Create timestamped session directory
        self.session_time = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = self.base_dir / f"session_{self.session_time}"
        self.session_dir.mkdir(exist_ok=True)
        
        # Data storage
        self.data_buffer = []
        self.current_entry = {}
        
        # Thread control
        self.running = False
        self.thread = None
        
        # File handles
        self.csv_file = None
        self.csv_writer = None
        
        # Data structure
        self.columns = [
            "timestamp",
            # Sensor readings (raw and filtered)
            "Thumb1_raw", "Thumb1_filtered",
            "Index1_raw", "Index1_filtered",
            "Middle1_raw", "Middle1_filtered",
            "Ring1_raw", "Ring1_filtered",
            "Pinky1_raw", "Pinky1_filtered",
            # Motor currents
            "Thumb_current", "Index_current", "Middle_current", 
            "Ring_current", "Pinky_current",
            # Control modes
            "Thumb_mode", "Index_mode", "Middle_mode", 
            "Ring_mode", "Pinky_mode",
            # IMU data (back of hand)
            "back_imu_roll", "back_imu_pitch", "back_imu_yaw",
            # IMU data (wrist)
            "wrist_imu_roll", "wrist_imu_pitch", "wrist_imu_yaw"
        ]
    
    def _setup_files(self):
        """Set up log files for the session"""
        # Create CSV file
        csv_path = self.session_dir / "sensor_data.csv"
        self.csv_file = open(csv_path, 'w', newline='')
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.columns)
        self.csv_writer.writeheader()
        
        # Write session metadata
        metadata = {
            "session_time": self.session_time,
            "log_rate": self.log_rate,
            "columns": self.columns,
            "start_time": datetime.now().isoformat()
        }
        
        with open(self.session_dir / "metadata.json", 'w') as f:
            json.dump(metadata, f, indent=4)
    
    def update_sensor_data(self, sensor_name, raw_value, filtered_value):
        """Update the current log entry with sensor data
        
        Args:
            sensor_name: Name of the sensor (e.g., "Thumb1")
            raw_value: Raw sensor reading
            filtered_value: Filtered sensor reading
        """
        self.current_entry[f"{sensor_name}_raw"] = raw_value
        self.current_entry[f"{sensor_name}_filtered"] = filtered_value
    
    def update_motor_data(self, finger_name, current):
        """Update the current log entry with motor current data
        
        Args:
            finger_name: Name of the finger (e.g., "Thumb")
            current: Motor current in Amperes
        """
        self.current_entry[f"{finger_name}_current"] = current
    
    def update_control_mode(self, finger_name, mode):
        """Update the current log entry with control mode data
        
        Args:
            finger_name: Name of the finger (e.g., "Thumb")
            mode: Control mode (as string)
        """
        self.current_entry[f"{finger_name}_mode"] = mode
        
    def update_imu_data(self, location, roll, pitch, yaw):
        """Update the current log entry with IMU orientation data
        
        Args:
            location: IMU location ("back" or "wrist")
            roll: Roll angle in degrees
            pitch: Pitch angle in degrees
            yaw: Yaw angle in degrees
        """
        self.current_entry[f"{location}_imu_roll"] = roll
        self.current_entry[f"{location}_imu_pitch"] = pitch
        self.current_entry[f"{location}_imu_yaw"] = yaw
    
    def _logger_thread(self):
        """Background thread for logging data at a fixed rate"""
        next_log_time = time.time()
        
        while self.running:
            current_time = time.time()
            
            # Check if it's time for the next log entry
            if current_time >= next_log_time:
                # Add timestamp to current entry
                self.current_entry["timestamp"] = current_time
                
                # Write to CSV
                self.csv_writer.writerow(self.current_entry)
                self.csv_file.flush()  # Ensure data is written to disk
                
                # Add to buffer
                self.data_buffer.append(self.current_entry.copy())
                
                # Reset current entry (keep timestamp for reference)
                self.current_entry = {"timestamp": current_time}
                
                # Calculate next log time
                next_log_time = current_time + self.log_interval
            
            # Small sleep to prevent CPU hogging
            time.sleep(0.001)
    
    def start(self):
        """Start the data logger"""
        if not self.running:
            self._setup_files()
            self.running = True
            self.thread = threading.Thread(target=self._logger_thread)
            self.thread.daemon = True
            self.thread.start()
            print(f"Data logger started, saving to {self.session_dir}")
    
    def stop(self):
        """Stop the data logger and finalize files"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        
        # Close files
        if self.csv_file:
            self.csv_file.close()
        
        # Write complete dataset to JSON for easier analysis
        with open(self.session_dir / "complete_dataset.json", 'w') as f:
            json.dump(self.data_buffer, f, indent=2)
        
        print(f"Data logger stopped. {len(self.data_buffer)} records saved.")
    
    def save_snapshot(self, description="snapshot"):
        """Save a specific snapshot with additional metadata
        
        Args:
            description: Text description of this snapshot
        """
        if not self.running:
            print("Logger not running, cannot save snapshot")
            return
        
        # Create snapshot entry with current data and description
        snapshot = self.current_entry.copy()
        snapshot["description"] = description
        snapshot["timestamp"] = time.time()
        
        # Save to snapshots file
        snapshot_file = self.session_dir / "snapshots.json"
        
        # Load existing snapshots or create new list
        if snapshot_file.exists():
            with open(snapshot_file, 'r') as f:
                try:
                    snapshots = json.load(f)
                except json.JSONDecodeError:
                    snapshots = []
        else:
            snapshots = []
        
        # Add new snapshot and save
        snapshots.append(snapshot)
        with open(snapshot_file, 'w') as f:
            json.dump(snapshots, f, indent=2)
        
        print(f"Snapshot saved: {description}")