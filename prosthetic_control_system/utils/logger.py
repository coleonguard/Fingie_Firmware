#!/usr/bin/env python3
"""
Data Logger for Prosthetic Control System.

This module implements the data logging functionality for the prosthetic
control system, creating newline-delimited JSON (ND-JSON) log files.
"""

import os
import time
import json
import logging
from typing import Dict, List, Any, Optional
import threading
import datetime

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("DataLogger")

class DataLogger:
    """
    Data logger for the prosthetic control system.
    
    This class manages logging of all sensor, control, and state data
    to a newline-delimited JSON file for later analysis.
    """
    
    def __init__(self, 
                 log_dir: str = None,
                 log_rate: int = 20,  # Hz
                 buffer_size: int = 100,
                 auto_flush: bool = True):
        """
        Initialize the data logger.
        
        Args:
            log_dir: Directory to store log files (defaults to ~/prosthetic_logs)
            log_rate: Rate at which to write logs (Hz)
            buffer_size: Number of entries to buffer before writing to disk
            auto_flush: Whether to automatically flush the buffer on a timer
        """
        # Set up logging directory
        if log_dir is None:
            self.log_dir = os.path.expanduser("~/prosthetic_logs")
        else:
            self.log_dir = log_dir
            
        os.makedirs(self.log_dir, exist_ok=True)
        
        # Logging parameters
        self.log_rate = log_rate
        self.log_interval = 1.0 / log_rate
        self.buffer_size = buffer_size
        self.auto_flush = auto_flush
        
        # Create log file
        self.log_file = None
        self.log_filename = self._create_log_file()
        
        # Buffer for log entries
        self.buffer = []
        
        # Thread control for auto-flushing
        self.running = False
        self.thread = None
        
        # Status
        self.entry_count = 0
        self.last_write_time = time.time()
        
        # Schema for validation
        self.schema = self._get_schema()
        
        logger.info(f"DataLogger initialized, writing to {self.log_filename}")
    
    def _create_log_file(self) -> str:
        """
        Create a new log file with timestamp name.
        
        Returns:
            Path to the created log file
        """
        timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        filename = os.path.join(self.log_dir, f"prosthetic_log_{timestamp}.ndjson")
        
        try:
            self.log_file = open(filename, 'w')
            return filename
        except Exception as e:
            logger.error(f"Failed to create log file: {e}")
            return None
    
    def _get_schema(self) -> Dict:
        """
        Define the JSON schema for log entries.
        
        Returns:
            Dictionary with the schema definition
        """
        return {
            "type": "object",
            "required": ["timestamp", "proximity", "imu", "fingers", "hand_state"],
            "properties": {
                "timestamp": {"type": "number"},
                "proximity": {
                    "type": "object",
                    "properties": {
                        "raw": {"type": "object"},
                        "filtered": {"type": "object"},
                        "status": {"type": "object"}
                    }
                },
                "imu": {
                    "type": "object",
                    "properties": {
                        "orientation": {
                            "type": "object",
                            "properties": {
                                "roll": {"type": "number"},
                                "pitch": {"type": "number"},
                                "yaw": {"type": "number"}
                            }
                        },
                        "acceleration": {
                            "type": "object",
                            "properties": {
                                "x": {"type": "number"},
                                "y": {"type": "number"},
                                "z": {"type": "number"}
                            }
                        },
                        "angular_rate": {
                            "type": "object",
                            "properties": {
                                "x": {"type": "number"},
                                "y": {"type": "number"},
                                "z": {"type": "number"}
                            }
                        },
                        "motion_state": {"type": "string"}
                    }
                },
                "fingers": {
                    "type": "object",
                    "patternProperties": {
                        "^[A-Za-z0-9_]+$": {
                            "type": "object",
                            "properties": {
                                "state": {"type": "string"},
                                "position": {"type": "number"},
                                "velocity": {"type": "number"},
                                "current": {"type": "number"},
                                "target_position": {"type": "number"},
                                "target_torque": {"type": "number"}
                            }
                        }
                    }
                },
                "hand_state": {"type": "string"},
                "fault": {"type": "string", "optional": True}
            }
        }
    
    def _validate_entry(self, entry: Dict) -> bool:
        """
        Validate an entry against the schema.
        
        This is a simple validation that just checks for required top-level keys.
        For full schema validation, use the jsonschema package.
        
        Args:
            entry: Dictionary to validate
            
        Returns:
            True if valid, False otherwise
        """
        required_keys = self.schema["required"]
        for key in required_keys:
            if key not in entry:
                logger.warning(f"Missing required key in log entry: {key}")
                return False
        return True
    
    def _flush_buffer(self):
        """Flush the buffer to disk"""
        if not self.buffer:
            return
            
        if self.log_file is None:
            logger.error("Cannot flush buffer: log file is not open")
            return
            
        try:
            for entry in self.buffer:
                json_str = json.dumps(entry)
                self.log_file.write(json_str + "\n")
            
            self.log_file.flush()
            self.last_write_time = time.time()
            self.buffer = []
            
        except Exception as e:
            logger.error(f"Failed to flush buffer: {e}")
    
    def _auto_flush_thread(self):
        """Thread for auto-flushing the buffer"""
        while self.running:
            if len(self.buffer) > 0:
                time_since_write = time.time() - self.last_write_time
                if time_since_write >= self.log_interval * 10:  # Flush every ~10 cycles worth
                    self._flush_buffer()
            
            # Sleep for a while
            time.sleep(self.log_interval * 5)  # Check at half the rate of auto-flush
    
    def start(self):
        """Start the auto-flush thread if enabled"""
        if self.auto_flush and not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._auto_flush_thread)
            self.thread.daemon = True
            self.thread.start()
            logger.info("DataLogger auto-flush thread started")
    
    def stop(self):
        """Stop the auto-flush thread and close the log file"""
        # Stop the thread
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        
        # Flush any remaining entries
        self._flush_buffer()
        
        # Close the log file
        if self.log_file:
            self.log_file.close()
            self.log_file = None
            
        logger.info(f"DataLogger stopped, wrote {self.entry_count} entries")
    
    def log_data(self, data: Dict):
        """
        Log a data entry.
        
        Args:
            data: Dictionary with data to log
        """
        # Skip if no log file
        if self.log_file is None:
            return
            
        # Validate the entry
        if not self._validate_entry(data):
            logger.warning("Skipping invalid log entry")
            return
            
        # Add to buffer
        self.buffer.append(data)
        self.entry_count += 1
        
        # Flush if buffer reaches threshold
        if len(self.buffer) >= self.buffer_size:
            self._flush_buffer()
    
    def get_stats(self) -> Dict:
        """
        Get statistics about the logger.
        
        Returns:
            Dictionary with statistics
        """
        return {
            "log_file": self.log_filename,
            "entries": self.entry_count,
            "buffer_size": len(self.buffer),
            "running": self.running
        }

# Test code
if __name__ == "__main__":
    print("Testing data logger...")
    
    # Create logger
    data_logger = DataLogger(buffer_size=5)  # Small buffer for testing
    data_logger.start()
    
    try:
        # Generate some test entries
        for i in range(20):
            entry = {
                "timestamp": time.time(),
                "proximity": {
                    "raw": {"Thumb1": 30, "Index1": 25},
                    "filtered": {"Thumb1": 30.5, "Index1": 25.2},
                    "status": {"Thumb1": "OK", "Index1": "OK"}
                },
                "imu": {
                    "orientation": {"roll": 1.0, "pitch": 2.0, "yaw": 3.0},
                    "acceleration": {"x": 0.1, "y": 0.2, "z": 9.8},
                    "angular_rate": {"x": 0.5, "y": 0.6, "z": 0.7},
                    "motion_state": "STATIC"
                },
                "fingers": {
                    "Thumb": {
                        "state": "APPROACH",
                        "position": 0.0,
                        "velocity": 0.0,
                        "current": 0.0,
                        "target_position": 0.0,
                        "target_torque": 0.0
                    },
                    "Index": {
                        "state": "PROPORTIONAL",
                        "position": 30.0,
                        "velocity": 5.0,
                        "current": 0.1,
                        "target_position": 35.0,
                        "target_torque": 0.0
                    }
                },
                "hand_state": "REACH"
            }
            
            # Add fault in some entries
            if i % 5 == 0:
                entry["fault"] = "COMM_LOSS"
                
            data_logger.log_data(entry)
            print(f"Logged entry {i+1}")
            
            # Wait a bit
            time.sleep(0.05)
        
        # Print stats
        stats = data_logger.get_stats()
        print(f"\nLogger stats: {stats}")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        
    finally:
        data_logger.stop()
        print(f"Test complete. Log written to {data_logger.log_filename}")