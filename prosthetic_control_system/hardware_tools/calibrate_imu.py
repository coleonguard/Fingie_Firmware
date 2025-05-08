#!/usr/bin/env python3
"""
IMU Calibration Tool.

This tool helps calibrate the Microstrain IMU by:
1. Measuring sensor orientation in reference positions
2. Computing calibration offsets
3. Saving calibration data for the IMU system
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
import statistics
import math

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("IMUCalibration")

# Add parent directory to path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Try to import IMU interface
try:
    from imu.imu_interface import IMUInterface, IMUData
except ImportError as e:
    logger.error(f"Failed to import IMU modules: {e}")
    logger.error("Make sure you are running from the project root directory")
    sys.exit(1)

# Check for plotting support
try:
    import matplotlib.pyplot as plt
    import numpy as np
    PLOTTING_AVAILABLE = True
except ImportError:
    PLOTTING_AVAILABLE = False
    logger.warning("Matplotlib not available. Install with 'pip install matplotlib' for plotting support.")

# Constants for calibration
DEFAULT_CALIBRATION_FILE = "imu_calibration.json"
DEFAULT_SAMPLES_PER_POSITION = 100
DEFAULT_STABILIZE_TIME = 3.0  # Seconds
GRAVITY = 9.81  # m/s²

# Reference orientations
REFERENCE_POSITIONS = {
    "upright": {
        "description": "Hand upright, palm facing forward",
        "expected_accel": [0.0, 0.0, -GRAVITY],
        "expected_roll": 0.0,
        "expected_pitch": 0.0,
        "expected_yaw": 0.0
    },
    "inverted": {
        "description": "Hand upside down, palm facing backward",
        "expected_accel": [0.0, 0.0, GRAVITY],
        "expected_roll": 180.0,
        "expected_pitch": 0.0,
        "expected_yaw": 180.0
    },
    "palm_down": {
        "description": "Hand horizontal, palm facing down",
        "expected_accel": [0.0, -GRAVITY, 0.0],
        "expected_roll": 0.0,
        "expected_pitch": 90.0,
        "expected_yaw": 0.0
    },
    "palm_up": {
        "description": "Hand horizontal, palm facing up",
        "expected_accel": [0.0, GRAVITY, 0.0],
        "expected_roll": 180.0,
        "expected_pitch": -90.0,
        "expected_yaw": 0.0
    }
}

class IMUCalibrator:
    """
    Tool for calibrating IMU sensors.
    
    This class provides utilities for calibrating the Microstrain IMU
    for accurate orientation measurements.
    """
    
    def __init__(
        self,
        port=None,
        secondary_port=None,
        sampling_rate=200,
        calibration_file=DEFAULT_CALIBRATION_FILE,
        auto_mode=False
    ):
        """
        Initialize the IMU calibrator.
        
        Args:
            port: Serial port for primary IMU
            secondary_port: Serial port for secondary IMU
            sampling_rate: IMU sampling rate in Hz
            calibration_file: Path to calibration file
            auto_mode: Whether to run in automatic mode
        """
        self.port = port
        self.secondary_port = secondary_port
        self.sampling_rate = sampling_rate
        self.calibration_file = calibration_file
        self.auto_mode = auto_mode
        
        # State
        self.running = False
        self.current_position = None  # Current reference position
        
        # Calibration data
        self.calibration_data = {
            "timestamp": None,
            "imu_info": {},
            "position_measurements": {},
            "offsets": {
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "accel_x": 0.0,
                "accel_y": 0.0,
                "accel_z": 0.0
            },
            "accelerometer_scale": 1.0,
            "gravity_reference": GRAVITY
        }
        
        # Sample data arrays
        self.samples = {
            "timestamp": [],
            "roll": [],
            "pitch": [],
            "yaw": [],
            "accel_x": [],
            "accel_y": [],
            "accel_z": [],
            "accel_magnitude": [],
            "gyro_x": [],
            "gyro_y": [],
            "gyro_z": [],
            "gyro_magnitude": []
        }
        
        # Initialize IMU interface
        logger.info("Initializing IMU...")
        try:
            self.imu = IMUInterface(
                sampling_rate=self.sampling_rate,
                primary_port=self.port,
                secondary_port=self.secondary_port,
                check_orientation=False  # Don't check orientation during calibration
            )
            logger.info("IMU interface initialized")
            
            # Get basic IMU info if possible
            if hasattr(self.imu, 'primary_imu') and self.imu.primary_imu:
                if hasattr(self.imu.primary_imu, 'modelName'):
                    model = self.imu.primary_imu.modelName()
                    self.calibration_data["imu_info"]["model"] = model
                    logger.info(f"IMU Model: {model}")
            
        except Exception as e:
            logger.error(f"Failed to initialize IMU: {e}")
            sys.exit(1)
    
    def start(self):
        """Start the IMU interface"""
        if not self.imu.running:
            self.imu.start()
            self.running = True
            logger.info("IMU interface started")
    
    def stop(self):
        """Stop the IMU interface"""
        if self.imu and self.imu.running:
            self.imu.stop()
            self.running = False
            logger.info("IMU interface stopped")
    
    def clear_samples(self):
        """Clear all collected samples"""
        for key in self.samples:
            self.samples[key] = []
    
    def collect_samples(self, position_name, num_samples=DEFAULT_SAMPLES_PER_POSITION, stabilize_time=DEFAULT_STABILIZE_TIME):
        """
        Collect IMU readings at a specific reference position.
        
        Args:
            position_name: Name of the reference position
            num_samples: Number of samples to collect
            stabilize_time: Time to wait for IMU to stabilize before sampling
        
        Returns:
            Dictionary of statistics for the collected samples
        """
        # Make sure the IMU is running
        if not self.running:
            self.start()
        
        # Check that the position is valid
        if position_name not in REFERENCE_POSITIONS:
            logger.error(f"Unknown reference position: {position_name}")
            return None
        
        # Save the current position for reference
        self.current_position = position_name
        position_info = REFERENCE_POSITIONS[position_name]
        
        # Clear previous samples
        self.clear_samples()
        
        try:
            # Wait for IMU to stabilize
            logger.info(f"Waiting {stabilize_time}s for IMU to stabilize in {position_name} position...")
            time.sleep(stabilize_time)
            
            # Collect samples
            logger.info(f"Collecting {num_samples} samples in {position_name} position...")
            for i in range(num_samples):
                # Read IMU data
                data = self.imu.get_data()
                
                # Store values
                self.samples["timestamp"].append(time.time())
                self.samples["roll"].append(data.roll)
                self.samples["pitch"].append(data.pitch)
                self.samples["yaw"].append(data.yaw)
                self.samples["accel_x"].append(data.accel_x)
                self.samples["accel_y"].append(data.accel_y)
                self.samples["accel_z"].append(data.accel_z)
                self.samples["accel_magnitude"].append(data.accel_magnitude)
                self.samples["gyro_x"].append(data.gyro_x)
                self.samples["gyro_y"].append(data.gyro_y)
                self.samples["gyro_z"].append(data.gyro_z)
                self.samples["gyro_magnitude"].append(data.gyro_magnitude)
                
                # Status update
                if (i + 1) % 20 == 0:
                    logger.info(f"Collected {i + 1}/{num_samples} samples")
                
                # Sleep for a short time
                time.sleep(1.0 / self.sampling_rate * 5)  # 5x slower than sampling rate
            
            # Calculate statistics
            stats = self._calculate_statistics(position_name)
            
            # Store data in calibration structure
            self.calibration_data["position_measurements"][position_name] = {
                "expected": position_info,
                "measured": {
                    "roll": stats["roll_mean"],
                    "pitch": stats["pitch_mean"],
                    "yaw": stats["yaw_mean"],
                    "accel_x": stats["accel_x_mean"],
                    "accel_y": stats["accel_y_mean"],
                    "accel_z": stats["accel_z_mean"],
                    "accel_magnitude": stats["accel_magnitude_mean"]
                },
                "std_dev": {
                    "roll": stats["roll_std"],
                    "pitch": stats["pitch_std"],
                    "yaw": stats["yaw_std"],
                    "accel_x": stats["accel_x_std"],
                    "accel_y": stats["accel_y_std"],
                    "accel_z": stats["accel_z_std"],
                    "accel_magnitude": stats["accel_magnitude_std"]
                }
            }
            
            logger.info(f"Completed sampling in {position_name} position")
            return stats
            
        except KeyboardInterrupt:
            logger.info("Sampling interrupted by user")
            return None
        except Exception as e:
            logger.error(f"Error during sampling: {e}")
            return None
    
    def _calculate_statistics(self, position_name):
        """
        Calculate statistics for collected samples.
        
        Args:
            position_name: Name of the reference position
            
        Returns:
            Dictionary of statistics
        """
        stats = {}
        
        # Ensure we have samples
        if not self.samples["roll"]:
            logger.error("No samples to calculate statistics from")
            return stats
        
        # Calculate statistics for each measurement
        for key in self.samples:
            if key == "timestamp":
                continue
                
            values = self.samples[key]
            mean_key = f"{key}_mean"
            std_key = f"{key}_std"
            error_key = f"{key}_error"
            
            if values:
                # Calculate mean and standard deviation
                stats[mean_key] = statistics.mean(values)
                stats[std_key] = statistics.stdev(values) if len(values) > 1 else 0
                
                # Calculate error compared to expected value
                if key in ["roll", "pitch", "yaw"]:
                    expected = REFERENCE_POSITIONS[position_name].get(f"expected_{key}", 0.0)
                    stats[error_key] = stats[mean_key] - expected
                elif key.startswith("accel_"):
                    axis = key[-1]  # x, y, or z
                    idx = "xyz".index(axis)
                    expected = REFERENCE_POSITIONS[position_name]["expected_accel"][idx]
                    stats[error_key] = stats[mean_key] - expected
        
        # Calculate overall stability
        # Lower is better - this tells us how stable the readings were
        gyro_stability = (stats.get("gyro_x_std", 0) + 
                         stats.get("gyro_y_std", 0) + 
                         stats.get("gyro_z_std", 0)) / 3.0
        
        accel_stability = (stats.get("accel_x_std", 0) + 
                          stats.get("accel_y_std", 0) + 
                          stats.get("accel_z_std", 0)) / 3.0
        
        stats["gyro_stability"] = gyro_stability
        stats["accel_stability"] = accel_stability
        stats["overall_stability"] = (gyro_stability + accel_stability * 10) / 2.0
        
        return stats
    
    def calculate_offsets(self):
        """
        Calculate calibration offsets from collected measurements.
        
        This method analyzes the collected position measurements to determine:
        1. Roll, pitch, and yaw offsets
        2. Accelerometer offsets
        3. Accelerometer scale factor
        
        Returns:
            Dictionary of calculated offsets
        """
        if not self.calibration_data["position_measurements"]:
            logger.error("No position measurements available to calculate offsets")
            return None
        
        # We need at least 2 positions for reasonable calibration
        if len(self.calibration_data["position_measurements"]) < 2:
            logger.warning("Only one position measured - calibration may be inaccurate")
        
        # Calculate offsets
        roll_offsets = []
        pitch_offsets = []
        yaw_offsets = []
        accel_x_offsets = []
        accel_y_offsets = []
        accel_z_offsets = []
        accel_scale_factors = []
        
        for pos_name, pos_data in self.calibration_data["position_measurements"].items():
            # Orientation offsets
            if "expected" in pos_data and "measured" in pos_data:
                expected = pos_data["expected"]
                measured = pos_data["measured"]
                
                # Calculate orientation offsets
                if "expected_roll" in expected:
                    roll_offsets.append(expected["expected_roll"] - measured["roll"])
                
                if "expected_pitch" in expected:
                    pitch_offsets.append(expected["expected_pitch"] - measured["pitch"])
                
                if "expected_yaw" in expected:
                    # Yaw is trickier due to wraparound - take smaller of two possible offsets
                    expected_yaw = expected["expected_yaw"]
                    measured_yaw = measured["yaw"]
                    
                    # Direct offset
                    direct_offset = expected_yaw - measured_yaw
                    
                    # Wraparound offset
                    if direct_offset > 180:
                        wraparound_offset = direct_offset - 360
                    elif direct_offset < -180:
                        wraparound_offset = direct_offset + 360
                    else:
                        wraparound_offset = direct_offset
                    
                    # Take the smaller offset
                    yaw_offset = wraparound_offset if abs(wraparound_offset) < abs(direct_offset) else direct_offset
                    yaw_offsets.append(yaw_offset)
                
                # Calculate accelerometer offsets
                if "expected_accel" in expected:
                    expected_accel = expected["expected_accel"]
                    
                    # X axis
                    accel_x_offsets.append(expected_accel[0] - measured["accel_x"])
                    
                    # Y axis
                    accel_y_offsets.append(expected_accel[1] - measured["accel_y"])
                    
                    # Z axis
                    accel_z_offsets.append(expected_accel[2] - measured["accel_z"])
                    
                    # Scale factor - ratio between expected and measured magnitude
                    expected_mag = math.sqrt(sum(x*x for x in expected_accel))
                    measured_mag = measured["accel_magnitude"]
                    
                    if measured_mag > 0:
                        accel_scale_factors.append(expected_mag / measured_mag)
        
        # Calculate average offsets
        offsets = {}
        
        if roll_offsets:
            offsets["roll"] = sum(roll_offsets) / len(roll_offsets)
        
        if pitch_offsets:
            offsets["pitch"] = sum(pitch_offsets) / len(pitch_offsets)
        
        if yaw_offsets:
            offsets["yaw"] = sum(yaw_offsets) / len(yaw_offsets)
        
        if accel_x_offsets:
            offsets["accel_x"] = sum(accel_x_offsets) / len(accel_x_offsets)
        
        if accel_y_offsets:
            offsets["accel_y"] = sum(accel_y_offsets) / len(accel_y_offsets)
        
        if accel_z_offsets:
            offsets["accel_z"] = sum(accel_z_offsets) / len(accel_z_offsets)
        
        if accel_scale_factors:
            offsets["accelerometer_scale"] = sum(accel_scale_factors) / len(accel_scale_factors)
        
        # Update calibration data
        self.calibration_data["offsets"] = {
            "roll": offsets.get("roll", 0.0),
            "pitch": offsets.get("pitch", 0.0),
            "yaw": offsets.get("yaw", 0.0),
            "accel_x": offsets.get("accel_x", 0.0),
            "accel_y": offsets.get("accel_y", 0.0),
            "accel_z": offsets.get("accel_z", 0.0)
        }
        self.calibration_data["accelerometer_scale"] = offsets.get("accelerometer_scale", 1.0)
        
        logger.info("Offset calculation complete")
        logger.info(f"Roll offset: {self.calibration_data['offsets']['roll']:.2f}°")
        logger.info(f"Pitch offset: {self.calibration_data['offsets']['pitch']:.2f}°")
        logger.info(f"Yaw offset: {self.calibration_data['offsets']['yaw']:.2f}°")
        logger.info(f"Accel X offset: {self.calibration_data['offsets']['accel_x']:.3f} m/s²")
        logger.info(f"Accel Y offset: {self.calibration_data['offsets']['accel_y']:.3f} m/s²")
        logger.info(f"Accel Z offset: {self.calibration_data['offsets']['accel_z']:.3f} m/s²")
        logger.info(f"Accelerometer scale: {self.calibration_data['accelerometer_scale']:.3f}")
        
        return offsets
    
    def save_calibration(self, filename=None):
        """
        Save calibration data to a file.
        
        Args:
            filename: Path to save file (uses default if None)
            
        Returns:
            Path to saved file
        """
        if not filename:
            filename = self.calibration_file
        
        # Update timestamp
        self.calibration_data["timestamp"] = datetime.now().isoformat()
        
        try:
            with open(filename, 'w') as f:
                json.dump(self.calibration_data, f, indent=2)
            logger.info(f"Calibration data saved to {filename}")
            return filename
        except Exception as e:
            logger.error(f"Failed to save calibration data: {e}")
            return None
    
    def load_calibration(self, filename=None):
        """
        Load calibration data from a file.
        
        Args:
            filename: Path to load file (uses default if None)
            
        Returns:
            Loaded calibration data or None if loading fails
        """
        if not filename:
            filename = self.calibration_file
        
        try:
            with open(filename, 'r') as f:
                self.calibration_data = json.load(f)
            logger.info(f"Calibration data loaded from {filename}")
            return self.calibration_data
        except Exception as e:
            logger.error(f"Failed to load calibration data: {e}")
            return None
    
    def plot_calibration_results(self):
        """
        Plot calibration results, including measured vs. expected values.
        
        This requires matplotlib to be installed.
        """
        if not PLOTTING_AVAILABLE:
            logger.error("Cannot plot results - matplotlib is not available")
            print("To plot results, install matplotlib with: pip install matplotlib")
            return
        
        if not self.calibration_data["position_measurements"]:
            logger.error("No calibration data available to plot")
            return
        
        # Create figure
        plt.figure(figsize=(12, 10))
        plt.suptitle("IMU Calibration Results", fontsize=16)
        
        # Create subplots
        ax1 = plt.subplot(2, 2, 1)  # Orientation
        ax2 = plt.subplot(2, 2, 2)  # Acceleration
        ax3 = plt.subplot(2, 1, 2)  # Position stability
        
        # Colors for different positions
        colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
        color_idx = 0
        
        # Organize data for plotting
        positions = []
        roll_expected = []
        roll_measured = []
        pitch_expected = []
        pitch_measured = []
        accel_x_expected = []
        accel_x_measured = []
        accel_y_expected = []
        accel_y_measured = []
        accel_z_expected = []
        accel_z_measured = []
        stabilities = []
        
        for pos_name, pos_data in self.calibration_data["position_measurements"].items():
            positions.append(pos_name)
            
            expected = pos_data["expected"]
            measured = pos_data["measured"]
            std_dev = pos_data["std_dev"]
            
            # Orientation data
            roll_expected.append(expected.get("expected_roll", 0))
            roll_measured.append(measured["roll"])
            
            pitch_expected.append(expected.get("expected_pitch", 0))
            pitch_measured.append(measured["pitch"])
            
            # Acceleration data
            expected_accel = expected.get("expected_accel", [0, 0, 0])
            accel_x_expected.append(expected_accel[0])
            accel_x_measured.append(measured["accel_x"])
            
            accel_y_expected.append(expected_accel[1])
            accel_y_measured.append(measured["accel_y"])
            
            accel_z_expected.append(expected_accel[2])
            accel_z_measured.append(measured["accel_z"])
            
            # Stability data (standard deviations)
            stability = (std_dev["roll"] + std_dev["pitch"] + 
                        std_dev["accel_x"] + std_dev["accel_y"] + std_dev["accel_z"]) / 5.0
            stabilities.append(stability)
            
            # Plot this position's data
            color = colors[color_idx % len(colors)]
            color_idx += 1
            
            # Add position to stability plot
            bar_pos = len(positions) - 1
            ax3.bar(bar_pos, stability, color=color, alpha=0.7, label=pos_name)
            
            # Add error bars to orientation plot
            ax1.errorbar(
                measured["roll"], measured["pitch"],
                xerr=std_dev["roll"], yerr=std_dev["pitch"],
                fmt='o', color=color, label=pos_name,
                capsize=5, markersize=8
            )
            
            # Add true position to orientation plot
            if "expected_roll" in expected and "expected_pitch" in expected:
                ax1.plot(
                    expected["expected_roll"], expected["expected_pitch"],
                    's', color=color, markersize=10, alpha=0.5
                )
                # Draw arrow from measured to expected
                ax1.annotate(
                    "", xy=(expected["expected_roll"], expected["expected_pitch"]),
                    xytext=(measured["roll"], measured["pitch"]),
                    arrowprops=dict(arrowstyle="->", color=color, alpha=0.5)
                )
        
        # Plot orientation data
        ax1.set_xlabel('Roll (degrees)')
        ax1.set_ylabel('Pitch (degrees)')
        ax1.set_title('Orientation Measurements')
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        
        # Plot acceleration data
        bar_width = 0.2
        bar_positions = np.arange(len(positions))
        
        ax2.bar(bar_positions - bar_width, accel_x_measured, bar_width, label='X Measured', alpha=0.7)
        ax2.bar(bar_positions, accel_y_measured, bar_width, label='Y Measured', alpha=0.7)
        ax2.bar(bar_positions + bar_width, accel_z_measured, bar_width, label='Z Measured', alpha=0.7)
        
        # Add expected values as horizontal lines for each position
        for i, pos in enumerate(positions):
            ax2.plot([i - bar_width*1.5, i - bar_width*0.5], [accel_x_expected[i], accel_x_expected[i]], 'k--', alpha=0.5)
            ax2.plot([i - bar_width*0.5, i + bar_width*0.5], [accel_y_expected[i], accel_y_expected[i]], 'k--', alpha=0.5)
            ax2.plot([i + bar_width*0.5, i + bar_width*1.5], [accel_z_expected[i], accel_z_expected[i]], 'k--', alpha=0.5)
        
        ax2.set_xticks(bar_positions)
        ax2.set_xticklabels(positions)
        ax2.set_ylabel('Acceleration (m/s²)')
        ax2.set_title('Acceleration Measurements')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Plot stability data
        ax3.set_xticks(np.arange(len(positions)))
        ax3.set_xticklabels(positions)
        ax3.set_ylabel('Stability (lower is better)')
        ax3.set_title('Position Stability (Standard Deviation)')
        ax3.grid(True, alpha=0.3)
        
        # Add calibration results as text
        offsets = self.calibration_data["offsets"]
        text = (
            f"Calibration Results:\n"
            f"Roll Offset: {offsets['roll']:.2f}°\n"
            f"Pitch Offset: {offsets['pitch']:.2f}°\n"
            f"Yaw Offset: {offsets['yaw']:.2f}°\n"
            f"Accel X Offset: {offsets['accel_x']:.3f} m/s²\n"
            f"Accel Y Offset: {offsets['accel_y']:.3f} m/s²\n"
            f"Accel Z Offset: {offsets['accel_z']:.3f} m/s²\n"
            f"Accel Scale: {self.calibration_data['accelerometer_scale']:.3f}"
        )
        plt.figtext(0.02, 0.02, text, fontsize=10, va='bottom')
        
        plt.tight_layout()
        plt.subplots_adjust(top=0.9, bottom=0.15, hspace=0.3)
        
        # Show plot
        plt.show()
        
    def run_automatic_calibration(self, positions=None):
        """
        Run an automatic calibration sequence.
        
        Args:
            positions: List of position names to calibrate (default: all positions)
            
        Returns:
            True if calibration was successful, False otherwise
        """
        if positions is None:
            positions = list(REFERENCE_POSITIONS.keys())
        
        try:
            # Start the IMU
            self.start()
            
            logger.info("Starting automatic calibration sequence")
            print("\n===== AUTOMATIC IMU CALIBRATION SEQUENCE =====")
            print("This will calibrate the IMU at multiple reference positions.")
            print("For each position, follow the instructions and hold the IMU")
            print("steady in the specified orientation.")
            print("\nCalibration positions: " + ", ".join(positions))
            
            # For each position, collect samples
            for position in positions:
                pos_info = REFERENCE_POSITIONS[position]
                
                print(f"\n---- Calibrating in {position} position ----")
                print(f"Description: {pos_info['description']}")
                input(f"Orient the IMU and press ENTER to continue...")
                
                stats = self.collect_samples(position)
                
                if not stats:
                    logger.error(f"Failed to collect samples in {position} position")
                    continue
                
                # Display results
                print(f"\nResults for {position} position:")
                print(f"  Roll: {stats['roll_mean']:.1f}° ± {stats['roll_std']:.1f}°")
                print(f"  Pitch: {stats['pitch_mean']:.1f}° ± {stats['pitch_std']:.1f}°")
                print(f"  Yaw: {stats['yaw_mean']:.1f}° ± {stats['yaw_std']:.1f}°")
                print(f"  Acceleration: X={stats['accel_x_mean']:.2f}, Y={stats['accel_y_mean']:.2f}, Z={stats['accel_z_mean']:.2f} m/s²")
                print(f"  Stability: {stats['overall_stability']:.2f} (lower is better)")
            
            # Calculate offsets
            offsets = self.calculate_offsets()
            
            if offsets:
                # Display offsets
                print("\n---- Calibrated Offsets ----")
                print(f"  Roll: {self.calibration_data['offsets']['roll']:.2f}°")
                print(f"  Pitch: {self.calibration_data['offsets']['pitch']:.2f}°")
                print(f"  Yaw: {self.calibration_data['offsets']['yaw']:.2f}°")
                print(f"  Accel X: {self.calibration_data['offsets']['accel_x']:.3f} m/s²")
                print(f"  Accel Y: {self.calibration_data['offsets']['accel_y']:.3f} m/s²")
                print(f"  Accel Z: {self.calibration_data['offsets']['accel_z']:.3f} m/s²")
                print(f"  Accel Scale: {self.calibration_data['accelerometer_scale']:.3f}")
            
            # Save calibration
            filename = self.save_calibration()
            
            if filename:
                print(f"\nCalibration saved to {filename}")
            
            # Plot results
            if PLOTTING_AVAILABLE:
                print("\nGenerating calibration plot...")
                self.plot_calibration_results()
            
            print("\nCalibration sequence complete!")
            return True
            
        except KeyboardInterrupt:
            print("\nCalibration interrupted by user")
            return False
        except Exception as e:
            logger.error(f"Error during automatic calibration: {e}")
            return False
        finally:
            # Stop the IMU
            self.stop()
    
    def run_manual_calibration(self):
        """
        Run a manual calibration sequence with interactive prompts.
        
        Returns:
            True if calibration was successful, False otherwise
        """
        try:
            # Start the IMU
            self.start()
            
            logger.info("Starting manual calibration sequence")
            print("\n===== MANUAL IMU CALIBRATION SEQUENCE =====")
            print("This will guide you through a manual calibration process.")
            print("You can collect data in various positions interactively.")
            
            # List of positions we've collected
            collected_positions = []
            
            while True:
                print("\n---- Manual Calibration Menu ----")
                print("1. Collect samples in a specific position")
                print("2. View current data")
                print("3. Calculate offsets")
                print("4. Save calibration")
                print("5. Load calibration")
                print("6. Plot results")
                print("0. Exit")
                
                choice = input("\nEnter your choice: ").strip()
                
                if choice == '1':
                    print("\nAvailable positions:")
                    for i, (pos_name, pos_info) in enumerate(REFERENCE_POSITIONS.items()):
                        print(f"{i+1}. {pos_name}: {pos_info['description']}")
                    
                    try:
                        pos_idx = int(input("\nSelect position (number): ").strip()) - 1
                        if 0 <= pos_idx < len(REFERENCE_POSITIONS):
                            position = list(REFERENCE_POSITIONS.keys())[pos_idx]
                            
                            print(f"\nSelected position: {position}")
                            print(f"Description: {REFERENCE_POSITIONS[position]['description']}")
                            input("Orient the IMU and press ENTER to continue...")
                            
                            stats = self.collect_samples(position)
                            
                            if stats:
                                collected_positions.append(position)
                                print(f"\nResults for {position} position:")
                                print(f"  Roll: {stats['roll_mean']:.1f}° ± {stats['roll_std']:.1f}°")
                                print(f"  Pitch: {stats['pitch_mean']:.1f}° ± {stats['pitch_std']:.1f}°")
                                print(f"  Yaw: {stats['yaw_mean']:.1f}° ± {stats['yaw_std']:.1f}°")
                                print(f"  Acceleration: X={stats['accel_x_mean']:.2f}, Y={stats['accel_y_mean']:.2f}, Z={stats['accel_z_mean']:.2f} m/s²")
                                print(f"  Stability: {stats['overall_stability']:.2f} (lower is better)")
                        else:
                            print("Invalid position number")
                    except ValueError:
                        print("Invalid input. Please enter a number.")
                
                elif choice == '2':
                    if not collected_positions:
                        print("No data collected yet.")
                    else:
                        print("\n---- Collected Data ----")
                        print(f"Positions: {collected_positions}")
                        
                        for position in collected_positions:
                            pos_data = self.calibration_data["position_measurements"].get(position, {})
                            if "measured" in pos_data:
                                measured = pos_data["measured"]
                                std_dev = pos_data["std_dev"]
                                
                                print(f"\n{position}:")
                                print(f"  Roll: {measured['roll']:.1f}° ± {std_dev['roll']:.1f}°")
                                print(f"  Pitch: {measured['pitch']:.1f}° ± {std_dev['pitch']:.1f}°")
                                print(f"  Yaw: {measured['yaw']:.1f}° ± {std_dev['yaw']:.1f}°")
                                print(f"  Accel: X={measured['accel_x']:.2f}, Y={measured['accel_y']:.2f}, Z={measured['accel_z']:.2f} m/s²")
                
                elif choice == '3':
                    print("Calculating offsets...")
                    offsets = self.calculate_offsets()
                    
                    if offsets:
                        print("\n---- Calibrated Offsets ----")
                        print(f"  Roll: {self.calibration_data['offsets']['roll']:.2f}°")
                        print(f"  Pitch: {self.calibration_data['offsets']['pitch']:.2f}°")
                        print(f"  Yaw: {self.calibration_data['offsets']['yaw']:.2f}°")
                        print(f"  Accel X: {self.calibration_data['offsets']['accel_x']:.3f} m/s²")
                        print(f"  Accel Y: {self.calibration_data['offsets']['accel_y']:.3f} m/s²")
                        print(f"  Accel Z: {self.calibration_data['offsets']['accel_z']:.3f} m/s²")
                        print(f"  Accel Scale: {self.calibration_data['accelerometer_scale']:.3f}")
                
                elif choice == '4':
                    filename = input(f"Enter filename (default: {self.calibration_file}): ").strip()
                    if not filename:
                        filename = self.calibration_file
                    
                    saved_file = self.save_calibration(filename)
                    if saved_file:
                        print(f"Calibration saved to {saved_file}")
                
                elif choice == '5':
                    filename = input(f"Enter filename (default: {self.calibration_file}): ").strip()
                    if not filename:
                        filename = self.calibration_file
                    
                    loaded_data = self.load_calibration(filename)
                    if loaded_data:
                        print(f"Calibration loaded from {filename}")
                
                elif choice == '6':
                    if not self.calibration_data["position_measurements"]:
                        print("No calibration data available to plot")
                    elif not PLOTTING_AVAILABLE:
                        print("Plotting is not available. Install matplotlib with: pip install matplotlib")
                    else:
                        print("Generating plot...")
                        self.plot_calibration_results()
                
                elif choice == '0':
                    print("Exiting manual calibration")
                    break
                
                else:
                    print("Invalid choice. Please try again.")
            
            return True
            
        except KeyboardInterrupt:
            print("\nCalibration interrupted by user")
            return False
        except Exception as e:
            logger.error(f"Error during manual calibration: {e}")
            return False
        finally:
            # Stop the IMU
            self.stop()

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description="IMU Calibration Tool")
    
    # Mode options
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument('--auto', action='store_true',
                           help="Run automatic calibration sequence")
    mode_group.add_argument('--manual', action='store_true',
                           help="Run manual calibration with interactive prompts")
    
    # Hardware options
    parser.add_argument('--port', type=str, default=None,
                       help="Serial port for primary IMU")
    parser.add_argument('--secondary-port', type=str, default=None,
                       help="Serial port for secondary IMU")
    parser.add_argument('--rate', type=int, default=200,
                       help="Sampling rate in Hz (default: 200)")
    
    # File options
    parser.add_argument('--file', type=str, default=DEFAULT_CALIBRATION_FILE,
                       help=f"Calibration file path (default: {DEFAULT_CALIBRATION_FILE})")
    parser.add_argument('--save', action='store_true',
                       help="Save calibration data after completion")
    parser.add_argument('--load', type=str,
                       help="Load existing calibration file")
    
    # Position options
    parser.add_argument('--positions', type=str,
                       help="Comma-separated list of positions to calibrate")
    
    args = parser.parse_args()
    
    # Create calibrator
    calibrator = IMUCalibrator(
        port=args.port,
        secondary_port=args.secondary_port,
        sampling_rate=args.rate,
        calibration_file=args.file,
        auto_mode=args.auto
    )
    
    # Load existing calibration if requested
    if args.load:
        calibrator.load_calibration(args.load)
    
    # Parse positions if provided
    positions = None
    if args.positions:
        positions = [pos.strip() for pos in args.positions.split(',')]
        # Validate positions
        for pos in positions:
            if pos not in REFERENCE_POSITIONS:
                logger.warning(f"Unknown position: {pos}")
        # Filter valid positions
        positions = [pos for pos in positions if pos in REFERENCE_POSITIONS]
        if positions:
            logger.info(f"Using custom positions: {positions}")
        else:
            positions = None
    
    # Handle interrupts gracefully
    def signal_handler(sig, frame):
        print("\nExiting...")
        calibrator.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # Run requested mode
        if args.auto:
            calibrator.run_automatic_calibration(positions)
        elif args.manual:
            calibrator.run_manual_calibration()
        else:
            # Default - run automatic mode
            print("No mode specified. Running automatic calibration.")
            calibrator.run_automatic_calibration(positions)
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    finally:
        calibrator.stop()
        
        # Save if requested
        if args.save and not (args.auto or args.manual):  # Don't double-save
            calibrator.save_calibration()

if __name__ == "__main__":
    main()