#!/usr/bin/env python3
"""
Proximity Sensor Calibration Tool.

This tool helps calibrate proximity sensors by:
1. Measuring sensor offsets at known distances
2. Determining optimal thresholds (approach and contact)
3. Generating calibration data for the proximity sensor system
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
logger = logging.getLogger("SensorCalibration")

# Add parent directory to path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Try to import proximity manager
try:
    from proximity.proximity_manager import ProximityManager, DEFAULT_SENSORS, MUX1_ADDRESS, MUX2_ADDRESS
except ImportError as e:
    logger.error(f"Failed to import proximity modules: {e}")
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
DEFAULT_CALIBRATION_FILE = "sensor_calibration.json"
DEFAULT_SAMPLES_PER_DISTANCE = 50
DEFAULT_SETTLE_TIME = 2.0  # Seconds
DEFAULT_DISTANCES = [5, 10, 20, 30, 40, 60, 80]  # mm
DEFAULT_APPROACH_THRESHOLD = 40  # mm
DEFAULT_CONTACT_THRESHOLD = 5    # mm
DEFAULT_STABLE_THRESHOLD = 2.0   # mm (std deviation)

class SensorCalibrator:
    """
    Tool for calibrating proximity sensors.
    
    This class provides utilities for calibrating VL6180X sensors for
    accurate distance measurements and determining optimal thresholds.
    """
    
    def __init__(
        self,
        bus_num=1,
        sampling_rate=20,
        calibration_file=DEFAULT_CALIBRATION_FILE,
        auto_mode=False
    ):
        """
        Initialize the sensor calibrator.
        
        Args:
            bus_num: I2C bus number
            sampling_rate: Sensor sampling rate in Hz
            calibration_file: Path to calibration file
            auto_mode: Whether to run in automatic mode
        """
        self.bus_num = bus_num
        self.sampling_rate = sampling_rate
        self.calibration_file = calibration_file
        self.auto_mode = auto_mode
        
        # State
        self.running = False
        self.current_distance = None  # Current reference distance
        
        # Calibration data
        self.calibration_data = {
            "timestamp": None,
            "sensor_map": {},
            "sensor_thresholds": {},
            "distance_responses": {},
            "statistics": {},
            "global_thresholds": {
                "approach": DEFAULT_APPROACH_THRESHOLD,
                "contact": DEFAULT_CONTACT_THRESHOLD
            }
        }
        
        # Sensor data arrays for collecting samples
        self.raw_readings = {}  # Raw sensor readings
        self.filtered_readings = {}  # Filtered sensor readings
        
        # Sensor mapping information for config file
        self.sensor_map = {
            "Thumb": ["Thumb1", "Thumb2"],
            "Index": ["Index1", "Index2"],
            "Middle": ["Middle1", "Middle2"],
            "Ring": ["Ring1", "Ring2"],
            "Pinky": ["Pinky1", "Pinky2"]
        }
        
        # Initialize proximity manager
        logger.info("Initializing proximity sensors...")
        try:
            self.proximity_manager = ProximityManager(
                sampling_rate=self.sampling_rate,
                bus_num=self.bus_num
            )
            self.sensor_names = self.proximity_manager.sensor_names
            # Initialize data structures
            self._init_data_structures()
            logger.info(f"Found {len(self.sensor_names)} sensors")
        except Exception as e:
            logger.error(f"Failed to initialize proximity manager: {e}")
            sys.exit(1)
    
    def _init_data_structures(self):
        """Initialize data structures for all detected sensors"""
        # Initialize data structures for each sensor
        for sensor in self.sensor_names:
            self.raw_readings[sensor] = []
            self.filtered_readings[sensor] = []
            self.calibration_data["distance_responses"][sensor] = {}
            self.calibration_data["statistics"][sensor] = {}
            
            # Default threshold settings
            self.calibration_data["sensor_thresholds"][sensor] = {
                "approach": DEFAULT_APPROACH_THRESHOLD,
                "contact": DEFAULT_CONTACT_THRESHOLD
            }
        
        # Populate sensor map
        self.calibration_data["sensor_map"] = self.sensor_map.copy()
    
    def start(self):
        """Start the proximity manager"""
        if not self.proximity_manager.running:
            self.proximity_manager.start()
            self.running = True
            logger.info("Proximity manager started")
    
    def stop(self):
        """Stop the proximity manager"""
        if self.proximity_manager and self.proximity_manager.running:
            self.proximity_manager.stop()
            self.running = False
            logger.info("Proximity manager stopped")
    
    def clear_samples(self):
        """Clear all collected samples"""
        for sensor in self.sensor_names:
            self.raw_readings[sensor] = []
            self.filtered_readings[sensor] = []
    
    def collect_samples(self, distance, num_samples=DEFAULT_SAMPLES_PER_DISTANCE, settle_time=DEFAULT_SETTLE_TIME):
        """
        Collect sensor readings at a specific reference distance.
        
        Args:
            distance: Reference distance in mm
            num_samples: Number of samples to collect
            settle_time: Time to wait for sensors to settle before sampling
        
        Returns:
            Dictionary of statistics by sensor
        """
        # Make sure the proximity manager is running
        if not self.running:
            self.start()
        
        # Save the current distance for reference
        self.current_distance = distance
        
        # Clear previous samples
        self.clear_samples()
        
        try:
            # Wait for sensors to settle
            logger.info(f"Waiting {settle_time}s for sensors to settle at {distance}mm...")
            time.sleep(settle_time)
            
            # Collect samples
            logger.info(f"Collecting {num_samples} samples at {distance}mm...")
            for i in range(num_samples):
                # Read all sensors
                raw_values = self.proximity_manager.get_all_values(filtered=False)
                filtered_values = self.proximity_manager.get_all_values(filtered=True)
                
                # Store values
                for sensor in self.sensor_names:
                    if sensor in raw_values:
                        self.raw_readings[sensor].append(raw_values[sensor])
                    if sensor in filtered_values:
                        self.filtered_readings[sensor].append(filtered_values[sensor])
                
                # Status update
                if (i + 1) % 10 == 0:
                    logger.info(f"Collected {i + 1}/{num_samples} samples")
                
                # Sleep for a short time
                time.sleep(1.0 / self.sampling_rate)
            
            # Calculate statistics
            stats = self._calculate_statistics(distance)
            
            # Store data in calibration structure
            for sensor in self.sensor_names:
                if sensor in stats:
                    self.calibration_data["distance_responses"][sensor][str(distance)] = stats[sensor]
                    self.calibration_data["statistics"][sensor][str(distance)] = {
                        "raw_mean": stats[sensor]["raw_mean"],
                        "raw_std": stats[sensor]["raw_std"],
                        "filtered_mean": stats[sensor]["filtered_mean"],
                        "filtered_std": stats[sensor]["filtered_std"]
                    }
            
            logger.info(f"Completed sampling at {distance}mm")
            return stats
            
        except KeyboardInterrupt:
            logger.info("Sampling interrupted by user")
            return None
        except Exception as e:
            logger.error(f"Error during sampling: {e}")
            return None
    
    def _calculate_statistics(self, distance):
        """
        Calculate statistics for collected samples.
        
        Args:
            distance: Reference distance in mm
            
        Returns:
            Dictionary of statistics by sensor
        """
        stats = {}
        
        for sensor in self.sensor_names:
            raw_values = self.raw_readings[sensor]
            filtered_values = self.filtered_readings[sensor]
            
            if len(raw_values) > 0 and len(filtered_values) > 0:
                # Calculate raw statistics
                raw_mean = statistics.mean(raw_values)
                raw_std = statistics.stdev(raw_values) if len(raw_values) > 1 else 0
                
                # Calculate filtered statistics
                filtered_mean = statistics.mean(filtered_values)
                filtered_std = statistics.stdev(filtered_values) if len(filtered_values) > 1 else 0
                
                # Calculate error metrics
                raw_error = raw_mean - distance
                filtered_error = filtered_mean - distance
                
                # Store statistics
                stats[sensor] = {
                    "distance": distance,
                    "raw_mean": raw_mean,
                    "raw_std": raw_std,
                    "filtered_mean": filtered_mean,
                    "filtered_std": filtered_std,
                    "raw_error": raw_error,
                    "filtered_error": filtered_error,
                    "raw_values": raw_values,
                    "filtered_values": filtered_values
                }
        
        return stats
    
    def determine_thresholds(self):
        """
        Determine optimal thresholds based on collected calibration data.
        
        This method analyzes the collected calibration data to determine:
        1. Approach thresholds for each sensor
        2. Contact thresholds for each sensor
        3. Global thresholds for the system
        
        Returns:
            Dictionary of thresholds by sensor
        """
        if not self.calibration_data["statistics"]:
            logger.error("No calibration data available to determine thresholds")
            return None
        
        # Thresholds for each sensor
        sensor_thresholds = {}
        all_approach_thresholds = []
        all_contact_thresholds = []
        
        for sensor in self.sensor_names:
            sensor_stats = self.calibration_data["statistics"].get(sensor, {})
            
            # We need enough distance points to determine thresholds
            if len(sensor_stats) < 3:
                logger.warning(f"Not enough data points for sensor {sensor} to determine thresholds")
                # Use defaults
                sensor_thresholds[sensor] = {
                    "approach": DEFAULT_APPROACH_THRESHOLD,
                    "contact": DEFAULT_CONTACT_THRESHOLD
                }
                continue
            
            # Extract distance and measurement data
            distances = []
            measurements = []
            
            for dist_str, stats in sensor_stats.items():
                try:
                    distance = float(dist_str)
                    # Use filtered mean as the measurement
                    measurement = stats["filtered_mean"]
                    
                    distances.append(distance)
                    measurements.append(measurement)
                except (ValueError, KeyError):
                    continue
            
            if len(distances) < 3:
                logger.warning(f"Not enough valid data points for sensor {sensor}")
                # Use defaults
                sensor_thresholds[sensor] = {
                    "approach": DEFAULT_APPROACH_THRESHOLD,
                    "contact": DEFAULT_CONTACT_THRESHOLD
                }
                continue
            
            # Sort data by distance
            dist_meas = sorted(zip(distances, measurements))
            distances = [d for d, _ in dist_meas]
            measurements = [m for _, m in dist_meas]
            
            # Find contact threshold - use largest measurement for smallest distance
            # (with a small buffer for reliability)
            smallest_dist = distances[0]
            smallest_meas = measurements[0]
            contact_threshold = smallest_meas + 2.0  # Add 2mm buffer
            
            # Find approach threshold - look for distance with good detection range
            # (typically around 30-50mm for VL6180X)
            approach_idx = -1
            for i, dist in enumerate(distances):
                if dist >= 30 and dist <= 50:
                    approach_idx = i
                    break
            
            # If no ideal distance found, use the middle point
            if approach_idx == -1:
                approach_idx = len(distances) // 2
            
            approach_threshold = measurements[approach_idx]
            
            # Store thresholds
            sensor_thresholds[sensor] = {
                "approach": approach_threshold,
                "contact": contact_threshold
            }
            
            # Collect for global average
            all_approach_thresholds.append(approach_threshold)
            all_contact_thresholds.append(contact_threshold)
        
        # Calculate global thresholds (average of all sensors)
        if all_approach_thresholds and all_contact_thresholds:
            global_approach = sum(all_approach_thresholds) / len(all_approach_thresholds)
            global_contact = sum(all_contact_thresholds) / len(all_contact_thresholds)
            
            self.calibration_data["global_thresholds"] = {
                "approach": global_approach,
                "contact": global_contact
            }
        
        # Update calibration data
        self.calibration_data["sensor_thresholds"] = sensor_thresholds
        
        logger.info("Threshold determination complete")
        logger.info(f"Global approach threshold: {self.calibration_data['global_thresholds']['approach']:.1f}mm")
        logger.info(f"Global contact threshold: {self.calibration_data['global_thresholds']['contact']:.1f}mm")
        
        return sensor_thresholds
    
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
        Plot calibration results, including sensor responses and thresholds.
        
        This requires matplotlib to be installed.
        """
        if not PLOTTING_AVAILABLE:
            logger.error("Cannot plot results - matplotlib is not available")
            print("To plot results, install matplotlib with: pip install matplotlib")
            return
        
        if not self.calibration_data["statistics"]:
            logger.error("No calibration data available to plot")
            return
        
        # Create figure
        plt.figure(figsize=(12, 8))
        
        # Plot distance responses for each sensor
        for i, sensor in enumerate(self.sensor_names):
            stats = self.calibration_data["statistics"].get(sensor, {})
            
            # Extract data
            distances = []
            raw_means = []
            filtered_means = []
            raw_stds = []
            filtered_stds = []
            
            for dist_str, dist_stats in stats.items():
                try:
                    distance = float(dist_str)
                    
                    distances.append(distance)
                    raw_means.append(dist_stats["raw_mean"])
                    filtered_means.append(dist_stats["filtered_mean"])
                    raw_stds.append(dist_stats["raw_std"])
                    filtered_stds.append(dist_stats["filtered_std"])
                except (ValueError, KeyError):
                    continue
            
            if not distances:
                continue
            
            # Sort by distance
            points = sorted(zip(distances, raw_means, filtered_means, raw_stds, filtered_stds))
            distances = [p[0] for p in points]
            raw_means = [p[1] for p in points]
            filtered_means = [p[2] for p in points]
            raw_stds = [p[3] for p in points]
            filtered_stds = [p[4] for p in points]
            
            # Plot filtered values with error bars
            plt.errorbar(
                distances, filtered_means, yerr=filtered_stds,
                fmt='o-', label=sensor,
                alpha=0.7, capsize=5
            )
        
        # Add ideal response line (y=x)
        min_dist = min(DEFAULT_DISTANCES)
        max_dist = max(DEFAULT_DISTANCES)
        plt.plot([min_dist, max_dist], [min_dist, max_dist], 'k--', alpha=0.5, label='Ideal')
        
        # Add global threshold lines
        approach = self.calibration_data["global_thresholds"]["approach"]
        contact = self.calibration_data["global_thresholds"]["contact"]
        
        plt.axhline(y=approach, color='r', linestyle='--', alpha=0.5)
        plt.axhline(y=contact, color='g', linestyle='--', alpha=0.5)
        
        plt.text(max_dist*0.8, approach*1.05, f'Approach ({approach:.1f}mm)', color='r')
        plt.text(max_dist*0.8, contact*1.1, f'Contact ({contact:.1f}mm)', color='g')
        
        # Add labels and legend
        plt.xlabel('Reference Distance (mm)')
        plt.ylabel('Measured Distance (mm)')
        plt.title('Sensor Calibration Results')
        plt.grid(True, alpha=0.3)
        plt.legend(loc='upper left')
        
        # Equal aspect ratio
        plt.axis('equal')
        plt.tight_layout()
        
        # Show plot
        plt.show()
    
    def run_automatic_calibration(self, distances=DEFAULT_DISTANCES):
        """
        Run an automatic calibration sequence.
        
        Args:
            distances: List of distances to calibrate at
            
        Returns:
            True if calibration was successful, False otherwise
        """
        try:
            # Start the proximity manager
            self.start()
            
            logger.info("Starting automatic calibration sequence")
            print("\n===== AUTOMATIC CALIBRATION SEQUENCE =====")
            print("This will calibrate the sensors at multiple distances.")
            print("For each distance, position an object in front of all sensors")
            print("at the specified distance and press ENTER when ready.")
            print("\nCalibration distances: " + ", ".join(f"{d}mm" for d in distances))
            
            # For each distance, collect samples
            for distance in distances:
                print(f"\n---- Calibrating at {distance}mm ----")
                input(f"Position an object at {distance}mm and press ENTER to continue...")
                
                stats = self.collect_samples(distance)
                
                if not stats:
                    logger.error(f"Failed to collect samples at {distance}mm")
                    continue
                
                # Display results
                print(f"\nResults for {distance}mm:")
                for sensor, sensor_stats in stats.items():
                    print(f"  {sensor}: Raw={sensor_stats['raw_mean']:.1f}±{sensor_stats['raw_std']:.1f}mm, "
                          f"Filtered={sensor_stats['filtered_mean']:.1f}±{sensor_stats['filtered_std']:.1f}mm")
            
            # Determine thresholds
            thresholds = self.determine_thresholds()
            
            if thresholds:
                # Display thresholds
                print("\n---- Calibrated Thresholds ----")
                for sensor, sensor_thresholds in thresholds.items():
                    print(f"  {sensor}: Approach={sensor_thresholds['approach']:.1f}mm, "
                          f"Contact={sensor_thresholds['contact']:.1f}mm")
            
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
            # Stop the proximity manager
            self.stop()
    
    def run_manual_calibration(self):
        """
        Run a manual calibration sequence with interactive prompts.
        
        Returns:
            True if calibration was successful, False otherwise
        """
        try:
            # Start the proximity manager
            self.start()
            
            logger.info("Starting manual calibration sequence")
            print("\n===== MANUAL CALIBRATION SEQUENCE =====")
            print("This will guide you through a manual calibration process.")
            print("You can specify distances and collect samples interactively.")
            
            # List of distances we've collected
            collected_distances = []
            
            while True:
                print("\n---- Manual Calibration Menu ----")
                print("1. Collect samples at a specific distance")
                print("2. View current data")
                print("3. Determine thresholds")
                print("4. Save calibration")
                print("5. Load calibration")
                print("6. Plot results")
                print("0. Exit")
                
                choice = input("\nEnter your choice: ").strip()
                
                if choice == '1':
                    try:
                        distance = float(input("Enter distance in mm: ").strip())
                        print(f"Collecting samples at {distance}mm...")
                        stats = self.collect_samples(distance)
                        
                        if stats:
                            collected_distances.append(distance)
                            print(f"\nResults for {distance}mm:")
                            for sensor, sensor_stats in stats.items():
                                print(f"  {sensor}: Raw={sensor_stats['raw_mean']:.1f}±{sensor_stats['raw_std']:.1f}mm, "
                                      f"Filtered={sensor_stats['filtered_mean']:.1f}±{sensor_stats['filtered_std']:.1f}mm")
                    except ValueError:
                        print("Invalid distance. Please enter a number.")
                
                elif choice == '2':
                    if not collected_distances:
                        print("No data collected yet.")
                    else:
                        print("\n---- Collected Data ----")
                        print(f"Distances: {collected_distances}")
                        print("\nSensor statistics:")
                        for sensor in self.sensor_names:
                            print(f"\n{sensor}:")
                            for dist in collected_distances:
                                stats = self.calibration_data["statistics"].get(sensor, {}).get(str(dist), {})
                                if stats:
                                    print(f"  {dist}mm: Raw={stats['raw_mean']:.1f}±{stats['raw_std']:.1f}mm, "
                                          f"Filtered={stats['filtered_mean']:.1f}±{stats['filtered_std']:.1f}mm")
                
                elif choice == '3':
                    print("Determining thresholds...")
                    thresholds = self.determine_thresholds()
                    
                    if thresholds:
                        print("\n---- Calibrated Thresholds ----")
                        print(f"Global: Approach={self.calibration_data['global_thresholds']['approach']:.1f}mm, "
                              f"Contact={self.calibration_data['global_thresholds']['contact']:.1f}mm")
                        print("\nPer-sensor thresholds:")
                        for sensor, sensor_thresholds in thresholds.items():
                            print(f"  {sensor}: Approach={sensor_thresholds['approach']:.1f}mm, "
                                  f"Contact={sensor_thresholds['contact']:.1f}mm")
                
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
                    if not self.calibration_data["statistics"]:
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
            # Stop the proximity manager
            self.stop()

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description="Proximity Sensor Calibration Tool")
    
    # Mode options
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument('--auto', action='store_true',
                           help="Run automatic calibration sequence")
    mode_group.add_argument('--manual', action='store_true',
                           help="Run manual calibration with interactive prompts")
    
    # Hardware options
    parser.add_argument('--bus', type=int, default=1,
                       help="I2C bus number (default: 1)")
    parser.add_argument('--rate', type=int, default=20,
                       help="Sampling rate in Hz (default: 20)")
    
    # File options
    parser.add_argument('--file', type=str, default=DEFAULT_CALIBRATION_FILE,
                       help=f"Calibration file path (default: {DEFAULT_CALIBRATION_FILE})")
    parser.add_argument('--save', action='store_true',
                       help="Save calibration data after completion")
    parser.add_argument('--load', type=str,
                       help="Load existing calibration file")
    
    # Distance options
    parser.add_argument('--distances', type=str,
                       help="Comma-separated list of calibration distances in mm")
    
    args = parser.parse_args()
    
    # Create calibrator
    calibrator = SensorCalibrator(
        bus_num=args.bus,
        sampling_rate=args.rate,
        calibration_file=args.file,
        auto_mode=args.auto
    )
    
    # Load existing calibration if requested
    if args.load:
        calibrator.load_calibration(args.load)
    
    # Parse distances if provided
    distances = DEFAULT_DISTANCES
    if args.distances:
        try:
            distances = [float(d.strip()) for d in args.distances.split(',')]
            logger.info(f"Using custom distances: {distances}")
        except ValueError:
            logger.warning(f"Invalid distance format: {args.distances}. Using defaults.")
    
    # Handle interrupts gracefully
    def signal_handler(sig, frame):
        print("\nExiting...")
        calibrator.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # Run requested mode
        if args.auto:
            calibrator.run_automatic_calibration(distances)
        elif args.manual:
            calibrator.run_manual_calibration()
        else:
            # Default - run automatic mode
            print("No mode specified. Running automatic calibration.")
            calibrator.run_automatic_calibration(distances)
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    finally:
        calibrator.stop()
        
        # Save if requested
        if args.save and not (args.auto or args.manual):  # Don't double-save
            calibrator.save_calibration()

if __name__ == "__main__":
    main()