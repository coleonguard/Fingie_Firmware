#!/usr/bin/env python3
"""
IMU Testing Tool.

This tool tests the connection to the Microstrain IMU and provides utilities
for streaming and visualizing IMU data.
"""

import os
import sys
import time
import argparse
import logging
import math
import threading
import signal
from datetime import datetime
import json

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("IMUTester")

# Add parent directory to path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Try to import IMUInterface
try:
    from imu.imu_interface import IMUInterface, IMUData, MSCL_AVAILABLE
except ImportError as e:
    logger.error(f"Failed to import IMUInterface: {e}")
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

# Constants
DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115200
DEFAULT_DURATION = 10  # seconds
DEFAULT_INTERVAL = 0.1  # seconds
DEFAULT_BUFFER_SIZE = 500  # for plot buffer

class IMUTester:
    """
    Tester for IMU hardware.
    
    This class provides utilities for testing IMU connection, streaming data,
    and displaying device information.
    """
    
    def __init__(
        self,
        port=DEFAULT_PORT,
        baud=DEFAULT_BAUD,
        raw_mode=False,
        info_only=False
    ):
        """
        Initialize the IMU tester.
        
        Args:
            port: Serial port for the IMU
            baud: Baud rate
            raw_mode: Whether to show raw data
            info_only: Only display device information
        """
        self.port = port
        self.baud = baud
        self.raw_mode = raw_mode
        self.info_only = info_only
        
        # Display configuration
        logger.info(f"IMU Tester Configuration:")
        logger.info(f"  Port:     {self.port}")
        logger.info(f"  Baud:     {self.baud}")
        logger.info(f"  Raw Mode: {self.raw_mode}")
        
        # Data for plotting
        self.timestamps = []
        self.roll_data = []
        self.pitch_data = []
        self.yaw_data = []
        self.accel_x_data = []
        self.accel_y_data = []
        self.accel_z_data = []
        self.gyro_x_data = []
        self.gyro_y_data = []
        self.gyro_z_data = []
        
        # Running state
        self.running = False
        self.thread = None
        self.last_data = None
        
        # Check dependencies
        if not MSCL_AVAILABLE:
            logger.warning("MSCL library not available. Using simulated IMU.")
            if not self.info_only:
                logger.warning("Install MSCL for full functionality: https://github.com/LORD-MicroStrain/MSCL")
        
        # Initialize IMU interface
        try:
            self.imu = IMUInterface(
                primary_port=self.port,
                check_orientation=False  # Don't enforce orientation during testing
            )
        except Exception as e:
            logger.error(f"Failed to initialize IMU: {e}")
            sys.exit(1)
    
    def get_device_info(self):
        """Get and display IMU device information"""
        if not MSCL_AVAILABLE:
            logger.info("Device information not available in simulated mode")
            return
        
        try:
            # Only use primary IMU
            if self.imu.primary_imu is None:
                logger.error("Primary IMU not available")
                return
            
            imu = self.imu.primary_imu
            
            # Device information
            info = {
                "Device Info": {
                    "Model Name": imu.modelName(),
                    "Model Number": imu.modelNumber(),
                    "Serial Number": imu.serialNumber(),
                    "Firmware Version": imu.firmwareVersion().str()
                },
                "Sensor Settings": {
                    "Active Channels": len(imu.getActiveChannelFields(imu.AHRS_IMU_NODE)),
                }
            }
            
            # Display info
            print("\n========== IMU Device Information ==========")
            for category, details in info.items():
                print(f"\n----- {category} -----")
                for key, value in details.items():
                    print(f"{key}: {value}")
            print("\n===========================================")
            
            return info
            
        except Exception as e:
            logger.error(f"Failed to get device information: {e}")
    
    def _update_data_buffers(self, data):
        """Update data buffers for plotting"""
        timestamp = time.time()
        
        # Add data to buffers
        self.timestamps.append(timestamp)
        self.roll_data.append(data.roll)
        self.pitch_data.append(data.pitch)
        self.yaw_data.append(data.yaw)
        self.accel_x_data.append(data.accel_x)
        self.accel_y_data.append(data.accel_y)
        self.accel_z_data.append(data.accel_z)
        self.gyro_x_data.append(data.gyro_x)
        self.gyro_y_data.append(data.gyro_y)
        self.gyro_z_data.append(data.gyro_z)
        
        # Limit buffer size
        if len(self.timestamps) > DEFAULT_BUFFER_SIZE:
            self.timestamps = self.timestamps[-DEFAULT_BUFFER_SIZE:]
            self.roll_data = self.roll_data[-DEFAULT_BUFFER_SIZE:]
            self.pitch_data = self.pitch_data[-DEFAULT_BUFFER_SIZE:]
            self.yaw_data = self.yaw_data[-DEFAULT_BUFFER_SIZE:]
            self.accel_x_data = self.accel_x_data[-DEFAULT_BUFFER_SIZE:]
            self.accel_y_data = self.accel_y_data[-DEFAULT_BUFFER_SIZE:]
            self.accel_z_data = self.accel_z_data[-DEFAULT_BUFFER_SIZE:]
            self.gyro_x_data = self.gyro_x_data[-DEFAULT_BUFFER_SIZE:]
            self.gyro_y_data = self.gyro_y_data[-DEFAULT_BUFFER_SIZE:]
            self.gyro_z_data = self.gyro_z_data[-DEFAULT_BUFFER_SIZE:]
        
        self.last_data = data
    
    def _format_data_display(self, data):
        """Format IMU data for display"""
        if self.raw_mode:
            # Raw data display
            return (
                f"Roll: {data.roll:7.2f}°  Pitch: {data.pitch:7.2f}°  Yaw: {data.yaw:7.2f}°  "
                f"Accel: X={data.accel_x:7.3f} Y={data.accel_y:7.3f} Z={data.accel_z:7.3f} m/s²  "
                f"Gyro: X={data.gyro_x:7.2f} Y={data.gyro_y:7.2f} Z={data.gyro_z:7.2f} °/s"
            )
        else:
            # Simplified display with indicators
            # Directions
            roll_dir = "←" if data.roll < 0 else "→"
            pitch_dir = "↓" if data.pitch < 0 else "↑"
            yaw_dir = "↶" if data.gyro_z < 0 else "↷"
            
            # Bar graphs (simple ASCII)
            roll_bar = self._get_bar(abs(data.roll), 0, 90, 10)
            pitch_bar = self._get_bar(abs(data.pitch), 0, 90, 10)
            
            # Motion indicators
            is_moving = data.gyro_magnitude > 10
            motion_str = "MOVING" if is_moving else "STATIC"
            
            # Check gravity direction for orientation
            gravity_dir = "Unknown"
            if data.accel_z < -7:
                gravity_dir = "Normal (Z down)"
            elif data.accel_z > 7:
                gravity_dir = "Inverted (Z up)"
            elif data.accel_x < -7:
                gravity_dir = "Sideways (X down)"
            elif data.accel_x > 7:
                gravity_dir = "Sideways (X up)"
            elif data.accel_y < -7:
                gravity_dir = "Sideways (Y down)"
            elif data.accel_y > 7:
                gravity_dir = "Sideways (Y up)"
            
            return (
                f"Orientation: {gravity_dir}  Motion: {motion_str}\n"
                f"Roll: {data.roll:6.1f}° {roll_dir} {roll_bar}\n"
                f"Pitch: {data.pitch:6.1f}° {pitch_dir} {pitch_bar}\n"
                f"Yaw: {data.yaw:6.1f}° {yaw_dir}"
            )
    
    def _get_bar(self, value, min_val, max_val, length):
        """Create a simple ASCII bar graph"""
        normalized = min(1.0, max(0.0, (value - min_val) / (max_val - min_val)))
        bar_length = int(normalized * length)
        return "[" + "#" * bar_length + " " * (length - bar_length) + "]"
    
    def _stream_thread(self, interval, duration, use_plot):
        """Thread for streaming IMU data"""
        self.imu.start()
        start_time = time.time()
        end_time = start_time + duration if duration > 0 else float('inf')
        
        try:
            # Create plot if requested
            if use_plot and PLOTTING_AVAILABLE:
                plt.ion()  # Interactive mode
                fig, axes = plt.subplots(3, 1, figsize=(10, 8))
                fig.suptitle("IMU Data Visualization")
                
                # Lines for plotting
                lines = {
                    'orientation': {
                        'roll': axes[0].plot([], [], 'r-', label='Roll')[0],
                        'pitch': axes[0].plot([], [], 'g-', label='Pitch')[0],
                        'yaw': axes[0].plot([], [], 'b-', label='Yaw')[0]
                    },
                    'accel': {
                        'x': axes[1].plot([], [], 'r-', label='X')[0],
                        'y': axes[1].plot([], [], 'g-', label='Y')[0],
                        'z': axes[1].plot([], [], 'b-', label='Z')[0]
                    },
                    'gyro': {
                        'x': axes[2].plot([], [], 'r-', label='X')[0],
                        'y': axes[2].plot([], [], 'g-', label='Y')[0],
                        'z': axes[2].plot([], [], 'b-', label='Z')[0]
                    }
                }
                
                # Set labels and legends
                axes[0].set_ylabel('Degrees')
                axes[0].set_title('Orientation')
                axes[0].legend()
                axes[0].grid(True)
                
                axes[1].set_ylabel('m/s²')
                axes[1].set_title('Acceleration')
                axes[1].legend()
                axes[1].grid(True)
                
                axes[2].set_xlabel('Time (s)')
                axes[2].set_ylabel('deg/s')
                axes[2].set_title('Angular Rate')
                axes[2].legend()
                axes[2].grid(True)
                
                plt.tight_layout()
                plt.subplots_adjust(top=0.9)
            
            # Main streaming loop
            last_plot_time = 0
            plot_interval = 0.1  # Update plot every 100ms to avoid slowdown
            
            while self.running and time.time() < end_time:
                # Get data
                data = self.imu.get_data()
                self._update_data_buffers(data)
                
                # Display data in terminal
                os.system('clear' if os.name == 'posix' else 'cls')
                print("\n===== IMU Data Stream =====")
                print(self._format_data_display(data))
                print("\nPress Ctrl+C to stop streaming")
                
                # Update plot if enough time has passed
                if use_plot and PLOTTING_AVAILABLE and time.time() - last_plot_time > plot_interval:
                    self._update_plot(lines, axes, fig)
                    last_plot_time = time.time()
                
                # Wait for next update
                time.sleep(interval)
                
        except Exception as e:
            logger.error(f"Error in streaming thread: {e}")
        
        finally:
            self.imu.stop()
            
            # Turn off interactive plotting
            if use_plot and PLOTTING_AVAILABLE:
                plt.ioff()
                plt.close('all')
    
    def _update_plot(self, lines, axes, fig):
        """Update the plot with new data"""
        if len(self.timestamps) < 2:
            return
        
        # Calculate relative timestamps
        rel_timestamps = [t - self.timestamps[0] for t in self.timestamps]
        
        # Update orientation plot
        lines['orientation']['roll'].set_data(rel_timestamps, self.roll_data)
        lines['orientation']['pitch'].set_data(rel_timestamps, self.pitch_data)
        lines['orientation']['yaw'].set_data(rel_timestamps, self.yaw_data)
        
        # Update acceleration plot
        lines['accel']['x'].set_data(rel_timestamps, self.accel_x_data)
        lines['accel']['y'].set_data(rel_timestamps, self.accel_y_data)
        lines['accel']['z'].set_data(rel_timestamps, self.accel_z_data)
        
        # Update gyro plot
        lines['gyro']['x'].set_data(rel_timestamps, self.gyro_x_data)
        lines['gyro']['y'].set_data(rel_timestamps, self.gyro_y_data)
        lines['gyro']['z'].set_data(rel_timestamps, self.gyro_z_data)
        
        # Adjust limits
        for i, ax in enumerate(axes):
            ax.relim()
            ax.autoscale_view()
            
            # Add a little padding to y-axis
            ymin, ymax = ax.get_ylim()
            pad = (ymax - ymin) * 0.1
            ax.set_ylim(ymin - pad, ymax + pad)
            
            # Set x-axis limits
            ax.set_xlim(0, rel_timestamps[-1])
        
        # Draw
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
    
    def save_data_to_file(self, filename=None):
        """Save the collected data to a file"""
        if not self.last_data:
            logger.warning("No data to save")
            return
        
        # Generate filename with timestamp if not provided
        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"imu_data_{timestamp}.json"
        
        # Create data object
        data_object = {
            "timestamp": datetime.now().isoformat(),
            "device_info": {
                "port": self.port,
                "simulated": not MSCL_AVAILABLE
            },
            "samples": {
                "timestamps": self.timestamps,
                "orientation": {
                    "roll": self.roll_data,
                    "pitch": self.pitch_data,
                    "yaw": self.yaw_data
                },
                "acceleration": {
                    "x": self.accel_x_data,
                    "y": self.accel_y_data,
                    "z": self.accel_z_data
                },
                "angular_rate": {
                    "x": self.gyro_x_data,
                    "y": self.gyro_y_data,
                    "z": self.gyro_z_data
                }
            }
        }
        
        # Write to file
        try:
            with open(filename, 'w') as f:
                json.dump(data_object, f, indent=2)
            logger.info(f"Data saved to {filename}")
            return filename
        except Exception as e:
            logger.error(f"Failed to save data: {e}")
            return None
    
    def start_streaming(self, interval=DEFAULT_INTERVAL, duration=DEFAULT_DURATION, use_plot=False):
        """
        Start streaming IMU data.
        
        Args:
            interval: Sampling interval in seconds
            duration: Duration in seconds (0 for infinite)
            use_plot: Whether to use real-time plotting
        """
        if self.running:
            logger.warning("Already streaming")
            return
        
        # Check plotting
        if use_plot and not PLOTTING_AVAILABLE:
            logger.warning("Plotting requested but matplotlib is not available. Continuing without plot.")
            use_plot = False
        
        # Start streaming
        self.running = True
        self.thread = threading.Thread(
            target=self._stream_thread,
            args=(interval, duration, use_plot)
        )
        self.thread.daemon = True
        self.thread.start()
        
        # Wait for thread to complete
        try:
            while self.running and self.thread.is_alive():
                time.sleep(0.1)
        except KeyboardInterrupt:
            logger.info("Streaming interrupted by user")
            self.stop_streaming()
            
            # Ask if user wants to save data
            if len(self.timestamps) > 0:
                try:
                    save = input("\nSave collected data? (y/n): ").strip().lower()
                    if save == 'y':
                        filename = input("Enter filename (leave empty for auto-generated): ").strip()
                        self.save_data_to_file(filename if filename else None)
                except:
                    pass
    
    def stop_streaming(self):
        """Stop streaming IMU data"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
            self.thread = None
        self.imu.stop()

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description="IMU Testing Tool")
    
    # Connection options
    parser.add_argument('--port', type=str, default=DEFAULT_PORT,
                        help=f"Serial port of IMU (default: {DEFAULT_PORT})")
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD,
                        help=f"Baud rate (default: {DEFAULT_BAUD})")
    
    # Mode options
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument('--info', action='store_true',
                           help="Display device information")
    mode_group.add_argument('--stream', action='store_true',
                           help="Stream IMU data")
    
    # Streaming options
    parser.add_argument('--duration', type=int, default=DEFAULT_DURATION,
                       help=f"Stream duration in seconds, 0 for infinite (default: {DEFAULT_DURATION})")
    parser.add_argument('--interval', type=float, default=DEFAULT_INTERVAL,
                       help=f"Sampling interval in seconds (default: {DEFAULT_INTERVAL})")
    parser.add_argument('--raw', action='store_true',
                       help="Show raw data")
    parser.add_argument('--plot', action='store_true',
                       help="Show real-time plot (requires matplotlib)")
    parser.add_argument('--save', type=str, default=None,
                       help="Save data to specified file after streaming")
    
    args = parser.parse_args()
    
    # Create tester
    tester = IMUTester(
        port=args.port,
        baud=args.baud,
        raw_mode=args.raw,
        info_only=args.info
    )
    
    # Handle interrupts gracefully
    def signal_handler(sig, frame):
        print("\nExiting...")
        tester.stop_streaming()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Run requested mode
    if args.info:
        tester.get_device_info()
    elif args.stream:
        try:
            tester.start_streaming(
                interval=args.interval,
                duration=args.duration,
                use_plot=args.plot
            )
            
            # Save data if requested
            if args.save and len(tester.timestamps) > 0:
                tester.save_data_to_file(args.save)
                
        except KeyboardInterrupt:
            print("\nStreaming interrupted")
            tester.stop_streaming()
    else:
        # Default: show info and start streaming
        tester.get_device_info()
        try:
            tester.start_streaming(
                interval=args.interval,
                duration=args.duration,
                use_plot=args.plot
            )
        except KeyboardInterrupt:
            print("\nStreaming interrupted")
            tester.stop_streaming()

if __name__ == "__main__":
    main()