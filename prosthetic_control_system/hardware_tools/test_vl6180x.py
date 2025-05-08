#!/usr/bin/env python3
"""
VL6180X Proximity Sensor Tester

This tool tests VL6180X proximity sensors directly.
It is used to verify the functionality of VL6180X sensors
and measure distances.
"""

import sys
import time
import argparse
import logging
import threading
import signal
from typing import Dict, List, Optional, Tuple

try:
    import smbus2 as smbus
except ImportError:
    try:
        import smbus
    except ImportError:
        print("Error: smbus2 or smbus module is required.")
        print("Please install with: pip install smbus2")
        sys.exit(1)

try:
    import matplotlib.pyplot as plt
    import numpy as np
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("VL6180XTester")

class VL6180XTester:
    """Tester for VL6180X proximity sensors"""
    
    # VL6180X register addresses
    IDENTIFICATION_MODEL_ID = 0x00
    SYSTEM_FRESH_OUT_OF_RESET = 0x016
    SYSRANGE_START = 0x018
    RESULT_RANGE_VAL = 0x062
    RESULT_RANGE_STATUS = 0x04d
    RESULT_INTERRUPT_STATUS_GPIO = 0x04f
    
    def __init__(
        self, 
        bus_num: int = 1, 
        address: int = 0x29, 
        mux_address: Optional[int] = None, 
        channel: Optional[int] = None,
        verbose: bool = False
    ):
        """
        Initialize the VL6180X tester.
        
        Args:
            bus_num: I2C bus number (default: 1)
            address: I2C address of the sensor (default: 0x29)
            mux_address: I2C address of the multiplexer (optional)
            channel: Multiplexer channel (optional)
            verbose: Enable verbose output
        """
        self.bus_num = bus_num
        self.address = address
        self.mux_address = mux_address
        self.channel = channel
        self.verbose = verbose
        self.bus = None
        self.initialized = False
        
        # For continuous reading
        self.running = False
        self.thread = None
        
        # Statistics
        self.readings = []
        self.min_reading = 255
        self.max_reading = 0
        self.sum_readings = 0
        self.count_readings = 0
        self.invalid_readings = 0
        
        # Set up logging
        if verbose:
            logger.setLevel(logging.DEBUG)
        
        # Initialize the bus
        try:
            self.bus = smbus.SMBus(bus_num)
            logger.debug(f"Opened I2C bus {bus_num}")
        except (IOError, FileNotFoundError) as e:
            logger.error(f"Failed to open I2C bus {bus_num}: {e}")
            raise
    
    def _select_mux_channel(self) -> bool:
        """
        Select channel on multiplexer if using one.
        
        Returns:
            True if successful or not using multiplexer, False otherwise
        """
        if self.mux_address is None or self.channel is None:
            return True
            
        try:
            # Channel is selected by writing a byte with the corresponding bit set
            channel_byte = 1 << self.channel
            self.bus.write_byte(self.mux_address, channel_byte)
            logger.debug(f"Selected channel {self.channel} on multiplexer 0x{self.mux_address:02x}")
            return True
        except IOError as e:
            logger.error(f"Failed to select channel {self.channel} on multiplexer: {e}")
            return False
    
    def initialize(self) -> bool:
        """
        Initialize the VL6180X sensor.
        
        Returns:
            True if successful, False otherwise
        """
        # Select multiplexer channel if using one
        if not self._select_mux_channel():
            return False
            
        try:
            # Check if the device is present
            model_id = self.bus.read_byte_data(self.address, self.IDENTIFICATION_MODEL_ID)
            
            if model_id != 0xB4:  # VL6180X model ID
                logger.error(f"Device at 0x{self.address:02x} is not a VL6180X (model ID: 0x{model_id:02x}, expected: 0xB4)")
                return False
                
            logger.info(f"Found VL6180X sensor at address 0x{self.address:02x}")
            
            # In a real implementation, we would properly initialize all the registers
            # This is a simplified initialization
            
            # Check if the device needs full initialization
            try:
                fresh_out_of_reset = self.bus.read_byte_data(self.address, self.SYSTEM_FRESH_OUT_OF_RESET)
                
                if fresh_out_of_reset:
                    logger.info("Sensor is fresh out of reset, initializing registers...")
                    # Here we would write initialization values to many registers
                    # This is just a placeholder for the real initialization
                    # Reset the fresh out of reset bit
                    self.bus.write_byte_data(self.address, self.SYSTEM_FRESH_OUT_OF_RESET, 0x00)
            except IOError as e:
                logger.warning(f"Could not check reset status: {e}")
                
            self.initialized = True
            return True
            
        except IOError as e:
            logger.error(f"Failed to initialize sensor: {e}")
            return False
    
    def read_distance(self) -> Optional[int]:
        """
        Read a single distance measurement.
        
        Returns:
            Distance in mm or None if measurement fails
        """
        if not self.initialized and not self.initialize():
            return None
            
        # Select multiplexer channel if using one
        if not self._select_mux_channel():
            return None
            
        try:
            # Start single range measurement
            self.bus.write_byte_data(self.address, self.SYSRANGE_START, 0x01)
            
            # Wait for measurement to complete
            for _ in range(10):  # Timeout after 10 checks
                status = self.bus.read_byte_data(self.address, self.RESULT_INTERRUPT_STATUS_GPIO)
                if status & 0x04:  # Range interrupt
                    break
                time.sleep(0.01)
            else:
                logger.warning("Timeout waiting for range measurement")
                return None
                
            # Clear interrupt
            self.bus.write_byte_data(self.address, self.RESULT_INTERRUPT_STATUS_GPIO, 0x07)
            
            # Read result
            status = self.bus.read_byte_data(self.address, self.RESULT_RANGE_STATUS)
            distance = self.bus.read_byte_data(self.address, self.RESULT_RANGE_VAL)
            
            # Check status
            status = (status >> 4) & 0x0F
            if status != 0:
                error_msgs = {
                    1: "VCSEL Continuity Test",
                    2: "VCSEL Watchdog Test",
                    3: "VCSEL Watchdog",
                    4: "PLL1 Lock",
                    5: "PLL2 Lock",
                    6: "Early Convergence Estimate",
                    7: "Max Convergence",
                    8: "No Target Ignore",
                    9: "Max Signal To Noise Ratio",
                    10: "Raw Ranging Algo Underflow",
                    11: "Raw Ranging Algo Overflow",
                    12: "Ranging Algo Underflow",
                    13: "Ranging Algo Overflow",
                    14: "Filtered Measuring Overflow"
                }
                
                logger.warning(f"Range status error: {status} ({error_msgs.get(status, 'Unknown')})")
                self.invalid_readings += 1
                return None
                
            # Update statistics
            self.readings.append(distance)
            if len(self.readings) > 100:
                self.readings = self.readings[-100:]  # Keep only the last 100 readings
                
            self.min_reading = min(self.min_reading, distance)
            self.max_reading = max(self.max_reading, distance)
            self.sum_readings += distance
            self.count_readings += 1
            
            return distance
            
        except IOError as e:
            logger.error(f"Failed to read distance: {e}")
            self.invalid_readings += 1
            return None
    
    def continuous_reading(self, interval: float = 0.1):
        """
        Continuously read distance measurements.
        
        Args:
            interval: Time interval between readings in seconds
        """
        self.running = True
        
        while self.running:
            distance = self.read_distance()
            
            if distance is not None:
                logger.info(f"Distance: {distance} mm")
            else:
                logger.warning("Failed to read distance")
                
            # Calculate statistics periodically
            if self.count_readings % 10 == 0 and self.count_readings > 0:
                self._print_statistics()
                
            if not self.running:
                break
                
            time.sleep(interval)
    
    def start_continuous_reading(self, interval: float = 0.1):
        """
        Start continuous reading in a separate thread.
        
        Args:
            interval: Time interval between readings in seconds
        """
        if self.thread is not None and self.thread.is_alive():
            logger.warning("Continuous reading already running")
            return
            
        self.thread = threading.Thread(target=self.continuous_reading, args=(interval,))
        self.thread.daemon = True
        self.thread.start()
        
        logger.info("Continuous reading started. Press Ctrl+C to stop.")
    
    def stop_continuous_reading(self):
        """Stop continuous reading"""
        self.running = False
        
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None
            
        logger.info("Continuous reading stopped")
    
    def _print_statistics(self):
        """Print statistics about the readings"""
        if self.count_readings == 0:
            logger.info("No readings yet")
            return
            
        avg = self.sum_readings / self.count_readings
        
        if len(self.readings) >= 2:
            # Calculate standard deviation
            std_dev = np.std(self.readings)
        else:
            std_dev = 0
            
        logger.info("=== Statistics ===")
        logger.info(f"Count: {self.count_readings}")
        logger.info(f"Invalid: {self.invalid_readings}")
        logger.info(f"Min: {self.min_reading} mm")
        logger.info(f"Max: {self.max_reading} mm")
        logger.info(f"Avg: {avg:.1f} mm")
        logger.info(f"Std Dev: {std_dev:.1f} mm")
        logger.info("=================")
    
    def plot_histogram(self, bins: int = 10):
        """
        Plot a histogram of readings.
        
        Args:
            bins: Number of bins
        """
        if not MATPLOTLIB_AVAILABLE:
            logger.error("Matplotlib is not available. Cannot plot histogram.")
            return
            
        if not self.readings:
            logger.warning("No readings to plot")
            return
            
        plt.figure(figsize=(10, 6))
        plt.hist(self.readings, bins=bins, alpha=0.7, color='blue')
        plt.title('VL6180X Distance Readings')
        plt.xlabel('Distance (mm)')
        plt.ylabel('Frequency')
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()
    
    def close(self):
        """Close the I2C bus"""
        if self.running:
            self.stop_continuous_reading()
            
        if self.bus is not None:
            self.bus.close()
            logger.debug(f"Closed I2C bus {self.bus_num}")
            self.bus = None

def main():
    """Main function"""
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="VL6180X Proximity Sensor Tester")
    parser.add_argument("--bus", type=int, default=1, help="I2C bus number (default: 1)")
    parser.add_argument("--address", type=lambda x: int(x, 0), default=0x29, 
                      help="I2C address of sensor (default: 0x29)")
    parser.add_argument("--mux-address", type=lambda x: int(x, 0), default=None,
                      help="I2C address of multiplexer (default: None)")
    parser.add_argument("--channel", type=int, default=None,
                      help="Multiplexer channel (0-7)")
    parser.add_argument("--continuous", action="store_true",
                      help="Continuous reading mode")
    parser.add_argument("--samples", type=int, default=10,
                      help="Number of samples to take (default: 10)")
    parser.add_argument("--interval", type=float, default=0.1,
                      help="Sampling interval in seconds (default: 0.1)")
    parser.add_argument("--plot", action="store_true",
                      help="Plot histogram of readings (requires matplotlib)")
    parser.add_argument("--verbose", action="store_true",
                      help="Verbose output")
    
    args = parser.parse_args()
    
    # Register signal handler for clean shutdown
    def signal_handler(sig, frame):
        logger.info("Interrupted by user")
        if 'tester' in locals() and tester is not None:
            tester.close()
        sys.exit(0)
        
    signal.signal(signal.SIGINT, signal_handler)
    
    # Create tester
    try:
        tester = VL6180XTester(
            args.bus, args.address, args.mux_address, args.channel, args.verbose
        )
        
        if not tester.initialize():
            logger.error("Failed to initialize sensor")
            return 1
            
        if args.continuous:
            # Continuous reading mode
            tester.start_continuous_reading(interval=args.interval)
            
            try:
                # Wait for user to interrupt
                while tester.running:
                    time.sleep(0.1)
            except KeyboardInterrupt:
                logger.info("Interrupted by user")
                
            tester.stop_continuous_reading()
            
        else:
            # Single reading mode
            logger.info(f"Taking {args.samples} samples...")
            
            for i in range(args.samples):
                distance = tester.read_distance()
                
                if distance is not None:
                    logger.info(f"Sample {i+1}/{args.samples}: Distance = {distance} mm")
                else:
                    logger.warning(f"Sample {i+1}/{args.samples}: Failed to read distance")
                    
                if i < args.samples - 1:
                    time.sleep(args.interval)
            
            # Print statistics
            tester._print_statistics()
            
        # Plot histogram if requested
        if args.plot and MATPLOTLIB_AVAILABLE:
            tester.plot_histogram()
            
    except Exception as e:
        logger.error(f"Error: {e}")
        if 'tester' in locals() and tester is not None:
            tester.close()
        return 1
        
    finally:
        if 'tester' in locals() and tester is not None:
            tester.close()
    
    return 0

if __name__ == "__main__":
    sys.exit(main())