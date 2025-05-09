#!/usr/bin/env python3
"""
I2C Multiplexer Tester

This tool tests I2C multiplexers and scans for connected devices.
It is used to verify the hardware connectivity of I2C multiplexers
and the devices connected to them.
"""

import sys
import time
import argparse
import logging
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

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("I2CMuxTester")

class I2CMuxTester:
    """Tester for I2C multiplexers"""
    
    def __init__(self, bus_num: int = 1, mux_address: int = 0x70, verbose: bool = False):
        """
        Initialize the I2C multiplexer tester.
        
        Args:
            bus_num: I2C bus number (default: 1)
            mux_address: I2C address of the multiplexer (default: 0x70)
            verbose: Enable verbose output
        """
        self.bus_num = bus_num
        self.mux_address = mux_address
        self.verbose = verbose
        self.bus = None
        self.current_channel = None
        
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
        
        # Check if multiplexer is present
        try:
            # Reading from the device just to see if it exists
            # In a real implementation, we would verify it's a TCA9548
            self.bus.read_byte(self.mux_address)
            logger.debug(f"Found multiplexer at address 0x{self.mux_address:02x}")
        except IOError:
            logger.error(f"No device found at address 0x{self.mux_address:02x}")
            raise
    
    def select_channel(self, channel: int) -> bool:
        """
        Select a channel on the multiplexer.
        
        Args:
            channel: Channel number (0-7)
            
        Returns:
            True if successful, False otherwise
        """
        if not 0 <= channel <= 7:
            logger.error(f"Invalid channel: {channel}, must be 0-7")
            return False
            
        try:
            # Channel is selected by writing a byte with the corresponding bit set
            channel_byte = 1 << channel
            self.bus.write_byte(self.mux_address, channel_byte)
            self.current_channel = channel
            logger.debug(f"Selected channel {channel}")
            return True
        except IOError as e:
            logger.error(f"Failed to select channel {channel}: {e}")
            return False
    
    def disable_all_channels(self) -> bool:
        """
        Disable all channels on the multiplexer.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            self.bus.write_byte(self.mux_address, 0x00)
            self.current_channel = None
            logger.debug("Disabled all channels")
            return True
        except IOError as e:
            logger.error(f"Failed to disable channels: {e}")
            return False
    
    def scan_bus(self, start: int = 1, end: int = 127) -> List[int]:
        """
        Scan the I2C bus for devices.
        
        Args:
            start: Start address (default: 1)
            end: End address (default: 127)
            
        Returns:
            List of device addresses found
        """
        devices = []
        
        for addr in range(start, end + 1):
            # Skip the multiplexer's own address
            if addr == self.mux_address:
                continue
                
            try:
                self.bus.read_byte(addr)
                devices.append(addr)
                logger.debug(f"Found device at address 0x{addr:02x}")
            except IOError:
                pass
                
        return devices
    
    def scan_all_channels(self) -> Dict[int, List[int]]:
        """
        Scan all channels for devices.
        
        Returns:
            Dictionary mapping channels to lists of device addresses
        """
        results = {}
        
        # First disable all channels
        self.disable_all_channels()
        
        # Scan each channel
        for channel in range(8):
            if self.select_channel(channel):
                logger.info(f"Scanning channel {channel}...")
                devices = self.scan_bus()
                results[channel] = devices
                
                if devices:
                    logger.info(f"  Found {len(devices)} device(s) on channel {channel}:")
                    for addr in devices:
                        logger.info(f"    - Device at 0x{addr:02x}")
                else:
                    logger.info(f"  No devices found on channel {channel}")
            else:
                logger.warning(f"Skipping channel {channel} (could not select)")
                
        return results
    
    def test_channel(self, channel: int) -> bool:
        """
        Test a specific channel.
        
        Args:
            channel: Channel number (0-7)
            
        Returns:
            True if devices were found, False otherwise
        """
        if not self.select_channel(channel):
            return False
            
        devices = self.scan_bus()
        
        if devices:
            logger.info(f"Found {len(devices)} device(s) on channel {channel}:")
            for addr in devices:
                logger.info(f"  - Device at 0x{addr:02x}")
            return True
        else:
            logger.info(f"No devices found on channel {channel}")
            return False
    
    def close(self):
        """Close the I2C bus"""
        if self.bus is not None:
            self.bus.close()
            logger.debug(f"Closed I2C bus {self.bus_num}")
            self.bus = None

def test_vl6180x_device(tester: I2CMuxTester, channel: int = None, address: int = 0x29) -> bool:
    """
    Test if a VL6180X device is present and working.
    
    Args:
        tester: I2CMuxTester instance
        channel: Channel number (optional)
        address: Device address (default: 0x29)
        
    Returns:
        True if device is working, False otherwise
    """
    # Select channel if specified
    if channel is not None:
        if not tester.select_channel(channel):
            return False
    
    try:
        # Check if the device is present
        # VL6180X should have its model ID in register 0x00
        model_id = tester.bus.read_byte_data(address, 0x00)
        
        if model_id == 0xB4:  # VL6180X model ID
            logger.info(f"Found VL6180X sensor at address 0x{address:02x}")
            
            # Read a sample distance
            # In a real implementation, we would properly initialize the sensor first
            # This is just a basic test
            try:
                # Read from result register (range value)
                distance = tester.bus.read_byte_data(address, 0x50)
                logger.info(f"Distance reading: {distance} mm")
                return True
            except IOError as e:
                logger.error(f"Failed to read distance: {e}")
                return False
        else:
            logger.error(f"Device at 0x{address:02x} is not a VL6180X (model ID: 0x{model_id:02x}, expected: 0xB4)")
            return False
            
    except IOError as e:
        logger.error(f"Failed to communicate with device at 0x{address:02x}: {e}")
        return False

def main():
    """Main function"""
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="I2C Multiplexer Tester")
    parser.add_argument("--bus", type=int, default=1, help="I2C bus number (default: 1)")
    parser.add_argument("--address", type=lambda x: int(x, 0), default=0x70, 
                      help="I2C address of multiplexer (default: 0x70)")
    parser.add_argument("--scan", action="store_true", help="Scan all channels for devices")
    parser.add_argument("--channel", type=int, help="Select specific channel (0-7)")
    parser.add_argument("--check-mux", action="store_true", help="Check multiplexers on addresses 0x73 and 0x77")
    parser.add_argument("--verbose", action="store_true", help="Verbose output")
    
    args = parser.parse_args()
    
    # Create tester
    try:
        tester = I2CMuxTester(args.bus, args.address, args.verbose)
        
        if args.check_mux:
            # Check multiplexers at 0x73 and 0x77
            logger.info("Checking multiplexers at addresses 0x73 and 0x77...")
            mux_addresses = [0x73, 0x77]
            for mux_addr in mux_addresses:
                try:
                    mux_tester = I2CMuxTester(args.bus, mux_addr, args.verbose)
                    logger.info(f"Found multiplexer at address 0x{mux_addr:02x}")
                    # Scan all channels on this multiplexer
                    results = mux_tester.scan_all_channels()
                    total_devices = sum(len(devices) for devices in results.values())
                    logger.info(f"Scan complete. Found {total_devices} device(s) across all channels on mux 0x{mux_addr:02x}.")
                    mux_tester.close()
                except Exception as e:
                    logger.warning(f"No multiplexer found at address 0x{mux_addr:02x}: {e}")
            
        elif args.scan:
            # Scan all channels
            results = tester.scan_all_channels()
            
            # Print summary
            total_devices = sum(len(devices) for devices in results.values())
            logger.info(f"\nScan complete. Found {total_devices} device(s) across all channels.")
            
        elif args.channel is not None:
            # Test specific channel
            if not tester.test_channel(args.channel):
                logger.warning(f"No devices found on channel {args.channel}")
        else:
            # No specific action, just check if multiplexer is present
            logger.info(f"Multiplexer found at address 0x{args.address:02x}")
            logger.info("Use --scan to scan all channels, --channel to test a specific channel, or --check-mux to check multiplexers at 0x73 and 0x77")
            
    except (IOError, FileNotFoundError) as e:
        logger.error(f"Error: {e}")
        return 1
    finally:
        if 'tester' in locals() and tester is not None:
            tester.close()
    
    return 0

if __name__ == "__main__":
    sys.exit(main())