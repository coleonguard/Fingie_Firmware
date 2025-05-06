#!/usr/bin/env python3
"""
Test script to monitor all VL6180X proximity sensors simultaneously.
This script follows best practices for multiplexer handling and sensor initialization.

Channel Mapping:
- MULT1 (I²C Address = 0x70)
  Channel 0 → Thumb1 (SDA-Thumb1, SCL-Thumb1)
  Channel 1 → Thumb2 (SDA-Thumb2, SCL-Thumb2)
  Channel 2 → Index1 (SDA-Index1, SCL-Index1)
  Channel 3 → Index2 (SDA-Index2, SCL-Index2)
  Channel 4 → Middle1 (SDA-Middle, SCL-Middle)
- MULT2 (I²C Address = 0x71)
  Channel 0 → Middle2 (SDA-Middle2, SCL-Middle2)
  Channel 1 → Ring1 (SDA-Ring1, SCL-Ring1)
  Channel 2 → Ring2 (SDA-Ring2, SCL-Ring2)
  Channel 3 → Pinky1 (SDA-Pinky1, SCL-Pinky1)
  Channel 4 → Pinky2 (SDA-Pinky2, SCL-Pinky2)
"""

import smbus2
import time
from collections import OrderedDict

# ------------------------------------------------------------------------
# Constants
# ------------------------------------------------------------------------
VL6180X_ADDRESS = 0x29   # Fixed I²C address of all VL6180X sensors

# Multiplexer addresses
MUX1_ADDRESS = 0x77  # MULT1 address from working middle2_test.py
MUX2_ADDRESS = 0x77  # MULT2 - using same address temporarily

# I²C bus on Raspberry Pi
I2C_BUS = 1

# ------------------------------------------------------------------------
# Sensor Configuration
# ------------------------------------------------------------------------
# For now, only test the known working sensor (Index2 on MUX1, channel 3)
SENSORS = OrderedDict([
    # Format: (multiplexer_addr, channel, name)
    ("index2",  (MUX1_ADDRESS, 3, "Index2")),
])

# ------------------------------------------------------------------------
# Initialize I²C bus
# ------------------------------------------------------------------------
bus = smbus2.SMBus(I2C_BUS)

# ------------------------------------------------------------------------
# Multiplexer Control
# ------------------------------------------------------------------------
def select_channel(mux_address, channel):
    """
    Enable a specific channel on the TCA9548A multiplexer.
    
    Args:
        mux_address: I²C address of the multiplexer (0x70-0x77)
        channel: Channel number to activate (0-7)
    """
    if not 0 <= channel <= 7:
        raise ValueError(f"Channel must be between 0-7, got {channel}")
    
    # Use bitmask to select exactly one channel
    bus.write_byte(mux_address, 1 << channel)
    time.sleep(0.01)  # Small delay for stable switching

def disable_all_channels(mux_address):
    """
    Disable all channels on the specified multiplexer.
    
    Args:
        mux_address: I²C address of the multiplexer
    """
    bus.write_byte(mux_address, 0x00)

# ------------------------------------------------------------------------
# VL6180X Register Access
# ------------------------------------------------------------------------
def write_byte(device_address, reg, data):
    """
    Write a single byte to a 16-bit register on the VL6180X sensor.
    
    Args:
        device_address: I²C address of the VL6180X (0x29)
        reg: 16-bit register address
        data: 8-bit data to write
    """
    # Split 16-bit register address into high and low bytes
    bus.write_i2c_block_data(
        device_address,
        (reg >> 8) & 0xFF,  # High byte
        [(reg & 0xFF), data]  # Low byte and data
    )

def read_byte(device_address, reg):
    """
    Read a single byte from a 16-bit register on the VL6180X sensor.
    
    Args:
        device_address: I²C address of the VL6180X (0x29)
        reg: 16-bit register address
        
    Returns:
        8-bit register value
    """
    # Split 16-bit register address into high and low bytes
    bus.write_i2c_block_data(
        device_address,
        (reg >> 8) & 0xFF,  # High byte
        [(reg & 0xFF)]  # Low byte
    )
    time.sleep(0.001)  # Small delay for stability
    return bus.read_byte(device_address)

# ------------------------------------------------------------------------
# VL6180X Sensor Operations
# ------------------------------------------------------------------------
def initialize_vl6180x():
    """
    Initialize the VL6180X sensor with recommended settings.
    Only runs if the sensor indicates it hasn't been initialized yet.
    """
    # Check if sensor needs initialization (fresh out of reset flag)
    if read_byte(VL6180X_ADDRESS, 0x016) == 1:
        # Private registers initialization sequence from datasheet
        write_byte(VL6180X_ADDRESS, 0x0207, 0x01)
        write_byte(VL6180X_ADDRESS, 0x0208, 0x01)
        write_byte(VL6180X_ADDRESS, 0x0096, 0x00)
        write_byte(VL6180X_ADDRESS, 0x0097, 0xfd)
        write_byte(VL6180X_ADDRESS, 0x00e3, 0x00)
        write_byte(VL6180X_ADDRESS, 0x00e4, 0x04)
        write_byte(VL6180X_ADDRESS, 0x00e5, 0x02)
        write_byte(VL6180X_ADDRESS, 0x00e6, 0x01)
        write_byte(VL6180X_ADDRESS, 0x00e7, 0x03)
        write_byte(VL6180X_ADDRESS, 0x00f5, 0x02)
        write_byte(VL6180X_ADDRESS, 0x00d9, 0x05)
        write_byte(VL6180X_ADDRESS, 0x00db, 0xce)
        write_byte(VL6180X_ADDRESS, 0x00dc, 0x03)
        write_byte(VL6180X_ADDRESS, 0x00dd, 0xf8)
        write_byte(VL6180X_ADDRESS, 0x009f, 0x00)
        write_byte(VL6180X_ADDRESS, 0x00a3, 0x3c)
        write_byte(VL6180X_ADDRESS, 0x00b7, 0x00)
        write_byte(VL6180X_ADDRESS, 0x00bb, 0x3c)
        write_byte(VL6180X_ADDRESS, 0x00b2, 0x09)
        write_byte(VL6180X_ADDRESS, 0x00ca, 0x09)
        write_byte(VL6180X_ADDRESS, 0x0198, 0x01)
        write_byte(VL6180X_ADDRESS, 0x01b0, 0x17)
        write_byte(VL6180X_ADDRESS, 0x01ad, 0x00)
        write_byte(VL6180X_ADDRESS, 0x00ff, 0x05)
        write_byte(VL6180X_ADDRESS, 0x0100, 0x05)
        write_byte(VL6180X_ADDRESS, 0x0199, 0x05)
        write_byte(VL6180X_ADDRESS, 0x01a6, 0x1b)
        write_byte(VL6180X_ADDRESS, 0x01ac, 0x3e)
        write_byte(VL6180X_ADDRESS, 0x01a7, 0x1f)
        write_byte(VL6180X_ADDRESS, 0x0030, 0x00)

        # Public registers - configure operation parameters
        write_byte(VL6180X_ADDRESS, 0x0011, 0x10)  # Enables polling for new sample
        write_byte(VL6180X_ADDRESS, 0x010a, 0x30)  # Set ALS integration time
        write_byte(VL6180X_ADDRESS, 0x003f, 0x46)  # Set ALS gain
        write_byte(VL6180X_ADDRESS, 0x0031, 0xFF)  # Set maximum LED current
        write_byte(VL6180X_ADDRESS, 0x0040, 0x63)  # Set range gain
        write_byte(VL6180X_ADDRESS, 0x002e, 0x01)  # Enable interrupts on new sample
        write_byte(VL6180X_ADDRESS, 0x001b, 0x09)  # Set interrupt polarity
        write_byte(VL6180X_ADDRESS, 0x003e, 0x31)  # Set ALS variable integration
        write_byte(VL6180X_ADDRESS, 0x0014, 0x24)  # Configure range check

        # Clear the "fresh out of reset" flag
        write_byte(VL6180X_ADDRESS, 0x016, 0x00)

def start_range():
    """Start a range measurement."""
    write_byte(VL6180X_ADDRESS, 0x018, 0x01)

def poll_range():
    """Wait until range measurement is complete."""
    max_retries = 100  # Safety limit
    retries = 0
    
    while retries < max_retries:
        status = read_byte(VL6180X_ADDRESS, 0x04F)
        range_status = status & 0x07
        
        if range_status == 0x04:  # Measurement ready
            return True
            
        retries += 1
        time.sleep(0.001)
    
    # If we reach here, measurement timed out
    return False

def clear_interrupts():
    """Clear all interrupts."""
    write_byte(VL6180X_ADDRESS, 0x015, 0x07)

def get_distance():
    """
    Perform a complete range measurement cycle.
    
    Returns:
        Distance in millimeters or None if measurement failed
    """
    start_range()
    if poll_range():
        # Read distance value
        distance = read_byte(VL6180X_ADDRESS, 0x062)
        # Read range status for error checking
        status = read_byte(VL6180X_ADDRESS, 0x04D)
        clear_interrupts()
        
        # Check if measurement is valid (status = 0 or 11)
        if status == 0 or status == 11:
            return distance
        else:
            return None
    else:
        clear_interrupts()
        return None

# ------------------------------------------------------------------------
# Sensor Initialization
# ------------------------------------------------------------------------
def initialize_all_sensors():
    """
    Initialize all sensors by cycling through each multiplexer channel.
    """
    initialized_sensors = []
    
    print("Initializing all sensors...")
    for sensor_id, (mux_addr, channel, name) in SENSORS.items():
        try:
            print(f"Initializing {name}... ", end="")
            
            # Select the channel on the appropriate multiplexer
            select_channel(mux_addr, channel)
            
            # Initialize the sensor
            initialize_vl6180x()
            
            # Verify sensor responds
            model_id = read_byte(VL6180X_ADDRESS, 0x000)
            revision_id = read_byte(VL6180X_ADDRESS, 0x001)
            
            if model_id == 0xB4:  # Expected model ID for VL6180X
                print(f"OK (Model: 0x{model_id:02X}, Rev: 0x{revision_id:02X})")
                initialized_sensors.append(sensor_id)
            else:
                print(f"FAILED (Invalid model ID: 0x{model_id:02X})")
        except Exception as e:
            print(f"ERROR: {str(e)}")
    
    # Disable all channels when done
    disable_all_channels(MUX1_ADDRESS)
    disable_all_channels(MUX2_ADDRESS)
    
    return initialized_sensors

# ------------------------------------------------------------------------
# Main Test Functions
# ------------------------------------------------------------------------
def read_all_sensors():
    """
    Read distance from all connected sensors.
    
    Returns:
        Dictionary mapping sensor IDs to distance values
    """
    results = {}
    
    for sensor_id, (mux_addr, channel, name) in SENSORS.items():
        try:
            # Select the appropriate multiplexer channel
            select_channel(mux_addr, channel)
            
            # Read distance
            distance = get_distance()
            results[sensor_id] = distance
            
        except Exception as e:
            results[sensor_id] = None
            print(f"Error reading {name}: {str(e)}")
    
    # Disable all channels when done
    disable_all_channels(MUX1_ADDRESS)
    disable_all_channels(MUX2_ADDRESS)
    
    return results

def display_results(results):
    """
    Display sensor readings in a formatted table.
    
    Args:
        results: Dictionary mapping sensor IDs to distance values
    """
    print("\n" + "=" * 40)
    print("PROXIMITY SENSOR READINGS")
    print("=" * 40)
    print(f"{'Sensor':<10} | {'Position':<8} | {'Distance (mm)':<12}")
    print("-" * 40)
    
    for sensor_id, (mux_addr, channel, name) in SENSORS.items():
        distance = results.get(sensor_id)
        if distance is not None:
            print(f"{sensor_id:<10} | {name:<8} | {distance:<12}")
        else:
            print(f"{sensor_id:<10} | {name:<8} | {'ERROR':<12}")
    
    print("=" * 40)

def main():
    """Main test function."""
    try:
        print("VL6180X Proximity Sensor Test - All Sensors")
        print("------------------------------------------")
        
        # Initialize all sensors
        working_sensors = initialize_all_sensors()
        
        if not working_sensors:
            print("No sensors were successfully initialized. Check connections.")
            return
        
        print(f"\nSuccessfully initialized {len(working_sensors)}/{len(SENSORS)} sensors.")
        print("Press Ctrl+C to exit.\n")
        
        # Main loop
        while True:
            # Read all sensors
            results = read_all_sensors()
            
            # Display results
            display_results(results)
            
            # Wait before next reading
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    except Exception as e:
        print(f"\nError: {str(e)}")
    finally:
        # Clean up
        try:
            disable_all_channels(MUX1_ADDRESS)
            disable_all_channels(MUX2_ADDRESS)
            bus.close()
            print("I2C bus closed. Test complete.")
        except:
            pass

if __name__ == "__main__":
    main()