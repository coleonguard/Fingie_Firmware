# Channel Mapping:
# MULT1 (I²C Address = 0x77)
#   Channel 0 → Thumb1 (SDA-Thumb1, SCL-Thumb1)
#   Channel 1 → Thumb2 (SDA-Thumb2, SCL-Thumb2)
#   Channel 2 → Index1 (SDA-Index1, SCL-Index1)
#   Channel 3 → Index2 (SDA-Index2, SCL-Index2)
#   Channel 4 → Middle1 (SDA-Middle, SCL-Middle)
# MULT2 (I²C Address = 0x73)
#   Channel 0 → Middle2 (SDA-Middle2, SCL-Middle2)
#   Channel 1 → Ring1 (SDA-Ring1, SCL-Ring1)
#   Channel 2 → Ring2 (SDA-Ring2, SCL-Ring2)
#   Channel 3 → Pinky1 (SDA-Pinky1, SCL-Pinky1)
#   Channel 4 → Pinky2 (SDA-Pinky2, SCL-Pinky2)

import smbus2
import time

# ------------------------------------------------------------------------
# I²C Addresses
# ------------------------------------------------------------------------
VL6180X_ADDRESS = 0x29   # Hardcoded address of each VL6180X sensor

# Your two multiplexers:
MUX1_ADDRESS = 0x70      # MULT1
MUX2_ADDRESS = 0x71      # MULT2

# ------------------------------------------------------------------------
# Raspberry Pi I²C Bus
# ------------------------------------------------------------------------
bus = smbus2.SMBus(1)    # On Pi 5, still I²C bus #1

# ------------------------------------------------------------------------
# Sensor-to-Multiplexer Mapping
# ------------------------------------------------------------------------
SENSORS = [
    # (multiplexer_addr, channel, "DescriptiveName")
    (MUX1_ADDRESS, 0, "Thumb1"),
    (MUX1_ADDRESS, 1, "Thumb2"),
    (MUX1_ADDRESS, 2, "Index1"),
    (MUX1_ADDRESS, 3, "Index2"),
    (MUX1_ADDRESS, 4, "Middle1"),
    (MUX2_ADDRESS, 0, "Middle2"),
    (MUX2_ADDRESS, 1, "Ring1"),
    (MUX2_ADDRESS, 2, "Ring2"),
    (MUX2_ADDRESS, 3, "Pinky1"),
    (MUX2_ADDRESS, 4, "Pinky2"),
]

# ------------------------------------------------------------------------
# I²C Helper Functions
# ------------------------------------------------------------------------
def select_channel(mux_address, channel):
    """
    Enable a specific channel on the TCA9548A multiplexer.
    The channel is selected using a bitmask (1 << channel).
    """
    bus.write_byte(mux_address, 1 << channel)
    time.sleep(0.01)  # Small delay for stable switching

def write_byte(device_address, reg, data):
    """
    Write a single byte to a register (16-bit address) on the VL6180X sensor.
    """
    bus.write_i2c_block_data(device_address,
                             (reg >> 8) & 0xFF,
                             [(reg & 0xFF), data])

def read_byte(device_address, reg):
    """
    Read a single byte from a register (16-bit address) on the VL6180X sensor.
    """
    bus.write_i2c_block_data(device_address,
                             (reg >> 8) & 0xFF,
                             [(reg & 0xFF)])
    time.sleep(0.01)
    return bus.read_byte(device_address)

# ------------------------------------------------------------------------
# VL6180X Sensor Initialization & Reading
# ------------------------------------------------------------------------
def initialize_vl6180x():
    """
    Perform the standard initialization sequence on the VL6180X sensor.
    Only run if the sensor indicates it has not yet been initialized.
    """
    if read_byte(VL6180X_ADDRESS, 0x016) == 1:
        # Full initialization sequence
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

        # Public registers
        write_byte(VL6180X_ADDRESS, 0x0011, 0x10)
        write_byte(VL6180X_ADDRESS, 0x010a, 0x30)
        write_byte(VL6180X_ADDRESS, 0x003f, 0x46)
        write_byte(VL6180X_ADDRESS, 0x0031, 0xFF)
        write_byte(VL6180X_ADDRESS, 0x0040, 0x63)
        write_byte(VL6180X_ADDRESS, 0x002e, 0x01)
        write_byte(VL6180X_ADDRESS, 0x001b, 0x09)
        write_byte(VL6180X_ADDRESS, 0x003e, 0x31)
        write_byte(VL6180X_ADDRESS, 0x0014, 0x24)

        # Final initialization bit
        write_byte(VL6180X_ADDRESS, 0x016, 0x00)

def start_range():
    """Start a range measurement."""
    write_byte(VL6180X_ADDRESS, 0x018, 0x01)

def poll_range():
    """Poll until a range measurement is ready."""
    while True:
        status = read_byte(VL6180X_ADDRESS, 0x04F)
        range_status = status & 0x07
        if range_status == 0x04:  # Measurement ready
            break
        time.sleep(0.01)

def clear_interrupts():
    """Clear sensor interrupts."""
    write_byte(VL6180X_ADDRESS, 0x015, 0x07)

def get_distance():
    """Get distance measurement (in mm)."""
    start_range()
    poll_range()
    dist = read_byte(VL6180X_ADDRESS, 0x062)  # Range result register
    clear_interrupts()
    return dist

# ------------------------------------------------------------------------
# Main Loop: Cycle Through All Sensors
# ------------------------------------------------------------------------
def test_all_sensors():
    try:
        while True:
            for (mux_addr, channel, sensor_name) in SENSORS:
                # Select the appropriate multiplexer channel
                select_channel(mux_addr, channel)

                # Initialize sensor if needed
                initialize_vl6180x()

                # Read distance
                distance_mm = get_distance()
                print(f"{sensor_name}: {distance_mm} mm")

                # Small delay between sensors
                time.sleep(0.2)

    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        # Optionally disable all channels on each multiplexer
        bus.write_byte(MUX1_ADDRESS, 0x00)
        bus.write_byte(MUX2_ADDRESS, 0x00)
        bus.close()

if __name__ == "__main__":
    test_all_sensors()
