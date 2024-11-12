import smbus2
import socket
import time

import errno  # Added to handle specific error codes

# I2C addresses
MUX_ADDRESS = 0x70       # TCA9548A multiplexer address
CHANNEL = 4              # Channel to enable on the multiplexer
VL6180X_ADDRESS = 0x29   # VL6180X sensor I2C address

# Initialize I2C bus
bus = smbus2.SMBus(1)

# UDP setup for transmission
UDP_IP = "127.0.0.1"
UDP_PORT = 6000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def select_channel(mux_address, channel):
    """Enable a specific channel on the TCA9548A multiplexer."""
    bus.write_byte(mux_address, 1 << channel)
    time.sleep(0.01)  # Small delay for stable switching

def write_byte(device_address, reg, data):
    """Write a single byte to a register on the VL6180X sensor."""
    try:
        bus.write_i2c_block_data(device_address, (reg >> 8) & 0xFF, [(reg & 0xFF), data])
    except OSError as e:
        if e.errno == errno.EIO:
            # Input/output error, sensor might not be connected
            raise ConnectionError("Sensor not connected.")
        else:
            raise

def read_byte(device_address, reg):
    """Read a single byte from a register on the VL6180X sensor."""
    try:
        bus.write_i2c_block_data(device_address, (reg >> 8) & 0xFF, [(reg & 0xFF)])
        time.sleep(0.01)
        return bus.read_byte(device_address)
    except OSError as e:
        if e.errno == errno.EIO:
            # Input/output error, sensor might not be connected
            raise ConnectionError("Sensor not connected.")
        else:
            raise

def initialize_vl6180x():
    """Initialize the VL6180X sensor based on setup instructions from the C code."""
    try:
        # If not already set up, initialize the device
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

            # Public registers as per the VL6180X datasheet
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

    except ConnectionError as e:
        print(e)
        return False  # Indicate initialization failed
    return True  # Initialization successful

def start_range():
    """Start a range measurement."""
    write_byte(VL6180X_ADDRESS, 0x018, 0x01)

def poll_range():
    """Poll the sensor until a range measurement is ready."""
    start_time = time.time()
    while True:
        try:
            status = read_byte(VL6180X_ADDRESS, 0x04F)
            range_status = status & 0x07
            if range_status == 0x04:  # New measurement ready
                break
        except ConnectionError as e:
            print(e)
            return None
        if time.time() - start_time > 0.1:  # Timeout after 100ms
            print("Polling timeout.")
            return None
        time.sleep(0.005)

def clear_interrupts():
    """Clear interrupts on the VL6180X sensor."""
    write_byte(VL6180X_ADDRESS, 0x015, 0x07)

def get_distance():
    """Get distance reading from the VL6180X sensor."""
    try:
        start_range()
        if poll_range() is None:
            return None
        distance = read_byte(VL6180X_ADDRESS, 0x062)  # Range result
        clear_interrupts()
        return distance
    except ConnectionError as e:
        print(e)
        return None

def transmit_distance():
    """Transmit distance measurement over UDP."""
    while True:
        try:
            select_channel(MUX_ADDRESS, CHANNEL)  # Enable channel on multiplexer
            if not initialize_vl6180x():  # Initialize the sensor
                print("Failed to initialize sensor. Retrying...")
                time.sleep(1)
                continue  # Retry initialization
            break  # Initialization successful
        except ConnectionError as e:
            print(e)
            time.sleep(1)
    
    try:
        while True:
            distance = get_distance()
            if distance is not None:
                print(f"Distance: {distance} mm")
                sock.sendto(str(distance).encode(), (UDP_IP, UDP_PORT))
            else:
                print("No distance data. Sensor might be disconnected.")
            time.sleep(0.01)  # 100 Hz
    except KeyboardInterrupt:
        print("Transmission interrupted by user.")
    finally:
        # Disable all channels on the multiplexer and close the bus
        bus.write_byte(MUX_ADDRESS, 0x00)
        bus.close()

if __name__ == "__main__":
    transmit_distance()
