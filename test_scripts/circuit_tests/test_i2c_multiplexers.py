import smbus2
import time

# I2C bus number (usually 1 on Raspberry Pi)
I2C_BUS = 1

# Addresses of the I2C multiplexers
MUX_ADDRESSES = [0x70, 0x71, 0x72]

# Initialize the I2C bus
bus = smbus2.SMBus(I2C_BUS)

def check_mux_connection(address):
    """Check if the I2C device at the specified address is connected."""
    try:
        bus.write_byte(address, 0)  # Attempt to write a byte to the device
        print(f"Multiplexer found at address: {hex(address)}")
    except OSError:
        print(f"Multiplexer NOT found at address: {hex(address)}")

def main():
    print("Testing I2C Multiplexers (TCA9548A)")

    # Loop through each multiplexer address and check its connection
    for address in MUX_ADDRESSES:
        check_mux_connection(address)
        time.sleep(0.5)  # Small delay for stability

if __name__ == "__main__":
    main()
