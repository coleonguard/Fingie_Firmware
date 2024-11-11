import spidev
import time

# Initialize SPI
spi = spidev.SpiDev()
spi.open(0, 0)  # Open SPI bus 0, device 0 (CE0)
spi.max_speed_hz = 1350000

def read_channel(channel):
    """Reads the analog value from the specified ADC channel (0-7)."""
    if channel < 0 or channel > 7:
        raise ValueError("Channel must be between 0 and 7")
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    data = ((adc[1] & 3) << 8) + adc[2]
    return data

try:
    print("Testing ADC (MCP3008)")
    while True:
        # Read and print values from all 8 channels
        for channel in range(8):
            value = read_channel(channel)
            print(f"Channel {channel}: {value}")
        time.sleep(1)  # Adjust delay as needed for testing

except KeyboardInterrupt:
    print("Test interrupted by user.")
finally:
    spi.close()
