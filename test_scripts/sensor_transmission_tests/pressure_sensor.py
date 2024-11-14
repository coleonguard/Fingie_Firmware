import RPi.GPIO as GPIO
import spidev
import socket
import time

# Initialize SPI for ADC
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1350000

# Corrected GPIO pin mappings for amux1 and amux2 control
AMUX1_PINS = {
    'A0': 17,  # Corrected from 0
    'A1': 23,  # Corrected from 4
    'A2': 26,  # Corrected from 25
    'A3': 16   # Corrected from 27
}

AMUX2_PINS = {
    'A0': 24,  # GPIO 24
    'A1': 6,   # GPIO 6
    'A2': 25,  # GPIO 25
    'A3': 12   # GPIO 12
}

# Initialize GPIO for both amuxes
GPIO.setmode(GPIO.BCM)
for pin in AMUX1_PINS.values():
    GPIO.setup(pin, GPIO.OUT)
for pin in AMUX2_PINS.values():
    GPIO.setup(pin, GPIO.OUT)

# Socket configuration for UDP transmission
UDP_IP = "127.0.0.1"
UDP_PORT = 6001
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def select_channel(mux_pins, channel):
    """Sets the given analog multiplexer to the specified channel."""
    binary = [int(x) for x in '{:04b}'.format(channel)]
    for i, pin in enumerate(mux_pins.values()):
        GPIO.output(pin, binary[i])

def read_adc_channel(channel):
    """Read the analog value from the specified ADC channel."""
    if channel < 0 or channel > 7:
        raise ValueError("Channel must be between 0 and 7")
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    return ((adc[1] & 3) << 8) + adc[2]

def read_and_transmit_all_channels():
    """Cycle through all channels on both amux1 and amux2, transmitting ADC values."""
    print("Transmitting all channels on amux1 and amux2... (Press Ctrl+C to stop)")

    try:
        while True:
            adc_values = []

            # Read from all channels on amux1
            for channel in range(16):
                select_channel(AMUX1_PINS, channel)
                time.sleep(0.01)  # Small delay for stable switching
                adc_value = read_adc_channel(0)  # Read from MCP3008 CH0
                adc_values.append(adc_value)

            # Read from all channels on amux2
            for channel in range(16):
                select_channel(AMUX2_PINS, channel)
                time.sleep(0.01)  # Small delay for stable switching
                adc_value = read_adc_channel(0)  # Read from MCP3008 CH0
                adc_values.append(adc_value)

            # Transmit all values as a space-separated string
            data = " ".join(map(str, adc_values))
            sock.sendto(data.encode(), (UDP_IP, UDP_PORT))
            time.sleep(0.01)  # 100 Hz transmission rate

    except KeyboardInterrupt:
        print("Transmission stopped by user.")
    finally:
        GPIO.cleanup()
        spi.close()
        sock.close()

if __name__ == "__main__":
    read_and_transmit_all_channels()
