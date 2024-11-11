import RPi.GPIO as GPIO
import spidev
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

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

# Setup for Matplotlib
plt.style.use("ggplot")
fig, ax = plt.subplots()
x_data = list(range(16))
y_data_amux1 = [0] * 16
y_data_amux2 = [0] * 16
line1, = ax.plot(x_data, y_data_amux1, label="amux1")
line2, = ax.plot(x_data, y_data_amux2, label="amux2")
ax.set_ylim(0, 1023)  # MCP3008 range from 0 to 1023
ax.set_title("Real-Time ADC Readings for amux1 and amux2")
ax.set_xlabel("Channel")
ax.set_ylabel("ADC Value")
ax.legend()

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

def update_plot(frame):
    """Update the plot with ADC values from all channels on amux1 and amux2."""
    y_data_amux1 = []
    y_data_amux2 = []

    # Read from all channels on amux1
    for channel in range(16):
        select_channel(AMUX1_PINS, channel)
        time.sleep(0.01)  # Small delay for stable switching
        adc_value = read_adc_channel(0)
        y_data_amux1.append(adc_value)

    # Read from all channels on amux2
    for channel in range(16):
        select_channel(AMUX2_PINS, channel)
        time.sleep(0.01)  # Small delay for stable switching
        adc_value = read_adc_channel(0)
        y_data_amux2.append(adc_value)

    # Update the data for both lines
    line1.set_ydata(y_data_amux1)
    line2.set_ydata(y_data_amux2)
    
    return line1, line2

# Use FuncAnimation to update the plot in real-time
ani = FuncAnimation(fig, update_plot, blit=True, interval=1000)

try:
    plt.show()
except KeyboardInterrupt:
    print("Real-time plot stopped by user.")
finally:
    GPIO.cleanup()
    spi.close()
