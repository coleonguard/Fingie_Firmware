# pressure_sensor.py
# order of array: ['THUMB_1', 'THUMB_2', 'p_agg_1', 'p_agg_2', 'p_agg_3', 'p_agg_4', 'p_agg_5', 'p_agg_6', 'p_agg_7', 'I_M', 'I_L_1', 'I_L_2', 'M_M', 'M_L_1', 'M_L_2', 'R_M', 'R_L_1', 'R_L_2', 'P_M', 'P_L_1', 'P_L_2']

import RPi.GPIO as GPIO
import spidev
import socket
import time

# Set up SPI for ADC (MCP3008)
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1350000

# AMUX pin configurations
AMUX1_PINS = {
    'A0': 17,
    'A1': 23,
    'A2': 26,
    'A3': 16
}

AMUX2_PINS = {
    'A0': 24,
    'A1': 6,
    'A2': 25,
    'A3': 12
}

# Channel to sensor name mappings for amux1
AMUX1_CHANNEL_MAP = {
    0: "THUMB_1",
    1: "THUMB_2",
    2: "p_agg_1",
    3: "p_agg_2",
    4: "p_agg_3",
    5: "p_agg_4",
    6: "p_agg_5",
    7: "p_agg_6",
    8: "p_agg_7",
    9: "N/A",
    10: "N/A",
    11: "N/A",
    12: "N/A",
    13: "N/A",
    14: "N/A",
    15: "N/A"
}

# Channel to sensor name mappings for amux2
AMUX2_CHANNEL_MAP = {
    0: "I_M",
    1: "I_L_1",
    2: "I_L_2",
    3: "M_M",
    4: "M_L_1",
    5: "M_L_2",
    6: "R_M",
    7: "R_L_1",
    8: "R_L_2",
    9: "P_M",
    10: "P_L_1",
    11: "P_L_2",
    12: "N/A",
    13: "N/A",
    14: "N/A",
    15: "N/A"
}


# Initialize GPIO
GPIO.setmode(GPIO.BCM)
for pin in AMUX1_PINS.values():
    GPIO.setup(pin, GPIO.OUT)
for pin in AMUX2_PINS.values():
    GPIO.setup(pin, GPIO.OUT)

# UDP configuration
UDP_IP = "127.0.0.1"
UDP_PORT = 6001
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def select_channel(mux_pins, channel):
    """Sets the given analog multiplexer to the specified channel."""
    binary = [int(x) for x in '{:04b}'.format(channel)]
    for i, pin in enumerate(mux_pins.values()):
        GPIO.output(pin, binary[i])

def read_adc_channel(channel):
    """Read the analog value from the specified ADC channel on MCP3008."""
    if channel < 0 or channel > 7:
        raise ValueError("Channel must be between 0 and 7")
    adc = spi.xfer2([1, (8 + channel) << 4, 0])
    return ((adc[1] & 3) << 8) + adc[2]

def get_all_sensor_values():
    """Returns a list of (sensor_name, adc_value) for all connected sensors."""
    readings = []

    # Read from amux1
    for ch in range(16):
        sensor_name = AMUX1_CHANNEL_MAP.get(ch, "N/A")
        if sensor_name != "N/A":
            select_channel(AMUX1_PINS, ch)
            time.sleep(0.001)  # small delay
            adc_value = read_adc_channel(0)  
            readings.append((sensor_name, adc_value))

    # Read from amux2
    for ch in range(16):
        sensor_name = AMUX2_CHANNEL_MAP.get(ch, "N/A")
        if sensor_name != "N/A":
            select_channel(AMUX2_PINS, ch)
            time.sleep(0.001)  # small delay
            adc_value = read_adc_channel(0)  
            readings.append((sensor_name, adc_value))

    return readings

if __name__ == "__main__":
    try:
        # Print out the order of sensors once so you can copy-paste it
        # This order matches the sequence in which readings are taken and transmitted
        sensor_order = [AMUX1_CHANNEL_MAP[ch] for ch in range(16) if AMUX1_CHANNEL_MAP[ch] != "N/A"] \
                     + [AMUX2_CHANNEL_MAP[ch] for ch in range(16) if AMUX2_CHANNEL_MAP[ch] != "N/A"]
        print("Order of the array (copy/paste this):")
        print(sensor_order)

        # Continuous transmission
        while True:
            readings = get_all_sensor_values()  # list of (sensor_name, value)
            # We only transmit the values, in order, separated by spaces
            # If you want to include sensor names in transmission, you can adapt this line.
            data_str = " ".join(str(val) for _, val in readings)
            sock.sendto(data_str.encode(), (UDP_IP, UDP_PORT))
            time.sleep(0.01)  # 100 Hz rate

    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        spi.close()
        sock.close()
