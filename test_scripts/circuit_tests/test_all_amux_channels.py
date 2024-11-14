import RPi.GPIO as GPIO
import spidev
import time

class KalmanFilter:
    def __init__(self, Q, R, A=1, H=1):
        self.Q = Q  # Process noise covariance
        self.R = R  # Measurement noise covariance
        self.A = A  # State transition factor
        self.H = H  # Measurement factor
        self.x = 0  # Initial estimate
        self.P = 1  # Initial estimate covariance

    def update(self, measurement):
        # Prediction phase
        x_pred = self.A * self.x
        P_pred = self.A * self.P * self.A + self.Q

        # Update phase
        K = P_pred * self.H / (self.H * P_pred * self.H + self.R)  # Kalman gain
        self.x = x_pred + K * (measurement - self.H * x_pred)      # Update estimate
        self.P = (1 - K * self.H) * P_pred                         # Update estimate covariance
        return self.x

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

# Kalman filter parameters
Q = 0.1    # Process noise covariance
R = 5.0    # Measurement noise covariance

# Create KalmanFilter instances for each channel
kalman_filters_amux1 = [KalmanFilter(Q, R) for _ in range(16)]
kalman_filters_amux2 = [KalmanFilter(Q, R) for _ in range(16)]

# Initialize GPIO for both amuxes
GPIO.setmode(GPIO.BCM)
for pin in AMUX1_PINS.values():
    GPIO.setup(pin, GPIO.OUT)
for pin in AMUX2_PINS.values():
    GPIO.setup(pin, GPIO.OUT)

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

def test_all_amux_channels():
    """Cycle through all channels on both amux1 and amux2, applying a Kalman filter to the ADC values."""
    print("Reading all channels on amux1 and amux2 with Kalman filtering... (Press Ctrl+C to stop)")

    try:
        while True:
            # Read from all channels on amux1
            print("amux1 channel values:")
            for channel in range(16):
                select_channel(AMUX1_PINS, channel)
                time.sleep(0.01)  # Reduced delay for faster switching
                raw_adc_value = read_adc_channel(0)  # Read from MCP3008 CH0
                filtered_value = kalman_filters_amux1[channel].update(raw_adc_value)
                print(f"  amux1 channel {channel}: raw={raw_adc_value}, filtered={filtered_value}")

            # Read from all channels on amux2
            print("amux2 channel values:")
            for channel in range(16):
                select_channel(AMUX2_PINS, channel)
                time.sleep(0.01)  # Reduced delay for faster switching
                raw_adc_value = read_adc_channel(0)  # Read from MCP3008 CH0
                filtered_value = kalman_filters_amux2[channel].update(raw_adc_value)
                print(f"  amux2 channel {channel}: raw={raw_adc_value}, filtered={filtered_value}")

            print("\n--- End of cycle ---\n")
            time.sleep(1)  # Delay before next cycle for readability

    except KeyboardInterrupt:
        print("Test stopped by user.")
    finally:
        GPIO.cleanup()
        spi.close()

if __name__ == "__main__":
    test_all_amux_channels()