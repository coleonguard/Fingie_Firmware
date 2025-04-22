import smbus2
import time

# ------------------------------------------------------------------------
# I²C Constants
# ------------------------------------------------------------------------
VL6180X_ADDRESS = 0x29   # Default sensor address
MUX1_ADDRESS = 0x77

MIDDLE2_CHANNEL_ON_MUX1 = 7  # <- Change this if Middle2 is on a different MUX1 channel

# ------------------------------------------------------------------------
# I²C Bus
# ------------------------------------------------------------------------
bus = smbus2.SMBus(1)

def select_channel(mux_address, channel):
    bus.write_byte(mux_address, 1 << channel)
    time.sleep(0.01)

def write_byte(device_address, reg, data):
    bus.write_i2c_block_data(device_address, (reg >> 8) & 0xFF, [(reg & 0xFF), data])

def read_byte(device_address, reg):
    bus.write_i2c_block_data(device_address, (reg >> 8) & 0xFF, [(reg & 0xFF)])
    time.sleep(0.01)
    return bus.read_byte(device_address)

def initialize_vl6180x():
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
        write_byte(VL6180X_ADDRESS, 0x0011, 0x10)
        write_byte(VL6180X_ADDRESS, 0x010a, 0x30)
        write_byte(VL6180X_ADDRESS, 0x003f, 0x46)
        write_byte(VL6180X_ADDRESS, 0x0031, 0xFF)
        write_byte(VL6180X_ADDRESS, 0x0040, 0x63)
        write_byte(VL6180X_ADDRESS, 0x002e, 0x01)
        write_byte(VL6180X_ADDRESS, 0x001b, 0x09)
        write_byte(VL6180X_ADDRESS, 0x003e, 0x31)
        write_byte(VL6180X_ADDRESS, 0x0014, 0x24)
        write_byte(VL6180X_ADDRESS, 0x016, 0x00)

def start_range():
    write_byte(VL6180X_ADDRESS, 0x018, 0x01)

def poll_range():
    while True:
        status = read_byte(VL6180X_ADDRESS, 0x04F)
        if status & 0x07 == 0x04:
            break
        time.sleep(0.01)

def clear_interrupts():
    write_byte(VL6180X_ADDRESS, 0x015, 0x07)

def get_distance():
    start_range()
    poll_range()
    dist = read_byte(VL6180X_ADDRESS, 0x062)
    clear_interrupts()
    return dist

def test_middle2_on_mux1():
    try:
        select_channel(MUX1_ADDRESS, MIDDLE2_CHANNEL_ON_MUX1)
        initialize_vl6180x()
        while True:
            dist = get_distance()
            print(f"Middle2 (via MUX1 Channel {MIDDLE2_CHANNEL_ON_MUX1}): {dist} mm")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Exiting.")
    finally:
        bus.write_byte(MUX1_ADDRESS, 0x00)
        bus.close()

if __name__ == "__main__":
    test_middle2_on_mux1()
