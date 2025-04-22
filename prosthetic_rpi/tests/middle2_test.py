#!/usr/bin/env python3
"""
Test a single VL6180X sensor on MULT1 (TCA9548A at 0x77), channel 3 (Index2 wiring).
"""

import smbus2
import time

# ------------------------------------------------------------------------
# I²C Constants
# ------------------------------------------------------------------------
VL6180X_ADDRESS      = 0x29    # Fixed I²C address of each VL6180X
MUX1_ADDRESS         = 0x77    # MULT1 strapped A2=A1=A0=HIGH
INDEX2_CHANNEL       = 3       # SD3/SC3 on the multiplexer

# ------------------------------------------------------------------------
# Initialize I²C bus (bus #1 on Raspberry Pi)
# ------------------------------------------------------------------------
bus = smbus2.SMBus(1)

# ------------------------------------------------------------------------
# Multiplexer channel selector
# ------------------------------------------------------------------------
def select_channel(mux_addr, channel):
    """
    Enable exactly one downstream channel on the TCA9548A.
    channel: 0–7 → writes (1 << channel) to mux_addr
    """
    bus.write_byte(mux_addr, 1 << channel)
    time.sleep(0.01)

# ------------------------------------------------------------------------
# Low‑level I²C register R/W for VL6180X
# ------------------------------------------------------------------------
def write_byte(device_addr, reg, data):
    """
    Write a single byte 'data' to 16-bit register 'reg' on the VL6180X.
    """
    hi = (reg >> 8) & 0xFF
    lo = reg & 0xFF
    bus.write_i2c_block_data(device_addr, hi, [lo, data])

def read_byte(device_addr, reg):
    """
    Read a single byte from 16-bit register 'reg' on the VL6180X.
    """
    hi = (reg >> 8) & 0xFF
    lo = reg & 0xFF
    bus.write_i2c_block_data(device_addr, hi, [lo])
    time.sleep(0.01)
    return bus.read_byte(device_addr)

# ------------------------------------------------------------------------
# VL6180X initialization sequence (from Adafruit example, with one fix)
# ------------------------------------------------------------------------
def initialize_vl6180x():
    # Only initialize if sensor reports "fresh out of reset" bit set
    if read_byte(VL6180X_ADDRESS, 0x016) == 1:
        # Private registers from application notes
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

        # Public registers (enable new-range-ready, set gains, etc.)
        write_byte(VL6180X_ADDRESS, 0x0011, 0x10)
        write_byte(VL6180X_ADDRESS, 0x010a, 0x30)
        write_byte(VL6180X_ADDRESS, 0x003f, 0x46)
        write_byte(VL6180X_ADDRESS, 0x0031, 0xFF)
        write_byte(VL6180X_ADDRESS, 0x0041, 0x63)   # <-- Fixed: was 0x0040
        write_byte(VL6180X_ADDRESS, 0x002e, 0x01)
        write_byte(VL6180X_ADDRESS, 0x001b, 0x09)
        write_byte(VL6180X_ADDRESS, 0x003e, 0x31)
        write_byte(VL6180X_ADDRESS, 0x0014, 0x24)

        # Clear the “fresh out of reset” flag
        write_byte(VL6180X_ADDRESS, 0x016, 0x00)

# ------------------------------------------------------------------------
# Single-range sequence methods
# ------------------------------------------------------------------------
def start_range():
    write_byte(VL6180X_ADDRESS, 0x018, 0x01)

def poll_range():
    while True:
        status = read_byte(VL6180X_ADDRESS, 0x04F)
        if (status & 0x07) == 0x04:   # range ready
            break
        time.sleep(0.01)

def clear_interrupts():
    write_byte(VL6180X_ADDRESS, 0x015, 0x07)

def get_distance():
    start_range()
    poll_range()
    dist = read_byte(VL6180X_ADDRESS, 0x062)  # range result
    clear_interrupts()
    return dist

# ------------------------------------------------------------------------
# Main test: select channel 3 on mux1 and loop forever
# ------------------------------------------------------------------------
def test_index2_on_mux1():
    try:
        # 1) Select the downstream channel
        select_channel(MUX1_ADDRESS, INDEX2_CHANNEL)

        # 2) Initialize sensor if needed
        initialize_vl6180x()

        # 3) Continuous read loop
        while True:
            d = get_distance()
            print(f"Index2 (MUX=0x{MUX1_ADDRESS:02X} CH={INDEX2_CHANNEL}): {d} mm")
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nInterrupted by user — disabling all channels and exiting.")

    finally:
        # turn off all channels
        bus.write_byte(MUX1_ADDRESS, 0x00)
        bus.close()

if __name__ == "__main__":
    test_index2_on_mux1()
