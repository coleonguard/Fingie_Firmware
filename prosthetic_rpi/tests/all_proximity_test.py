#!/usr/bin/env python3
"""
Read every VL6180X in the hand-sensor stack:
 ├─ MULT1 (0x77): Thumb1, Thumb2, Index1, Index2, Middle1
 └─ MULT2 (0x73): Middle2, Ring1, Ring2, Pinky1, Pinky2
Each entry is (human-readable name, multiplexer I²C addr, channel number).
"""

import smbus2, time, collections, sys, signal

VL6180X_ADDRESS = 0x29          # Fixed address for every VL6180X

# ---------------------------------------------------------------------------
# 1.  Sensor inventory (name, mux-addr, channel)
# ---------------------------------------------------------------------------
SENSORS = [
    ("Thumb1",  0x77, 0),
    ("Thumb2",  0x77, 1),
    ("Index1",  0x77, 2),
    ("Index2",  0x77, 3),
    ("Middle1", 0x77, 4),
    ("Middle2", 0x73, 0),
    ("Ring1",   0x73, 1),
    ("Ring2",   0x73, 2),
    ("Pinky1",  0x73, 3),
    ("Pinky2",  0x73, 4),
]

bus            = smbus2.SMBus(1)
initialised    = collections.defaultdict(bool)   # (mux,chan) → True/False

# ---------------------------------------------------------------------------
# 2.  Common helpers (same as your single-sensor test)
# ---------------------------------------------------------------------------
def select_channel(mux_addr: int, channel: int) -> None:
    bus.write_byte(mux_addr, 1 << channel)
    time.sleep(0.001)

def write_byte(device_addr: int, reg: int, data: int) -> None:
    hi, lo = (reg >> 8) & 0xFF, reg & 0xFF
    bus.write_i2c_block_data(device_addr, hi, [lo, data])

def read_byte(device_addr: int, reg: int) -> int:
    hi, lo = (reg >> 8) & 0xFF, reg & 0xFF
    bus.write_i2c_block_data(device_addr, hi, [lo])
    time.sleep(0.0005)
    return bus.read_byte(device_addr)

# (unchanged) — VL6180X initial register sequence
def initialise_vl6180x() -> None:
    if read_byte(VL6180X_ADDRESS, 0x016) != 1:
        return                                            # already done
    for reg, val in [
        (0x0207,0x01),(0x0208,0x01),(0x0096,0x00),(0x0097,0xfd),
        (0x00e3,0x00),(0x00e4,0x04),(0x00e5,0x02),(0x00e6,0x01),
        (0x00e7,0x03),(0x00f5,0x02),(0x00d9,0x05),(0x00db,0xce),
        (0x00dc,0x03),(0x00dd,0xf8),(0x009f,0x00),(0x00a3,0x3c),
        (0x00b7,0x00),(0x00bb,0x3c),(0x00b2,0x09),(0x00ca,0x09),
        (0x0198,0x01),(0x01b0,0x17),(0x01ad,0x00),(0x00ff,0x05),
        (0x0100,0x05),(0x0199,0x05),(0x01a6,0x1b),(0x01ac,0x3e),
        (0x01a7,0x1f),(0x0030,0x00),
        (0x0011,0x10),(0x010a,0x30),(0x003f,0x46),(0x0031,0xFF),
        (0x0041,0x63),(0x002e,0x01),(0x001b,0x09),(0x003e,0x31),
        (0x0014,0x24)
    ]:
        write_byte(VL6180X_ADDRESS, reg, val)
    write_byte(VL6180X_ADDRESS, 0x016, 0x00)              # clear reset

def start_range()        : write_byte(VL6180X_ADDRESS, 0x018, 0x01)
def clear_interrupts()   : write_byte(VL6180X_ADDRESS, 0x015, 0x07)

def wait_ready() -> None:
    while (read_byte(VL6180X_ADDRESS, 0x04F) & 0x07) != 0x04:
        time.sleep(0.0005)

def get_distance() -> int:
    start_range()
    wait_ready()
    dist = read_byte(VL6180X_ADDRESS, 0x062)
    clear_interrupts()
    return dist

# ---------------------------------------------------------------------------
# 3.  Clean shutdown on Ctrl-C
# ---------------------------------------------------------------------------
def tidy_exit(signum=None, frame=None):
    try:
        # disable all mux outputs we touched
        for mux in {m for _, m, _ in SENSORS}:
            bus.write_byte(mux, 0x00)
    finally:
        bus.close()
        sys.exit(0)

signal.signal(signal.SIGINT, tidy_exit)
signal.signal(signal.SIGTERM, tidy_exit)

# ---------------------------------------------------------------------------
# 4.  Main polling loop
# ---------------------------------------------------------------------------
while True:
    for name, mux, chan in SENSORS:
        try:
            select_channel(mux, chan)

            # one-time init per sensor
            if not initialised[(mux, chan)]:
                initialise_vl6180x()
                initialised[(mux, chan)] = True

            dmm = get_distance()
            print(f"{name:<7} (0x{mux:02X}/CH{chan}): {dmm:3d} mm")

        except OSError as e:
            # e.g. open-circuit sensor — keep going, mark as not ready
            print(f"{name:<7} (0x{mux:02X}/CH{chan}): I²C error → {e}")

    time.sleep(0.2)      # ~5 Hz overall update rate for ten sensors
