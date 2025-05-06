#!/usr/bin/env python3
"""
Poll ten VL6180X range sensors behind two TCA9548A multiplexers.
Never pauses on a bad sensor; prints per-sensor error messages
and a summary of functional / non-functional units each sweep.
"""

import smbus2, time, signal, sys, collections

VL6180X_ADDR = 0x29

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

bus             = smbus2.SMBus(1)
initialised     = collections.defaultdict(bool)   # (mux,chan) → bool

def select_channel(mux, ch):            # -------- mux helper
    bus.write_byte(mux, 1 << ch)
    time.sleep(0.0005)

def wb(reg, val):                       # -------- VL6180X helpers
    hi, lo = reg >> 8 & 0xFF, reg & 0xFF
    bus.write_i2c_block_data(VL6180X_ADDR, hi, [lo, val])

def rb(reg):
    hi, lo = reg >> 8 & 0xFF, reg & 0xFF
    bus.write_i2c_block_data(VL6180X_ADDR, hi, [lo])
    time.sleep(0.0003)
    return bus.read_byte(VL6180X_ADDR)

def init_vl6180x():
    if rb(0x016) != 1:          # “fresh-out-of-reset” bit clear → skip
        return
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
    ]: wb(reg, val)
    wb(0x016, 0x00)             # clear reset bit

def start_range():     wb(0x018, 0x01)
def clear_int():       wb(0x015, 0x07)

def wait_ready(timeout_ms=20):
    t0 = time.time()
    while (rb(0x04F) & 0x07) != 0x04:
        if (time.time() - t0)*1000 > timeout_ms:
            raise TimeoutError("range-ready timeout")
        time.sleep(0.0003)

def read_distance():
    start_range()
    wait_ready()
    dist = rb(0x062)
    clear_int()
    return dist

# ---------------- graceful exit ----------------
def shutdown(*_):
    try:
        for mux in {m for _, m, _ in SENSORS}:
            bus.write_byte(mux, 0x00)
    finally:
        bus.close()
        sys.exit(0)

signal.signal(signal.SIGINT,  shutdown)
signal.signal(signal.SIGTERM, shutdown)

# ---------------- main loop ----------------
while True:
    ok, bad = [], []
    for name, mux, ch in SENSORS:
        try:
            select_channel(mux, ch)
            if not initialised[(mux, ch)]:
                init_vl6180x()
                initialised[(mux, ch)] = True

            d = read_distance()
            print(f"{name:<7} {d:3d} mm")
            ok.append(name)

        except Exception as e:
            print(f"{name:<7} ERROR → {e}")
            bad.append(name)

    # ---- sweep summary ----
    if bad:
        print(f"Summary: OK={ok}  BAD={bad}\n")
    else:
        print(f"Summary: all {len(ok)} sensors OK\n")

    time.sleep(0.2)   # ~5 Hz overall update rate
