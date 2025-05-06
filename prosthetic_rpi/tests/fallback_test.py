#!/usr/bin/env python3
"""
fallback_test.py  –  ten-sensor VL6180X poller with neighbour substitution

Wiring
------
MULT1 (0x77): Thumb1 Thumb2 Index1 Index2 Middle1
MULT2 (0x73): Middle2 Ring1  Ring2  Pinky1  Pinky2

Features
--------
• For each measurement we wait up to MAX_READY_MS (20 ms) for the
  “range ready” flag.  On timeout we retry N_RETRIES times.
• If a sensor still fails, we substitute the first available reading
  from a hand-tuned proximity list (same finger first, then neighbours).
  Substituted values are shown with a '~' prefix.
• A per-sweep summary lists OK, substituted (SUB), and still-bad (BAD).
"""

import smbus2, time, signal, sys, collections

# --------------------------------------------------------------------------- #
#  User-tunable constants
MAX_READY_MS = 20      # ↑ You found 20 ms works—hard-coded here
N_RETRIES    = 3       # extra attempts before giving up this sweep
SWEEP_HZ     = 5       # ≈ hand update rate (controls sleep at end)

VL6180X_ADDR = 0x29
bus          = smbus2.SMBus(1)

SENSORS = [  # (name, mux-addr, channel)
    ("Thumb1",  0x77, 0), ("Thumb2",  0x77, 1),
    ("Index1",  0x77, 2), ("Index2",  0x77, 3),
    ("Middle1", 0x77, 4), ("Middle2", 0x73, 0),
    ("Ring1",   0x73, 1), ("Ring2",   0x73, 2),
    ("Pinky1",  0x73, 3), ("Pinky2",  0x73, 4),
]

FALLBACK = {
    "Thumb1":["Thumb2","Index1","Index2","Middle1"],
    "Thumb2":["Thumb1","Index2","Index1","Middle2"],
    "Index1":["Index2","Thumb2","Middle1","Middle2"],
    "Index2":["Index1","Middle2","Thumb2","Middle1"],
    "Middle1":["Middle2","Index1","Ring1","Ring2"],
    "Middle2":["Middle1","Index2","Ring2","Ring1"],
    "Ring1"  :["Ring2","Middle1","Pinky1","Pinky2"],
    "Ring2"  :["Ring1","Middle2","Pinky2","Pinky1"],
    "Pinky1" :["Pinky2","Ring1","Ring2","Middle1"],
    "Pinky2" :["Pinky1","Ring2","Ring1","Middle2"],
}

# --------------------------------------------------------------------------- #
def mux_select(addr, ch):
    bus.write_byte(addr, 1 << ch)
    time.sleep(0.0005)

def wb(reg, val):
    hi, lo = reg >> 8 & 0xFF, reg & 0xFF
    bus.write_i2c_block_data(VL6180X_ADDR, hi, [lo, val])

def rb(reg):
    hi, lo = reg >> 8 & 0xFF, reg & 0xFF
    bus.write_i2c_block_data(VL6180X_ADDR, hi, [lo])
    time.sleep(0.0003)
    return bus.read_byte(VL6180X_ADDR)

def vl_init():
    if rb(0x016) != 1:   # “fresh-out-of-reset”
        return
    for r, v in [
        (0x0207,1),(0x0208,1),(0x0096,0),(0x0097,0xfd),
        (0x00e3,0),(0x00e4,4),(0x00e5,2),(0x00e6,1),(0x00e7,3),
        (0x00f5,2),(0x00d9,5),(0x00db,0xce),(0x00dc,3),(0x00dd,0xf8),
        (0x009f,0),(0x00a3,0x3c),(0x00b7,0),(0x00bb,0x3c),(0x00b2,9),
        (0x00ca,9),(0x0198,1),(0x01b0,0x17),(0x01ad,0),(0x00ff,5),
        (0x0100,5),(0x0199,5),(0x01a6,0x1b),(0x01ac,0x3e),(0x01a7,0x1f),
        (0x0030,0),(0x0011,0x10),(0x010a,0x30),(0x003f,0x46),(0x0031,0xFF),
        (0x0041,0x63),(0x002e,1),(0x001b,9),(0x003e,0x31),(0x0014,0x24)
    ]:
        wb(r, v)
    wb(0x016, 0)

def start_range(): wb(0x018, 1)
def clear_int()  : wb(0x015, 7)

def wait_ready():
    t0 = time.time()
    while (rb(0x04F) & 0x07) != 0x04:
        if (time.time() - t0) * 1000 > MAX_READY_MS:
            raise TimeoutError
        time.sleep(0.0003)

def get_distance():
    for _ in range(N_RETRIES):
        try:
            start_range()
            wait_ready()
            d = rb(0x062)
            clear_int()
            return d
        except TimeoutError:
            time.sleep(0.002)   # Let the sensor settle before retry
    raise TimeoutError("range-ready timeout")

# --------------------------------------------------------------------------- #
def shutdown(*_):
    for mux in {m for _, m, _ in SENSORS}:
        bus.write_byte(mux, 0x00)
    bus.close()
    sys.exit(0)

signal.signal(signal.SIGINT,  shutdown)
signal.signal(signal.SIGTERM, shutdown)

# --------------------------------------------------------------------------- #
init_done = collections.defaultdict(bool)     # (mux, ch) → bool

while True:
    raw, ok, subs, bad = {}, [], [], []

    # 1. Grab raw readings
    for name, mux, ch in SENSORS:
        try:
            mux_select(mux, ch)
            if not init_done[(mux, ch)]:
                vl_init()
                init_done[(mux, ch)] = True
            raw[name] = get_distance()
            ok.append(name)
        except Exception:
            raw[name] = None

    # 2. Substitute where needed
    final = {}
    for name, *_ in SENSORS:
        if raw[name] is not None:
            final[name] = raw[name]
        else:
            for nb in FALLBACK[name]:
                if raw.get(nb) is not None:
                    final[name] = raw[nb]
                    subs.append(name)
                    break
            else:
                final[name] = None
                bad.append(name)

    # 3. Print per-sensor values
    for name, *_ in SENSORS:
        v = final[name]
        if v is None:
            print(f"{name:<7} N/A")
        elif name in subs:
            print(f"{name:<7} ~{v:3d} mm")
        else:
            print(f"{name:<7}  {v:3d} mm")

    print(f"Summary: OK={ok}  SUB={subs}  BAD={bad}\n")

    time.sleep(1 / SWEEP_HZ)
