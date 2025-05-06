#!/usr/bin/env python3
"""
VL6180X HAND-SENSOR POLLER — with proximity fall-back

• 10 range sensors sit behind two TCA9548A muxes:
      MULT1 (0x77): Thumb1 Thumb2 Index1 Index2 Middle1
      MULT2 (0x73): Middle2 Ring1 Ring2 Pinky1 Pinky2
• Every 0.2 s we sweep all ten devices.
• **If a sensor raises an I²C/timeout error we substitute its reading with
  the first OK value from a hard-wired proximity list** (same finger’s
  mate → adjacent finger → next adjacent …).
  Substituted values are printed with a “~” prefix.
• End-of-sweep summary distinguishes  OK vs SUBSTITUTED vs UNAVAILABLE.
"""
# ---------------------------------------------------------------------

import smbus2, time, signal, sys, collections

VL6180X_ADDR = 0x29
bus          = smbus2.SMBus(1)               # Pi I²C-1

# ------------ wiring table ---------------------------------------------------
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

# ------------ static proximity lists (same finger first) ---------------------
FALLBACK = {
    "Thumb1":  ["Thumb2", "Index1", "Index2", "Middle1"],
    "Thumb2":  ["Thumb1", "Index2", "Index1", "Middle2"],
    "Index1":  ["Index2", "Thumb2", "Middle1", "Middle2"],
    "Index2":  ["Index1", "Middle2", "Thumb2", "Middle1"],
    "Middle1": ["Middle2", "Index1", "Ring1",  "Ring2"],
    "Middle2": ["Middle1", "Index2", "Ring2",  "Ring1"],
    "Ring1":   ["Ring2",   "Middle1","Pinky1", "Pinky2"],
    "Ring2":   ["Ring1",   "Middle2","Pinky2", "Pinky1"],
    "Pinky1":  ["Pinky2",  "Ring1",  "Ring2",  "Middle1"],
    "Pinky2":  ["Pinky1",  "Ring2",  "Ring1",  "Middle2"],
}

# ------------ helpers --------------------------------------------------------
def mux_select(addr, ch):          # choose downstream channel
    bus.write_byte(addr, 1 << ch);  time.sleep(0.0005)

def wb(reg, val):                  # write byte to VL6180X 16-bit reg
    hi, lo = reg >> 8 & 0xFF, reg & 0xFF
    bus.write_i2c_block_data(VL6180X_ADDR, hi, [lo, val])

def rb(reg):                       # read byte
    hi, lo = reg >> 8 & 0xFF, reg & 0xFF
    bus.write_i2c_block_data(VL6180X_ADDR, hi, [lo]);  time.sleep(0.0003)
    return bus.read_byte(VL6180X_ADDR)

def vl_init():                     # one-time power-on sequence
    if rb(0x016) != 1:   # fresh-out-of-reset bit
        return
    for r,v in [(0x0207,1),(0x0208,1),(0x0096,0),(0x0097,0xfd),
                (0x00e3,0),(0x00e4,4),(0x00e5,2),(0x00e6,1),
                (0x00e7,3),(0x00f5,2),(0x00d9,5),(0x00db,0xce),
                (0x00dc,3),(0x00dd,0xf8),(0x009f,0),(0x00a3,0x3c),
                (0x00b7,0),(0x00bb,0x3c),(0x00b2,9),(0x00ca,9),
                (0x0198,1),(0x01b0,0x17),(0x01ad,0),(0x00ff,5),
                (0x0100,5),(0x0199,5),(0x01a6,0x1b),(0x01ac,0x3e),
                (0x01a7,0x1f),(0x0030,0),
                (0x0011,0x10),(0x010a,0x30),(0x003f,0x46),(0x0031,0xFF),
                (0x0041,0x63),(0x002e,1),(0x001b,9),(0x003e,0x31),
                (0x0014,0x24)]:
        wb(r,v)
    wb(0x016,0)                   # clear reset bit

def start_range():  wb(0x018,1)
def clear_int():    wb(0x015,7)

def wait_ready(ms=20):
    t0=time.time()
    while (rb(0x04F)&7)!=4:
        if (time.time()-t0)*1000>ms:
            raise TimeoutError("range-ready timeout")
        time.sleep(0.0003)

def get_distance():
    start_range();  wait_ready();  d=rb(0x062);  clear_int();  return d

# ------------ graceful exit --------------------------------------------------
def shutdown(*_):
    for mux in {m for _,m,_ in SENSORS}:  bus.write_byte(mux,0)
    bus.close();  sys.exit(0)

import sys, signal
signal.signal(signal.SIGINT, shutdown);  signal.signal(signal.SIGTERM, shutdown)

# ------------ runtime state --------------------------------------------------
init_done = collections.defaultdict(bool)   # (mux,ch) → bool

# ------------ main loop ------------------------------------------------------
while True:
    raw   = {}          # immediate measurements or None on failure
    ok    = []          # sensors read directly
    bad   = []          # sensors still missing after substitution
    subs  = []          # substituted sensors

    # 1. collect raw readings
    for name, mux, ch in SENSORS:
        try:
            mux_select(mux, ch)
            if not init_done[(mux,ch)]:
                vl_init();  init_done[(mux,ch)] = True
            raw[name] = get_distance();  ok.append(name)
        except Exception as e:
            raw[name] = None     # mark as missing this sweep

    # 2. substitute missing with nearest healthy neighbour
    final = {}
    for name, _mux, _ch in SENSORS:
        if raw[name] is not None:
            final[name] = raw[name]
        else:
            # search fallback list for first healthy neighbour
            for nb in FALLBACK[name]:
                if raw.get(nb) is not None:
                    final[name] = raw[nb];  subs.append(name);  break
            else:
                final[name] = None;         bad.append(name)

    # 3. pretty print
    for name, *_ in SENSORS:
        val = final[name]
        if val is None:
            print(f"{name:<7} N/A")
        elif name in subs:
            print(f"{name:<7} ~{val:3d} mm")   # '~' marks substitution
        else:
            print(f"{name:<7}  {val:3d} mm")

    print(f"Summary: OK={ok}  SUB={subs}  BAD={bad}\n")
    time.sleep(0.2)          # ≈5 Hz whole-hand update
