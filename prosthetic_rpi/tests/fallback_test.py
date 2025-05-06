#!/usr/bin/env python3
"""
VL6180X HAND-SENSOR POLLER (v2) — long-listen + retries

Adds:
• MAX_READY_MS   – configurable time to poll ‘range ready’ bit
• N_RETRIES      – how many times to re-attempt a measurement that
                   timed-out before marking the sensor as missing
Both can be set via CLI:  --wait 200  --retries 5
See bottom of file for defaults.
"""

import smbus2, time, signal, sys, collections, argparse

VL6180X_ADDR = 0x29
bus          = smbus2.SMBus(1)

# --------------------------------------------------------------------------- #
SENSORS = [  # (name, mux-addr, channel)
    ("Thumb1",  0x77, 0), ("Thumb2",  0x77, 1),
    ("Index1",  0x77, 2), ("Index2",  0x77, 3),
    ("Middle1", 0x77, 4), ("Middle2", 0x73, 0),
    ("Ring1",   0x73, 1), ("Ring2",   0x73, 2),
    ("Pinky1",  0x73, 3), ("Pinky2",  0x73, 4),
]

FALLBACK = {   # same-finger first, then nearest neighbours
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
    bus.write_byte(addr, 1 << ch);  time.sleep(0.0005)

def wb(reg,val):
    hi,lo = reg>>8&0xFF, reg&0xFF
    bus.write_i2c_block_data(VL6180X_ADDR, hi,[lo,val])

def rb(reg):
    hi,lo = reg>>8&0xFF, reg&0xFF
    bus.write_i2c_block_data(VL6180X_ADDR, hi,[lo]);  time.sleep(0.0003)
    return bus.read_byte(VL6180X_ADDR)

def vl_init():
    if rb(0x016)!=1: return
    for r,v in [(0x0207,1),(0x0208,1),(0x0096,0),(0x0097,0xfd),
                (0x00e3,0),(0x00e4,4),(0x00e5,2),(0x00e6,1),(0x00e7,3),
                (0x00f5,2),(0x00d9,5),(0x00db,0xce),(0x00dc,3),(0x00dd,0xf8),
                (0x009f,0),(0x00a3,0x3c),(0x00b7,0),(0x00bb,0x3c),(0x00b2,9),
                (0x00ca,9),(0x0198,1),(0x01b0,0x17),(0x01ad,0),(0x00ff,5),
                (0x0100,5),(0x0199,5),(0x01a6,0x1b),(0x01ac,0x3e),(0x01a7,0x1f),
                (0x0030,0),(0x0011,0x10),(0x010a,0x30),(0x003f,0x46),(0x0031,0xFF),
                (0x0041,0x63),(0x002e,1),(0x001b,9),(0x003e,0x31),(0x0014,0x24)]:
        wb(r,v)
    wb(0x016,0)

def start_range(): wb(0x018,1)
def clear_int()  : wb(0x015,7)

def wait_ready(max_ms):
    t0=time.time()
    while (rb(0x04F)&7)!=4:
        if (time.time()-t0)*1000>max_ms:
            raise TimeoutError
        time.sleep(0.0003)

def try_read(max_ms, retries):
    for attempt in range(retries):
        try:
            start_range();  wait_ready(max_ms)
            d = rb(0x062);  clear_int();  return d
        except TimeoutError:
            time.sleep(0.002)  # settle before retry
    raise TimeoutError("range-ready timeout (> {} ms x{} tries)".format(max_ms,retries))

# --------------------------------------------------------------------------- #
def shutdown(*_):
    for mux in {m for _,m,_ in SENSORS}: bus.write_byte(mux,0)
    bus.close();  sys.exit(0)
signal.signal(signal.SIGINT, shutdown);  signal.signal(signal.SIGTERM, shutdown)

# --------------------------------------------------------------------------- #
def main(max_ms, retries):
    init_done = collections.defaultdict(bool)
    while True:
        raw, ok, subs, bad = {}, [], [], []
        # 1) collect
        for name, mux, ch in SENSORS:
            try:
                mux_select(mux,ch)
                if not init_done[(mux,ch)]:
                    vl_init();  init_done[(mux,ch)] = True
                raw[name] = try_read(max_ms, retries);  ok.append(name)
            except Exception as e:
                raw[name] = None

        # 2) substitute
        final={}
        for name, *_ in SENSORS:
            if raw[name] is not None:
                final[name]=raw[name]
            else:
                for nb in FALLBACK[name]:
                    if raw.get(nb) is not None:
                        final[name]=raw[nb]; subs.append(name); break
                else:
                    final[name]=None; bad.append(name)

        # 3) print
        for name,*_ in SENSORS:
            v=final[name]
            if v is None: print(f"{name:<7} N/A")
            elif name in subs: print(f"{name:<7} ~{v:3d} mm")
            else: print(f"{name:<7}  {v:3d} mm")
        print(f"Summary: OK={ok}  SUB={subs}  BAD={bad}\n")
        time.sleep(0.2)

# --------------------------------------------------------------------------- #
if __name__=="__main__":
    ap=argparse.ArgumentParser()
    ap.add_argument("--wait",   type=int, default=120, help="max ms to poll ‘range-ready’")
    ap.add_argument("--retries",type=int, default=3,   help="attempts per sensor")
    args=ap.parse_args()
    main(args.wait, args.retries)
