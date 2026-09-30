#!/usr/bin/env python3
"""CleanMate Pi console. Usage: python3 cleanmate_pi.py [/dev/ttyUSB0]
Type commands (d 0.15 | s | r | g 170 | k | quit). Live status prints twice a second.
Everything received is logged to cleanmate_log.csv.   Needs: pip install pyserial"""
import sys, time, threading, csv
import serial

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
ser = serial.Serial(port, 115200, timeout=0.2)
STATE = {0: "IDLE", 1: "RUN", 2: "PAUSE", 3: "FAULT"}
FAULTS = ["none", "comms-lost", "gap-too-near", "gap-lost", "long-error", "obstacle", "low-battery",
          "stall", "partner-fault", "link-lost"]
wlock = threading.Lock()
last = {}
lock = threading.Lock()
running = True
log = open("cleanmate_log.csv", "a", newline="")
w = csv.writer(log)
w.writerow(["pc_time", "id", "state", "seq", "t_ms", "encL", "encR", "heading", "dist_m", "v_mps",
            "gap_mm", "fl_mm", "fr_mm", "batt_mv", "flags"])


def reader():
    while running:
        try:
            line = ser.readline().decode(errors="ignore").strip()
        except Exception:
            continue
        if not line:
            continue
        if line.startswith("T,"):
            f = line.split(",")[1:]
            if len(f) == 14:
                with lock:
                    last[int(f[0])] = (time.time(), f)
                w.writerow([f"{time.time():.3f}"] + f)
                log.flush()
        else:
            print("\n" + line)


def heartbeat():
    while running:
        try:
            with wlock:
                ser.write(b"h\n")
        except Exception:
            pass
        time.sleep(0.2)


def status():
    while running:
        time.sleep(0.5)
        with lock:
            parts = []
            for rid, name in ((1, "LEAD"), (2, "FOLL")):
                if rid in last:
                    ts, f = last[rid]
                    age = time.time() - ts
                    st = STATE.get(int(f[1]), '?')
                    if int(f[1]) == 3:
                        code = (int(f[13]) >> 3) & 0xF
                        st = "FAULT:" + (FAULTS[code] if code < len(FAULTS) else str(code))
                    parts.append(f"{name} {st:5s} v={float(f[8]):.2f} "
                                 f"d={float(f[7]):.2f} yaw={float(f[6]):+.1f} gap={f[9]} "
                                 f"batt={int(f[12])/1000:.1f}V{' STALE' if age > 1 else ''}")
                else:
                    parts.append(f"{name} --")
        print("\r" + " | ".join(parts) + "   ", end="", flush=True)


threading.Thread(target=reader, daemon=True).start()
threading.Thread(target=status, daemon=True).start()
threading.Thread(target=heartbeat, daemon=True).start()
print("connected to", port, "- commands: d 0.15 | s | r | g 170 | k | quit")
try:
    while True:
        cmd = input().strip()
        if cmd == "quit":
            break
        if cmd:
            with wlock:
                ser.write((cmd + "\n").encode())
except (KeyboardInterrupt, EOFError):
    pass
finally:
    try:
        ser.write(b"s\n")
    except Exception:
        pass
    running = False
