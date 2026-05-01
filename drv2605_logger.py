"""
drv2605_logger.py

Reads the DRV2605 benchmark UART output and saves a CSV.
Run this WHILE the STM32 is running the benchmark firmware.

Usage:
    python drv2605_logger.py --port COM3          (Windows)
    python drv2605_logger.py --port /dev/ttyACM0  (Linux/Mac)

Output:
    drv2605_results.csv   <- raw data, one row per trigger event
    drv2605_summary.txt   <- mean, std, 99th pct, max printed at end

Columns in CSV:
    event      - event number (1-based)
    i2c_cyc    - DWT cycle count of I2C GO transaction
    i2c_us     - microseconds at 170 MHz
"""

import argparse
import csv
import os
import time
import statistics
import serial

# ── CLI args ──────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument("--port",  default="/dev/tty.usbmodem103",   help="Serial port")
parser.add_argument("--baud",  default=115200,   type=int)
parser.add_argument("--out",   default="drv2605_results.csv")
parser.add_argument("--total", default=10000,    type=int,
                    help="Stop after this many events (must match firmware)")
args = parser.parse_args()

# ── Open serial ───────────────────────────────────────────────────────────────
print(f"Opening {args.port} @ {args.baud}...")
ser = serial.Serial(args.port, args.baud, timeout=2)
time.sleep(0.5)
ser.reset_input_buffer()

# ── Open CSV ──────────────────────────────────────────────────────────────────
csv_file = open(args.out, "w", newline="")
writer   = csv.writer(csv_file)
writer.writerow(["event", "i2c_cyc", "i2c_us"])

i2c_us_list = []
event_count  = 0

print("Waiting for BOOT...")

try:
    while True:
        raw = ser.readline()
        if not raw:
            continue

        try:
            line = raw.decode("ascii", errors="replace").strip()
        except Exception:
            continue

        if not line:
            continue

        # ── Print everything so you can see it live ──────────────────────────
        print(line)

        # ── Skip comment/header lines ────────────────────────────────────────
        if line.startswith("#") or line.startswith("BOOT") or line.startswith("DRV"):
            # Still print WCET summaries — they're useful live feedback
            continue

        # ── Parse data line: event,i2c_cyc,i2c_us ───────────────────────────
        parts = line.split(",")
        if len(parts) != 3:
            continue

        try:
            event   = int(parts[0])
            i2c_cyc = int(parts[1])
            i2c_us  = float(parts[2])
        except ValueError:
            continue

        writer.writerow([event, i2c_cyc, i2c_us])
        i2c_us_list.append(i2c_us)
        event_count += 1

        # ── Progress every 500 events ─────────────────────────────────────────
        if event_count % 500 == 0:
            print(f"  → {event_count}/{args.total} events captured")

        # ── Stop when done ────────────────────────────────────────────────────
        if event_count >= args.total:
            print("\nAll events captured. Computing statistics...")
            break

except KeyboardInterrupt:
    print("\nInterrupted by user.")

finally:
    csv_file.close()
    ser.close()

# ── Summary statistics ────────────────────────────────────────────────────────
if len(i2c_us_list) < 2:
    print("Not enough data for statistics.")
else:
    n      = len(i2c_us_list)
    mean   = statistics.mean(i2c_us_list)
    stdev  = statistics.stdev(i2c_us_list)
    mn     = min(i2c_us_list)
    mx     = max(i2c_us_list)

    sorted_vals = sorted(i2c_us_list)
    p99_idx = int(0.99 * n)
    p99     = sorted_vals[p99_idx]

    summary = (
        f"DRV2605 I2C GO Transaction — Latency Summary\n"
        f"{'='*48}\n"
        f"  Events measured : {n}\n"
        f"  Mean            : {mean:.3f} µs\n"
        f"  Std Dev         : {stdev:.3f} µs\n"
        f"  Min             : {mn:.3f} µs\n"
        f"  99th percentile : {p99:.3f} µs\n"
        f"  Max (WCET)      : {mx:.3f} µs\n"
        f"{'='*48}\n"
        f"  1 kHz budget    : 1000.000 µs\n"
        f"  Budget headroom : {1000.0 - mx:.3f} µs\n"
        f"\n"
        f"NOTE: WCET above is i2c_cyc only (t0→t1).\n"
        f"      Mechanical onset (t2) is read from oscilloscope\n"
        f"      by measuring PA5 LOW edge to motor terminal voltage rise.\n"
    )

    print(summary)

    with open("drv2605_summary.txt", "w") as f:
        f.write(summary)
    print(f"Summary saved → drv2605_summary.txt")

print(f"Data saved → {args.out}")
