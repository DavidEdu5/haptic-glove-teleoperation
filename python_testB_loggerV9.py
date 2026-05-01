#!/usr/bin/env python3
import sys, time
from pathlib import Path
import serial

def main():
    if len(sys.argv) < 2:
        print("Usage: python python_testB_loggerV9.py <port> [baud]")
        sys.exit(1)

    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    duration_s = 60.0

    outdir = Path.home() / "Documents" / "glove_logs"
    outdir.mkdir(parents=True, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    outfile = outdir / f"testB_{ts}.csv"

    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(2.0)
    ser.reset_input_buffer()

    t0 = time.time()
    last_pc_ms = None
    n_ok = n_bad = 0

    with open(outfile, "w", encoding="utf-8") as f:
        f.write("pc_ms,stm32_ms,raw_adc,ema_adc,milli,servo_pulse,serial_dt_ms\n")
        while True:
            now = time.time()
            if now - t0 >= duration_s:
                break

            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue

            p = line.split(",")
            if len(p) != 5:
                n_bad += 1
                continue

            try:
                stm32_ms = int(p[0]); raw = int(p[1]); ema = int(p[2]); mil = int(p[3]); srv = int(p[4])
            except ValueError:
                n_bad += 1
                continue

            pc_ms = now * 1000.0
            if last_pc_ms is None:
                serial_dt_ms = 0.0
            else:
                serial_dt_ms = pc_ms - last_pc_ms
            last_pc_ms = pc_ms

            f.write(f"{pc_ms:.3f},{stm32_ms},{raw},{ema},{mil},{srv},{serial_dt_ms:.3f}\n")
            n_ok += 1

    ser.close()
    print(f"[DONE] {outfile}")
    print(f"[STATS] good={n_ok}, bad={n_bad}, rate~{n_ok/duration_s:.1f} lines/s")

if __name__ == "__main__":
    main()