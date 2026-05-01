#!/usr/bin/env python3
"""
Test C Unity<->Python<->STM32 bridge + logger  VERSION 5
---------------------------------------------------------
Key fix over V4:
  - Python clamps the value sent TO Unity using vmax_held
    So Unity NEVER receives a value past the wall boundary
    = finger physically cannot appear through the wall
  - vmax_held still only decreases inside wall (spike immune)
  - Wall releases cleanly on exit
  - Calibration: F200, B3000, I0 on startup

Usage (Mac):
  python testC_unity_bridge_logger_Version5.py /dev/cu.usbmodem103 115200 300 127.0.0.1 5015 5016
"""

import sys
import time
import csv
import socket
import threading
from pathlib import Path
from collections import deque

import serial

# ---------------------------- Config ----------------------------
SEND_RATE_HZ          = 100.0
SER_TIMEOUT           = 0.05
SOCK_TIMEOUT          = 0.01
V_SEND_MIN_INTERVAL_S = 0.005
LIVE_PRINT_INTERVAL_S = 1.0

# ------------------------- Shared State -------------------------
state_lock = threading.Lock()

latest_stm32 = {
    "pc_ms": 0.0,
    "stm32_ms": 0,
    "raw": 0,
    "ema": 0,
    "milli": 0,
    "servo": 0,
    "vmax_milli_fw": 1000,
    "flags": 0,
    "spike_mag": 0,
    "spike_count": 0,
    "spike_max_mag": 0,
    "alpha": 0.5,
    "serial_dt_ms": 0.0,
}

latest_unity = {
    "seq": -1,
    "send_ms": 0.0,
    "unity_recv_ms": 0.0,
    "vmax": 1.0,
    "inside_wall": 0,
    "echo_pc_ms": 0.0,
    "echo_rtt_ms_est": 0.0,
}

stm32_row_count = 0
stop_flag       = False

# ---- Wall boundary state (Python-side, spike-immune) -----------
wall_lock  = threading.Lock()
vmax_held  = 1.0
inside_now = False

# -------------------------- Utilities --------------------------
def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def now_ms():
    return time.perf_counter() * 1000.0

def parse_stm32_line(line: str):
    p = line.strip().split(",")
    if len(p) != 11:
        return None
    try:
        return {
            "stm32_ms":      int(p[0]),
            "raw":           int(p[1]),
            "ema":           int(p[2]),
            "milli":         int(p[3]),
            "servo":         int(p[4]),
            "vmax_milli_fw": int(p[5]),
            "flags":         int(p[6]),
            "spike_mag":     int(p[7]),
            "spike_count":   int(p[8]),
            "spike_max_mag": int(p[9]),
            "alpha":         float(p[10]),
        }
    except ValueError:
        return None

def parse_unity_echo(msg: str):
    p = msg.strip().split(",")
    if len(p) != 5:
        return None
    try:
        return {
            "seq":           int(p[0]),
            "send_ms":       float(p[1]),
            "unity_recv_ms": float(p[2]),
            "vmax":          float(p[3]),
            "inside_wall":   int(p[4]),
        }
    except ValueError:
        return None

# ---------------------- Worker: Serial RX ----------------------
def serial_reader(ser, log_queue):
    global stop_flag, stm32_row_count
    last_pc_ms = None
    while not stop_flag:
        try:
            raw_line = ser.readline().decode(errors="ignore").strip()
            if not raw_line:
                continue
            if raw_line.startswith("#"):
                print(f"  [FW] {raw_line}")
                continue
            parsed = parse_stm32_line(raw_line)
            if parsed is None:
                continue

            pc = now_ms()
            dt = 0.0 if last_pc_ms is None else (pc - last_pc_ms)
            last_pc_ms = pc

            with state_lock:
                latest_stm32.update(parsed)
                latest_stm32["pc_ms"]       = pc
                latest_stm32["serial_dt_ms"] = dt
                stm32_row_count += 1
                row = {"pc_ms": pc, **latest_stm32, **latest_unity}

            log_queue.append(row)
        except Exception:
            continue

# -------------------- Worker: UDP Echo RX ----------------------
def unity_echo_listener(sock_echo, ser, log_queue):
    global stop_flag, vmax_held, inside_now
    last_v_sent_milli = 1000
    last_v_sent_t     = 0.0

    while not stop_flag:
        try:
            data, _addr = sock_echo.recvfrom(2048)
            txt    = data.decode(errors="ignore").strip()
            parsed = parse_unity_echo(txt)
            if parsed is None:
                continue

            pc      = now_ms()
            rtt_est = pc - parsed["send_ms"]

            unity_vmax   = clamp(parsed["vmax"], 0.0, 1.0)
            unity_inside = parsed["inside_wall"]

            # ---------------------------------------------------
            # Wall boundary — spike immune
            # vmax_held only decreases while inside wall
            # Resets to 1.0 on exit
            # ---------------------------------------------------
            with wall_lock:
                if unity_inside:
                    inside_now = True
                    if unity_vmax < vmax_held:
                        vmax_held = unity_vmax
                else:
                    if inside_now:
                        vmax_held  = 1.0
                        inside_now = False
                v_to_send = vmax_held

            v_milli = int(round(v_to_send * 1000.0))

            with state_lock:
                latest_unity.update(parsed)
                latest_unity["echo_pc_ms"]      = pc
                latest_unity["echo_rtt_ms_est"]  = rtt_est

            # Send Vxxx to STM32
            tnow = time.perf_counter()
            if (v_milli != last_v_sent_milli) or ((tnow - last_v_sent_t) >= 0.2):
                if (tnow - last_v_sent_t) >= V_SEND_MIN_INTERVAL_S:
                    cmd = f"V{v_milli}\n".encode("ascii", errors="ignore")
                    try:
                        ser.write(cmd)
                        ser.flush()
                        last_v_sent_milli = v_milli
                        last_v_sent_t     = tnow
                    except Exception:
                        pass

            with state_lock:
                row = {"pc_ms": pc, **latest_stm32, **latest_unity}
            log_queue.append(row)

        except socket.timeout:
            continue
        except Exception:
            continue

# -------------------- Worker: UDP Send Unity -------------------
def unity_sender(sock_tx, unity_addr):
    global stop_flag
    seq    = 0
    period = 1.0 / SEND_RATE_HZ
    next_t = time.perf_counter()

    while not stop_flag:
        with state_lock:
            raw_value = clamp(latest_stm32["milli"] / 1000.0, 0.0, 1.0)

        # KEY FIX V5: clamp value to wall boundary BEFORE sending to Unity
        # Unity will never receive a value past the wall = no visual penetration
        with wall_lock:
            value = clamp(raw_value, 0.0, vmax_held)

        send_ms = now_ms()
        payload = f"{seq},{send_ms:.3f},{value:.4f}".encode("utf-8")
        try:
            sock_tx.sendto(payload, unity_addr)
        except Exception:
            pass

        seq    += 1
        next_t += period
        sleep_s = next_t - time.perf_counter()
        if sleep_s > 0:
            time.sleep(sleep_s)
        else:
            next_t = time.perf_counter()

# -------------------- Live terminal printer --------------------
def live_printer(duration_s, t0):
    global stop_flag, stm32_row_count
    last_print     = time.perf_counter()
    no_data_warned = False

    while not stop_flag:
        time.sleep(0.2)
        now     = time.perf_counter()
        elapsed = now - t0

        if (now - last_print) >= LIVE_PRINT_INTERVAL_S:
            last_print = now

            with state_lock:
                raw_val   = latest_stm32["raw"]
                ema_val   = latest_stm32["ema"]
                milli_val = latest_stm32["milli"]
                vmax_fw   = latest_stm32["vmax_milli_fw"]
                flags     = latest_stm32["flags"]
                u_seq     = latest_unity["seq"]
                rows      = stm32_row_count

            with wall_lock:
                vh      = vmax_held
                in_wall = inside_now

            wall_clamp  = bool(flags & 2)
            spike_fired = bool(flags & 1)

            if rows == 0 and elapsed > 5.0 and not no_data_warned:
                print("\n  *** WARNING: No STM32 data after 5s — check board reset ***\n")
                no_data_warned = True

            parts = []
            if wall_clamp:  parts.append("WALL-CLAMPED")
            if spike_fired: parts.append("SPIKE")
            if in_wall:     parts.append("IN-WALL")
            status = " ".join(parts)

            print(
                f"  t={elapsed:5.1f}s | "
                f"raw={raw_val:4d}  ema={ema_val:4d}  "
                f"milli={milli_val:4d}  vmax_fw={vmax_fw:4d}  "
                f"vmax_held={vh:.3f} | "
                f"seq={u_seq}  rows={rows}"
                + (f"  [{status}]" if status else "")
            )

# ----------------------------- Main -----------------------------
def main():
    global stop_flag

    default_port  = "/dev/cu.usbmodem103"
    serial_port   = sys.argv[1] if len(sys.argv) > 1 else default_port
    baud          = int(sys.argv[2])   if len(sys.argv) > 2 else 115200
    duration_s    = float(sys.argv[3]) if len(sys.argv) > 3 else 300.0
    unity_ip      = sys.argv[4]        if len(sys.argv) > 4 else "127.0.0.1"
    unity_rx_port = int(sys.argv[5])   if len(sys.argv) > 5 else 5015
    py_echo_port  = int(sys.argv[6])   if len(sys.argv) > 6 else 5016

    outdir  = Path.home() / "Documents" / "glove_logs"
    outdir.mkdir(parents=True, exist_ok=True)
    ts      = time.strftime("%Y%m%d_%H%M%S")
    outfile = outdir / f"testC_unity_bridge_{ts}.csv"

    print(f"[INIT] Opening serial {serial_port} @ {baud}...")
    ser = serial.Serial(serial_port, baud, timeout=SER_TIMEOUT)
    time.sleep(1.5)
    ser.reset_input_buffer()

    # Startup commands — I0=no invert, F150=flat floor, B3000=bent ceiling
    startup_cmds = ["I0\n", "F150\n", "B3000\n", "A0.5\n", "V1000\n", "R\n"]
    for cmd in startup_cmds:
        ser.write(cmd.encode("ascii", errors="ignore"))
        ser.flush()
        time.sleep(0.05)
    print(f"[INIT] Sent: {[c.strip() for c in startup_cmds]}")
    print(f"[INIT] Calibration: FLAT=150, BENT=3000, INVERT=OFF")
    print(f"[INIT] V5 FIX: value sent to Unity is pre-clamped to vmax_held")
    print()

    sock_tx   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_echo = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_echo.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock_echo.bind(("0.0.0.0", py_echo_port))
    sock_echo.settimeout(SOCK_TIMEOUT)

    unity_addr = (unity_ip, unity_rx_port)
    log_queue  = deque(maxlen=200000)
    t0         = time.perf_counter()

    th_ser   = threading.Thread(target=serial_reader,       args=(ser, log_queue),            daemon=True)
    th_echo  = threading.Thread(target=unity_echo_listener, args=(sock_echo, ser, log_queue), daemon=True)
    th_tx    = threading.Thread(target=unity_sender,        args=(sock_tx, unity_addr),       daemon=True)
    th_print = threading.Thread(target=live_printer,        args=(duration_s, t0),            daemon=True)

    th_ser.start(); th_echo.start(); th_tx.start(); th_print.start()

    print(f"[RUN] Serial={serial_port}@{baud}, duration={duration_s:.0f}s")
    print(f"[RUN] Unity RX={unity_ip}:{unity_rx_port}, Echo RX=0.0.0.0:{py_echo_port}")
    print("[RUN] Finger CANNOT pass through wall — pre-clamped before send")
    print("[RUN] Press Ctrl+C to stop early and save CSV.\n")
    print(f"  {'t':>6}  | {'raw':>4}  {'ema':>4}  {'milli':>5}  {'vmax_fw':>7}  {'vmax_held':>9} | seq  rows  status")
    print("  " + "-"*90)

    try:
        while (time.perf_counter() - t0) < duration_s:
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n[STOP] Saving CSV...")
    finally:
        stop_flag = True
        time.sleep(0.3)
        try: ser.write(b"V1000\n"); ser.flush()
        except Exception: pass
        try: sock_echo.close()
        except Exception: pass
        try: sock_tx.close()
        except Exception: pass
        try: ser.close()
        except Exception: pass

    # Write CSV
    fields = [
        "pc_ms",
        "stm32_ms","raw","ema","milli","servo","vmax_milli_fw","flags",
        "spike_mag","spike_count","spike_max_mag","alpha","serial_dt_ms",
        "seq","send_ms","unity_recv_ms","vmax","inside_wall","echo_pc_ms","echo_rtt_ms_est"
    ]

    with open(outfile, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        count = 0
        while log_queue:
            row = log_queue.popleft()
            for k in fields:
                if k not in row:
                    row[k] = ""
            w.writerow(row)
            count += 1

    print(f"\n[DONE] {outfile}")
    print(f"[ROWS] {count} rows saved")
    print(f"\nTo analyze: python testC_analyze.py \"{outfile}\" --plot")

if __name__ == "__main__":
    main()
