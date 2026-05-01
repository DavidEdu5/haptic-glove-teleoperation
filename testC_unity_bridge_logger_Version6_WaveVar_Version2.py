#!/usr/bin/env python3
"""
Test C Unity<->Python<->STM32 bridge + logger  VERSION 6 — WAVE VARIABLES (+TDPA parser)
--------------------------------------------------------------------------
New in this patched build:
  - Backward-compatible STM32 parser:
      * old 11-field lines supported
      * new extended 18-field lines supported
  - TDPA telemetry parsed/logged:
      servo_feedback, theta_cmd, theta_actual, omega, F_est, E_tdpa, error
  - CSV columns extended with TDPA fields
  - Live print shows E_tdpa and TDPA-VIOLATION tag if E_tdpa < 0
"""

import sys, time, csv, math, json, socket, threading, argparse, statistics
from pathlib import Path
from datetime import datetime
from collections import deque

import serial

# ========================== Constants ===========================
SEND_RATE_HZ          = 100.0
SER_TIMEOUT           = 0.05
SOCK_TIMEOUT          = 0.01
V_SEND_MIN_INTERVAL_S = 0.005
LIVE_PRINT_INTERVAL_S = 1.0
CALIB_FILE            = Path.home() / ".glove_calibration.json"
CALIB_WINDOW_S        = 2.0
CALIB_STABLE_THRESH   = 30
CALIB_FLAT_DEADZONE   = 20
CALIB_MIN_SPAN        = 200

# ======================== Shared State ==========================
state_lock = threading.Lock()

latest_stm32 = {
    "pc_ms": 0.0, "stm32_ms": 0, "raw": 0, "ema": 0, "milli": 0,
    "servo": 0, "vmax_milli_fw": 1000, "flags": 0, "spike_mag": 0,
    "spike_count": 0, "spike_max_mag": 0, "alpha": 0.5, "serial_dt_ms": 0.0,
    # TDPA extended fields
    "servo_feedback": 0, "theta_cmd": 0.0, "theta_actual": 0.0,
    "omega": 0.0, "F_est": 0.0, "E_tdpa": 0.0, "error": 0.0,
}
latest_unity = {
    "seq": -1, "send_ms": 0.0, "unity_recv_ms": 0.0,
    "vmax": 1.0, "inside_wall": 0, "echo_pc_ms": 0.0, "echo_rtt_ms_est": 0.0,
}
latest_wave = {
    "u_forward": 0.0, "u_return": 0.0, "E_packet": 0.0,
    "E_cumulative": 0.0, "vmax_decoded": 1.0, "wave_passive": 1, "v_sent": 0.0,
}

stm32_row_count = 0
stop_flag       = False
wall_lock       = threading.Lock()
vmax_held       = 1.0
inside_now      = False
wave_lock       = threading.Lock()
E_cumulative    = 0.0

# ========================== Utilities ===========================
def clamp(x, lo, hi):   return lo if x < lo else hi if x > hi else x
def now_ms():            return time.perf_counter() * 1000.0

def send_cmd(ser, cmd: str):
    ser.write((cmd + "\n").encode("ascii", errors="ignore"))
    ser.flush()
    time.sleep(0.05)

def parse_stm32_line(line: str):
    p = line.strip().split(",")

    # Old firmware line: 11 fields
    # New firmware line: 18 fields (11 + 7 TDPA fields)
    if len(p) not in (11, 18):
        return None

    try:
        base = {
            "stm32_ms":      int(p[0]),   "raw":           int(p[1]),
            "ema":           int(p[2]),   "milli":         int(p[3]),
            "servo":         int(p[4]),   "vmax_milli_fw": int(p[5]),
            "flags":         int(p[6]),   "spike_mag":     int(p[7]),
            "spike_count":   int(p[8]),   "spike_max_mag": int(p[9]),
            "alpha":         float(p[10]),
            # defaults for backward compatibility
            "servo_feedback": 0,
            "theta_cmd":      0.0,
            "theta_actual":   0.0,
            "omega":          0.0,
            "F_est":          0.0,
            "E_tdpa":         0.0,
            "error":          0.0,
        }

        if len(p) >= 18:
            base.update({
                "servo_feedback": int(p[11]),
                "theta_cmd":      float(p[12]),
                "theta_actual":   float(p[13]),
                "omega":          float(p[14]),
                "F_est":          float(p[15]),
                "E_tdpa":         float(p[16]),
                "error":          float(p[17]),
            })

        return base
    except ValueError:
        return None

def parse_unity_echo(msg: str):
    p = msg.strip().split(",")
    if len(p) != 5:
        return None
    try:
        return {
            "seq":           int(p[0]),   "send_ms":       float(p[1]),
            "unity_recv_ms": float(p[2]), "vmax":          float(p[3]),
            "inside_wall":   int(p[4]),
        }
    except ValueError:
        return None

# ======================= Calibration ===========================
def print_bar(value: int, lo: int, hi: int, width: int = 40):
    frac   = max(0.0, min(1.0, (value - lo) / max(hi - lo, 1)))
    filled = int(frac * width)
    print(f"  [{'█'*filled}{'░'*(width-filled)}]  ADC={value:4d}", end="\r", flush=True)

def capture_adc(ser, duration_s: float, lo: int, hi: int) -> list:
    samples  = []
    deadline = time.perf_counter() + duration_s
    while time.perf_counter() < deadline:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split(",")
            if len(parts) >= 11:
                raw = int(parts[1])
                samples.append(raw)
                print_bar(raw, lo, hi)
        except Exception:
            continue
    print()
    return samples

def run_calibration_wizard(ser) -> dict:
    print("\n╔══════════════════════════════════════════════════════╗")
    print("║        FLEX SENSOR CALIBRATION WIZARD               ║")
    print("╚══════════════════════════════════════════════════════╝\n")

    for cmd in ["I0", "F50", "B4000", "A0.5", "V1000"]:
        send_cmd(ser, cmd)
    time.sleep(0.2)
    ser.reset_input_buffer()

    print("─" * 54)
    print("  STEP 1 of 2 — FLAT (open hand)")
    print("─" * 54)
    print("  Lay your hand FLAT and OPEN on a surface.")
    input("  Hold still → press ENTER to capture...\n")

    flat_samples = capture_adc(ser, CALIB_WINDOW_S, 0, 4095)
    if len(flat_samples) < 10:
        print("  ✗ No data received — check STM32 is connected.\n")
        sys.exit(1)

    flat_mean = int(statistics.mean(flat_samples))
    flat_std  = statistics.stdev(flat_samples) if len(flat_samples) > 1 else 0.0
    flat_val  = flat_mean + CALIB_FLAT_DEADZONE

    print(f"  mean={flat_mean}  std={flat_std:.1f}  → FLAT set to {flat_val}")
    if flat_std > CALIB_STABLE_THRESH:
        print(f"  ⚠ Noisy signal — try holding your hand more still")

    print()
    print("─" * 54)
    print("  STEP 2 of 2 — BENT (tight fist)")
    print("─" * 54)
    print("  Make the TIGHTEST FIST you can.")
    input("  Hold still → press ENTER to capture...\n")

    bent_samples = capture_adc(ser, CALIB_WINDOW_S, flat_val, 4095)
    if len(bent_samples) < 10:
        print("  ✗ No data received — check STM32 is connected.\n")
        sys.exit(1)

    bent_mean = int(statistics.mean(bent_samples))
    bent_std  = statistics.stdev(bent_samples) if len(bent_samples) > 1 else 0.0
    bent_val  = bent_mean

    print(f"  mean={bent_mean}  std={bent_std:.1f}  → BENT set to {bent_val}")
    if bent_std > CALIB_STABLE_THRESH:
        print(f"  ⚠ Noisy signal — try holding your fist more still")

    span = bent_val - flat_val
    print()
    print("─" * 54)
    print(f"  FLAT={flat_val}   BENT={bent_val}   SPAN={span}")

    if span < CALIB_MIN_SPAN:
        print(f"  ✗ Span too small ({span}) — sensor may not be connected.")
        print("    Check wiring then re-run with --recalibrate\n")
        sys.exit(1)
    elif span < 500:
        print(f"  ⚠ Small span ({span}) — results may be noisy")
    else:
        print(f"  ✓ Good span ({span}) — calibration looks healthy")

    cal = {
        "flat":      flat_val,
        "bent":      bent_val,
        "alpha":     0.5,
        "invert":    0,
        "timestamp": datetime.now().isoformat(timespec="seconds"),
    }
    with open(CALIB_FILE, "w") as f:
        json.dump(cal, f, indent=2)

    print(f"\n  ✓ Saved to {CALIB_FILE}")
    print("─" * 54 + "\n")
    return cal

def load_or_calibrate(ser, force_recal: bool) -> dict:
    cal = None

    if not force_recal and CALIB_FILE.exists():
        try:
            with open(CALIB_FILE) as f:
                cal = json.load(f)
            print(f"[CAL]  Loaded: FLAT={cal['flat']}  BENT={cal['bent']}"
                  f"  (saved {cal.get('timestamp', '?')})")
        except Exception as e:
            print(f"[CAL]  Could not read calibration ({e}) — running wizard")
            cal = None

    if cal is None:
        if not force_recal:
            print("[CAL]  No calibration file found — launching wizard...")
        cal = run_calibration_wizard(ser)

    send_cmd(ser, f"I{cal.get('invert', 0)}")
    send_cmd(ser, f"F{cal['flat']}")
    send_cmd(ser, f"B{cal['bent']}")
    send_cmd(ser, f"A{cal.get('alpha', 0.5):.2f}")
    send_cmd(ser, "V1000")
    send_cmd(ser, "R")
    return cal

# ====================== Wave Variable Layer ====================
def wave_encode(v: float, F: float, b: float) -> float:
    return (v + b * F) / math.sqrt(2.0 * b)

def wave_decode(u_r: float, v: float, b: float) -> float:
    return clamp((u_r * math.sqrt(2.0 * b) - v) / b, 0.0, 1.0)

def wave_energy_packet(u_f: float, u_r: float) -> float:
    return u_f**2 - u_r**2

# ==================== Worker: Serial RX ========================
def serial_reader(ser, log_queue):
    global stop_flag, stm32_row_count
    last_pc_ms = None
    while not stop_flag:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            if not line or line.startswith("#"):
                continue
            parsed = parse_stm32_line(line)
            if parsed is None:
                continue
            pc = now_ms()
            dt = 0.0 if last_pc_ms is None else (pc - last_pc_ms)
            last_pc_ms = pc
            with state_lock:
                latest_stm32.update(parsed)
                latest_stm32["pc_ms"]        = pc
                latest_stm32["serial_dt_ms"] = dt
                stm32_row_count += 1
                row = {"pc_ms": pc, **latest_stm32, **latest_unity, **latest_wave}
            log_queue.append(row)
        except Exception:
            continue

# =================== Worker: UDP Echo RX =======================
def unity_echo_listener(sock_echo, ser, log_queue, b, use_wave):
    global stop_flag, vmax_held, inside_now, E_cumulative
    last_v_sent_milli = 1000
    last_v_sent_t     = 0.0

    while not stop_flag:
        try:
            data, _ = sock_echo.recvfrom(2048)
            parsed  = parse_unity_echo(data.decode(errors="ignore"))
            if parsed is None:
                continue

            pc           = now_ms()
            rtt_est      = pc - parsed["send_ms"]
            unity_vmax   = clamp(parsed["vmax"], 0.0, 1.0)
            unity_inside = parsed["inside_wall"]

            if use_wave:
                with state_lock:
                    u_f_last = latest_wave.get("u_forward", 0.0)
                    v_sent   = latest_wave.get("v_sent",    0.0)
                u_r   = wave_encode(v_sent, unity_vmax, b)
                E_pkt = wave_energy_packet(u_f_last, u_r)
                with wave_lock:
                    E_cumulative += E_pkt
                    E_cum_snap    = E_cumulative
                with state_lock:
                    latest_wave.update({
                        "u_return":     u_r,
                        "E_packet":     E_pkt,
                        "E_cumulative": E_cum_snap,
                        "vmax_decoded": unity_vmax,
                        "wave_passive": 1 if E_pkt >= 0.0 else 0,
                    })
            else:
                with state_lock:
                    latest_wave["u_return"]     = unity_vmax
                    latest_wave["E_packet"]     = 0.0
                    latest_wave["wave_passive"] = 1

            effective_vmax = unity_vmax

            with wall_lock:
                if unity_inside:
                    inside_now = True
                    if effective_vmax < vmax_held:
                        vmax_held = effective_vmax
                else:
                    if inside_now:
                        vmax_held  = 1.0
                        inside_now = False
                v_to_send = vmax_held

            v_milli = int(round(v_to_send * 1000.0))
            with state_lock:
                latest_unity.update(parsed)
                latest_unity["echo_pc_ms"]      = pc
                latest_unity["echo_rtt_ms_est"] = rtt_est
                row = {"pc_ms": pc, **latest_stm32, **latest_unity, **latest_wave}
            log_queue.append(row)

            tnow = time.perf_counter()
            if (v_milli != last_v_sent_milli) or ((tnow - last_v_sent_t) >= 0.2):
                if (tnow - last_v_sent_t) >= V_SEND_MIN_INTERVAL_S:
                    try:
                        ser.write(f"V{v_milli}\n".encode("ascii", errors="ignore"))
                        ser.flush()
                        last_v_sent_milli = v_milli
                        last_v_sent_t     = tnow
                    except Exception:
                        pass

        except socket.timeout:
            continue
        except Exception:
            continue

# =================== Worker: UDP Send Unity ====================
def unity_sender(sock_tx, unity_addr, b, use_wave):
    global stop_flag
    seq    = 0
    period = 1.0 / SEND_RATE_HZ
    next_t = time.perf_counter()

    while not stop_flag:
        with state_lock:
            raw_value = clamp(latest_stm32["milli"] / 1000.0, 0.0, 1.0)
        with wall_lock:
            F_current = vmax_held
            value     = clamp(raw_value, 0.0, vmax_held)

        if use_wave:
            u_f = wave_encode(value, F_current, b)
            with state_lock:
                latest_wave["u_forward"] = u_f
                latest_wave["v_sent"]    = value
            payload_val = value
        else:
            payload_val = value

        send_ms = now_ms()
        try:
            sock_tx.sendto(
                f"{seq},{send_ms:.3f},{payload_val:.6f}".encode("utf-8"),
                unity_addr
            )
        except Exception:
            pass

        seq    += 1
        next_t += period
        sleep_s = next_t - time.perf_counter()
        if sleep_s > 0:
            time.sleep(sleep_s)
        else:
            next_t = time.perf_counter()

# ================ Live Terminal Printer ========================
def live_printer(duration_s, t0, use_wave):
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
                e_tdpa    = latest_stm32["E_tdpa"]
                u_seq     = latest_unity["seq"]
                rows      = stm32_row_count
                E_cum     = latest_wave["E_cumulative"]
                passive   = latest_wave["wave_passive"]
            with wall_lock:
                vh      = vmax_held
                in_wall = inside_now

            if rows == 0 and elapsed > 5.0 and not no_data_warned:
                print("\n  *** WARNING: No STM32 data after 5 s — check board ***\n")
                no_data_warned = True

            tags = []
            if flags & 2:                tags.append("WALL-CLAMPED")
            if flags & 1:                tags.append("SPIKE")
            if in_wall:                  tags.append("IN-WALL")
            if use_wave and not passive: tags.append("WAVE-VIOLATION!")
            if e_tdpa < 0.0:             tags.append("TDPA-VIOLATION!")

            print(
                f"  t={elapsed:5.1f}s | "
                f"raw={raw_val:4d}  ema={ema_val:4d}  milli={milli_val:4d}  "
                f"vmax_fw={vmax_fw:4d}  vmax_held={vh:.3f} | "
                f"seq={u_seq}  rows={rows}  E_tdpa={e_tdpa:+.4f}"
                + (f"  E_cum={E_cum:+.4f}" if use_wave else "")
                + (f"  [{' '.join(tags)}]" if tags else "")
            )

# ============================ Main ==============================
def main():
    global stop_flag

    ap = argparse.ArgumentParser(description="V6 Wave-Variable Haptic Bridge (+TDPA log parser)")
    ap.add_argument("serial_port",   nargs="?", default="/dev/cu.usbmodem103")
    ap.add_argument("baud",          nargs="?", type=int,   default=115200)
    ap.add_argument("duration_s",    nargs="?", type=float, default=300.0)
    ap.add_argument("unity_ip",      nargs="?", default="127.0.0.1")
    ap.add_argument("unity_rx_port", nargs="?", type=int,   default=5015)
    ap.add_argument("py_echo_port",  nargs="?", type=int,   default=5016)
    ap.add_argument("--wave-b",      type=float, default=1.0,
                    help="Wave impedance constant b > 0 (default 1.0)")
    ap.add_argument("--no-wave",     action="store_true",
                    help="Disable wave encoding — scalar fallback")
    ap.add_argument("--recalibrate", action="store_true",
                    help="Force re-run of calibration wizard even if file exists")
    args = ap.parse_args()

    b        = max(args.wave_b, 1e-6)
    use_wave = not args.no_wave

    print(f"[INIT] Opening serial {args.serial_port} @ {args.baud}...")
    ser = serial.Serial(args.serial_port, args.baud, timeout=SER_TIMEOUT)
    time.sleep(1.5)
    ser.reset_input_buffer()

    cal = load_or_calibrate(ser, args.recalibrate)

    print(f"[INIT] Mode      : {'WAVE VARIABLES (b=' + str(b) + ')' if use_wave else 'SCALAR fallback'}")
    print(f"[INIT] Stability : Lyapunov (STM32 local) + Wave passivity (UDP channel)")
    print(f"[INIT] Cal       : FLAT={cal['flat']}  BENT={cal['bent']}  "
          f"ALPHA={cal.get('alpha', 0.5)}  INVERT={cal.get('invert', 0)}")
    print()

    outdir  = Path.home() / "Documents" / "glove_logs"
    outdir.mkdir(parents=True, exist_ok=True)
    outfile = outdir / f"testC_v6_wave_{time.strftime('%Y%m%d_%H%M%S')}.csv"

    sock_tx   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_echo = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_echo.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock_echo.bind(("0.0.0.0", args.py_echo_port))
    sock_echo.settimeout(SOCK_TIMEOUT)

    unity_addr = (args.unity_ip, args.unity_rx_port)
    log_queue  = deque(maxlen=200000)
    t0         = time.perf_counter()

    for target, targs in [
        (serial_reader,       (ser, log_queue)),
        (unity_echo_listener, (sock_echo, ser, log_queue, b, use_wave)),
        (unity_sender,        (sock_tx, unity_addr, b, use_wave)),
        (live_printer,        (args.duration_s, t0, use_wave)),
    ]:
        threading.Thread(target=target, args=targs, daemon=True).start()

    print(f"[RUN] {args.serial_port}@{args.baud}  "
          f"Unity={args.unity_ip}:{args.unity_rx_port}  "
          f"Echo=0.0.0.0:{args.py_echo_port}")
    print(f"[RUN] Wave={use_wave}  b={b:.3f}  duration={args.duration_s:.0f}s")
    print("[RUN] Press Ctrl+C to stop early and save CSV.\n")

    try:
        while (time.perf_counter() - t0) < args.duration_s:
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n[STOP] Saving CSV...")
    finally:
        stop_flag = True
        time.sleep(0.3)
        try:
            ser.write(b"V1000\n")
            ser.flush()
        except Exception:
            pass
        for s in (sock_echo, sock_tx):
            try:
                s.close()
            except Exception:
                pass
        try:
            ser.close()
        except Exception:
            pass

    fields = [
        "pc_ms",
        "stm32_ms", "raw", "ema", "milli", "servo", "vmax_milli_fw", "flags",
        "spike_mag", "spike_count", "spike_max_mag", "alpha", "serial_dt_ms",
        # TDPA extension
        "servo_feedback", "theta_cmd", "theta_actual", "omega", "F_est", "E_tdpa", "error",
        # Unity / wave
        "seq", "send_ms", "unity_recv_ms", "vmax", "inside_wall",
        "echo_pc_ms", "echo_rtt_ms_est",
        "u_forward", "u_return", "E_packet", "E_cumulative", "vmax_decoded", "wave_passive",
    ]
    with open(outfile, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        count = 0
        while log_queue:
            row = log_queue.popleft()
            for k in fields:
                if k not in row:
                    row[k] = ""
            w.writerow(row)
            count += 1

    print(f"\n[DONE] {outfile}  ({count} rows)")
    if use_wave:
        print(f"[WAVE] Final E_cumulative = {E_cumulative:+.6f}  "
              f"→ {'PASSIVE ✓' if E_cumulative >= 0 else 'VIOLATION ✗'}")

if __name__ == "__main__":
    main()