#!/usr/bin/env python3
import argparse
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def xcorr_delay_ms(x, y, fs):
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    x -= np.mean(x)
    y -= np.mean(y)
    c = np.correlate(y, x, mode="full")
    lags = np.arange(-len(x)+1, len(x))
    k = np.argmax(c)
    lag = lags[k]
    return (lag / fs) * 1000.0

def rolling_delay(df, fs=100.0, win_s=6.0, step_s=1.0):
    n = len(df)
    w = int(win_s * fs)
    s = int(step_s * fs)
    t_mid = []
    d_ms = []
    if n < w:
        return np.array([]), np.array([])
    for i in range(0, n - w + 1, s):
        seg = df.iloc[i:i+w]
        d = xcorr_delay_ms(seg["raw_adc"].values, seg["ema_adc"].values, fs)
        t = seg["t_s"].iloc[0] + (seg["t_s"].iloc[-1] - seg["t_s"].iloc[0]) / 2.0
        t_mid.append(t)
        d_ms.append(d)
    return np.array(t_mid), np.array(d_ms)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", help="input testB csv")
    ap.add_argument("--outdir", default="reports", help="output folder")
    args = ap.parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(args.csv).dropna()
    # use stm32_ms as primary timeline
    df["t_s"] = (df["stm32_ms"] - df["stm32_ms"].iloc[0]) / 1000.0

    # Estimate sampling from stm32 timestamp diffs
    dt = np.diff(df["stm32_ms"].values) / 1000.0
    dt = dt[dt > 0]
    fs = 1.0 / np.median(dt) if len(dt) else 100.0

    # Global delay
    d_global = xcorr_delay_ms(df["raw_adc"].values, df["ema_adc"].values, fs)

    # Rolling delay
    t_mid, d_roll = rolling_delay(df, fs=fs, win_s=6.0, step_s=1.0)
    med_abs = float(np.median(np.abs(d_roll))) if len(d_roll) else np.nan
    p95_abs = float(np.percentile(np.abs(d_roll), 95)) if len(d_roll) else np.nan

    # Plot 1 raw vs ema
    plt.figure(figsize=(10,4))
    plt.plot(df["t_s"], df["raw_adc"], label="raw_adc", alpha=0.7)
    plt.plot(df["t_s"], df["ema_adc"], label="ema_adc", alpha=0.9)
    plt.xlabel("Time (s)")
    plt.ylabel("ADC")
    plt.title("Raw vs EMA")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outdir / "01_raw_vs_ema.png", dpi=150)
    plt.close()

    # Plot 2 derivative shock
    d_raw = np.gradient(df["raw_adc"].values.astype(float))
    d_ema = np.gradient(df["ema_adc"].values.astype(float))
    plt.figure(figsize=(10,4))
    plt.plot(df["t_s"], d_raw, label="d(raw)")
    plt.plot(df["t_s"], d_ema, label="d(ema)")
    plt.xlabel("Time (s)")
    plt.ylabel("dADC/sample")
    plt.title("Derivative (shock attenuation view)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outdir / "02_shock_derivative.png", dpi=150)
    plt.close()

    # Plot 3 rolling delay
    plt.figure(figsize=(10,4))
    if len(t_mid):
        plt.plot(t_mid, d_roll, marker="o", label="rolling delay (ms)")
    plt.axhline(0, color="k", linewidth=1)
    plt.xlabel("Time (s)")
    plt.ylabel("Delay (ms)")
    plt.title("Rolling phase delay (raw -> ema)")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outdir / "03_rolling_delay.png", dpi=150)
    plt.close()

    # Plot 4 output channels
    plt.figure(figsize=(10,4))
    plt.plot(df["t_s"], df["milli"], label="milli (0..1000)")
    plt.plot(df["t_s"], df["servo_pulse"], label="servo_pulse (us)")
    plt.xlabel("Time (s)")
    plt.title("Control outputs")
    plt.legend()
    plt.tight_layout()
    plt.savefig(outdir / "04_milli_servo.png", dpi=150)
    plt.close()

    summary = []
    summary.append(f"Input file: {args.csv}")
    summary.append(f"Samples: {len(df)}")
    summary.append(f"Estimated fs: {fs:.3f} Hz")
    summary.append(f"Global xcorr delay (raw->ema): {d_global:.3f} ms")
    summary.append(f"Rolling |delay| median: {med_abs:.3f} ms")
    summary.append(f"Rolling |delay| p95: {p95_abs:.3f} ms")
    summary.append("PASS criterion: rolling |delay| median <= 5.0 ms")
    summary.append(f"PASS: {'YES' if (not np.isnan(med_abs) and med_abs <= 5.0) else 'NO'}")

    (outdir / "summary.txt").write_text("\n".join(summary), encoding="utf-8")
    print("\n".join(summary))
    print(f"\nSaved plots + summary in: {outdir}")

if __name__ == "__main__":
    main()