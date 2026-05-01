#!/usr/bin/env python3
import sys, time, collections
import serial
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets, QtCore

def main():
    if len(sys.argv) < 2:
        print("Usage: python live_scope_testB.py <port> [baud]")
        sys.exit(1)

    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    ser = serial.Serial(port, baud, timeout=0.02)
    time.sleep(1.5)
    ser.reset_input_buffer()

    app = QtWidgets.QApplication([])
    win = pg.GraphicsLayoutWidget(title="STM32 Live Scope")
    win.resize(1200, 800)
    win.show()

    p1 = win.addPlot(title="ADC Raw / EMA")
    p1.addLegend()
    c_raw = p1.plot(pen=pg.mkPen('y', width=2), name="raw_adc")
    c_ema = p1.plot(pen=pg.mkPen('c', width=2), name="ema_adc")
    p1.showGrid(x=True, y=True)

    win.nextRow()
    p2 = win.addPlot(title="Output")
    p2.addLegend()
    c_milli = p2.plot(pen=pg.mkPen('m', width=2), name="milli")
    c_servo = p2.plot(pen=pg.mkPen('g', width=2), name="servo_pulse")
    p2.showGrid(x=True, y=True)

    max_points = 2000
    t = collections.deque(maxlen=max_points)
    raw = collections.deque(maxlen=max_points)
    ema = collections.deque(maxlen=max_points)
    milli = collections.deque(maxlen=max_points)
    servo = collections.deque(maxlen=max_points)

    t0 = time.time()

    def update():
        for _ in range(300):
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                break
            parts = line.split(",")
            if len(parts) != 5:
                continue
            try:
                _ms = int(parts[0])
                _raw = int(parts[1])
                _ema = int(parts[2])
                _mil = int(parts[3])
                _srv = int(parts[4])
            except:
                continue

            tt = time.time() - t0
            t.append(tt)
            raw.append(_raw)
            ema.append(_ema)
            milli.append(_mil)
            servo.append(_srv)

        if len(t) > 2:
            c_raw.setData(t, raw)
            c_ema.setData(t, ema)
            c_milli.setData(t, milli)
            c_servo.setData(t, servo)

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(20)  # ~50 FPS

    app.exec()
    ser.close()

if __name__ == "__main__":
    main()