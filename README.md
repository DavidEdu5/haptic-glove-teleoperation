# haptic-glove-teleoperation
Software for EEEN30330 Individual Project (2025/26), University of Manchester
Deterministic Bilateral Control of a Reduced-DOF Haptic Interface
for High-Transparency Tele-operation — Software Repository
1. Introduction
This repository contains all software developed for the third-year dissertation project: Deterministic Bilateral Control of a Reduced-DOF Haptic Interface for High-Transparency Tele-operation (University of Manchester, EEEN30330, 2026).

The system enables a human operator wearing a single-finger flex-sensor glove to interact with a virtual wall rendered in Unity. Bend angle is read from a resistive flex sensor via an STM32 microcontroller, transmitted over serial to a Python host bridge, forwarded as a UDP datagram to the Unity game engine, and returned as a force cue that drives a servo motor pressing against the operator's finger. Passivity is enforced on the communication channel using a Time-Domain Passivity Approach (TDPA) and wave-variable coding, ensuring stability regardless of round-trip network latency.

The software is divided into four subsystems:
•	STM32 Firmware (C) — sensor acquisition, EMA filtering, servo PWM generation, and TDPA observer running on an STM32F4 microcontroller at 1 kHz.
•	Python Host Bridge (Python 3) — bidirectional serial-to-UDP bridge, wave-variable encoder/decoder, calibration management, and CSV data logger.
•	Unity Virtual Environment (C#) — UDP receiver, kinematic finger model, wall-collision detection, and haptic feedback dispatcher.
•	Analysis Toolchain (Python 3) — offline post-processing scripts that parse logged CSV files and produce the figures used in the dissertation.

2. Contextual Overview
The system architecture follows a bilateral teleoperation loop. The diagram below describes the data-flow path:

  [Flex Sensor]                                                  
      |  ADC (12-bit, 1 kHz)
  [STM32F4]  ← EMA filter + TDPA observer + servo PWM
      |  UART @ 115200 baud (DMA, 100 Hz log)
  [Python Bridge]  ← wave-variable encode/decode + logger
      |  UDP (localhost, port 5005 / 5007)
  [Unity 3D]  ← finger model + wall detection + vmax feedback
      |  UDP echo (RTT measurement)
  [Python Bridge]  ← TDPA violation monitor
      |  servo command back over serial
  [Servo Motor]  ← physical force cue on operator's finger

Key design decisions: the control loop runs at 1 kHz on the STM32 but logs at 100 Hz to limit serial bandwidth; the Python bridge runs two threads (serial reader and Unity socket) sharing a mutex-protected state dictionary; Unity receives position (flex ∈ [0, 1]) and replies with vmax (the maximum permissible velocity) and inside_wall (Boolean collision flag).

3. Repository Structure

haptic-glove-teleoperation/
├── firmware/                  # STM32CubeIDE project
│   ├── main.c                 # Core control loop + EMA + TDPA
│   └── main_with_slew_limiter.c  # Variant with servo slew-rate cap
├── bridge/                    # Python host bridge
│   ├── haptic.py              # Original minimal bridge (v1)
│   ├── testC_unity_bridge_logger_Version5.py   # Bridge v5
│   └── testC_unity_bridge_logger_Version6_WaveVar_Version2.py  # Bridge v6 (wave vars + TDPA)
├── unity/                     # Unity C# scripts
│   ├── UnityReceiver.cs       # UDP listener + finger kinematics
│   ├── Wall_detector.cs       # Trigger collision callbacks
│   └── ButtonInteractable.cs  # vmax feedback sender
├── analysis/                  # Offline data analysis scripts
│   ├── python_testB_loggerV9.py      # Test B logger
│   ├── analysis_testB_report1.py     # Filter efficacy analysis
│   ├── plot_test_b.py               # Test B figure generator
│   ├── live_scope_testB.py          # Real-time oscilloscope display
│   └── drv2605_logger.py            # DRV2605 haptic driver logger
├── data/                      # Representative sample CSV logs
└── README.md                  # This file

4. Installation Instructions
Prerequisites: Python 3.10 or later, STM32CubeIDE 1.15+ (for firmware), Unity 2022 LTS or later (for the virtual environment), and a USB-connected STM32F4 board.

4.1  Python Dependencies
Install all Python dependencies using pip:

pip install pyserial numpy pandas matplotlib scipy

No virtual environment is strictly required but is recommended:
python -m venv venv
source venv/bin/activate   # Windows: venv\Scripts\activate
pip install pyserial numpy pandas matplotlib scipy

4.2  STM32 Firmware
•	Open STM32CubeIDE and import the firmware/ directory as an existing project.
•	Connect the STM32F4 board via USB/ST-Link.
•	Build and flash using the Run button (or Ctrl+F11).
•	The board will immediately begin streaming CSV-formatted telemetry at 115200 baud on USART2.

4.3  Unity Project
•	Open Unity Hub and add the unity/ directory as a project (Unity 2022 LTS).
•	Open the main scene; attach UnityReceiver.cs, Wall_detector.cs, and ButtonInteractable.cs to the appropriate GameObjects as described in the Inspector.
•	Ensure UDP port 5005 (receive) and 5007 (send) are not blocked by a firewall.

5. How to Run the Software
The system is started in two stages: first flash the firmware (one-time), then launch the Python bridge, then press Play in Unity.

5.1  Launch the Python Bridge
Identify the STM32 serial port (e.g. /dev/cu.usbmodem103 on macOS, COM3 on Windows), then run:

python bridge/testC_unity_bridge_logger_Version6_WaveVar_Version2.py \
       --port /dev/cu.usbmodem103 --baud 115200

Optional flags:
•	--inject-latency 20  — artificially injects 20 ms of round-trip latency for robustness testing.
•	--wave-mode          — enables wave-variable encoding on the vmax channel.
•	--log-dir ./data     — saves timestamped CSV logs to the specified directory.

5.2  Press Play in Unity
With the bridge running, press Play in the Unity Editor. The finger model will animate in response to glove bend, and the servo will resist when the virtual wall is contacted.

5.3  Run Test-B Filter Characterisation (Offline)
To reproduce the filter-efficacy plots from Section 4.1 of the dissertation:

python analysis/python_testB_loggerV9.py --port /dev/cu.usbmodem103
python analysis/analysis_testB_report1.py data/testB_20260421_152128.csv
python analysis/plot_test_b.py data/testB_20260421_152128.csv

6. Technical Details
6.1  EMA Filter (STM32)
A first-order Exponential Moving Average (EMA) is applied to the raw 12-bit ADC value at 1 kHz to suppress high-frequency flex-sensor noise before servo commanding:

EMA[n] = alpha * raw[n] + (1 - alpha) * EMA[n-1]

where alpha = 0.50 (configurable at compile time via EMA_ALPHA). The filter introduces a group delay of approximately (1/alpha - 1) / f_s = 1 ms at the nominal sample rate, confirmed experimentally in Test B to attenuate shock inputs by 64.6 % with a 12 ms whole-run delay (see Section 4.1).

6.2  Wave-Variable Channel (Python Bridge)
Wave variables are computed per-packet to enforce passivity on the velocity channel transmitted to Unity. The forward and return wave variables are:

u_forward = (vmax + F_est) / sqrt(2 * b)
u_return  = (vmax - F_est) / sqrt(2 * b)

where b is the wave impedance (tuned to 1.0 N·s/m for this system) and F_est is the estimated contact force from the TDPA observer running on the STM32. Cumulative energy E_cumulative is tracked per session; if E_cumulative drops below zero (indicating a passivity violation), a correction term is injected into the next outgoing packet.

6.3  Time-Domain Passivity Approach (TDPA) — STM32 Observer
The TDPA observer runs in the 1 kHz ISR on the STM32. It integrates instantaneous power P(t) = F(t) · v(t) over time to compute the energy stored in the haptic channel. The energy variable E_tdpa is transmitted to the Python bridge in each telemetry frame. A violation (E_tdpa < 0) causes the bridge to log a TDPA-VIOLATION event and reduce the commanded vmax to zero until E_tdpa recovers to a positive value.

6.4  Serial Protocol
The STM32 outputs newline-delimited ASCII frames at 100 Hz (decimated from the 1 kHz control loop using LOG_DECIM = 10). Each frame is either 11 or 18 comma-separated fields:

stm32_ms, raw, ema, milli, servo, vmax_milli_fw, flags,
spike_mag, spike_count, spike_max_mag, alpha          [11-field, v1]

... + servo_feedback, theta_cmd, theta_actual, omega, F_est, E_tdpa, error  [18-field, v2]

The Python bridge is backward-compatible: it detects frame length and parses accordingly, populating default values of zero for the TDPA fields when running against older firmware.

7. Known Issues and Future Improvements
7.1  Known Limitations
•	Serial timing jitter: Windows USB-serial drivers introduce non-deterministic inter-frame gaps up to 30 ms (p95). The bridge compensates using the STM32 hardware timestamp (stm32_ms) rather than PC clock for latency attribution, but jitter above ~15 ms can cause brief passivity violations.
•	Single-finger only: the current design captures MCP, PIP, and DIP joints of one finger using a single flex sensor. Extending to a full hand would require five additional ADC channels and corresponding servo actuators.
•	Unity-only virtual environment: the wall model is a flat plane; more complex geometry (e.g. a deformable surface or a second operator's hand) would require changes to Wall_detector.cs and the vmax feedback protocol.
•	Slew-rate limiter (main_with_slew_limiter.c): the variant with the servo slew-rate cap showed reduced peak force fidelity in Test D; it is provided for completeness but is not the default firmware used in the submitted results.

7.2  Future Improvements
•	Replace the resistive flex sensor with an IMU-based angle estimator to eliminate the sensor hysteresis that currently contributes ~±3 % steady-state error in the milli channel.
•	Implement a Kalman filter on the STM32 in place of the fixed-alpha EMA to achieve adaptive noise suppression without manual alpha tuning.
•	Extend the TDPA observer to handle multi-DOF energy accounting when additional fingers are added.
•	Port the Python bridge to a real-time operating system (e.g. Zephyr) running on a second STM32, eliminating the non-deterministic Python GIL and OS scheduler jitter from the communication path.
•	Evaluate the DRV2605L ERM/LRA haptic driver (logged in drv2605_logger.py) as a complementary high-frequency vibrotactile feedback channel alongside the servo-based kinesthetic feedback.
<img width="468" height="638" alt="image" src="https://github.com/user-attachments/assets/a102bac4-7ad8-4c4d-b614-41fe362671d8" />
