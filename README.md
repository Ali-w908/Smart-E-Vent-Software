# Smart E-Ventilator Software (Smart-E-Vent-Software)

Welcome to the firmware and software repository for the **Smart E-Ventilator** project. This repository contains the professional-grade Arduino firmware for controlling the ventilator, alongside the essential hardware testing suites and structural documentation.

## Repository Structure

```
Smart-E-Vent-Software/
├── Code/
│   ├── Ventilator_Firmware/                # The main production firmware (Arduino UNO)
│   └── UNO-Test-Suite2-Flow-Hall-Motor/    # Hardware integration test suite
├── Documentation/
│   ├── Software-Architecture-Document.md   # Detailed layer-by-layer software design (SAD)
│   └── Software-Requirements-Specification.md # Functional & non-functional requirements (SRS)
└── README.md
```

## Architecture Overview

The firmware (`Ventilator_Firmware`) is strictly **non-blocking** and organized into a professional three-layer architecture to decouple hardware specifics from high-level ventilation logic:

1. **Application Layer (`FSM_App`)**: A Finite State Machine controlling the ventilation cycle (BOOT → CALIBRATE → READY → INHALE ⇄ EXHALE). It handles Volume Control (VCV) and Pressure Control (PCV) modes.
2. **Service Layer (`Kinematics`, `Filters`, `Safety`)**: 
   - Generates trapezoidal motion profiles to prevent motor stalling.
   - Filters sensor noise (EMA algorithm) and calculates Venturi tube flow.
   - Monitors physical limits (driver ALM+, overpressure) and triggers alarms.
3. **Hardware Abstraction Layer (HAL)**: Direct interaction with pins, the CS-D508 stepper driver, MPX5010DP flow sensor, and A3144 Hall Effect sensor.

> **Read the SAD (Software Architechture Document) inside `Documentation/` for a complete breakdown of the states and system architecture.**

## How to Compile & Upload

We target the **Arduino UNO (ATmega328P)**. The code uses only standard AVR libraries and has zero external dependencies.

1. Open `Code/Ventilator_Firmware/Ventilator_Firmware.ino` in the Arduino IDE.
2. Select **Arduino UNO** as the board.
3. Click **Upload**.
   - *(Alternatively, on Raspberry Pi use `arduino-cli compile --fqbn arduino:avr:uno Ventilator_Firmware`)*

## Serial Command Interface (Development)

Connect via Serial Monitor at **115200 baud** to monitor telemetry and send commands in real-time.

### Telemetry Output
The FSM prints live status every 250ms during active ventilation:
```
INH P=0.12kPa  Flow=96.4L/min  Stp=312/1600
EXH P=0.00kPa  Flow=0.0L/min   Stp=890/1600
```

### Supported Commands
- `S` / `X` : Start / Stop ventilation
- `V` / `P` : Switch to VCV mode / PCV mode
- `B<nn>`   : Set BPM (e.g., `B20` sets 20 Breaths Per Minute)
- `R<nn>`   : Set I:E ratio (e.g., `R30` sets 1:3.0)
- `T<nnn>`  : Set Tidal Volume steps (e.g., `T1600`)
- `I<nn>`   : Set target Peak Inspiratory Pressure (e.g., `I25` for 2.5 kPa)
- `+` / `-` : Quickly bump BPM up or down by 1
- `G`       : Toggle Graph Mode (formats output for the Arduino Serial Plotter)
- `?`       : Print the full device status and current parameters

## Testing & Calibration

- On boot, the machine auto-calibrates by advancing the motor until the **A3144 Hall Effect limit switch** is triggered, establishing the maximum compression bound.
- The `UNO-Test-Suite2-Flow-Hall-Motor` folder contains a standalone sketch used during R&D to verify isolated hardware integration. Use this if you are hooking up new hardware and want to verify wiring before flashing the full FSM.

## Documentation References

For team members catching up, please review the files in the `Documentation/` folder:
- **`Software-Requirements-Specification.md`**: Outlines *what* the system must do, safety bounds, and operating parameters.
- **`Software-Architecture-Document.md`**: Outlines *how* the system achieves it, detailing the HAL, Services, and FSM.
