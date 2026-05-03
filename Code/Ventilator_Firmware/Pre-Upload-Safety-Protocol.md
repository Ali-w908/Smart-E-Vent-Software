# Pre-Upload Safety Verification Protocol
**Project:** Smart E-Ventilator Firmware v1.0  
**Date:** 25 March 2026  
**Purpose:** Ensure the firmware is safe to upload and run on the physical Arduino UNO + CS-D508 + NEMA 23 hardware, without risk of damage.

---

## Phase 0: Static Code Audit (Before Plugging Anything In)

Read through each file and confirm the following matches your physical wiring. **Every pin number below must match what is physically connected.**

| What to Check | File | Expected Value | ✅ |
|---|---|---|---|
| Motor PUL pin | `HAL_Board.h` line `PIN_MOTOR_PUL` | `2` (→ CS-D508 PUL+) | ☐ |
| Motor DIR pin | `HAL_Board.h` line `PIN_MOTOR_DIR` | `3` (→ CS-D508 DIR+) | ☐ |
| Motor ENA pin | `HAL_Board.h` line `PIN_MOTOR_ENA` | `4` (disconnected, safe) | ☐ |
| Motor ALM pin | `HAL_Board.h` line `PIN_MOTOR_ALM` | `9` (← CS-D508 ALM+, INPUT_PULLUP) | ☐ |
| Buzzer pin | `HAL_Board.h` line `PIN_BUZZER` | `5` | ☐ |
| Green LED pin | `HAL_Board.h` line `PIN_LED_GREEN` | `6` | ☐ |
| Yellow LED pin | `HAL_Board.h` line `PIN_LED_YELLOW` | `7` | ☐ |
| Red LED pin | `HAL_Board.h` line `PIN_LED_RED` | `8` | ☐ |
| Flow Sensor pin | `HAL_Board.h` line `PIN_FLOW_SENSOR` | `A0` | ☐ |
| Hall Sensor pin | `HAL_Board.h` line `PIN_HALL_SENSOR` | `A2` | ☐ |

> **Rule: If ANY pin is wrong, FIX IT before compiling. A wrong pin means an OUTPUT signal could be driven into a component expecting INPUT, potentially causing a short.**

---

## Phase 1: Compile-Only Check

Compile the sketch **without uploading**. This catches syntax errors and verifies the code fits in the UNO's 32 KB flash / 2 KB RAM.

```bash
# On the Raspberry Pi (or Arduino IDE → Verify button)
arduino-cli compile --fqbn arduino:avr:uno Ventilator_Firmware/
```

**What to look for:**
- ✅ `Sketch uses XXXX bytes (XX%) of program storage space.`
- ✅ `Global variables use XXXX bytes (XX%) of dynamic memory.`
- ❌ Any compile errors → fix before proceeding.

---

## Phase 2: Upload With Motor Power OFF (Safe Dry Run)

This test verifies the code boots, calibration starts, and sensors read correctly — **without the motor actually moving**.

### Setup:
1. **DO NOT plug in the 220V PSU.** The 36V rail stays completely dead.
2. Connect Arduino UNO to your laptop via USB cable only.
3. All sensors (Flow sensor, Hall sensor) + LEDs + Buzzer remain connected as normal.

### Upload and Observe:
1. Upload the sketch via USB.
2. Open Serial Monitor at **115200 baud**.
3. **Expected output:**
   ```
   Smart E-Ventilator Firmware v1.0
   [BOOT] All modules initialised.
   [FSM] Calibration started — advancing to Hall limit...
   ```
4. The Yellow LED should turn on (calibrating state).
5. The motor step pin (D2) will pulse — but since the CS-D508 has no power, **nothing moves**. This is safe.

### Test the Hall Sensor:
6. While calibration is running, bring a magnet near the A3144 sensor.
7. **Expected output:**
   ```
   [CAL] Hall triggered at step XXXX
   [CAL] Home established. System READY.
   ```
8. Green LED should turn on.

### Test the Fault Timeout:
9. Reset the Arduino (press the hardware reset button). Do NOT bring a magnet.
10. Wait 30 seconds.
11. **Expected output:**
    ```
    [CAL] FAULT: Hall sensor not found.
    ```
12. Red LED + Buzzer should activate.

### Test the Flow Sensor:
13. Reset again. After calibration succeeds (bring magnet), type `?` to see status.
14. Gently blow into the MPX5010DP P1 port while monitoring serial. The EMA filter should show rising kPa values.

> **At this point you have verified: boot sequence, sensor reads, Hall detection, fault handling, LEDs, and buzzer — all without any motor power.**

---

## Phase 3: Motor Power ON (Supervised First Run)

Only proceed here after Phase 2 is 100% clean.

### Setup:
1. Arduino stays on USB (laptop power).
2. Plug in the 220V PSU to power the CS-D508 driver and motor.
3. **Stand clear of the motor arm.** The NEMA 23 is powerful.
4. Have your hand ready on the 220V power switch to kill motor power instantly if needed.

### First Calibration With Motor:
1. Reset the Arduino.
2. The motor should slowly advance (compress direction).
3. When the magnet is detected by the Hall sensor → motor stops, then retracts.
4. `[CAL] Home established. System READY.` should appear.

### First Breath Cycle:
5. Type `S` in Serial Monitor to start ventilation.
6. Watch the motor compress and retract. Listen for smooth motion (no grinding/stalling).
7. Type `X` to stop immediately if anything looks wrong.
8. Type `?` to verify state.

### Emergency Procedure (If Anything Goes Wrong):
- **Turn off the 220V power switch** — this instantly de-powers the motor. The Arduino stays alive on USB, serial output continues, but nothing moves.
- Press the Arduino reset button if the code appears hung.

---

## Summary: What Makes This Code Safe

| Safety Feature | How It Protects You |
|---|---|
| Motor starts **disabled** | `HAL_Motor_Init()` calls `Disable()`. Motor won't move until the FSM explicitly enables it. |
| Calibration speed is slow | `KIN_CALIBRATE_INTERVAL_US = 2000` → ~250 steps/sec, very slow and controllable. |
| Hall sensor = hard stop | Calibration instantly stops the motor the moment the magnet is detected. |
| 30-second calibration timeout | If the Hall sensor never triggers, the code enters FAULT and stops everything. |
| `Safety_Update()` runs every 3ms | Continuously checks the CS-D508 ALM+ pin. If the driver faults, motor is killed instantly. |
| Overpressure check | If airway pressure exceeds 4.0 kPa (~40 cmH2O), the motor stops. |
| Watchdog timer (250ms) | If the code ever hangs/freezes, the MCU resets itself. |
| No `delay()` in control loops | The system is always responsive — it can never get "stuck" ignoring a fault. |
| ENA pin is physically disconnected | Even if the code writes to D4, the wire isn't connected — the driver stays enabled by default. |
