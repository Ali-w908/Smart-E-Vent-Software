# Software Architecture Document (SAD)
**Project:** Smart E-Ventilator (Arduino Subsystem)
**Date:** 25 March 2026
**Version:** 1.0

## 1. Introduction
This document defines the software architecture for the Arduino UNO microcontroller driving the Smart E-Ventilator. The architecture is built to ensure strict timing, absolute fault-tolerance, and high modularity. 

To achieve this, the software is divided into three distinct layers:
1. **Application Layer:** The high-level logic and Finite State Machine (FSM).
2. **Service Layer:** Mid-level logic processing such as kinematic trajectory calculation and data filtering.
3. **Hardware Abstraction Layer (HAL):** The lowest level that interacts directly with the physical pins and registers of the ESP/Arduino.

## 2. Architectural Layers

### 2.1 Hardware Abstraction Layer (HAL)
The purpose of the HAL is to decouple the business logic of the ventilator from the underlying silicon. If the project upgrades from an Arduino UNO to an STM32 or ESP32, only the HAL needs to be rewritten.

**Modules in the HAL:**
* **`HAL_GPIO`**: Wraps fundamental digital and analog pin controls (e.g., `HAL_DigitalWrite`, `HAL_AnalogRead`).
* **`HAL_MotorDriver`**: Interfaces specifically with the CS-D508. Contains raw step pulse generation, direction switching, and enable/disable toggles.
* **`HAL_Sensors`**: Interfaces with the MPX5010DP Flow Sensor and AT3503 Hall Sensor. Handles the raw conversion of 10-bit ADC values to voltages/raw ticks.
* **`HAL_Time`**: Wraps the microcontroller's hardware timers (e.g., `millis()`, `micros()`).

### 2.2 Service Layer
This layer takes raw data from the HAL and turns it into useful information, or takes physical requests from the Application Layer and turns them into actions.

**Modules in the Service Layer:**
* **`Kinematics_Engine`**: Calculates the trapezoidal acceleration and deceleration profiles. It determines *when* the next step pulse must be fired to achieve a smooth stroke curve, preventing the NEMA 23 motor from stalling under the heavy load of the Ambu bag.
* **`Signal_Filter`**: Applies Exponential Moving Averages (EMA) to the noisy analog reads from the Flow Sensor. Converts raw voltage into differential pressure (kPa) and Flow Rate (L/min).
* **`Safety_Monitor`**: A daemon running parallel to the main FSM that constantly polls the `ALM+` pin via the HAL and checks if the differential pressure exceeds safe Peak Inspiratory Pressure (PIP) limits. Can trigger hard interrupts.

### 2.3 Application Layer 
The topmost layer determines *what* the ventilator should be doing at any given millisecond. 

**Module:** **`Ventilator_FSM`** (Finite State Machine)
This module does not know what a "pin" or a "stepper pulse" is. It commands the Service and HAL layers based on its current state and operational mode (Volume Control Ventilation [VCV] vs. Pressure Control Ventilation [PCV]).

**States:**
1. **`STATE_BOOT`**: Invokes HAL initialization. Checks sensor baselines.
2. **`STATE_CALIBRATE`**: Commands `Kinematics_Engine` to carefully advance the compression arm until `HAL_Sensors` reports the Hall Effect is triggered (Max Compression Bound). Then retracts a predefined number of steps to establish "Home" (Zero Position).
3. **`STATE_INHALE`**: 
   - **If VCV Mode:** Commands `Kinematics_Engine` to advance to the target step position corresponding to the required Tidal Volume over the calculated Inspiratory Time.
   - **If PCV Mode:** Commands `Kinematics_Engine` to advance based on real-time feedback from the `Signal_Filter` (Pressure Sensor) until the Peak Inspiratory Pressure (PIP) is reached, adjusting speed dynamically to maintain PIP across the Inspiratory Time.
4. **`STATE_EXHALE`**: Commands `Kinematics_Engine` to return to the calculated "Home" position over the Expiratory Time.
5. **`STATE_FAULT`**: Triggered by the `Safety_Monitor` (e.g., Driver ALM+, over-pressure). Commands `HAL_MotorDriver` to disable the motor and sounds the `HAL_GPIO` alarm buzzer.

## 3. Data Flow & Timing Architecture

### 3.1 Non-Blocking execution
The architecture explicitly forbids blocking functions (like `delay()`). All timing is managed via state polling against `HAL_Time`.
```cpp
// Correct representation of non-blocking step generation across layers
if (HAL_Time_GetMicros() - lastStepTime >= currentDelayRequired) {
    HAL_MotorDriver_Step();
    lastStepTime = HAL_Time_GetMicros();
}
```

### 3.2 Decoupled Control Loops
The software will operate two conceptual loops running asynchronously within the main Arduino loop:
1. **The Fast Loop (~50-100 microseconds):** Dedicated entirely to the `Kinematics_Engine` and `HAL_MotorDriver` to ensure precise micro-stepping without jitter.
2. **The Slow Loop (~2-5 milliseconds):** Executes sensor polling, FSM state evaluation, and UI updates.

## 4. Directory and File Structure Draft

```
/Code/Ventilator_Firmware/
├── Ventilator_Firmware.ino   # Main loop() and setup()
├── FSM_App.h/.cpp            # Application Layer (State Machine)
├── Kinematics.h/.cpp         # Service Layer (Motor Acceleration profiles)
├── Filters.h/.cpp            # Service Layer (Signal processing)
├── Safety.h/.cpp             # Service Layer (Fault handling)
├── HAL_Motor.h/.cpp          # HAL Layer (Step generation, CS-D508 specific)
├── HAL_Sensors.h/.cpp        # HAL Layer (ADC reads, I2C/SPI wrappers)
└── HAL_Board.h/.cpp          # HAL Layer (Pin defines, Timers, WDT)
```
