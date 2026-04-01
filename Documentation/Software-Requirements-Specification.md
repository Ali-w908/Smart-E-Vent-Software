# Software Requirements Specification (SRS)
**Project:** Smart E-Ventilator (Arduino Subsystem)
**Date:** 25 March 2026
**Version:** 1.0

## 1. Introduction
### 1.1 Purpose
This Software Requirements Specification (SRS) documents the functional and non-functional requirements for the Arduino-based Smart E-Ventilator firmware. This firmware is responsible for the critical, real-time life-support operations, specifically motor control, sensor data acquisition, and immediate hard-fault safety mechanisms. It operates independently from the higher-level Raspberry Pi user interface model to ensure failure of the UI does not halt patient ventilation.

### 1.2 Scope
The scope of this document is strictly limited to the Arduino UNO microcontroller software. It covers:
- Reading analog and digital sensors (Flow, Hall Effect).
- Driving a stepper motor via the CS-D508 driver.
- Maintaining strict ventilation timing based on target BPM and I:E ratios.
- Fallback/emergency states and hardware alarms.

## 2. Functional Requirements
### 2.1 Ventilation Mechanics (FR-01)
* **FR-01.1:** The system shall compress and decompress the Ambu bag using a stepper motor.
* **FR-01.2:** The system shall support both **Volume Control Ventilation (VCV)** and **Pressure Control Ventilation (PCV)** modes.
  * *VCV Mode:* The motor compresses the bag to deliver a precise target Tidal Volume (calculated by motor stroke length).
  * *PCV Mode:* The motor compresses the bag until a target Peak Inspiratory Pressure (PIP) is achieved, maintaining this pressure over the inspiratory time.
* **FR-01.3:** The system shall support configurable Breaths Per Minute (BPM) ranging from 10 to 30.
* **FR-01.4:** The system shall support a configurable Inspiration to Expiration (I:E) ratio (e.g., 1:2, 1:3).

### 2.2 Sensor Data Acquisition (FR-02)
* **FR-02.1:** The system shall read differential pressure from the MPX5010DP flow sensor to calculate flow rate and airway pressure.
* **FR-02.2:** The system shall utilize an AT3503 Hall Effect sensor to detect the **maximum physical compression point**. This acts as a forward limit switch to prevent mechanical overcompression and damage to the Ambu bag. Extents of retraction are calculated purely via stepper motor step counts from this known absolute reference.
* **FR-02.3:** The software shall implement signal filtering (e.g., Exponential Moving Average) to smooth raw ADC sensor reads.

### 2.3 Safety and Hardware Fault Detection (FR-03)
* **FR-03.1:** The system shall continuously monitor the `ALM+` pin from the CS-D508 motor driver for overcurrent, overvoltage, or stall faults.
* **FR-03.2:** The system shall enter a safe `ALARM` state within 10 milliseconds if a driver fault is detected.
* **FR-03.3:** In the `ALARM` state, the motor driver shall be immediately disabled via the `ENA+` pin.
* **FR-03.4:** The system shall sound the SFM-20B piezo buzzer continuously and illuminate the Red Status LED during a critical alarm.
* **FR-03.5:** The system shall implement a Watchdog Timer (WDT) to automatically reboot the microcontroller if the main instruction loop hangs for more than 250ms.

### 2.4 Calibration and Limits (FR-04)
* **FR-04.1:** On startup, the system shall safely establish its absolute positioning by advancing the motor until the Hall Effect sensor reliably triggers at the maximum compression limit.
* **FR-04.2:** Once the maximum compression point is logged, the motor shall retract a predefined number of steps to establish its fully open "zero/retracted" position.
* **FR-04.3:** The system shall not begin regular ventilation cycles until this stroke calibration sequence is successfully completed.

## 3. Non-Functional Requirements
### 3.1 Real-Time Performance (NFR-01)
* **NFR-01.1:** The software shall be strictly non-blocking. The use of blocking `delay()` functions is strictly prohibited in the main control loops.
* **NFR-01.2:** The main FSM loop must execute at a frequency of no less than 1kHz (1ms per cycle) to guarantee rapid response to the Hall sensor during homing and fault detection.

### 3.2 Reliability (NFR-02)
* **NFR-02.1:** The system shall log failed sensor reads.
* **NFR-02.2:** The system shall support a "Degraded Mode" where, if the flow sensor or I2C sensors fail, it continues to deliver the baseline Tidal Volume based exclusively on stepper motor kinematics.

### 3.3 Extensibility (NFR-03)
* **NFR-03.1:** The architecture shall utilize a Hardware Abstraction Layer (HAL) to ensure that specific sensors or motor driver hardware can be swapped in the future without requiring a rewrite of the core Finite State Machine logic.

---

## 4. Hardware Interfaces
* **Motor Driver:** Pulse (PUL), Direction (DIR), Enable (ENA), and Alarm (ALM) pins. All connections are opto-isolated 5V logic.
* **Flow Sensor:** 0-5V Analog input via internal 10-bit ADC.
* **Hall Effect Sensor:** Analog input (or digital comparator threshold trigger).
* **UI LED & Buzzer:** Direct Digital Output to 220Ω LEDs and a Piezo element.
