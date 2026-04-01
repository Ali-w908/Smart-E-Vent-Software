// ===========================================================
// HAL_Sensors.h — Hardware Abstraction Layer: Sensor Reads
// Smart E-Ventilator Firmware v1.0
// ===========================================================
#ifndef HAL_SENSORS_H
#define HAL_SENSORS_H

#include <Arduino.h>

// =============================================================
// CALIBRATION CONSTANTS
// =============================================================
// MPX5010DP Flow Sensor
#define FLOW_SENSOR_OFFSET_V    0.332f    // Resting voltage (from calibration)
#define FLOW_SENSOR_SCALE       0.45f     // kPa per Volt (transfer function)
#define ADC_TO_VOLTS            (5.0f / 1023.0f)

// A3144 Hall Effect Sensor + 10KΩ Pull-Up Resistor
// No magnet  = pin pulled HIGH by resistor → ADC ≈ 1023 (5 V)
// Magnet     = A3144 sinks to GND          → ADC ≈ 9–16  (≈0 V)
// Threshold  = midpoint.  Below this = magnet detected.
#define HALL_TRIGGER_THRESHOLD  512

// =============================================================
// PUBLIC API
// =============================================================
void     HAL_Sensors_Init();
uint16_t HAL_Sensors_ReadFlowRaw();         // Raw 10-bit ADC
float    HAL_Sensors_ReadFlowVoltage();     // Converted to Volts
uint16_t HAL_Sensors_ReadHallRaw();         // Raw 10-bit ADC
bool     HAL_Sensors_IsHallTriggered();     // true = at max compression limit

#endif // HAL_SENSORS_H
