// ===========================================================
// HAL_Sensors.cpp — Hardware Abstraction Layer: Sensor Reads
// Smart E-Ventilator Firmware v1.0
// ===========================================================
#include "HAL_Sensors.h"
#include "HAL_Board.h"

void HAL_Sensors_Init() {
    // Analog pins do not require pinMode on ATmega328P.
    // Perform a few dummy reads to let the ADC multiplexer settle.
    analogRead(PIN_FLOW_SENSOR);
    analogRead(PIN_HALL_SENSOR);
    delay(10);
}

// =============================================================
// Flow Sensor (MPX5010DP on A0)
// =============================================================
uint16_t HAL_Sensors_ReadFlowRaw() {
    return analogRead(PIN_FLOW_SENSOR);
}

float HAL_Sensors_ReadFlowVoltage() {
    return (float)HAL_Sensors_ReadFlowRaw() * ADC_TO_VOLTS;
}

// =============================================================
// Hall Effect Sensor (AT3503 on A2)
// =============================================================
uint16_t HAL_Sensors_ReadHallRaw() {
    return analogRead(PIN_HALL_SENSOR);
}

bool HAL_Sensors_IsHallTriggered() {
    // A3144 + 10K pull-up: LOW ADC value = magnet detected
    uint16_t val = HAL_Sensors_ReadHallRaw();
    return (val < HALL_TRIGGER_THRESHOLD);
}
