// ===========================================================
// HAL_Motor.cpp — Hardware Abstraction Layer: CS-D508 Driver
// Smart E-Ventilator Firmware v1.0
// ===========================================================
#include "HAL_Motor.h"
#include "HAL_Board.h"

void HAL_Motor_Init() {
    // Pin modes already configured by HAL_Board_Init().
    // Start with motor disabled for safety during boot.
    HAL_Motor_Disable();
}

void HAL_Motor_Enable() {
    // NOTE: The CS-D508 ENA+ wire is currently physically disconnected.
    // The driver defaults to "Enabled" when the ENA port is empty.
    // This function is preserved for when ENA is reconnected.
    digitalWrite(PIN_MOTOR_ENA, HIGH);
}

void HAL_Motor_Disable() {
    digitalWrite(PIN_MOTOR_ENA, LOW);
}

void HAL_Motor_SetDirection(uint8_t dir) {
    digitalWrite(PIN_MOTOR_DIR, dir);
}

void HAL_Motor_StepPulse() {
    // CS-D508 datasheet requires minimum 2.5µs pulse width.
    // We use 5µs for reliable triggering.
    digitalWrite(PIN_MOTOR_PUL, HIGH);
    delayMicroseconds(5);
    digitalWrite(PIN_MOTOR_PUL, LOW);
}

bool HAL_Motor_ReadAlarm() {
    // ALM+ is active LOW (pin has internal pull-up).
    // LOW = fault condition from driver.
    return (digitalRead(PIN_MOTOR_ALM) == LOW);
}
