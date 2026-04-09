// ===========================================================
// HAL_Board.h — Hardware Abstraction Layer: Board & Pin Defs
// Smart E-Ventilator Firmware v1.0
// ===========================================================
#ifndef HAL_BOARD_H
#define HAL_BOARD_H

#include <Arduino.h>

// =============================================================
// PIN DEFINITIONS
// Source: Electrical-System-Specification.md Rev 3.0
// =============================================================

// --- Motor Driver (CS-D508) ---
#define PIN_MOTOR_PUL       2     // Step Pulse  → PUL+
#define PIN_MOTOR_DIR       3     // Direction   → DIR+
#define PIN_MOTOR_ENA       4     // Enable      → ENA+ (physically disconnected)
#define PIN_MOTOR_ALM       9     // Alarm       ← ALM+ (INPUT_PULLUP)

// --- User Interface ---
#define PIN_BUZZER          5     // SFM-20B Piezo
#define PIN_LED_GREEN       6
#define PIN_LED_YELLOW      7
#define PIN_LED_RED         8

// --- Sensors ---
#define PIN_FLOW_SENSOR     A0    // MPX5010DP Differential Pressure
#define PIN_HALL_SENSOR     A2    // AT3503 Hall Effect

// =============================================================
// MOTOR DRIVER CONFIGURATION
// =============================================================
#define MOTOR_PULSES_PER_REV    800   // CS-D508 DIP: OFF,ON,ON,ON

// =============================================================
// MECHANICAL CALIBRATION (measured empirically)
// Full Ambu bag compression = 1.25 motor turns = 450 degrees.
// 1.25 rev × 800 pulses/rev = 1000 pulses for full compression.
// =============================================================
#define MECH_FULL_COMPRESS_TURNS    1.25f
#define MECH_FULL_COMPRESS_STEPS    ((int32_t)(MECH_FULL_COMPRESS_TURNS * MOTOR_PULSES_PER_REV))  // 1000

// =============================================================
// HAL TIME WRAPPERS
// =============================================================
uint32_t HAL_GetMillis();
uint32_t HAL_GetMicros();

// =============================================================
// BOARD INIT & WATCHDOG
// =============================================================
void HAL_Board_Init();
void HAL_WDT_Enable();
void HAL_WDT_Reset();
void HAL_WDT_Disable();

#endif // HAL_BOARD_H
