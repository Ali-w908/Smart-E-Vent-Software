// ===========================================================
// HAL_Board.cpp — Hardware Abstraction Layer: Board & Pin Defs
// Smart E-Ventilator Firmware v1.0
// ===========================================================
#include "HAL_Board.h"
#include <avr/wdt.h>

// =============================================================
// HAL_Board_Init — Set pin modes and default states
// =============================================================
void HAL_Board_Init() {
    // Motor driver pins
    pinMode(PIN_MOTOR_PUL, OUTPUT);
    pinMode(PIN_MOTOR_DIR, OUTPUT);
    pinMode(PIN_MOTOR_ENA, OUTPUT);
    pinMode(PIN_MOTOR_ALM, INPUT_PULLUP);

    // User interface pins
    pinMode(PIN_BUZZER,     OUTPUT);
    pinMode(PIN_LED_GREEN,  OUTPUT);
    pinMode(PIN_LED_YELLOW, OUTPUT);
    pinMode(PIN_LED_RED,    OUTPUT);

    // Analog sensor pins (A0, A2) do not require pinMode on ATmega328P

    // Set all outputs LOW to start
    digitalWrite(PIN_MOTOR_PUL, LOW);
    digitalWrite(PIN_MOTOR_DIR, LOW);
    digitalWrite(PIN_MOTOR_ENA, LOW);   // Driver defaults ON when ENA disconnected
    digitalWrite(PIN_BUZZER,    LOW);
    digitalWrite(PIN_LED_GREEN, LOW);
    digitalWrite(PIN_LED_YELLOW,LOW);
    digitalWrite(PIN_LED_RED,   LOW);
}

// =============================================================
// Time Wrappers
// =============================================================
uint32_t HAL_GetMillis() { return millis(); }
uint32_t HAL_GetMicros() { return micros(); }

// =============================================================
// Watchdog Timer (ATmega328P avr/wdt.h)
// =============================================================
void HAL_WDT_Enable()  { wdt_enable(WDTO_250MS); }
void HAL_WDT_Reset()   { wdt_reset(); }
void HAL_WDT_Disable() { wdt_disable(); }
