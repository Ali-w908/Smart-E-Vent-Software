// ===========================================================
//  Ventilator_Firmware.ino
//  Smart E-Ventilator — Main Entry Point
//  Version 1.1 — 25 March 2026
//
//  Architecture
//  ┌─────────────────────────────────────────┐
//  │  APPLICATION  — FSM_App (State Machine) │
//  ├─────────────────────────────────────────┤
//  │  SERVICE      — Kinematics, Filters,    │
//  │                 Safety                  │
//  ├─────────────────────────────────────────┤
//  │  HAL          — HAL_Board, HAL_Motor,   │
//  │                 HAL_Sensors             │
//  └─────────────────────────────────────────┘
//
//  Board : Arduino UNO  (ATmega328P @ 16 MHz)
//  Driver: CS-D508      (800 pulses/rev)
//  Motor : CS-M22331    (NEMA 23, 5 A/phase)
// ===========================================================
#include "HAL_Board.h"
#include "HAL_Motor.h"
#include "HAL_Sensors.h"
#include "Kinematics.h"
#include "Filters.h"
#include "Safety.h"
#include "FSM_App.h"

// =============================================================
// LOCAL SETTING MIRRORS  (for serial display / adjustment)
// =============================================================
static uint8_t _bpm      = 15;
static float   _ieRatio  = 2.0f;
static int32_t _tvSteps  = 1600;
static float   _pipKpa   = 2.5f;

// =============================================================
// HELPER: Read an integer from Serial input buffer
// Reads everything available until a non-digit, returns the number.
// =============================================================
static int32_t _readSerialInt() {
    int32_t val = 0;
    delay(50);   // brief wait for remaining chars to arrive
    while (Serial.available()) {
        char c = Serial.peek();
        if (c >= '0' && c <= '9') {
            val = val * 10 + (Serial.read() - '0');
        } else {
            Serial.read();  // discard non-digit
            break;
        }
    }
    return val;
}

// =============================================================
// HELPER: Read a float from Serial (simple: reads as int / 10)
// E.g., typing "25" after the command char → 2.5
// =============================================================
static float _readSerialFloat10() {
    int32_t raw = _readSerialInt();
    return (float)raw / 10.0f;
}

// =============================================================
// HELPER: Print all current settings
// =============================================================
static void _printStatus() {
    const VentSettings* s = FSM_GetSettings();

    Serial.println(F("\n--- Current Settings ---"));

    Serial.print(F("  State : "));
    switch (FSM_GetState()) {
        case STATE_BOOT:      Serial.println(F("BOOT"));      break;
        case STATE_CALIBRATE: Serial.println(F("CALIBRATE")); break;
        case STATE_READY:     Serial.println(F("READY"));     break;
        case STATE_INHALE:    Serial.println(F("INHALE"));    break;
        case STATE_EXHALE:    Serial.println(F("EXHALE"));    break;
        case STATE_FAULT:     Serial.println(F("FAULT"));     break;
    }

    Serial.print(F("  Mode  : "));
    Serial.println(FSM_GetMode() == MODE_VCV ? F("VCV") : F("PCV"));

    Serial.print(F("  BPM   : ")); Serial.println(s->bpm);
    Serial.print(F("  I:E   : 1:")); Serial.println(s->ieRatio, 1);
    Serial.print(F("  TV    : ")); Serial.print(s->tidalVolumeSteps);
    Serial.println(F(" steps"));
    Serial.print(F("  PIP   : ")); Serial.print(s->targetPIP_kPa, 1);
    Serial.println(F(" kPa"));
    Serial.print(F("  MaxCmp: ")); Serial.print(s->maxCompressSteps);
    Serial.println(F(" steps"));
    Serial.print(F("  T_inh : ")); Serial.print(FSM_GetInhaleTimeMs());
    Serial.println(F(" ms"));
    Serial.print(F("  T_exh : ")); Serial.print(FSM_GetExhaleTimeMs());
    Serial.println(F(" ms"));
    Serial.print(F("  P_now : ")); Serial.print(FSM_GetCurrentPressure(), 2);
    Serial.println(F(" kPa"));
    Serial.print(F("  Flow  : ")); Serial.print(FSM_GetCurrentFlowLPM(), 1);
    Serial.println(F(" L/min"));

    Serial.println(F("------------------------\n"));
}

// =============================================================
// SERIAL COMMAND INTERFACE
//
// Single-char commands:
//   S / s  -- Start ventilation
//   X / x  -- Stop ventilation
//   V / v  -- Switch to VCV
//   P / p  -- Switch to PCV
//   G / g  -- Toggle graph mode (Serial Plotter format)
//   ?      -- Print full status
//
// Parameterised commands (char + number):
//   B<nn>  — Set BPM        e.g. B20   → 20 BPM
//   R<nn>  — Set I:E ratio  e.g. R30   → 1:3.0
//   T<nnn> — Set tidal vol  e.g. T800  → 800 steps
//   I<nn>  — Set PIP        e.g. I25   → 2.5 kPa
//
// Simple adjust:
//   +  —  BPM + 1
//   -  —  BPM - 1
// =============================================================
static void _processSerialCommand() {
    if (!Serial.available()) return;

    char cmd = Serial.read();
    switch (cmd) {
        // --- Start / Stop ---
        case 'S': case 's':
            FSM_StartVentilation();
            break;
        case 'X': case 'x':
            FSM_StopVentilation();
            break;

        // --- Mode ---
        case 'V': case 'v':
            FSM_SetMode(MODE_VCV);
            Serial.println(F("[CMD] Mode -> VCV"));
            break;
        case 'P': case 'p':
            FSM_SetMode(MODE_PCV);
            Serial.println(F("[CMD] Mode -> PCV"));
            break;

        // --- Graph mode toggle ---
        case 'G': case 'g': {
            static bool graphOn = false;
            graphOn = !graphOn;
            FSM_SetGraphMode(graphOn);
            Serial.println(graphOn ? F("[CMD] Graph mode ON  (use Serial Plotter)") :
                                     F("[CMD] Graph mode OFF (text telemetry)"));
            break;
        }

        // --- BPM quick adjust ---
        case '+':
            _bpm = constrain(_bpm + 1, 10, 30);
            FSM_SetBPM(_bpm);
            Serial.print(F("[CMD] BPM = ")); Serial.println(_bpm);
            break;
        case '-':
            _bpm = constrain(_bpm - 1, 10, 30);
            FSM_SetBPM(_bpm);
            Serial.print(F("[CMD] BPM = ")); Serial.println(_bpm);
            break;

        // --- BPM set exact ---
        case 'B': case 'b': {
            int32_t val = _readSerialInt();
            if (val >= 10 && val <= 30) {
                _bpm = (uint8_t)val;
                FSM_SetBPM(_bpm);
                Serial.print(F("[CMD] BPM = ")); Serial.println(_bpm);
            } else {
                Serial.println(F("[ERR] BPM must be 10-30. Usage: B20"));
            }
            break;
        }

        // --- I:E Ratio ---
        case 'R': case 'r': {
            float val = _readSerialFloat10();   // e.g. R20 → 2.0
            if (val >= 1.0f && val <= 4.0f) {
                _ieRatio = val;
                FSM_SetIERatio(_ieRatio);
                Serial.print(F("[CMD] I:E = 1:")); Serial.println(_ieRatio, 1);
            } else {
                Serial.println(F("[ERR] I:E must be 1.0-4.0. Usage: R20 for 1:2.0"));
            }
            break;
        }

        // --- Tidal Volume (steps) ---
        case 'T': case 't': {
            int32_t val = _readSerialInt();
            if (val >= 100 && val <= 5000) {
                _tvSteps = val;
                FSM_SetTidalVolumeSteps(_tvSteps);
                Serial.print(F("[CMD] Tidal Vol = ")); Serial.print(_tvSteps);
                Serial.println(F(" steps"));
            } else {
                Serial.println(F("[ERR] TV must be 100-5000. Usage: T1200"));
            }
            break;
        }

        // --- Target PIP (kPa × 10) ---
        case 'I': case 'i': {
            float val = _readSerialFloat10();   // e.g. I25 → 2.5
            if (val >= 0.5f && val <= 5.0f) {
                _pipKpa = val;
                FSM_SetTargetPIP(_pipKpa);
                Serial.print(F("[CMD] PIP = ")); Serial.print(_pipKpa, 1);
                Serial.println(F(" kPa"));
            } else {
                Serial.println(F("[ERR] PIP must be 0.5-5.0 kPa. Usage: I25 for 2.5"));
            }
            break;
        }

        // --- Status ---
        case '?':
            _printStatus();
            break;
    }
}

// =============================================================
// SETUP
// =============================================================
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println(F("\n=========================================="));
    Serial.println(F("  Smart E-Ventilator Firmware v1.1"));
    Serial.println(F("  HAL + Service + FSM Architecture"));
    Serial.println(F("==========================================\n"));

    // HAL Layer
    HAL_Board_Init();
    HAL_Motor_Init();
    HAL_Sensors_Init();

    // Service Layer
    Kin_Init();
    Safety_Init();

    // Application Layer
    FSM_Init();

    // Enable Watchdog Timer (250 ms timeout)
    HAL_WDT_Enable();

    Serial.println(F("[BOOT] All modules initialised."));
    Serial.println(F("Commands:"));
    Serial.println(F("  S=Start  X=Stop  V=VCV  P=PCV  G=Graph  ?=Status"));
    Serial.println(F("  B<nn>=BPM  R<nn>=I:E  T<nnn>=TidalVol  I<nn>=PIP"));
    Serial.println(F("  + / - = BPM up/down\n"));
}

// =============================================================
// LOOP
// =============================================================
void loop() {
    HAL_WDT_Reset();
    FSM_Update();
    _processSerialCommand();
}
