// ===========================================================
// FSM_App.cpp — Application Layer: Ventilator State Machine
// Smart E-Ventilator Firmware v1.0
//
// This module orchestrates the entire breathing cycle.
// It does NOT touch pins directly — all hardware interaction
// goes through HAL_Motor, HAL_Sensors, and Safety.
// ===========================================================
#include "FSM_App.h"
#include "HAL_Board.h"
#include "HAL_Motor.h"
#include "HAL_Sensors.h"
#include "Kinematics.h"
#include "Filters.h"
#include "Safety.h"

// =============================================================
// COMPILE-TIME DEFAULTS
// =============================================================
#define DEFAULT_BPM                 15
#define DEFAULT_IE_RATIO            2.0f    // 1:2
// Full Ambu compression = MECH_FULL_COMPRESS_STEPS (1000).
// Default tidal = 80 % of full compression → 800 steps.
#define DEFAULT_TIDAL_STEPS         (int32_t)(MECH_FULL_COMPRESS_STEPS * 0.80f)  // 80% of full
#define DEFAULT_TARGET_PIP_KPA      2.5f    // ~25 cmH2O
#define CALIBRATE_RETRACT_STEPS     400     // Steps back from max to Home
#define CALIBRATE_TIMEOUT_MS        30000   // 30 s max for calibration
#define SENSOR_POLL_INTERVAL_MS     3       // ~333 Hz sensor loop
#define TELEMETRY_PRINT_INTERVAL_MS 250     // Print to Serial every 250ms

// =============================================================
// PRIVATE STATE
// =============================================================
static VentState    _state            = STATE_BOOT;
static VentMode     _mode             = MODE_VCV;
static VentSettings _settings;
static EMA_Filter   _flowFilter;

// Timing helpers
static uint32_t _stateEntryMs   = 0;
static uint32_t _lastSensorMs   = 0;
static uint32_t _lastTelemetryMs = 0;

// Computed per breath
static uint32_t _inhaleTimeMs   = 0;
static uint32_t _exhaleTimeMs   = 0;

// Per-breath bookkeeping
static int32_t  _currentInhaleSteps = 0;
static float    _currentPressureKpa = 0.0f;
static float    _currentFlowVoltage = 0.0f;
static float    _currentFlowLPM     = 0.0f;

// Output mode
static bool     _graphMode          = false;  // true = Serial Plotter format

// Calibration sub-state
static bool     _calibRetracting    = false;

// =============================================================
// HELPER: recompute breath phase timing from current settings
// =============================================================
static void _computeBreathTiming() {
    uint32_t breathPeriodMs = 60000UL / _settings.bpm;
    float    totalRatio     = 1.0f + _settings.ieRatio;
    _inhaleTimeMs = (uint32_t)(breathPeriodMs / totalRatio);
    _exhaleTimeMs = breathPeriodMs - _inhaleTimeMs;
}

// =============================================================
// HELPER: begin a new inhale phase
// =============================================================
static void _startInhale() {
    _computeBreathTiming();

    int32_t targetSteps;
    if (_mode == MODE_VCV) {
        targetSteps = _settings.tidalVolumeSteps;
        if (targetSteps > _settings.maxCompressSteps)
            targetSteps = _settings.maxCompressSteps;
    } else {
        // PCV: plan full stroke; FSM will stop early on pressure target
        targetSteps = _settings.maxCompressSteps;
    }

    HAL_Motor_SetDirection(MOTOR_DIR_COMPRESS);
    Kin_PlanMove(targetSteps, _inhaleTimeMs);

    Safety_SetLEDs(true, false, false);     // Green = inhaling
    _state        = STATE_INHALE;
    _stateEntryMs = HAL_GetMillis();
}

// =============================================================
// FSM_Init
// =============================================================
void FSM_Init() {
    _state = STATE_BOOT;
    _mode  = MODE_VCV;

    _settings.bpm              = DEFAULT_BPM;
    _settings.ieRatio          = DEFAULT_IE_RATIO;
    _settings.tidalVolumeSteps = DEFAULT_TIDAL_STEPS;
    _settings.targetPIP_kPa   = DEFAULT_TARGET_PIP_KPA;
    _settings.maxCompressSteps = 0;   // set during calibration

    Filter_EMA_Init(&_flowFilter, 0.15f);   // moderate smoothing
    _computeBreathTiming();
    _stateEntryMs     = HAL_GetMillis();
    _calibRetracting  = false;
}

// =============================================================
// FSM_Update — Called every loop() iteration
// =============================================================
void FSM_Update() {
    uint32_t now = HAL_GetMillis();

    // ---- Slow-loop: periodic sensor read ----
    if ((now - _lastSensorMs) >= SENSOR_POLL_INTERVAL_MS) {
        _lastSensorMs = now;
        _currentFlowVoltage = HAL_Sensors_ReadFlowVoltage();
        float rawKpa = Filter_VoltageToKpa(_currentFlowVoltage);
        _currentPressureKpa = Filter_EMA_Update(&_flowFilter, rawKpa);
        _currentFlowLPM     = Filter_KpaToFlowLPM(_currentPressureKpa);

        Safety_Update(_currentPressureKpa);
    }

    // ---- Telemetry print (during active ventilation only) ----
    if ((_state == STATE_INHALE || _state == STATE_EXHALE) &&
        (now - _lastTelemetryMs) >= TELEMETRY_PRINT_INTERVAL_MS) {
        _lastTelemetryMs = now;

        if (_graphMode) {
            // Serial Plotter format: tab-separated numbers
            // Labels: Pressure(kPa)  Flow(L/min)  Steps
            Serial.print(_currentPressureKpa, 2);
            Serial.print('\t');
            Serial.print(_currentFlowLPM, 1);
            Serial.print('\t');
            Serial.println(Kin_GetStepsCompleted());
        } else {
            // Human-readable text format
            Serial.print(_state == STATE_INHALE ? F("INH ") : F("EXH "));
            Serial.print(F("P="));
            Serial.print(_currentPressureKpa, 2);
            Serial.print(F("kPa  Flow="));
            Serial.print(_currentFlowLPM, 1);
            Serial.print(F("L/min  Stp="));
            Serial.print(Kin_GetStepsCompleted());
            Serial.print(F("/"));
            Serial.println(_currentInhaleSteps > 0 ? _currentInhaleSteps : _settings.tidalVolumeSteps);
        }
    }

    // ---- Global fault override ----
    if (Safety_IsFaulted() && _state != STATE_FAULT) {
        Kin_Stop();
        _state        = STATE_FAULT;
        _stateEntryMs = now;
        Serial.println(F("[FSM] FAULT detected — motor disabled."));
        return;
    }

    // ---- State machine ----
    switch (_state) {

    // ----------------------------------------------------------
    case STATE_BOOT:
        Safety_SetLEDs(false, true, false);             // Yellow
        HAL_Motor_Enable();
        HAL_Motor_SetDirection(MOTOR_DIR_COMPRESS);
        Kin_PlanConstantMove(KIN_CALIBRATE_INTERVAL_US);
        _calibRetracting = false;
        _state           = STATE_CALIBRATE;
        _stateEntryMs    = now;
        Serial.println(F("[FSM] Calibration started — advancing to Hall limit..."));
        break;

    // ----------------------------------------------------------
    case STATE_CALIBRATE:
        if (!_calibRetracting) {
            // Phase A: advance until Hall triggers
            if (HAL_Sensors_IsHallTriggered()) {
                Kin_Stop();
                _settings.maxCompressSteps = Kin_GetStepsCompleted();
                Serial.print(F("[CAL] Hall triggered at step "));
                Serial.println(_settings.maxCompressSteps);

                // Phase B: retract to Home
                HAL_Motor_SetDirection(MOTOR_DIR_RETRACT);
                Kin_PlanMove(CALIBRATE_RETRACT_STEPS, 2000);
                _calibRetracting = true;
            }
            else if ((now - _stateEntryMs) > CALIBRATE_TIMEOUT_MS) {
                Kin_Stop();
                Safety_SetFault(FAULT_HALL_NOT_FOUND);
                Serial.println(F("[CAL] FAULT: Hall sensor not found."));
            }
            else {
                Kin_Update();
            }
        } else {
            // Phase B: wait for retraction to finish
            Kin_Update();
            if (Kin_IsComplete()) {
                Serial.println(F("[CAL] Home established. System READY."));
                Safety_SetLEDs(true, false, false);     // Green
                _state        = STATE_READY;
                _stateEntryMs = now;
            }
        }
        break;

    // ----------------------------------------------------------
    case STATE_READY:
        // Idle — waiting for FSM_StartVentilation().
        break;

    // ----------------------------------------------------------
    case STATE_INHALE: {
        Kin_Update();

        // PCV: stop advancing once target PIP is reached
        if (_mode == MODE_PCV &&
            _currentPressureKpa >= _settings.targetPIP_kPa) {
            Kin_Stop();
        }

        // Transition to exhale when move is done OR time expires
        uint32_t elapsed = now - _stateEntryMs;
        if (Kin_IsComplete() || elapsed >= _inhaleTimeMs) {
            _currentInhaleSteps = Kin_GetStepsCompleted();

            HAL_Motor_SetDirection(MOTOR_DIR_RETRACT);
            Kin_PlanMove(_currentInhaleSteps, _exhaleTimeMs);

            Safety_SetLEDs(false, true, false);         // Yellow = exhaling
            _state        = STATE_EXHALE;
            _stateEntryMs = now;
        }
        break;
    }

    // ----------------------------------------------------------
    case STATE_EXHALE: {
        Kin_Update();

        uint32_t elapsed = now - _stateEntryMs;
        if (Kin_IsComplete() || elapsed >= _exhaleTimeMs) {
            // Breath cycle complete — start next inhale
            _startInhale();
        }
        break;
    }

    // ----------------------------------------------------------
    case STATE_FAULT:
        // Motor already disabled by Safety_Update().
        // Stay here until Safety_ClearFault() + FSM reset.
        HAL_Motor_Disable();
        break;
    }
}

// =============================================================
// GETTERS
// =============================================================
VentState FSM_GetState() { return _state; }
VentMode  FSM_GetMode()  { return _mode;  }

// =============================================================
// SETTERS (safe to call while ventilating — take effect next breath)
// =============================================================
void FSM_SetMode(VentMode mode) { _mode = mode; }

void FSM_SetBPM(uint8_t bpm) {
    _settings.bpm = constrain(bpm, 10, 30);
    _computeBreathTiming();
}

void FSM_SetIERatio(float ratio) {
    _settings.ieRatio = ratio;
    _computeBreathTiming();
}

void FSM_SetTidalVolumeSteps(int32_t steps) {
    // Clamp to physical limits: minimum useful stroke → full Ambu compression
    _settings.tidalVolumeSteps = constrain(steps, 100, MECH_FULL_COMPRESS_STEPS);
}

void FSM_SetTargetPIP(float kpa) {
    _settings.targetPIP_kPa = kpa;
}

// =============================================================
// START / STOP
// =============================================================
void FSM_StartVentilation() {
    if (_state != STATE_READY || !Kin_IsComplete()) return;
    Serial.println(F("[FSM] Ventilation STARTED."));
    _startInhale();
}

void FSM_StopVentilation() {
    Kin_Stop();
    HAL_Motor_Disable();
    Safety_SetLEDs(true, false, false);
    _state        = STATE_READY;
    _stateEntryMs = HAL_GetMillis();
    Serial.println(F("[FSM] Ventilation STOPPED."));
}

// =============================================================
// GETTERS for telemetry
// =============================================================
const VentSettings* FSM_GetSettings()       { return &_settings; }
float     FSM_GetCurrentPressure()           { return _currentPressureKpa; }
float     FSM_GetCurrentFlowLPM()            { return _currentFlowLPM; }
uint32_t  FSM_GetInhaleTimeMs()              { return _inhaleTimeMs; }
uint32_t  FSM_GetExhaleTimeMs()              { return _exhaleTimeMs; }
void      FSM_SetGraphMode(bool enabled)     { _graphMode = enabled; }

