// ===========================================================
// Kinematics.cpp — Service Layer: Trapezoidal Motion Profiles
// Smart E-Ventilator Firmware v1.0
// ===========================================================
#include "Kinematics.h"
#include "HAL_Board.h"
#include "HAL_Motor.h"

// =============================================================
// PRIVATE STATE
// =============================================================
static MoveProfile _move;

// =============================================================
// INIT
// =============================================================
void Kin_Init() {
    memset(&_move, 0, sizeof(_move));
    _move.active = false;
}

// =============================================================
// Kin_PlanMove — Trapezoidal velocity profile
//
// Given totalSteps and totalTimeMs, the function pre-computes:
//   • accelSteps / cruiseSteps / decelSteps
//   • cruiseIntervalUs  (step period at peak speed)
//   • startIntervalUs   (step period at ramp start/end)
//
// During Kin_Update(), the current step interval is linearly
// interpolated between startInterval and cruiseInterval
// depending on which phase the move is in.
// =============================================================
void Kin_PlanMove(int32_t totalSteps, uint32_t totalTimeMs) {
    if (totalSteps <= 0 || totalTimeMs == 0) return;

    _move.targetSteps    = totalSteps;
    _move.stepsCompleted = 0;

    // --- phase step counts ---
    _move.accelSteps = (int32_t)(totalSteps * KIN_ACCEL_FRACTION);
    _move.decelSteps = (int32_t)(totalSteps * KIN_DECEL_FRACTION);
    if (_move.accelSteps < 1) _move.accelSteps = 1;
    if (_move.decelSteps < 1) _move.decelSteps = 1;
    _move.cruiseSteps = totalSteps - _move.accelSteps - _move.decelSteps;
    if (_move.cruiseSteps < 1) _move.cruiseSteps = 1;

    // --- cruise interval ---
    // Area under trapezoidal curve = totalSteps
    // effectiveSteps = 0.5·accel + cruise + 0.5·decel  (at cruise velocity)
    uint32_t totalTimeUs = (uint32_t)totalTimeMs * 1000UL;
    float effectiveSteps = 0.5f * _move.accelSteps
                         + (float)_move.cruiseSteps
                         + 0.5f * _move.decelSteps;
    _move.cruiseIntervalUs = (uint32_t)(totalTimeUs / effectiveSteps);

    // Clamp to safe bounds
    if (_move.cruiseIntervalUs < KIN_MIN_STEP_INTERVAL_US)
        _move.cruiseIntervalUs = KIN_MIN_STEP_INTERVAL_US;
    if (_move.cruiseIntervalUs > KIN_MAX_STEP_INTERVAL_US)
        _move.cruiseIntervalUs = KIN_MAX_STEP_INTERVAL_US;

    // Start/end interval = 3× cruise (gentle ramp)
    _move.startIntervalUs = _move.cruiseIntervalUs * 3;
    if (_move.startIntervalUs > KIN_MAX_STEP_INTERVAL_US)
        _move.startIntervalUs = KIN_MAX_STEP_INTERVAL_US;

    _move.lastStepTimeUs = HAL_GetMicros();
    _move.active         = true;
}

// =============================================================
// Kin_PlanConstantMove — Constant speed, no ramp
// Used for calibration.  Runs until Kin_Stop() is called.
// =============================================================
void Kin_PlanConstantMove(uint32_t intervalUs) {
    _move.targetSteps    = 999999;   // effectively infinite
    _move.stepsCompleted = 0;
    _move.accelSteps     = 0;
    _move.cruiseSteps    = 999999;
    _move.decelSteps     = 0;
    _move.cruiseIntervalUs = intervalUs;
    _move.startIntervalUs  = intervalUs;
    _move.lastStepTimeUs   = HAL_GetMicros();
    _move.active           = true;
}

// =============================================================
// _computeCurrentInterval — Linear interpolation per phase
// =============================================================
static uint32_t _computeCurrentInterval() {
    int32_t s = _move.stepsCompleted;

    // --- Acceleration phase ---
    if (s < _move.accelSteps && _move.accelSteps > 0) {
        uint32_t range     = _move.startIntervalUs - _move.cruiseIntervalUs;
        uint32_t reduction = (range * (uint32_t)s) / (uint32_t)_move.accelSteps;
        return _move.startIntervalUs - reduction;
    }

    // --- Deceleration phase ---
    int32_t decelStart = _move.targetSteps - _move.decelSteps;
    if (s >= decelStart && _move.decelSteps > 0) {
        int32_t  decelStep = s - decelStart;
        uint32_t range     = _move.startIntervalUs - _move.cruiseIntervalUs;
        uint32_t increase  = (range * (uint32_t)decelStep) / (uint32_t)_move.decelSteps;
        return _move.cruiseIntervalUs + increase;
    }

    // --- Cruise phase ---
    return _move.cruiseIntervalUs;
}

// =============================================================
// Kin_Update — Call from the fast loop.
// Returns true if a step was fired this tick.
// =============================================================
bool Kin_Update() {
    if (!_move.active) return false;

    if (_move.stepsCompleted >= _move.targetSteps) {
        _move.active = false;
        return false;
    }

    uint32_t now      = HAL_GetMicros();
    uint32_t interval = _computeCurrentInterval();

    if ((now - _move.lastStepTimeUs) >= interval) {
        HAL_Motor_StepPulse();
        _move.lastStepTimeUs = now;
        _move.stepsCompleted++;
        return true;
    }
    return false;
}

// =============================================================
// Utility
// =============================================================
void    Kin_Stop()              { _move.active = false; }
bool    Kin_IsComplete()        { return !_move.active; }
int32_t Kin_GetStepsCompleted() { return _move.stepsCompleted; }
