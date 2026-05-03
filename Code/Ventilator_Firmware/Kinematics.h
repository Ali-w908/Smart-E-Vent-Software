// ===========================================================
// Kinematics.h — Service Layer: Trapezoidal Motion Profiles
// Smart E-Ventilator Firmware v1.0
// ===========================================================
#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>

// =============================================================
// PROFILE TUNING CONSTANTS
// =============================================================
#define KIN_ACCEL_FRACTION          0.20f   // 20 % of move = acceleration
#define KIN_DECEL_FRACTION          0.20f   // 20 % of move = deceleration
// Remaining 60 % = cruise

#define KIN_MIN_STEP_INTERVAL_US    700     // Fastest allowed (tested limit)
#define KIN_MAX_STEP_INTERVAL_US    1450    // Slowest allowed (stall boundary)
#define KIN_CALIBRATE_INTERVAL_US   1200    // Calibration speed

// =============================================================
// MOVE PROFILE STRUCTURE
// =============================================================
typedef struct {
    int32_t  targetSteps;        // Total steps for this move
    int32_t  stepsCompleted;     // Steps fired so far
    int32_t  accelSteps;         // Steps in acceleration phase
    int32_t  cruiseSteps;        // Steps in cruise phase
    int32_t  decelSteps;         // Steps in deceleration phase
    uint32_t cruiseIntervalUs;   // Step interval during cruise (µs)
    uint32_t startIntervalUs;    // Step interval at start / end (slow, µs)
    uint32_t lastStepTimeUs;     // micros() timestamp of last step
    bool     active;             // Is a move in progress?
} MoveProfile;

// =============================================================
// PUBLIC API
// =============================================================
void    Kin_Init();

// Plan a profiled move: totalSteps over totalTimeMs with trapezoidal ramp.
void    Kin_PlanMove(int32_t totalSteps, uint32_t totalTimeMs);

// Plan a constant-speed move (used during calibration).
// Runs indefinitely until Kin_Stop() is called externally.
void    Kin_PlanConstantMove(uint32_t intervalUs);

// Call every fast-loop tick.  Returns true if a step was fired this tick.
bool    Kin_Update();

// Immediately abort the current move.
void    Kin_Stop();

// Query whether the move has finished.
bool    Kin_IsComplete();

// How many steps have been completed in the current / last move.
int32_t Kin_GetStepsCompleted();

#endif // KINEMATICS_H
