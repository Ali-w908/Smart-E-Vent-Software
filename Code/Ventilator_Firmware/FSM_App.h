// ===========================================================
// FSM_App.h — Application Layer: Ventilator State Machine
// Smart E-Ventilator Firmware v1.0
// ===========================================================
#ifndef FSM_APP_H
#define FSM_APP_H

#include <Arduino.h>

// =============================================================
// VENTILATOR STATES
// =============================================================
typedef enum {
    STATE_BOOT,         // Power-on initialization
    STATE_CALIBRATE,    // Find max compression via Hall sensor
    STATE_READY,        // Calibrated, waiting to start
    STATE_INHALE,       // Motor compressing Ambu bag
    STATE_EXHALE,       // Motor retracting
    STATE_FAULT         // Critical alarm — motor disabled
} VentState;

// =============================================================
// VENTILATION MODES
// =============================================================
typedef enum {
    MODE_VCV,   // Volume Control Ventilation
    MODE_PCV    // Pressure Control Ventilation
} VentMode;

// =============================================================
// VENTILATOR SETTINGS
// =============================================================
typedef struct {
    uint8_t  bpm;                   // Breaths per minute (10–30)
    float    ieRatio;               // I:E denominator (e.g. 2.0 → 1:2)
    int32_t  tidalVolumeSteps;      // VCV target stroke (motor steps)
    float    targetPIP_kPa;         // PCV target peak pressure (kPa)
    int32_t  maxCompressSteps;      // Physical limit from calibration
} VentSettings;

// =============================================================
// PUBLIC API
// =============================================================
void      FSM_Init();
void      FSM_Update();             // Call every loop() iteration

VentState FSM_GetState();
VentMode  FSM_GetMode();

void      FSM_SetMode(VentMode mode);
void      FSM_SetBPM(uint8_t bpm);
void      FSM_SetIERatio(float ratio);
void      FSM_SetTidalVolumeSteps(int32_t steps);
void      FSM_SetTargetPIP(float kpa);

void      FSM_StartVentilation();   // READY -> INHALE
void      FSM_StopVentilation();    // Any  -> READY

// Getters for telemetry / serial display
const VentSettings* FSM_GetSettings();
float     FSM_GetCurrentPressure();
float     FSM_GetCurrentFlowLPM();
uint32_t  FSM_GetInhaleTimeMs();
uint32_t  FSM_GetExhaleTimeMs();
void      FSM_SetGraphMode(bool enabled);

#endif // FSM_APP_H
