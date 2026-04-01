// ===========================================================
// Filters.cpp — Service Layer: Signal Processing
// Smart E-Ventilator Firmware v1.1
// ===========================================================
#include "Filters.h"
#include "HAL_Sensors.h"
#include <math.h>

// =============================================================
// EMA Filter
// =============================================================
void Filter_EMA_Init(EMA_Filter* f, float alpha) {
    f->alpha       = alpha;
    f->value       = 0.0f;
    f->initialized = false;
}

float Filter_EMA_Update(EMA_Filter* f, float rawValue) {
    if (!f->initialized) {
        f->value       = rawValue;
        f->initialized = true;
    } else {
        f->value = f->alpha * rawValue + (1.0f - f->alpha) * f->value;
    }
    return f->value;
}

float Filter_EMA_GetValue(const EMA_Filter* f) {
    return f->value;
}

// =============================================================
// MPX5010DP Transfer Function
// Datasheet:  Vout = Vs * (0.09 * P + 0.04)
// Solved:     P(kPa) = (Vout - offset) / scale
// =============================================================
float Filter_VoltageToKpa(float voltage) {
    float kpa = (voltage - FLOW_SENSOR_OFFSET_V) / FLOW_SENSOR_SCALE;
    return (kpa < 0.0f) ? 0.0f : kpa;
}

// =============================================================
// Venturi Tube Flow Calculation (Bernoulli's Principle)
//
// Given:
//   D1 = 22 mm  (wide section)    -> A1 = pi * (0.011)^2 = 3.8013e-4 m^2
//   D2 = 12 mm  (narrow section)  -> A2 = pi * (0.006)^2 = 1.1310e-4 m^2
//   rho = 1.225 kg/m^3 (air at sea level, 15 C)
//
// Bernoulli + Continuity:
//   Q = A2 * sqrt( 2 * deltaP / (rho * (1 - (A2/A1)^2)) )
//
// Pre-compute:
//   beta  = D2/D1 = 0.5455
//   beta4 = beta^4 = 0.0886
//   denom = rho * (1 - beta4) = 1.225 * 0.9114 = 1.1165
//   coeff = 2 / denom = 1.7912
//   K     = A2 * sqrt(coeff) * 60000  [converts m^3/s -> L/min]
//         = 1.131e-4 * 1.3383 * 60000
//         = 9.08   L/min per sqrt(kPa)
//
// NOTE: deltaP in Pa = kPa * 1000, so:
//   Q = A2 * sqrt(2 * deltaP_Pa / denom) * 60000
//   Q = A2 * sqrt(2 * kPa * 1000 / denom) * 60000
//   Q = A2 * sqrt(2000/denom) * sqrt(kPa) * 60000
//   Q = K_final * sqrt(kPa)
//
//   K_final = 1.131e-4 * sqrt(2000 / 1.1165) * 60000
//           = 1.131e-4 * sqrt(1791.3) * 60000
//           = 1.131e-4 * 42.323 * 60000
//           = 287.2  L/min per sqrt(kPa)
//
// IMPORTANT: This is the *theoretical* value. Real-world
// discharge coefficient Cd is typically 0.95-0.98 for a
// well-machined Venturi. We apply Cd = 0.97 by default.
// =============================================================

// Pre-computed constant (see derivation above)
const float VENTURI_K = 287.2f * 0.97f;   // ~278.6 L/min per sqrt(kPa)

float Filter_KpaToFlowLPM(float kpa) {
    if (kpa <= 0.0f) return 0.0f;
    return VENTURI_K * sqrtf(kpa);
}
