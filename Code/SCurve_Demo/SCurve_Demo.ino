// =============================================================
//  S-CURVE MOTION DEMO
//  Smart E-Ventilator — Visual S-Curve Verification
//
//  Runs the motor through an exaggerated S-Curve profile
//  over ~4 seconds so you can visually confirm smoothness.
//
//  The motor should:
//    1. Start VERY slowly (gentle ease-in)
//    2. Smoothly accelerate to cruise speed
//    3. Hold cruise briefly
//    4. Smoothly decelerate (gentle ease-out)
//    5. Stop VERY gently
//
//  Compare: 'S' = S-Curve,  'T' = Trapezoidal (linear ramp)
//           'R' = Reverse back to start
// =============================================================

#include <math.h>

#define PIN_MOTOR_PUL     2
#define PIN_MOTOR_DIR     3
#define PIN_MOTOR_ENA     4

#define MOTOR_DIR_COMPRESS LOW
#define MOTOR_DIR_RETRACT  HIGH

// --- Demo Parameters ---
#define DEMO_STEPS        1600      // 2 full revolutions at 800 ppr
#define ACCEL_FRAC        0.50f     // 50% accel (no cruise — all ramp)
#define CRUISE_FRAC       0.00f     // 0% cruise
#define DECEL_FRAC        0.50f     // 50% decel

#define SLOWEST_US        8000      // Starting speed: VERY slow (125 stp/s)
#define FASTEST_US        400       // Top speed: fast (2500 stp/s)

// Pre-calculated phase boundaries
static int32_t accelEnd;
static int32_t decelStart;

void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }

    pinMode(PIN_MOTOR_PUL, OUTPUT);
    pinMode(PIN_MOTOR_DIR, OUTPUT);
    pinMode(PIN_MOTOR_ENA, OUTPUT);

    digitalWrite(PIN_MOTOR_ENA, HIGH);  // Disabled
    digitalWrite(PIN_MOTOR_PUL, LOW);
    digitalWrite(PIN_MOTOR_DIR, MOTOR_DIR_COMPRESS);

    accelEnd   = (int32_t)(DEMO_STEPS * ACCEL_FRAC);
    decelStart = DEMO_STEPS - (int32_t)(DEMO_STEPS * DECEL_FRAC);

    Serial.println(F(""));
    Serial.println(F("============================================"));
    Serial.println(F("   S-CURVE MOTION DEMO"));
    Serial.println(F("============================================"));
    Serial.println();
    Serial.print(F("  Steps:  ")); Serial.println(DEMO_STEPS);
    Serial.print(F("  Accel:  ")); Serial.print(accelEnd); Serial.println(F(" steps (35%)"));
    Serial.print(F("  Cruise: ")); Serial.print(decelStart - accelEnd); Serial.println(F(" steps (30%)"));
    Serial.print(F("  Decel:  ")); Serial.print(DEMO_STEPS - decelStart); Serial.println(F(" steps (35%)"));
    Serial.print(F("  Slow:   ")); Serial.print(SLOWEST_US); Serial.println(F(" us/step"));
    Serial.print(F("  Fast:   ")); Serial.print(FASTEST_US); Serial.println(F(" us/step"));
    Serial.println();
    Serial.println(F("COMMANDS:"));
    Serial.println(F("  S = Run S-Curve profile (cosine easing)"));
    Serial.println(F("  T = Run Trapezoidal profile (linear ramp)"));
    Serial.println(F("  R = Reverse (retract) back to start"));
    Serial.println(F("  ? = Show step interval table"));
    Serial.println();
}

// ---------------------------------------------------------------
// Calculate step interval with S-CURVE (cosine easing)
// ---------------------------------------------------------------
uint32_t calcInterval_SCurve(int32_t step) {
    float minFreq = 1000000.0f / SLOWEST_US;
    float maxFreq = 1000000.0f / FASTEST_US;

    if (step < accelEnd) {
        // S-Curve acceleration: cosine easing
        float frac = (float)step / (float)accelEnd;
        float sCurve = (1.0f - cos(frac * M_PI)) * 0.5f;
        float freq = minFreq + sCurve * (maxFreq - minFreq);
        return (uint32_t)(1000000.0f / freq);
    } else if (step >= decelStart) {
        // S-Curve deceleration: mirror
        float frac = (float)(step - decelStart) / (float)(DEMO_STEPS - decelStart);
        float sCurve = (1.0f - cos(frac * M_PI)) * 0.5f;
        float freq = maxFreq - sCurve * (maxFreq - minFreq);
        return (uint32_t)(1000000.0f / freq);
    } else {
        return FASTEST_US;
    }
}

// ---------------------------------------------------------------
// Calculate step interval with TRAPEZOIDAL (linear INTERVAL ramp)
// This creates constant acceleration → INSTANT JERK at transitions
// You should hear/feel a sharp "snap" when accel begins and ends
// ---------------------------------------------------------------
uint32_t calcInterval_Trap(int32_t step) {
    if (step < accelEnd) {
        // Linear ramp of interval: SLOWEST → FASTEST
        float frac = (float)step / (float)accelEnd;
        return (uint32_t)(SLOWEST_US - frac * (SLOWEST_US - FASTEST_US));
    } else if (step >= decelStart) {
        // Linear ramp of interval: FASTEST → SLOWEST
        float frac = (float)(step - decelStart) / (float)(DEMO_STEPS - decelStart);
        return (uint32_t)(FASTEST_US + frac * (SLOWEST_US - FASTEST_US));
    } else {
        return FASTEST_US;
    }
}

// ---------------------------------------------------------------
// Execute a motion profile
// ---------------------------------------------------------------
void runProfile(bool sCurve) {
    const char* name = sCurve ? "S-CURVE" : "TRAPEZOIDAL";
    Serial.print(F("\n>>> Running ")); Serial.print(name);
    Serial.println(F(" profile..."));

    digitalWrite(PIN_MOTOR_ENA, LOW);   // Enable
    digitalWrite(PIN_MOTOR_DIR, MOTOR_DIR_COMPRESS);
    delay(20);

    unsigned long totalStart = millis();

    for (int32_t i = 0; i < DEMO_STEPS; i++) {
        uint32_t interval = sCurve ? calcInterval_SCurve(i) : calcInterval_Trap(i);

        // Fire step
        digitalWrite(PIN_MOTOR_PUL, HIGH);
        delayMicroseconds(5);
        digitalWrite(PIN_MOTOR_PUL, LOW);

        // Wait for the interval
        if (interval > 16000) {
            // Use delay() for very slow steps to avoid micros() overflow
            delay(interval / 1000);
        } else {
            delayMicroseconds(interval);
        }

        // Print progress every 80 steps
        if (i % 80 == 0) {
            const char* phase;
            if (i < accelEnd) phase = "ACCEL";
            else if (i >= decelStart) phase = "DECEL";
            else phase = "CRUISE";

            Serial.print(F("  Step ")); Serial.print(i);
            Serial.print(F("/")); Serial.print(DEMO_STEPS);
            Serial.print(F("  Phase=")); Serial.print(phase);
            Serial.print(F("  Interval=")); Serial.print(interval);
            Serial.print(F("us  Speed=")); Serial.print(1000000UL / interval);
            Serial.println(F(" stp/s"));
        }
    }

    unsigned long elapsed = millis() - totalStart;
    Serial.print(F(">>> DONE! Total time: ")); Serial.print(elapsed);
    Serial.println(F(" ms\n"));
}

// ---------------------------------------------------------------
// Retract back to start
// ---------------------------------------------------------------
void retract() {
    Serial.println(F("\n>>> Retracting..."));
    digitalWrite(PIN_MOTOR_ENA, LOW);
    digitalWrite(PIN_MOTOR_DIR, MOTOR_DIR_RETRACT);
    delay(20);

    for (int32_t i = 0; i < DEMO_STEPS; i++) {
        uint32_t interval = calcInterval_SCurve(i);
        digitalWrite(PIN_MOTOR_PUL, HIGH);
        delayMicroseconds(5);
        digitalWrite(PIN_MOTOR_PUL, LOW);
        if (interval > 16000) {
            delay(interval / 1000);
        } else {
            delayMicroseconds(interval);
        }
    }
    Serial.println(F(">>> Retract complete.\n"));
}

// ---------------------------------------------------------------
// Print interval table for comparison
// ---------------------------------------------------------------
void printTable() {
    Serial.println(F("\n--- Step Interval Comparison ---"));
    Serial.println(F("Step\tS-Curve_us\tTrap_us\t\tS_stp/s\t\tT_stp/s"));

    for (int32_t i = 0; i < DEMO_STEPS; i += 40) {
        uint32_t sInt = calcInterval_SCurve(i);
        uint32_t tInt = calcInterval_Trap(i);
        Serial.print(i);
        Serial.print(F("\t")); Serial.print(sInt);
        Serial.print(F("\t\t")); Serial.print(tInt);
        Serial.print(F("\t\t")); Serial.print(1000000UL / sInt);
        Serial.print(F("\t\t")); Serial.println(1000000UL / tInt);
    }
    Serial.println(F("--- End ---\n"));
}

void loop() {
    if (!Serial.available()) return;
    char cmd = Serial.read();

    switch (cmd) {
        case 'S': case 's':
            runProfile(true);   // S-Curve
            break;
        case 'T': case 't':
            runProfile(false);  // Trapezoidal
            break;
        case 'R': case 'r':
            retract();
            break;
        case '?':
            printTable();
            break;
        case '\n': case '\r': break;
        default: break;
    }
}
