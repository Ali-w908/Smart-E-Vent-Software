// =============================================================
//  FLOW SENSOR NOISE DIAGNOSTIC
//  Smart E-Ventilator — Systematic ADC Noise Source Isolation
//
//  Purpose: Identify what causes phantom flow readings by testing
//  the flow sensor under different conditions, one variable at a time.
//
//  Tests:
//    1. Baseline     — Motor OFF, everything still
//    2. Motor Enabled — Motor energized (holding torque) but not stepping
//    3. Motor Stepping, Tubes OFF   — Motor moves, sensor tubes disconnected
//    4. Motor Stepping, Tubes ON    — Motor moves, tubes connected, no bag
//    5. Motor Stepping, Full Setup  — Motor compresses bag normally
//
//  Each test samples the ADC for 5 seconds and reports statistics.
// =============================================================

#define PIN_FLOW_SENSOR   A0
#define PIN_MOTOR_PUL     2
#define PIN_MOTOR_DIR     3
#define PIN_MOTOR_ENA     4

#define MOTOR_DIR_COMPRESS LOW

// Sampling parameters
#define INTEGRATE_US      40000UL   // 40ms averaging window (same as Motor_Torque_Test)
#define TEST_DURATION_MS  5000      // 5 seconds per test
#define WARMUP_MS         3000      // 3s ADC warm-up

// Motor step parameters (match Motor_Torque_Test)
#define CRUISE_INTERVAL_US 900
#define STEPS_PER_TEST     500      // How many steps to fire during motor tests

// ---- Globals ----
static float zeroADC = 0.0f;

// ---- Result structure (must be before any function that uses it) ----
struct TestResult {
    float mean;
    float minVal;
    float maxVal;
    float peakToPeak;
    float stdDev;
    int   numSamples;
};

// =============================================================
// Read flow sensor for 40ms, return averaged ADC (sub-LSB precision)
// =============================================================
static float readFlow40ms() {
    long sum = 0;
    int count = 0;
    unsigned long start = micros();
    while (micros() - start < INTEGRATE_US) {
        sum += analogRead(PIN_FLOW_SENSOR);
        count++;
    }
    return (float)sum / (float)count;
}

// =============================================================
// Run a sampling session for TEST_DURATION_MS, collect statistics
// =============================================================

static TestResult runSamplingSession(bool motorStepping) {
    const float V_PER_ADC = 5.0f / 1023.0f;
    const float V_TO_KPA  = 1.0f / 0.45f;
    const float K_FLOW    = 6.09f;

    float samples[150];    // 5000ms / 40ms = 125 samples max
    int n = 0;

    // If motor stepping, interleave steps with sampling
    int stepCount = 0;
    unsigned long lastStepUs = micros();

    unsigned long startMs = millis();
    while (millis() - startMs < TEST_DURATION_MS && n < 150) {
        // Fire motor steps if requested
        if (motorStepping) {
            unsigned long now = micros();
            while (micros() - now < INTEGRATE_US) {
                // Fire steps at cruise interval
                if (micros() - lastStepUs >= CRUISE_INTERVAL_US && stepCount < STEPS_PER_TEST) {
                    digitalWrite(PIN_MOTOR_PUL, HIGH);
                    delayMicroseconds(5);
                    digitalWrite(PIN_MOTOR_PUL, LOW);
                    lastStepUs = micros();
                    stepCount++;
                }
                analogRead(PIN_FLOW_SENSOR);  // Keep sampling
            }
            // Now do the actual averaged read
            samples[n] = readFlow40ms();
        } else {
            samples[n] = readFlow40ms();
        }
        n++;

        // Print live ADC every sample
        float deltaADC = samples[n-1] - zeroADC;
        float dV = deltaADC * V_PER_ADC;
        float dP_kPa = dV * V_TO_KPA;
        float flow = 0.0f;
        if (fabs(deltaADC) >= 1.0f) {
            float effectiveADC = (deltaADC > 0) ? (deltaADC - 1.0f) : (deltaADC + 1.0f);
            float dV2 = effectiveADC * V_PER_ADC;
            float dP2 = dV2 * V_TO_KPA;
            float sign = (dP2 > 0) ? 1.0f : -1.0f;
            flow = sign * K_FLOW * sqrt(fabs(dP2) * 1000.0f);
        }

        Serial.print(n);
        Serial.print(F(","));
        Serial.print(samples[n-1], 2);
        Serial.print(F(","));
        Serial.print(deltaADC, 2);
        Serial.print(F(","));
        Serial.println(flow, 1);
    }

    // Calculate statistics
    TestResult r;
    r.numSamples = n;
    r.minVal = 9999.0f;
    r.maxVal = -9999.0f;
    float sum = 0.0f;

    for (int i = 0; i < n; i++) {
        sum += samples[i];
        if (samples[i] < r.minVal) r.minVal = samples[i];
        if (samples[i] > r.maxVal) r.maxVal = samples[i];
    }
    r.mean = sum / n;
    r.peakToPeak = r.maxVal - r.minVal;

    // Std deviation
    float varSum = 0.0f;
    for (int i = 0; i < n; i++) {
        float diff = samples[i] - r.mean;
        varSum += diff * diff;
    }
    r.stdDev = sqrt(varSum / n);

    return r;
}

static void printResult(const char* testName, TestResult r) {
    const float V_PER_ADC = 5.0f / 1023.0f;

    Serial.println();
    Serial.println(F("──────────────────────────────────────────"));
    Serial.print(F("  RESULT: ")); Serial.println(testName);
    Serial.println(F("──────────────────────────────────────────"));
    Serial.print(F("  Samples:      ")); Serial.println(r.numSamples);
    Serial.print(F("  Zero ADC:     ")); Serial.println(zeroADC, 2);
    Serial.print(F("  Mean ADC:     ")); Serial.println(r.mean, 2);
    Serial.print(F("  Min ADC:      ")); Serial.println(r.minVal, 2);
    Serial.print(F("  Max ADC:      ")); Serial.println(r.maxVal, 2);
    Serial.print(F("  Peak-to-Peak: ")); Serial.print(r.peakToPeak, 2);
    Serial.println(F(" counts"));
    Serial.print(F("  Std Dev:      ")); Serial.print(r.stdDev, 3);
    Serial.println(F(" counts"));
    Serial.print(F("  Mean Δ Zero:  ")); Serial.print(r.mean - zeroADC, 2);
    Serial.println(F(" counts"));
    Serial.print(F("  P-P Voltage:  ")); Serial.print(r.peakToPeak * V_PER_ADC * 1000.0f, 1);
    Serial.println(F(" mV"));
    Serial.println(F("──────────────────────────────────────────\n"));
}

// =============================================================
// SETUP
// =============================================================
void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }

    pinMode(PIN_MOTOR_PUL, OUTPUT);
    pinMode(PIN_MOTOR_DIR, OUTPUT);
    pinMode(PIN_MOTOR_ENA, OUTPUT);
    pinMode(PIN_FLOW_SENSOR, INPUT);

    digitalWrite(PIN_MOTOR_ENA, HIGH);  // Disabled
    digitalWrite(PIN_MOTOR_PUL, LOW);
    digitalWrite(PIN_MOTOR_DIR, MOTOR_DIR_COMPRESS);

    Serial.println(F(""));
    Serial.println(F("============================================"));
    Serial.println(F("   FLOW SENSOR NOISE DIAGNOSTIC"));
    Serial.println(F("   Smart E-Ventilator"));
    Serial.println(F("============================================"));
    Serial.println();

    // ADC warm-up
    Serial.println(F("--- ADC WARM-UP (3s) ---"));
    for (int i = 0; i < WARMUP_MS; i++) {
        analogRead(PIN_FLOW_SENSOR);
        delay(1);
    }
    Serial.println(F("  Done.\n"));

    // Auto-zero (16 × 40ms reads averaged)
    Serial.println(F("--- AUTO-ZERO ---"));
    float zSum = 0.0f;
    for (int i = 0; i < 16; i++) {
        zSum += readFlow40ms();
    }
    zeroADC = zSum / 16.0f;
    Serial.print(F("  Zero ADC = ")); Serial.print(zeroADC, 2);
    Serial.print(F("  (")); Serial.print(zeroADC * 5.0f / 1023.0f, 4);
    Serial.println(F(" V)\n"));

    Serial.println(F("TESTS:"));
    Serial.println(F("  1 = Baseline (motor OFF, everything still)"));
    Serial.println(F("  2 = Motor Enabled (energized, not stepping)"));
    Serial.println(F("  3 = Motor Stepping (DISCONNECT tubes first!)"));
    Serial.println(F("  4 = Motor Stepping (tubes ON, no bag contact)"));
    Serial.println(F("  5 = Motor Stepping (full setup, compressing bag)"));
    Serial.println(F("  Z = Re-zero the sensor"));
    Serial.println(F("\nEach test runs 5s. CSV format: sample,adc,delta,flow_Lmin\n"));
    Serial.println(F("Send test number to begin.\n"));
}

// =============================================================
// MAIN LOOP
// =============================================================
void loop() {
    if (!Serial.available()) return;
    char cmd = Serial.read();

    switch (cmd) {
        case '1': {
            Serial.println(F("\n===== TEST 1: BASELINE (Motor OFF) ====="));
            Serial.println(F("Ensure: Motor disabled, no movement, no airflow."));
            Serial.println(F("sample,adc,delta,flow_Lmin"));
            digitalWrite(PIN_MOTOR_ENA, HIGH);  // Disabled
            TestResult r = runSamplingSession(false);
            printResult("BASELINE (Motor OFF)", r);
            break;
        }

        case '2': {
            Serial.println(F("\n===== TEST 2: MOTOR ENABLED (holding, not stepping) ====="));
            Serial.println(F("Ensure: Motor enabled (holding torque), no stepping."));
            Serial.println(F("sample,adc,delta,flow_Lmin"));
            digitalWrite(PIN_MOTOR_ENA, LOW);   // Enabled (holding torque)
            delay(100);  // Let current stabilize
            TestResult r = runSamplingSession(false);
            printResult("MOTOR ENABLED (no stepping)", r);
            digitalWrite(PIN_MOTOR_ENA, HIGH);  // Disable after test
            break;
        }

        case '3': {
            Serial.println(F("\n===== TEST 3: MOTOR STEPPING, TUBES DISCONNECTED ====="));
            Serial.println(F("*** DISCONNECT flow sensor tubes FIRST! ***"));
            Serial.println(F("This isolates ELECTRICAL noise (EMI) from motor."));
            Serial.println(F("Starting in 3 seconds..."));
            delay(3000);
            Serial.println(F("sample,adc,delta,flow_Lmin"));
            digitalWrite(PIN_MOTOR_ENA, LOW);
            digitalWrite(PIN_MOTOR_DIR, MOTOR_DIR_COMPRESS);
            delay(10);
            TestResult r = runSamplingSession(true);
            printResult("MOTOR STEPPING, TUBES OFF (EMI test)", r);
            digitalWrite(PIN_MOTOR_ENA, HIGH);
            break;
        }

        case '4': {
            Serial.println(F("\n===== TEST 4: MOTOR STEPPING, TUBES ON, NO BAG ====="));
            Serial.println(F("Ensure: Tubes reconnected. Mechanism moves but"));
            Serial.println(F("        does NOT touch/compress the Ambu bag."));
            Serial.println(F("This isolates VIBRATION from compression."));
            Serial.println(F("Starting in 3 seconds..."));
            delay(3000);
            Serial.println(F("sample,adc,delta,flow_Lmin"));
            digitalWrite(PIN_MOTOR_ENA, LOW);
            digitalWrite(PIN_MOTOR_DIR, MOTOR_DIR_COMPRESS);
            delay(10);
            TestResult r = runSamplingSession(true);
            printResult("MOTOR STEPPING, TUBES ON, NO BAG (vibration test)", r);
            digitalWrite(PIN_MOTOR_ENA, HIGH);
            break;
        }

        case '5': {
            Serial.println(F("\n===== TEST 5: FULL SETUP (compressing bag) ====="));
            Serial.println(F("Ensure: Everything connected. Motor will compress bag."));
            Serial.println(F("Starting in 3 seconds..."));
            delay(3000);
            Serial.println(F("sample,adc,delta,flow_Lmin"));
            digitalWrite(PIN_MOTOR_ENA, LOW);
            digitalWrite(PIN_MOTOR_DIR, MOTOR_DIR_COMPRESS);
            delay(10);
            TestResult r = runSamplingSession(true);
            printResult("FULL SETUP (compressing bag)", r);
            digitalWrite(PIN_MOTOR_ENA, HIGH);
            break;
        }

        case 'Z': case 'z': {
            Serial.println(F("\n--- RE-ZEROING ---"));
            float zSum = 0.0f;
            for (int i = 0; i < 16; i++) {
                zSum += readFlow40ms();
            }
            zeroADC = zSum / 16.0f;
            Serial.print(F("  New Zero ADC = ")); Serial.println(zeroADC, 2);
            break;
        }

        case '\n': case '\r': break;
        default: break;
    }
}
