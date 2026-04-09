// ===========================================================
// Motor_Torque_Test.ino — Standalone Motor Torque & Load Test
// Smart E-Ventilator — Pre-Integration Safety Test
//
// PURPOSE:
//   Incrementally test motor compression on the Ambu bag
//   mechanism, starting from tiny strokes and working up.
//   Completely independent from the main firmware FSM.
//
// SAFETY FEATURES:
//   - Starts at only 10% compression (100 steps)
//   - User must manually command each increase
//   - ALM+ monitoring OFF by default (pin disconnected)
//   - Hall Effect = HOME position (not max compression)
//   - Max compression enforced by step count only
//   - Emergency stop via 'X' at any time
//   - Adjustable speed (step interval)
//
// SERIAL COMMANDS (115200 baud):
//   F  — Compress forward by current stroke distance
//   B  — Retract back to starting position
//   C  — Full cycle: compress then retract (one breath)
//   +  — Increase stroke by 50 steps
//   -  — Decrease stroke by 50 steps
//   >  — Speed up (decrease step interval by 50µs)
//   <  — Slow down (increase step interval by 50µs)
//   H  — Home: slowly retract until Hall triggers (use magnet)
//   A  — Toggle ALM+ monitoring ON/OFF
//   ?  — Print current settings and status
//   X  — EMERGENCY STOP (immediate motor disable)
//   1  — Set stroke to 10% (100 steps)
//   2  — Set stroke to 25% (250 steps)
//   3  — Set stroke to 50% (500 steps)
//   4  — Set stroke to 75% (750 steps)
//   5  — Set stroke to 100% (1000 steps)
// ===========================================================

// =============================================================
// PIN DEFINITIONS (same as main firmware)
// =============================================================
#define PIN_MOTOR_PUL       2
#define PIN_MOTOR_DIR       3
#define PIN_MOTOR_ENA       4
#define PIN_MOTOR_ALM       9

#define PIN_BUZZER          5
#define PIN_LED_GREEN       6
#define PIN_LED_YELLOW      7
#define PIN_LED_RED         8

#define PIN_HALL_SENSOR     A2

// =============================================================
// MOTOR CONSTANTS
// =============================================================
#define MOTOR_DIR_COMPRESS  LOW    // Toward Ambu bag (forward)
#define MOTOR_DIR_RETRACT   HIGH   // Away from Ambu bag (backward)
#define PULSES_PER_REV      800
#define FULL_COMPRESS_STEPS 1000    // 1.25 turns measured empirically

// Hall sensor: A3144 with 10K pull-up. HIGH (~1023) = no magnet, LOW (~9-16) = magnet
#define HALL_TRIGGER_THRESHOLD  512

// =============================================================
// TEST STATE
// =============================================================
static int32_t  strokeSteps     = 100;      // Start at 10% — very conservative
static uint32_t stepIntervalUs  = 800;      // Start slow (800µs per step)
static int32_t  currentPosition = 0;        // Track position relative to Home
static bool     motorEnabled    = false;
static bool     faultDetected   = false;
static bool     isHomed         = false;    // Must home before compressing
static bool     almEnabled      = false;    // ALM+ monitoring OFF by default

// Timing for move execution
static bool     moveActive      = false;
static int32_t  moveTarget      = 0;
static int32_t  moveCompleted   = 0;
static uint8_t  moveDirection   = MOTOR_DIR_COMPRESS;
static uint32_t lastStepUs      = 0;

// For cycle mode (compress then retract automatically)
static bool     cycleMode       = false;
static bool     cyclePhase      = false;    // false = compressing, true = retracting

// =============================================================
// FORWARD DECLARATIONS
// =============================================================
static void _handleCommand(char cmd);
static void _enableMotor();
static void _disableMotor();
static void _startCompress();
static void _startRetract();
static void _stopMove();
static void _emergencyStop();
static void _homeSequence();
static void _printStrokeSet();
static void _printStatus();
static void _buzzWarning();
static void _buzzError();

// =============================================================
// SETUP
// =============================================================
void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }

    // Motor pins
    pinMode(PIN_MOTOR_PUL, OUTPUT);
    pinMode(PIN_MOTOR_DIR, OUTPUT);
    pinMode(PIN_MOTOR_ENA, OUTPUT);
    pinMode(PIN_MOTOR_ALM, INPUT_PULLUP);

    // UI pins
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_YELLOW, OUTPUT);
    pinMode(PIN_LED_RED, OUTPUT);

    // Hall sensor
    pinMode(PIN_HALL_SENSOR, INPUT);

    // Motor starts DISABLED
    digitalWrite(PIN_MOTOR_ENA, HIGH);   // HIGH = disabled for CS-D508
    digitalWrite(PIN_MOTOR_PUL, LOW);
    digitalWrite(PIN_MOTOR_DIR, LOW);

    // Green LED on = powered, ready
    digitalWrite(PIN_LED_GREEN, HIGH);
    digitalWrite(PIN_LED_YELLOW, LOW);
    digitalWrite(PIN_LED_RED, LOW);

    Serial.println(F(""));
    Serial.println(F("============================================"));
    Serial.println(F("   MOTOR TORQUE & LOAD TEST"));
    Serial.println(F("   Smart E-Ventilator Safety Test"));
    Serial.println(F("============================================"));
    Serial.println(F(""));
    Serial.println(F("NOTE: Hall sensor = HOME (retracted) position."));
    Serial.println(F("      Place magnet near Hall sensor to mark Home."));
    Serial.println(F("      ALM+ monitoring is OFF (pin disconnected)."));
    Serial.println(F(""));
    Serial.println(F("COMMANDS:"));
    Serial.println(F("  H = Home (retract to Hall sensor / set home)"));
    Serial.println(F("  F = Compress forward    B = Retract back"));
    Serial.println(F("  C = Full cycle (fwd+back)"));
    Serial.println(F("  + = More stroke (+50)   - = Less stroke (-50)"));
    Serial.println(F("  > = Speed up            < = Slow down"));
    Serial.println(F("  A = Toggle ALM+ monitor ? = Status"));
    Serial.println(F("  X = EMERGENCY STOP"));
    Serial.println(F("  1-5 = Quick set 10/25/50/75/100%"));
    Serial.println(F(""));
    Serial.println(F(">>> Step 1: Send 'H' then place magnet near Hall to home."));
    Serial.println(F(""));
    _printStatus();
}

// =============================================================
// MAIN LOOP
// =============================================================
void loop() {
    // --- Safety: Check alarm only if ALM+ monitoring is enabled ---
    if (almEnabled && !faultDetected && digitalRead(PIN_MOTOR_ALM) == LOW) {
        _emergencyStop();
        faultDetected = true;
        Serial.println(F(""));
        Serial.println(F("!!! DRIVER ALARM DETECTED (ALM+ LOW) !!!"));
        Serial.println(F("!!! Motor disabled. Power cycle to reset. !!!"));
        _buzzError();
    }

    // --- Execute active move ---
    if (moveActive && !faultDetected) {
        uint32_t now = micros();
        if ((now - lastStepUs) >= stepIntervalUs) {
            lastStepUs = now;

            // Hall sensor is only checked during 'H' homing command.
            // Retraction stops are based purely on step count.

            // Safety during COMPRESSION: enforce step limit
            if (moveDirection == MOTOR_DIR_COMPRESS && currentPosition >= FULL_COMPRESS_STEPS) {
                _stopMove();
                Serial.println(F(""));
                Serial.println(F("[SAFETY] Max compression reached (1000 steps / 1.25 turns)!"));
                _buzzWarning();
                if (cycleMode) {
                    _startRetract();
                } else {
                    _disableMotor();
                }
                return;
            }

            // Fire one step
            digitalWrite(PIN_MOTOR_PUL, HIGH);
            delayMicroseconds(5);
            digitalWrite(PIN_MOTOR_PUL, LOW);

            moveCompleted++;

            // Track absolute position
            if (moveDirection == MOTOR_DIR_COMPRESS) {
                currentPosition++;
            } else {
                currentPosition = max(currentPosition - 1, (int32_t)0);
            }

            // Check if move is complete
            if (moveCompleted >= moveTarget) {
                _stopMove();
                Serial.print(F("[DONE] "));
                Serial.print(moveCompleted); Serial.print(F(" steps "));
                Serial.print(moveDirection == MOTOR_DIR_COMPRESS ? F("FWD") : F("REV"));
                Serial.print(F("  |  Position: ")); Serial.println(currentPosition);

                // Cycle mode: auto-switch from compress to retract
                if (cycleMode && !cyclePhase) {
                    Serial.println(F("[CYCLE] Pausing 500ms at peak, then retracting..."));
                    delay(500);
                    _startRetract();
                } else {
                    cycleMode = false;
                    _disableMotor();
                }
            }
        }

        // Blink yellow LED during movement
        digitalWrite(PIN_LED_YELLOW, (millis() / 200) % 2);
    }

    // --- Serial commands ---
    if (Serial.available()) {
        char cmd = Serial.read();
        _handleCommand(cmd);
    }
}

// =============================================================
// COMMAND HANDLER
// =============================================================
static void _handleCommand(char cmd) {
    switch (cmd) {
        case 'F': case 'f':
            if (faultDetected) { Serial.println(F("[ERR] Fault active. Power cycle.")); break; }
            if (moveActive) { Serial.println(F("[ERR] Move in progress!")); break; }
            if (!isHomed) {
                Serial.println(F("[WARN] Not homed yet! Proceeding anyway — be careful."));
                Serial.println(F("       (Send 'H' first to home properly)"));
            }
            if (currentPosition + strokeSteps > FULL_COMPRESS_STEPS) {
                int32_t allowed = FULL_COMPRESS_STEPS - currentPosition;
                Serial.print(F("[WARN] Would exceed max. Clamping to "));
                Serial.print(allowed); Serial.println(F(" steps."));
                strokeSteps = allowed;
                if (strokeSteps <= 0) {
                    Serial.println(F("[ERR] Already at max compression!"));
                    break;
                }
            }
            Serial.print(F("[FWD] Compressing ")); Serial.print(strokeSteps);
            Serial.print(F(" steps (to pos ")); Serial.print(currentPosition + strokeSteps);
            Serial.println(F(")..."));
            _startCompress();
            break;

        case 'B': case 'b':
            if (faultDetected) { Serial.println(F("[ERR] Fault active.")); break; }
            if (moveActive) { Serial.println(F("[ERR] Move in progress!")); break; }
            if (currentPosition <= 0) { Serial.println(F("[ERR] Already at home position.")); break; }
            Serial.print(F("[REV] Retracting ")); Serial.print(currentPosition);
            Serial.println(F(" steps to home..."));
            _startRetract();
            break;

        case 'C': case 'c':
            if (faultDetected) { Serial.println(F("[ERR] Fault active.")); break; }
            if (moveActive) { Serial.println(F("[ERR] Move in progress!")); break; }
            if (!isHomed) {
                Serial.println(F("[WARN] Not homed! Proceeding carefully..."));
            }
            if (currentPosition + strokeSteps > FULL_COMPRESS_STEPS) {
                strokeSteps = FULL_COMPRESS_STEPS - currentPosition;
            }
            Serial.print(F("[CYCLE] Compress ")); Serial.print(strokeSteps);
            Serial.print(F(" steps at ")); Serial.print(stepIntervalUs);
            Serial.println(F("us/step, then retract."));
            cycleMode = true;
            cyclePhase = false;
            _startCompress();
            break;

        case '+':
            strokeSteps = min(strokeSteps + 50, (int32_t)FULL_COMPRESS_STEPS);
            _printStrokeSet();
            break;

        case '-':
            strokeSteps = max(strokeSteps - 50, (int32_t)50);
            _printStrokeSet();
            break;

        case '>': case '.':
            stepIntervalUs = max(stepIntervalUs - 50, (uint32_t)150);
            Serial.print(F("[SET] Speed: ")); Serial.print(stepIntervalUs);
            Serial.print(F("us/step  ("));
            Serial.print(1000000UL / stepIntervalUs);
            Serial.println(F(" steps/sec)"));
            break;

        case '<': case ',':
            stepIntervalUs = min(stepIntervalUs + 50, (uint32_t)5000);
            Serial.print(F("[SET] Speed: ")); Serial.print(stepIntervalUs);
            Serial.print(F("us/step  ("));
            Serial.print(1000000UL / stepIntervalUs);
            Serial.println(F(" steps/sec)"));
            break;

        case 'H': case 'h':
            if (faultDetected) { Serial.println(F("[ERR] Fault active.")); break; }
            if (moveActive) { Serial.println(F("[ERR] Move in progress!")); break; }
            _homeSequence();
            break;

        case 'A': case 'a':
            almEnabled = !almEnabled;
            Serial.print(F("[SET] ALM+ monitoring: "));
            Serial.println(almEnabled ? F("ON — connect ALM+ wire!") : F("OFF (default)"));
            break;

        case '1': strokeSteps = 100;  _printStrokeSet(); break;
        case '2': strokeSteps = 250;  _printStrokeSet(); break;
        case '3': strokeSteps = 500;  _printStrokeSet(); break;
        case '4': strokeSteps = 750;  _printStrokeSet(); break;
        case '5': strokeSteps = 1000; _printStrokeSet(); break;

        case '?': _printStatus(); break;

        case 'X': case 'x':
            _emergencyStop();
            Serial.println(F("[STOP] Emergency stop. Motor disabled."));
            break;

        case '\n': case '\r':
            break;

        default:
            break;
    }
}

// =============================================================
// MOTOR CONTROL HELPERS
// =============================================================
static void _enableMotor() {
    digitalWrite(PIN_MOTOR_ENA, LOW);   // LOW = enabled for CS-D508
    motorEnabled = true;
    delay(10);
}

static void _disableMotor() {
    digitalWrite(PIN_MOTOR_ENA, HIGH);  // HIGH = disabled
    motorEnabled = false;
    digitalWrite(PIN_LED_YELLOW, LOW);
}

static void _startCompress() {
    _enableMotor();
    digitalWrite(PIN_MOTOR_DIR, MOTOR_DIR_COMPRESS);
    delay(5);
    moveDirection = MOTOR_DIR_COMPRESS;
    moveTarget    = strokeSteps;
    moveCompleted = 0;
    lastStepUs    = micros();
    moveActive    = true;
}

static void _startRetract() {
    _enableMotor();
    digitalWrite(PIN_MOTOR_DIR, MOTOR_DIR_RETRACT);
    delay(5);
    moveDirection = MOTOR_DIR_RETRACT;
    // Retract the number of steps we actually compressed
    moveTarget    = (cycleMode && !cyclePhase) ? moveCompleted : currentPosition;
    cyclePhase    = true;
    moveCompleted = 0;
    lastStepUs    = micros();
    moveActive    = true;
}

static void _stopMove() {
    moveActive = false;
    digitalWrite(PIN_LED_YELLOW, LOW);
}

static void _emergencyStop() {
    moveActive = false;
    cycleMode  = false;
    _disableMotor();
    digitalWrite(PIN_LED_RED, HIGH);
    _buzzError();
}

// =============================================================
// HOMING SEQUENCE
//
// The Hall sensor marks HOME (fully retracted) position.
// Two options:
//   Option A: Motor retracts slowly until Hall triggers.
//   Option B: User manually places magnet near Hall,
//             and we simply mark the current position as 0.
//
// This implementation: Option A first, then falls back to
// manual if Hall is already triggered (magnet already near).
// =============================================================
static void _homeSequence() {
    int hallVal = analogRead(PIN_HALL_SENSOR);

    // Check if magnet is already near the Hall sensor
    if (hallVal < HALL_TRIGGER_THRESHOLD) {
        // Already at home — just mark it
        currentPosition = 0;
        isHomed = true;
        Serial.println(F("[HOME] Hall sensor already triggered!"));
        Serial.println(F("       Position set to 0. Ready to compress."));
        return;
    }

    // Magnet not detected — retract motor slowly to find home
    Serial.println(F("[HOME] Hall not triggered. Retracting to find home..."));
    Serial.println(F("       Place magnet near Hall sensor as motor retracts,"));
    Serial.println(F("       or press 'X' to abort."));
    Serial.println(F(""));

    _enableMotor();
    digitalWrite(PIN_MOTOR_DIR, MOTOR_DIR_RETRACT);
    delay(5);

    int stepsRetracted = 0;
    int maxRetractSteps = FULL_COMPRESS_STEPS + 400;  // Generous safety margin

    while (stepsRetracted < maxRetractSteps) {
        // Check Hall sensor
        hallVal = analogRead(PIN_HALL_SENSOR);
        if (hallVal < HALL_TRIGGER_THRESHOLD) {
            Serial.print(F("[HOME] Hall triggered after "));
            Serial.print(stepsRetracted);
            Serial.println(F(" retract steps."));
            currentPosition = 0;
            isHomed = true;
            Serial.println(F("       HOME position set. Ready to compress!"));
            _disableMotor();
            return;
        }

        // Fire one step (slow)
        digitalWrite(PIN_MOTOR_PUL, HIGH);
        delayMicroseconds(5);
        digitalWrite(PIN_MOTOR_PUL, LOW);
        delayMicroseconds(1200);  // Slow homing speed

        stepsRetracted++;

        // Print progress every 200 steps
        if (stepsRetracted % 200 == 0) {
            Serial.print(F("       Retracted ")); Serial.print(stepsRetracted);
            Serial.print(F(" steps, Hall = ")); Serial.println(analogRead(PIN_HALL_SENSOR));
        }

        // Check for emergency serial abort
        if (Serial.available()) {
            char c = Serial.peek();
            if (c == 'X' || c == 'x') {
                Serial.read();
                Serial.println(F("[HOME] Aborted by user."));
                _disableMotor();
                return;
            }
        }
    }

    Serial.println(F("[HOME] WARNING: Max retract reached without Hall trigger!"));
    Serial.println(F("       Try placing the magnet near the Hall sensor"));
    Serial.println(F("       and send 'H' again."));
    _disableMotor();
    _buzzWarning();
}

// =============================================================
// UI HELPERS
// =============================================================
static void _printStrokeSet() {
    Serial.print(F("[SET] Stroke = ")); Serial.print(strokeSteps);
    Serial.print(F(" / ")); Serial.print(FULL_COMPRESS_STEPS);
    Serial.print(F(" (")); Serial.print((strokeSteps * 100) / FULL_COMPRESS_STEPS);
    Serial.println(F("%)"));
}

static void _printStatus() {
    Serial.println(F(""));
    Serial.println(F("--- Torque Test Status ---"));
    Serial.print(F("  Homed      : ")); Serial.println(isHomed ? F("YES") : F("NO — send H"));

    Serial.print(F("  Stroke     : ")); Serial.print(strokeSteps);
    Serial.print(F(" / ")); Serial.print(FULL_COMPRESS_STEPS);
    Serial.print(F(" steps (")); Serial.print((strokeSteps * 100) / FULL_COMPRESS_STEPS);
    Serial.println(F("%)"));

    Serial.print(F("  Turns      : ")); Serial.print((float)strokeSteps / PULSES_PER_REV, 2);
    Serial.println(F(" / 1.25"));

    Serial.print(F("  Speed      : ")); Serial.print(stepIntervalUs);
    Serial.print(F(" us/step (")); Serial.print(1000000UL / stepIntervalUs);
    Serial.println(F(" steps/sec)"));

    float stepsPerSec = 1000000.0f / stepIntervalUs;
    float rpm = (stepsPerSec / PULSES_PER_REV) * 60.0f;
    Serial.print(F("  Motor RPM  : ")); Serial.println(rpm, 1);

    Serial.print(F("  Position   : ")); Serial.print(currentPosition);
    Serial.print(F(" / ")); Serial.println(FULL_COMPRESS_STEPS);

    Serial.print(F("  Motor      : ")); Serial.println(motorEnabled ? F("ENABLED") : F("DISABLED"));

    Serial.print(F("  ALM+ Mon.  : ")); Serial.print(almEnabled ? F("ON") : F("OFF"));
    if (almEnabled) {
        Serial.print(F("  Pin = ")); Serial.println(digitalRead(PIN_MOTOR_ALM) ? F("OK") : F("FAULT!"));
    } else {
        Serial.println(F("  (send A to toggle)"));
    }

    int hallVal = analogRead(PIN_HALL_SENSOR);
    Serial.print(F("  Hall Sensor: ")); Serial.print(hallVal);
    Serial.println(hallVal < HALL_TRIGGER_THRESHOLD ? F(" (MAGNET = HOME)") : F(" (clear)"));

    Serial.println(F("--------------------------"));
    Serial.println(F(""));
}

static void _buzzWarning() {
    for (int i = 0; i < 3; i++) {
        digitalWrite(PIN_BUZZER, HIGH);
        delay(100);
        digitalWrite(PIN_BUZZER, LOW);
        delay(100);
    }
}

static void _buzzError() {
    for (int i = 0; i < 5; i++) {
        digitalWrite(PIN_BUZZER, HIGH);
        delay(200);
        digitalWrite(PIN_BUZZER, LOW);
        delay(100);
    }
}
