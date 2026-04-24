// --- LIBRARIES ---
#include <Wire.h>
#include <Adafruit_BMP280.h>

// --- PIN ASSIGNMENTS ---
const int PIN_BUZZER     = 5;
const int PIN_LED_GREEN  = 6;
const int PIN_LED_YELLOW = 7;
const int PIN_LED_RED    = 8;

const int PIN_FLOW_SENSOR = A0; // MPX5010DP
const int PIN_HALL_SENSOR = A2; // AT3503

// MOTOR CONTROL PINS TO CS-D508
const int PIN_MOTOR_STEP  = 2; // PUL+
const int PIN_MOTOR_DIR   = 3; // DIR+
const int PIN_MOTOR_EN    = 4; // ENA+
const int PIN_MOTOR_ALARM = 9; // ALM+

// --- BMP280 SENSORS (I2C via Level Shifter) ---
// Sensor 1 (Ambient): SDO→GND  = address 0x76
// Sensor 2 (Airway):  SDO→3.3V = address 0x77
// Both on A4(SDA) and A5(SCL) through bi-directional level shifter
Adafruit_BMP280 bmpAmbient;   // 0x76
Adafruit_BMP280 bmpAirway;    // 0x77
bool bmpAmbientOk = false;
bool bmpAirwayOk  = false;

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor to open

  Serial.println(F("\n--- SMART VENTILATOR: INCREMENTAL HARDWARE TEST ---"));
  Serial.println(F("Send commands in the Serial Monitor to run tests:"));
  Serial.println(F("  1 = UI (LEDs & Buzzer)"));
  Serial.println(F("  2 = Hall Sensor"));
  Serial.println(F("  3 = Flow Sensor (MPX5010DP)"));
  Serial.println(F("  4 = Motor Control"));
  Serial.println(F("  5 = Pressure Sensors (BMP280 x2)"));
  
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);

  pinMode(PIN_MOTOR_STEP, OUTPUT);
  pinMode(PIN_MOTOR_DIR, OUTPUT);
  pinMode(PIN_MOTOR_EN, OUTPUT);
  pinMode(PIN_MOTOR_ALARM, INPUT_PULLUP);

  // Leave LEDs and Motor off to start
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_YELLOW, LOW);
  digitalWrite(PIN_LED_RED, LOW);
  digitalWrite(PIN_MOTOR_EN, LOW); // Usually LOW = Enabled on cheap drivers, HIGH = Disabled

  // --- Init I2C and BMP280 sensors ---
  Wire.begin();
  Serial.println(F("\n[INIT] Scanning I2C bus..."));
  _scanI2C();

  Serial.print(F("[INIT] BMP280 Ambient (0x76): "));
  bmpAmbientOk = bmpAmbient.begin(0x76);
  Serial.println(bmpAmbientOk ? F("OK") : F("NOT FOUND"));

  Serial.print(F("[INIT] BMP280 Airway  (0x77): "));
  bmpAirwayOk = bmpAirway.begin(0x77);
  Serial.println(bmpAirwayOk ? F("OK") : F("NOT FOUND"));

  if (bmpAmbientOk) {
    bmpAmbient.setSampling(Adafruit_BMP280::MODE_NORMAL,
                           Adafruit_BMP280::SAMPLING_X2,   // temp oversampling
                           Adafruit_BMP280::SAMPLING_X16,  // pressure oversampling
                           Adafruit_BMP280::FILTER_X4,     // IIR filter
                           Adafruit_BMP280::STANDBY_MS_63);
  }
  if (bmpAirwayOk) {
    bmpAirway.setSampling(Adafruit_BMP280::MODE_NORMAL,
                          Adafruit_BMP280::SAMPLING_X2,
                          Adafruit_BMP280::SAMPLING_X16,
                          Adafruit_BMP280::FILTER_X4,
                          Adafruit_BMP280::STANDBY_MS_63);
  }

  Serial.println(F(""));
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == '1') {
      testUIElements();
    } 
    else if (command == '2') {
      testHallSensor();
    }
    else if (command == '3') {
      testFlowSensor();
    }
    else if (command == '4') {
      testMotorControl();
    }
    else if (command == '5') {
      testPressureSensors();
    }
  }
}

// =============================================================
// I2C Bus Scanner — lists all detected devices
// =============================================================
void _scanI2C() {
  uint8_t count = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("  Found device at 0x"));
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
      count++;
    }
  }
  if (count == 0) {
    Serial.println(F("  No I2C devices found! Check wiring & level shifter."));
  } else {
    Serial.print(F("  Total: ")); Serial.print(count);
    Serial.println(F(" device(s)"));
  }
}

// ==========================================
// TEST 1: User Interface (LEDs & Buzzer)
// ==========================================
void testUIElements() {
  Serial.println(F("\n[TEST 1] Testing UI Indicators..."));
  
  Serial.println(F("  -> Green LED ON"));
  digitalWrite(PIN_LED_GREEN, HIGH);
  delay(1000);
  digitalWrite(PIN_LED_GREEN, LOW);

  Serial.println(F("  -> Yellow LED ON"));
  digitalWrite(PIN_LED_YELLOW, HIGH);
  delay(1000);
  digitalWrite(PIN_LED_YELLOW, LOW);

  Serial.println(F("  -> Red LED ON"));
  digitalWrite(PIN_LED_RED, HIGH);
  delay(1000);
  digitalWrite(PIN_LED_RED, LOW);

  Serial.println(F("  -> Buzzer Beep"));
  tone(PIN_BUZZER, 2000, 500); // 2000Hz for 0.5s
  delay(500);
  
  Serial.println(F("[TEST 1] Complete."));
}

// ==========================================
// TEST 2: AT3503 Hall Effect Sensor
// ==========================================
void testHallSensor() {
  Serial.println(F("\n[TEST 2] Reading Hall Sensor for 10 seconds..."));
  Serial.println(F("Bring a magnet close to the sensor now!"));
  
  unsigned long startTime = millis();
  while (millis() - startTime < 20000) { // Run for 10 seconds
    int hallValue = analogRead(PIN_HALL_SENSOR);
  
    Serial.print(F("  Hall ADC Value: "));
    Serial.print(hallValue);
  
    // Normal resting point is ~512. Detect >30 point variance.
    if (hallValue < 542) {
      Serial.println(F("  *** MAGNET DETECTED! ***"));
      digitalWrite(PIN_LED_GREEN, HIGH); // Magnet present
      digitalWrite(PIN_LED_RED, LOW);
    } else {
      Serial.println(F("  (No magnet)"));
      digitalWrite(PIN_LED_GREEN, LOW);
      digitalWrite(PIN_LED_RED, HIGH); // No magnet
    }
    delay(250);
  }
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED, LOW);
  Serial.println(F("[TEST 2] Complete."));
}

// ==========================================
// TEST 3: MPX5010DP Flow Sensor
//
//  STEP 1: 50Hz-immune oversampling
//  - Each read integrates for exactly 40ms
//    (2 full 50Hz mains cycles = noise cancels)
//  - ~357 ADC samples per read
//  - Returns float average (sub-LSB resolution)
//
//  Controls: Z=re-zero  X=exit
// ==========================================

#define INTEGRATE_US  40000UL   // 40ms = 2x 50Hz cycles
#define WARMUP_MS     3000      // 3s AREF warm-up
#define ZERO_READS    16        // Averaged reads for zero
#define EMA_ALPHA     0.20f     // EMA smoothing
#define DEAD_ZONE_V   0.003f    // ~3mV dead zone in ADC volts

// Read sensor for exactly 40ms, return average ADC as float.
// ~357 reads averaged → sub-LSB precision, 50Hz immune.
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

void testFlowSensor() {
  Serial.println(F("\n[TEST 3] Flow Sensor (40ms 50Hz-immune)"));

  // ---- AREF warm-up: 3s continuous reads ----
  analogReference(DEFAULT);
  Serial.println(F("--- AREF WARM-UP (3s) ---"));
  unsigned long wStart = millis();
  while (millis() - wStart < WARMUP_MS) {
    analogRead(PIN_FLOW_SENSOR);  // Continuous, no delay
  }
  Serial.println(F("  Done.\n"));

  // ---- Verify: show 10 consecutive 40ms reads ----
  Serial.println(F("--- VERIFY: 10 x 40ms reads (should be stable) ---"));
  for (int i = 0; i < 10; i++) {
    float avg = readFlow40ms();
    float v = avg * 5.0f / 1023.0f;
    Serial.print(F("  read "));
    Serial.print(i);
    Serial.print(F(": ADC="));
    Serial.print(avg, 1);
    Serial.print(F("  V="));
    Serial.println(v, 4);
  }

  // ---- Auto-zero ----
  Serial.println(F("\n--- AUTO-ZERO ---"));
  float zeroSum = 0.0f;
  for (int i = 0; i < ZERO_READS; i++) {
    zeroSum += readFlow40ms();
  }
  float zeroADC = zeroSum / ZERO_READS;
  float zeroV = zeroADC * 5.0f / 1023.0f;

  Serial.print(F("  Zero ADC="));
  Serial.print(zeroADC, 2);
  Serial.print(F("  V="));
  Serial.println(zeroV, 4);

  if (zeroV > 0.5f) {
    Serial.println(F("  [!] Zero HIGH - check wiring"));
  } else {
    Serial.println(F("  Zero OK!"));
  }

  // ---- Live measurement (60s) ----
  Serial.println(F("\n  Squeeze bag!  Z=re-zero  X=exit\n"));
  Serial.println(F("time_ms,adc_avg,zero_f,flow_raw,flow_ema"));

  const float V_PER_ADC = 5.0f / 1023.0f;
  const float V_TO_KPA  = 1.0f / 0.45f;    // dP(kPa) = deltaV / 0.45
  const float K_FLOW    = 6.09f;
  const float DEAD_ZONE_ADC = 2.0f;        // 2 ADC counts dead zone (~10mV)

  float emaFlow = 0.0f;
  float zeroF = zeroADC;
  
  // Adaptive zero variables
  float emaDerivative = 0.0f;
  float lastRaw = zeroADC;
  int quietSamples = 0;

  unsigned long t0 = millis();

  while (millis() - t0 < 60000) { // Run for 60 seconds
    float raw = readFlow40ms();
    
    // --- Adaptive Zero (Derivative method) ---
    float dRaw = fabs(raw - lastRaw);
    lastRaw = raw;
    emaDerivative = 0.2f * dRaw + 0.8f * emaDerivative;
    
    // If the signal is very flat (derivative < 2.0 counts/40ms)
    // AND we have been flat for > 480ms (12 samples)
    // AND it's close to the original zero (prevent zeroing out a steady breath)
    if (emaDerivative < 2.0f && fabs(raw - zeroF) < 15.0f) {
      quietSamples++;
      if (quietSamples > 12) {
        zeroF = 0.05f * raw + 0.95f * zeroF; // Slowly pull baseline to current reading
      }
    } else {
      quietSamples = 0;
    }

    // Normal calculation: blowing into P1 increases ADC
    float deltaADC = raw - zeroF;

    // --- Dead zone & Flow Calculation ---
    float flowRaw;
    if (fabs(deltaADC) < DEAD_ZONE_ADC) {
      flowRaw = 0.0f;
    } else {
      // Subtract dead zone to prevent sudden jump at the threshold
      float effectiveADC = (deltaADC > 0) ? (deltaADC - DEAD_ZONE_ADC) : (deltaADC + DEAD_ZONE_ADC);
      float dV = effectiveADC * V_PER_ADC;
      
      float dP_kPa = dV * V_TO_KPA;
      if (dP_kPa > 0.0f) {
        flowRaw = K_FLOW * sqrt(dP_kPa * 1000.0f);
      } else {
        flowRaw = -K_FLOW * sqrt(-dP_kPa * 1000.0f);
      }
    }

    // --- EMA ---
    emaFlow = EMA_ALPHA * flowRaw + (1.0f - EMA_ALPHA) * emaFlow;

    // --- CSV output ---
    unsigned long tNow = millis() - t0;
    Serial.print(tNow);
    Serial.print(',');
    Serial.print(raw, 1);
    Serial.print(',');
    Serial.print(zeroF, 2);
    Serial.print(',');
    Serial.print(flowRaw, 1);
    Serial.print(',');
    Serial.println(emaFlow, 1);

    // --- LEDs ---
    if (emaFlow > 1.0f) {
      digitalWrite(PIN_LED_GREEN, HIGH);
      digitalWrite(PIN_LED_RED, LOW);
    } else {
      digitalWrite(PIN_LED_GREEN, LOW);
      digitalWrite(PIN_LED_RED, HIGH);
    }

    // --- Commands ---
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'X' || c == 'x') break;
      if (c == 'Z' || c == 'z') {
        zeroF = raw;
        emaFlow = 0.0f;
        quietSamples = 0;
        Serial.print(F("  [RE-ZERO] ADC="));
        Serial.println(raw, 1);
      }
    }
  }

  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED, LOW);
  Serial.println(F("\n[TEST 3] Complete.\n"));
}

// ==========================================
// TEST 4: CS-D508 Motor Driver & Stepper
// ==========================================
void testMotorControl() {
  Serial.println(F("\n[TEST 4] Activating Motor Control Mode..."));
  Serial.println(F("Type 'L' (Left/CCW), 'R' (Right/CW), or 'S' (Stop)."));
  Serial.println(F("Type '+' (Faster), '-' (Slower), or 'X' to exit."));
  
  digitalWrite(PIN_MOTOR_EN, HIGH); // Enable motor (Adjust if your driver uses LOW to enable)
  
  // Start stopped
  digitalWrite(PIN_LED_YELLOW, HIGH);
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED, LOW);

  bool motorRunning = false;
  int delayTime = 500; // Starting pulse delay in microseconds (500us is a good medium speed)
  const int MIN_DELAY = 100;  // Absolute maximum safely allowed speed (lowest delay)
  const int MAX_DELAY = 4000; // Absolute minimum speed (highest delay)

  while(true) {
    // 1. Check if the user typed a new command
    if (Serial.available() > 0) {
      char cmd = Serial.read();
    
      if (cmd == 'L' || cmd == 'l') {
        Serial.println(F("  -> Spinning Left (Counter-Clockwise)"));
        digitalWrite(PIN_MOTOR_DIR, LOW);
        motorRunning = true;
        digitalWrite(PIN_LED_RED, HIGH);
        digitalWrite(PIN_LED_GREEN, LOW);
        digitalWrite(PIN_LED_YELLOW, LOW);
      }
      else if (cmd == 'R' || cmd == 'r') {
        Serial.println(F("  -> Spinning Right (Clockwise)"));
        digitalWrite(PIN_MOTOR_DIR, HIGH);
        motorRunning = true;
        digitalWrite(PIN_LED_GREEN, HIGH);
        digitalWrite(PIN_LED_RED, LOW);
        digitalWrite(PIN_LED_YELLOW, LOW);
      }
      else if (cmd == 'S' || cmd == 's') {
        Serial.println(F("  -> Motor Stopped."));
        motorRunning = false;
        digitalWrite(PIN_LED_YELLOW, HIGH);
        digitalWrite(PIN_LED_GREEN, LOW);
        digitalWrite(PIN_LED_RED, LOW);
      }
      else if (cmd == '+' || cmd == '=') {
        delayTime -= 100; // Decrease delay to speed up
        if (delayTime < MIN_DELAY) delayTime = MIN_DELAY;
        Serial.print(F("  -> Speed Increased! Current pulse delay: ")); Serial.print(delayTime); Serial.println(F(" us"));
      }
      else if (cmd == '-' || cmd == '_') {
        delayTime += 100; // Increase delay to slow down
        if (delayTime > MAX_DELAY) delayTime = MAX_DELAY;
        Serial.print(F("  -> Speed Decreased! Current pulse delay: ")); Serial.print(delayTime); Serial.println(F(" us"));
      }
      else if (cmd == 'X' || cmd == 'x') {
        Serial.println(F("  -> Exiting Motor Test. Disabling Motor."));
        motorRunning = false;
        digitalWrite(PIN_MOTOR_EN, LOW); // Disable motor to save power/heat
        digitalWrite(PIN_LED_YELLOW, LOW);
        digitalWrite(PIN_LED_GREEN, LOW);
        digitalWrite(PIN_LED_RED, LOW);
        break; // Exit the loop and return to the main menu
      }
    }

    // 2. Check for Driver ALARM (Fault condition)
    if (digitalRead(PIN_MOTOR_ALARM) == LOW) {
      Serial.println(F("\n  CRITICAL FAULT: MOTOR DRIVER ALARM TRIGGERED!"));
      Serial.println(F("     Driver may be overheated, over-voltage, or stalled."));
      motorRunning = false;
      digitalWrite(PIN_MOTOR_EN, LOW); // Instantly disable motor
      digitalWrite(PIN_LED_GREEN, LOW);
      digitalWrite(PIN_LED_YELLOW, LOW);
      digitalWrite(PIN_LED_RED, HIGH); // Flash red LED
      tone(PIN_BUZZER, 1000, 2000); // Loud 2-second beep
      delay(2000); 
      digitalWrite(PIN_LED_RED, LOW);
      break; // Exit the motor test instantly for safety
    }

    // 3. Pulse the motor extremely fast if it is supposed to be running
    if (motorRunning) {
      digitalWrite(PIN_MOTOR_STEP, HIGH);
      delayMicroseconds(delayTime); // Fast/slow wait based on user input
      digitalWrite(PIN_MOTOR_STEP, LOW);
      delayMicroseconds(delayTime);
    }
  }
}

// ==========================================
// TEST 5: BMP280 Dual Pressure Sensors
// ==========================================
void testPressureSensors() {
  Serial.println(F("\n[TEST 5] BMP280 Dual Pressure Sensor Test"));
  Serial.println(F("========================================="));

  // --- Phase A: Status Check ---
  Serial.println(F("\n--- Phase A: Sensor Status ---"));

  Serial.print(F("  Ambient  (0x76): "));
  if (!bmpAmbientOk) {
    // Try re-init in case it was plugged in after boot
    bmpAmbientOk = bmpAmbient.begin(0x76);
  }
  Serial.println(bmpAmbientOk ? F("CONNECTED") : F("NOT FOUND"));

  Serial.print(F("  Airway   (0x77): "));
  if (!bmpAirwayOk) {
    bmpAirwayOk = bmpAirway.begin(0x77);
  }
  Serial.println(bmpAirwayOk ? F("CONNECTED") : F("NOT FOUND"));

  if (!bmpAmbientOk && !bmpAirwayOk) {
    Serial.println(F("\n  [ERROR] No BMP280 sensors detected!"));
    Serial.println(F("  Check:"));
    Serial.println(F("    - Level shifter wiring (LV=3.3V, HV=5V)"));
    Serial.println(F("    - SDA→A4, SCL→A5 through level shifter"));
    Serial.println(F("    - Sensor VCC = 3.3V (NOT 5V directly)"));
    Serial.println(F("    - Common GND between all devices"));
    Serial.println(F("    - SDO pin: GND=0x76, 3.3V=0x77"));

    Serial.println(F("\n  Re-scanning I2C bus:"));
    _scanI2C();
    Serial.println(F("[TEST 5] Aborted."));
    return;
  }

  // --- Phase B: Single-shot readings ---
  Serial.println(F("\n--- Phase B: Single-Shot Readings ---"));

  if (bmpAmbientOk) {
    float tA = bmpAmbient.readTemperature();
    float pA = bmpAmbient.readPressure() / 100.0f;  // Pa to hPa
    Serial.print(F("  Ambient:  T="));
    Serial.print(tA, 1); Serial.print(F("°C  P="));
    Serial.print(pA, 2); Serial.println(F(" hPa"));
  }

  if (bmpAirwayOk) {
    float tW = bmpAirway.readTemperature();
    float pW = bmpAirway.readPressure() / 100.0f;
    Serial.print(F("  Airway:   T="));
    Serial.print(tW, 1); Serial.print(F("°C  P="));
    Serial.print(pW, 2); Serial.println(F(" hPa"));
  }

  if (bmpAmbientOk && bmpAirwayOk) {
    float pA = bmpAmbient.readPressure() / 100.0f;
    float pW = bmpAirway.readPressure() / 100.0f;
    float delta = pW - pA;
    Serial.print(F("  Delta (Airway - Ambient): "));
    Serial.print(delta, 2); Serial.println(F(" hPa"));
    Serial.print(F("  (That's ")); Serial.print(delta * 1.01972f, 2);
    Serial.println(F(" cmH2O)"));
  }

  // --- Phase C: Sanity check ---
  Serial.println(F("\n--- Phase C: Sanity Check ---"));
  if (bmpAmbientOk) {
    float pA = bmpAmbient.readPressure() / 100.0f;
    if (pA > 900.0f && pA < 1100.0f) {
      Serial.print(F("  Ambient pressure ")); Serial.print(pA, 1);
      Serial.println(F(" hPa — LOOKS CORRECT (sea-level range)"));
    } else {
      Serial.print(F("  Ambient pressure ")); Serial.print(pA, 1);
      Serial.println(F(" hPa — WARNING: outside normal range!"));
    }
  }
  if (bmpAirwayOk) {
    float pW = bmpAirway.readPressure() / 100.0f;
    if (pW > 900.0f && pW < 1100.0f) {
      Serial.print(F("  Airway  pressure ")); Serial.print(pW, 1);
      Serial.println(F(" hPa — LOOKS CORRECT"));
    } else {
      Serial.print(F("  Airway  pressure ")); Serial.print(pW, 1);
      Serial.println(F(" hPa — WARNING: outside normal range!"));
    }
  }

  // --- Phase D: Continuous reading ---
  Serial.println(F("\n--- Phase D: Continuous Reading (30 seconds) ---"));

  // Auto-zero BMP280 pair: capture resting offset between sensors
  float bmpZeroHpa = 0.0f;
  if (bmpAmbientOk && bmpAirwayOk) {
    float sumDelta = 0;
    for (int i = 0; i < 8; i++) {
      float a = bmpAmbient.readPressure() / 100.0f;
      float w = bmpAirway.readPressure() / 100.0f;
      sumDelta += (w - a);
      delay(70);
    }
    bmpZeroHpa = sumDelta / 8.0f;
    Serial.print(F("  BMP280 zero offset: "));
    Serial.print(bmpZeroHpa, 2); Serial.println(F(" hPa (subtracted)"));
  }

  Serial.println(F("NOW squeeze the Ambu bag!"));
  Serial.println(F("Press X to exit early.\n"));

  Serial.println(F("    Time   | Ambient(hPa) | Airway(hPa)  | Delta(hPa) | Delta(cmH2O)"));
  Serial.println(F("  ---------+--------------+--------------+------------+-------------"));

  unsigned long startTime = millis();
  unsigned long elapsed = 0;

  while (elapsed < 30000) {
    elapsed = millis() - startTime;

    float pAmb = bmpAmbientOk ? bmpAmbient.readPressure() / 100.0f : 0.0f;
    float pAir = bmpAirwayOk  ? bmpAirway.readPressure() / 100.0f  : 0.0f;
    float delta = (pAir - pAmb) - bmpZeroHpa;  // Subtract resting offset
    float deltaCmH2O = delta * 1.01972f;  // 1 hPa ≈ 1.01972 cmH2O

    // Format: time | ambient | airway | delta hPa | delta cmH2O
    Serial.print(F("  "));
    // Elapsed seconds with 1 decimal
    Serial.print(elapsed / 1000.0f, 1);
    Serial.print(F("s"));
    if (elapsed < 10000) Serial.print(' ');
    Serial.print(F("  | "));

    if (bmpAmbientOk) {
      Serial.print(pAmb, 2);
    } else {
      Serial.print(F("  N/A   "));
    }
    Serial.print(F("      | "));

    if (bmpAirwayOk) {
      Serial.print(pAir, 2);
    } else {
      Serial.print(F("  N/A   "));
    }
    Serial.print(F("      | "));

    if (bmpAmbientOk && bmpAirwayOk) {
      if (delta >= 0) Serial.print(' ');
      Serial.print(delta, 2);
      Serial.print(F("     | "));
      if (deltaCmH2O >= 0) Serial.print(' ');
      Serial.print(deltaCmH2O, 2);
    } else {
      Serial.print(F("  N/A      |   N/A"));
    }
    Serial.println();

    // Visual indicator: Green if delta > 0.5 hPa (pressure applied)
    if (bmpAmbientOk && bmpAirwayOk) {
      if (delta > 0.5f) {
        digitalWrite(PIN_LED_GREEN, HIGH);
        digitalWrite(PIN_LED_RED, LOW);
      } else if (delta < -0.5f) {
        digitalWrite(PIN_LED_RED, HIGH);
        digitalWrite(PIN_LED_GREEN, LOW);
      } else {
        digitalWrite(PIN_LED_GREEN, LOW);
        digitalWrite(PIN_LED_RED, LOW);
        digitalWrite(PIN_LED_YELLOW, HIGH);
      }
    }

    // Check for early exit
    if (Serial.available()) {
      char c = Serial.peek();
      if (c == 'X' || c == 'x') {
        Serial.read();
        Serial.println(F("\n  [EXIT] Stopped early."));
        break;
      }
      Serial.read(); // consume other chars
    }

    delay(500);  // 2 readings per second
  }

  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED, LOW);
  digitalWrite(PIN_LED_YELLOW, LOW);

  Serial.println(F("\n[TEST 5] Complete."));
}

