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

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor to open

  Serial.println("\n--- SMART VENTILATOR: INCREMENTAL HARDWARE TEST ---");
  Serial.println("Send commands '1', '2', '3', or '4' in the Serial Monitor to run tests.");
  
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
  }
}

// ==========================================
// TEST 1: User Interface (LEDs & Buzzer)
// ==========================================
void testUIElements() {
  Serial.println("\n[TEST 1] Testing UI Indicators...");
  
  Serial.println("  -> Green LED ON");
  digitalWrite(PIN_LED_GREEN, HIGH);
  delay(1000);
  digitalWrite(PIN_LED_GREEN, LOW);

  Serial.println("  -> Yellow LED ON");
  digitalWrite(PIN_LED_YELLOW, HIGH);
  delay(1000);
  digitalWrite(PIN_LED_YELLOW, LOW);

  Serial.println("  -> Red LED ON");
  digitalWrite(PIN_LED_RED, HIGH);
  delay(1000);
  digitalWrite(PIN_LED_RED, LOW);

  Serial.println("  -> Buzzer Beep");
  tone(PIN_BUZZER, 2000, 500); // 2000Hz for 0.5s
  delay(500);
  
  Serial.println("[TEST 1] Complete.");
}

// ==========================================
// TEST 2: AT3503 Hall Effect Sensor
// ==========================================
void testHallSensor() {
  Serial.println("\n[TEST 2] Reading Hall Sensor for 10 seconds...");
  Serial.println("Bring a magnet close to the sensor now!");
  
  unsigned long startTime = millis();
  while (millis() - startTime < 20000) { // Run for 10 seconds
    int hallValue = analogRead(PIN_HALL_SENSOR);
  
    Serial.print("  Hall ADC Value: ");
    Serial.print(hallValue);
  
    // Normal resting point is ~512. Detect >30 point variance.
    if (hallValue < 542) {
      Serial.println("  *** MAGNET DETECTED! ***");
      digitalWrite(PIN_LED_GREEN, HIGH); // Magnet present
      digitalWrite(PIN_LED_RED, LOW);
    } else {
      Serial.println("  (No magnet)");
      digitalWrite(PIN_LED_GREEN, LOW);
      digitalWrite(PIN_LED_RED, HIGH); // No magnet
    }
    delay(250);
  }
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED, LOW);
  Serial.println("[TEST 2] Complete.");
}

// ==========================================
// TEST 3: MPX5010DP Analog Flow Sensor
// ==========================================
void testFlowSensor() {
  Serial.println("\n[TEST 3] Reading Flow Sensor for 10 seconds...");
  Serial.println("Gently blow air into the High-Pressure port (P1) now!");
  
  unsigned long startTime = millis();
  while (millis() - startTime < 10000) { // Run for 10 seconds
    int flowVout = analogRead(PIN_FLOW_SENSOR);
  
    // Convert ADC (0-1023) to Voltage (0-5.0V)
    float voltage = flowVout * (5.0 / 1023.0);
  
    // MPX5010DP Transfer Function (Vout = VS * (0.09 * P + 0.04))
    // Solving for P (kPa) when Vs = 5.0V:
    // P = (Vout - 0.2) / 0.45
    // *Calibration Update:* Your specific sensor rests at ~0.332V, not 0.20V.
    float pressure_kPa = (voltage - 0.332) / 0.45;
  
    // If negative (sensor noise or slight vacuum), clamp to 0
    if (pressure_kPa < 0) pressure_kPa = 0;

    Serial.print("  ADC: "); Serial.print(flowVout);
    Serial.print(" | Volts: "); Serial.print(voltage, 3);
    Serial.print(" | Diff Pressure: "); Serial.print(pressure_kPa, 3);
    Serial.println(" kPa");
  
    // Light Green if flow detected, Red if still
    if (pressure_kPa > 0.05) { // 0.05kPa threshold to ignore pure sensor noise
      digitalWrite(PIN_LED_GREEN, HIGH);
      digitalWrite(PIN_LED_RED, LOW);
    } else {
      digitalWrite(PIN_LED_GREEN, LOW);
      digitalWrite(PIN_LED_RED, HIGH);
    }

    delay(250);
  }
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED, LOW);
  Serial.println("[TEST 3] Complete.");
}

// ==========================================
// TEST 4: CS-D508 Motor Driver & Stepper
// ==========================================
void testMotorControl() {
  Serial.println("\n[TEST 4] Activating Motor Control Mode...");
  Serial.println("Type 'L' (Left/CCW), 'R' (Right/CW), or 'S' (Stop).");
  Serial.println("Type '+' (Faster), '-' (Slower), or 'X' to exit.");
  
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
        Serial.println("  -> Spinning Left (Counter-Clockwise)");
        digitalWrite(PIN_MOTOR_DIR, LOW);
        motorRunning = true;
        digitalWrite(PIN_LED_RED, HIGH);
        digitalWrite(PIN_LED_GREEN, LOW);
        digitalWrite(PIN_LED_YELLOW, LOW);
      }
      else if (cmd == 'R' || cmd == 'r') {
        Serial.println("  -> Spinning Right (Clockwise)");
        digitalWrite(PIN_MOTOR_DIR, HIGH);
        motorRunning = true;
        digitalWrite(PIN_LED_GREEN, HIGH);
        digitalWrite(PIN_LED_RED, LOW);
        digitalWrite(PIN_LED_YELLOW, LOW);
      }
      else if (cmd == 'S' || cmd == 's') {
        Serial.println("  -> Motor Stopped.");
        motorRunning = false;
        digitalWrite(PIN_LED_YELLOW, HIGH);
        digitalWrite(PIN_LED_GREEN, LOW);
        digitalWrite(PIN_LED_RED, LOW);
      }
      else if (cmd == '+' || cmd == '=') {
        delayTime -= 100; // Decrease delay to speed up
        if (delayTime < MIN_DELAY) delayTime = MIN_DELAY;
        Serial.print("  -> Speed Increased! Current pulse delay: "); Serial.print(delayTime); Serial.println(" us");
      }
      else if (cmd == '-' || cmd == '_') {
        delayTime += 100; // Increase delay to slow down
        if (delayTime > MAX_DELAY) delayTime = MAX_DELAY;
        Serial.print("  -> Speed Decreased! Current pulse delay: "); Serial.print(delayTime); Serial.println(" us");
      }
      else if (cmd == 'X' || cmd == 'x') {
        Serial.println("  -> Exiting Motor Test. Disabling Motor.");
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
      Serial.println("\n  🚨 CRITICAL FAULT: MOTOR DRIVER ALARM TRIGGERED! 🚨");
      Serial.println("     Driver may be overheated, over-voltage, or stalled.");
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

