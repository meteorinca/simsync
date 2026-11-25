// Safe Auto-Calibration Motor Sketch for Arduino Uno + IBT-2 + Pots
// Motor 2 pot feedback is software-inverted to account for reverse mounting.

// Motor 1 Pins
const int RPWM1 = 5;
const int LPWM1 = 6;
const int EN1   = 4;
const int POT1  = A0;

// Motor 2 Pins (Pot inverted via software due to reverse mounting)
const int RPWM2 = 9;
const int LPWM2 = 10;
const int EN2   = 8;
const int POT2  = A1;

// Motor 3 Pins
const int RPWM3 = 3;
const int LPWM3 = 11;
const int EN3   = 7;
const int POT3  = A2;

void setup() {
    Serial.begin(9600);
    
    pinMode(RPWM1, OUTPUT); pinMode(LPWM1, OUTPUT); pinMode(EN1, OUTPUT);
    pinMode(RPWM2, OUTPUT); pinMode(LPWM2, OUTPUT); pinMode(EN2, OUTPUT);
    pinMode(RPWM3, OUTPUT); pinMode(LPWM3, OUTPUT); pinMode(EN3, OUTPUT);

    stopAllMotors();
    
    Serial.println("=== Safe Auto-Calibration Mode Ready (M2 Inverted) ===");
    Serial.println("Commands:");
    Serial.println("  - Type 'm1cal', 'm2cal', or 'm3cal' for auto-direction test");
    Serial.println("  - Type 'stop' to halt all motors");
    Serial.println("  - Type e.g. '1f30' for manual safe move (max PWM 50)");
    Serial.println("-----------------------------------------------------");
}

void loop() {
    // Read pots (Inverting M2 using 1023 - value)
    int p1 = analogRead(POT1);
    int p2 = 1023 - analogRead(POT2); // Software inversion for Motor 2
    int p3 = analogRead(POT3);

    // Print live potentiometer feedback every 500ms
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
        lastPrint = millis();
        Serial.print("Pots -> A0(M1): ");
        Serial.print(p1);
        Serial.print(" | A1(M2 Inv): ");
        Serial.print(p2);
        Serial.print(" | A2(M3): ");
        Serial.println(p3);
    }

    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.equalsIgnoreCase("stop")) {
            stopAllMotors();
            Serial.println(">>> All motors stopped.");
            return;
        }

        // Handle auto-calibration commands
        if (input.equalsIgnoreCase("m1cal")) {
            runAutoCalibration(1, RPWM1, LPWM1, EN1, POT1, false);
            return;
        } else if (input.equalsIgnoreCase("m2cal")) {
            runAutoCalibration(2, RPWM2, LPWM2, EN2, POT2, true); // true = inverted
            return;
        } else if (input.equalsIgnoreCase("m3cal")) {
            runAutoCalibration(3, RPWM3, LPWM3, EN3, POT3, false);
            return;
        }

        // Manual fallback command format: e.g., "2f30"
        if (input.length() >= 3) {
            int motorNum = input.substring(0, 1).toInt();
            char direction = input.charAt(1);
            int speed = input.substring(2).toInt();
            
            speed = constrain(speed, 0, 50);

            runMotor(motorNum, direction, speed);
            
            Serial.print("-> Manual: Motor ");
            Serial.print(motorNum);
            Serial.print(" | Dir: ");
            Serial.print(direction);
            Serial.print(" | Speed (Capped 50): ");
            Serial.println(speed);
        } else {
            Serial.println("Invalid format. Use m1cal, stop, or 1f30.");
        }
    }
}

void runAutoCalibration(int motorNum, int rpwm, int lpwm, int en, int potPin, bool isInverted) {
    Serial.print("\n[Auto-Cal] Testing Motor ");
    Serial.print(motorNum);
    Serial.println("...");

    int rawStart = analogRead(potPin);
    int startVal = isInverted ? (1023 - rawStart) : rawStart;
    
    // Pulse Forward at safe low speed (PWM = 35)
    digitalWrite(en, HIGH);
    analogWrite(rpwm, 35);
    analogWrite(lpwm, 0);
    
    // Short burst duration: 250 milliseconds
    delay(250); 
    
    // Immediate stop
    stopAllMotors();
    
    int rawEnd = analogRead(potPin);
    int endVal = isInverted ? (1023 - rawEnd) : rawEnd;
    int delta = endVal - startVal;

    Serial.print("  Start Pot: "); Serial.print(startVal);
    Serial.print(" | End Pot: "); Serial.print(endVal);
    Serial.print(" | Delta: "); Serial.println(delta);

    if (delta > 3) {
        Serial.println("  RESULT: RIGHT! Forward command increases pot value.");
    } else if (delta < -3) {
        Serial.println("  RESULT: NOT RIGHT (REVERSED)! Forward command decreases pot value.");
    } else {
        Serial.println("  RESULT: NO MOVEMENT DETECTED. Check wiring or mechanical linkage.");
    }
    Serial.println("-----------------------------------------------------\n");
}

void runMotor(int motor, char dir, int spd) {
    int rpwm = (motor == 1) ? RPWM1 : (motor == 2) ? RPWM2 : RPWM3;
    int lpwm = (motor == 1) ? LPWM1 : (motor == 2) ? LPWM2 : LPWM3;
    int en   = (motor == 1) ? EN1   : (motor == 2) ? EN2   : EN3;

    digitalWrite(en, HIGH);

    if (dir == 'f' || dir == 'F') {
        analogWrite(rpwm, spd);
        analogWrite(lpwm, 0);
    } else if (dir == 'r' || dir == 'R') {
        analogWrite(rpwm, 0);
        analogWrite(lpwm, spd);
    }
}

void stopAllMotors() {
    digitalWrite(EN1, LOW); digitalWrite(RPWM1, LOW); digitalWrite(LPWM1, LOW);
    digitalWrite(EN2, LOW); digitalWrite(RPWM2, LOW); digitalWrite(LPWM2, LOW);
    digitalWrite(EN3, LOW); digitalWrite(RPWM3, LOW); digitalWrite(LPWM3, LOW);
}