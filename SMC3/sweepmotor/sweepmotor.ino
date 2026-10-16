// IBT-2 (BTS7960) Motor Driver Test
// Forward 2s, pause, Backward 2s, then stop forever

// --- Pin definitions ---
const int RPWM = 5;    // PWM pin for Forward (was D2 – NOT PWM capable)
const int LPWM = 9;    // PWM pin for Reverse (D9 is PWM capable)
const int EN   = 3;    // Enable pin – tie R_EN and L_EN together on the IBT-2

// "Slow speed" – 0-255. Start low, raise if the motor won't turn.
const int SLOW_SPEED = 90;

void setup() {
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(EN, OUTPUT);

  // Enable the driver (must be HIGH for the motor to run)
  digitalWrite(EN, HIGH);

  // Ensure motor is stopped at power-up
  stopMotor();
  delay(500);
}

void loop() {
  // --- Forward 2 seconds ---
  forward(SLOW_SPEED);
  delay(2000);

  // --- Brief pause (optional, avoids abrupt reversal) ---
  stopMotor();
  delay(300);

  // --- Backward 2 seconds ---
  backward(SLOW_SPEED);
  delay(2000);

  // --- Stop and hold forever ---
  stopMotor();
  while (true) { }   // halt here so it doesn't loop
}

// --- Helper functions ---
void forward(int speed) {
  analogWrite(LPWM, 0);        // Ensure reverse is off
  analogWrite(RPWM, speed);    // Forward PWM
}

void backward(int speed) {
  analogWrite(RPWM, 0);        // Ensure forward is off
  analogWrite(LPWM, speed);    // Reverse PWM
}

void stopMotor() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}
