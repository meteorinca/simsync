//****************************************************************************************************************
// pot_noise_analyzer.ino — Potentiometer Noise & Motor EMI Analyzer for Arduino Uno R3 + 3x IBT-2 (BTS7960)
//
// Designed for testing pot signal noise with pots DISCONNECTED from the motor arms (so pots stay stationary
// while motors run). Matches SMC3_IBT2.ino pinout, PWM timer frequencies, and ADC prescaler so measured noise
// reflects real controller operating conditions.
//
// ===============================================================================================================
// PINOUT (Matches altSMC3/SMC3_IBT2/SMC3_IBT2.ino)
// ===============================================================================================================
//   Motor | RPWM | LPWM | EN  | Pot | Timer  | PWM Freq
//   ------+------+------+-----+-----+--------+----------
//   M1    | D5   | D6   | D4  | A0  | Timer0 | ~976 Hz (default; keeps millis/micros accurate)
//   M2    | D9   | D10  | D8  | A1  | Timer1 | 8 kHz (Phase & Freq Correct, ICR1 TOP; pot software-inverted)
//   M3    | D3   | D11  | D7  | A2  | Timer2 | 31.25 kHz (prescaler 1)
//
// ===============================================================================================================
// SERIAL MONITOR USAGE (115200 baud, Newline or Both NL & CR)
// ===============================================================================================================
//   analyze         Run full noise test driving all 3 motors (PWM 50)
//   analyze 1       Run noise test driving only Motor 1 (still measures all 3 pots for crosstalk)
//   analyze 2       Run noise test driving only Motor 2
//   analyze 3       Run noise test driving only Motor 3
//   stop            Immediately abort test and disable all motors
//
// Test sequence (5 seconds total):
//   1. BEFORE  (1.0 s) : All motors disabled — baseline pot noise
//   2. FORWARD (1.0 s) : Selected motor(s) forward at PWM 50 (hard-capped at 50)
//   3. STOP    (1.0 s) : All motors stopped/disabled
//   4. REVERSE (1.0 s) : Selected motor(s) reverse at PWM 50 (hard-capped at 50)
//   5. AFTER   (1.0 s) : All motors disabled — post-motion pot noise
//****************************************************************************************************************

#include <Arduino.h>
#include <math.h>

// ===============================================================================================================
// HARDWARE CONFIGURATION
// ===============================================================================================================

const uint8_t  NUM_MOTORS       = 3;
const uint32_t SERIAL_BAUD      = 115200;

const uint8_t  MAX_PWM          = 50;      // Hard safety cap
const uint8_t  TEST_PWM         = 50;      // Drive PWM used during analyze (<= MAX_PWM)
const uint32_t PHASE_DURATION_MS = 1000;   // 1 second per phase
const uint32_t SAMPLE_PERIOD_US = 1000;    // ~1 kHz sampling (~1000 samples/phase/pot)

const uint8_t  TIMER1_FREQ_KHZ  = 8;       // Matches SMC3_IBT2 default for M2 (D9/D10)
const uint8_t  TIMER2_FREQ_KHZ  = 31;      // Matches SMC3_IBT2 default for M3 (D3/D11)

const int      SPIKE_THRESHOLD  = 4;       // ADC counts away from baseline mean counted as a spike

#ifndef cbi
  #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
  #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

struct MotorPins {
  uint8_t rpwmPin;
  uint8_t lpwmPin;
  uint8_t enPin;
  uint8_t potPin;
  bool    invertPot;
};

const MotorPins MOTORS[NUM_MOTORS] = {
  { 5,  6, 4, A0, false },  // M1
  { 9, 10, 8, A1, true  },  // M2 (pot software-inverted)
  { 3, 11, 7, A2, false }   // M3
};

// ===============================================================================================================
// ONLINE STATISTICS (O(1) RAM per pot per phase — safe for Uno's 2 KB SRAM)
// ===============================================================================================================

struct PhaseStats {
  uint16_t count;
  int16_t  minVal;
  int16_t  maxVal;
  int16_t  prevVal;
  uint16_t maxStep;
  uint16_t spikes;
  float    mean;
  float    m2;       // Sum of squared deviations (Welford's algorithm)

  void reset() {
    count   = 0;
    minVal  = 1023;
    maxVal  = 0;
    prevVal = 0;
    maxStep = 0;
    spikes  = 0;
    mean    = 0.0f;
    m2      = 0.0f;
  }

  void addSample(int16_t x, float refMean, bool trackSpikes) {
    if (count == 0) {
      minVal  = x;
      maxVal  = x;
      prevVal = x;
    } else {
      if (x < minVal) minVal = x;
      if (x > maxVal) maxVal = x;
      uint16_t step = (uint16_t)abs(x - prevVal);
      if (step > maxStep) maxStep = step;
      prevVal = x;
    }

    count++;
    float delta = (float)x - mean;
    mean += delta / (float)count;
    float delta2 = (float)x - mean;
    m2 += delta * delta2;

    if (trackSpikes && fabsf((float)x - refMean) >= (float)SPIKE_THRESHOLD) {
      spikes++;
    }
  }

  int16_t peakToPeak() const {
    return (count > 0) ? (maxVal - minVal) : 0;
  }

  float stdDev() const {
    if (count < 2) return 0.0f;
    return sqrtf(m2 / (float)(count - 1));
  }
};

enum PhaseId : uint8_t {
  PHASE_BEFORE  = 0,
  PHASE_FORWARD = 1,
  PHASE_STOP    = 2,
  PHASE_REVERSE = 3,
  PHASE_AFTER   = 4,
  NUM_PHASES    = 5
};

PhaseStats results[NUM_PHASES][NUM_MOTORS];

// ===============================================================================================================
// PWM TIMERS & IBT-2 MOTOR DRIVE (Matches SMC3_IBT2.ino)
// ===============================================================================================================

void initPWMTimer1(uint8_t freqkHz) {
  uint8_t clamped = constrain(freqkHz, 1, 25);
  uint8_t sreg = SREG;
  cli();
  TCCR1A = 0;
  TCCR1B = _BV(WGM13);
  ICR1   = (uint16_t)((F_CPU / 2UL) / ((uint32_t)clamped * 1000UL));
  TCNT1  = 0;
  OCR1A  = 0;
  OCR1B  = 0;
  TCCR1B = _BV(WGM13) | _BV(CS10);
  SREG   = sreg;
}

void initPWMTimer2(uint8_t freqkHz) {
  uint8_t prescale = (freqkHz >= 15) ? 0x01 : 0x02;
  TCCR2B = (TCCR2B & B11111000) | prescale;
}

void writeTimer1PWM(uint8_t pin, uint8_t duty) {
  if (duty == 0) {
    if (pin == 9)  cbi(TCCR1A, COM1A1);
    if (pin == 10) cbi(TCCR1A, COM1B1);
    digitalWrite(pin, LOW);
    return;
  }
  if (duty == 255) {
    if (pin == 9)  cbi(TCCR1A, COM1A1);
    if (pin == 10) cbi(TCCR1A, COM1B1);
    digitalWrite(pin, HIGH);
    return;
  }
  uint16_t top = ICR1;
  if (top == 0) top = 1000;
  uint16_t ocr = (uint16_t)(((uint32_t)duty * top) / 255UL);
  if (ocr == 0) ocr = 1;
  if (pin == 9) {
    OCR1A = ocr;
    sbi(TCCR1A, COM1A1);
  } else if (pin == 10) {
    OCR1B = ocr;
    sbi(TCCR1A, COM1B1);
  }
}

void writeMotorPinPWM(uint8_t pin, uint8_t duty) {
  if (pin == 9 || pin == 10) {
    writeTimer1PWM(pin, duty);
  } else {
    analogWrite(pin, duty);
  }
}

int16_t readPot(uint8_t idx) {
  int16_t raw = (int16_t)analogRead(MOTORS[idx].potPin);
  return MOTORS[idx].invertPot ? (1023 - raw) : raw;
}

void disableMotor(uint8_t idx) {
  writeMotorPinPWM(MOTORS[idx].rpwmPin, 0);
  writeMotorPinPWM(MOTORS[idx].lpwmPin, 0);
  digitalWrite(MOTORS[idx].enPin, LOW);
}

void stopAllMotors() {
  for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
    disableMotor(i);
  }
}

// signedPwm: -MAX_PWM..+MAX_PWM (hard-clamped), motorMask: bit0=M1, bit1=M2, bit2=M3
void driveMotors(uint8_t motorMask, int16_t signedPwm) {
  int16_t clamped = constrain(signedPwm, -(int16_t)MAX_PWM, (int16_t)MAX_PWM);

  for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
    if ((motorMask & (1 << i)) == 0 || clamped == 0) {
      disableMotor(i);
      continue;
    }

    // Clear any latched BTS7960 fault and enable
    digitalWrite(MOTORS[i].enPin, LOW);
    digitalWrite(MOTORS[i].enPin, HIGH);

    if (clamped > 0) {
      writeMotorPinPWM(MOTORS[i].lpwmPin, 0);
      writeMotorPinPWM(MOTORS[i].rpwmPin, (uint8_t)clamped);
    } else {
      writeMotorPinPWM(MOTORS[i].rpwmPin, 0);
      writeMotorPinPWM(MOTORS[i].lpwmPin, (uint8_t)(-clamped));
    }
  }
}

// ===============================================================================================================
// ABORT CHECK & PHASE RECORDING
// ===============================================================================================================

bool checkSerialAbort() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 's' || c == 'S' || c == 'x' || c == 'X') {
      stopAllMotors();
      while (Serial.available() > 0) Serial.read();
      return true;
    }
  }
  return false;
}

bool recordPhase(PhaseId phase, uint8_t motorMask, int16_t signedPwm, bool useBaselineRef) {
  for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
    results[phase][i].reset();
  }

  driveMotors(motorMask, signedPwm);

  uint32_t startMs    = millis();
  uint32_t nextTickUs = micros();

  while ((uint32_t)(millis() - startMs) < PHASE_DURATION_MS) {
    if (checkSerialAbort()) {
      return false;
    }

    uint32_t nowUs = micros();
    if ((int32_t)(nowUs - nextTickUs) >= 0) {
      nextTickUs += SAMPLE_PERIOD_US;
      for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        int16_t val = readPot(i);
        float refMean = useBaselineRef ? results[PHASE_BEFORE][i].mean : 0.0f;
        results[phase][i].addSample(val, refMean, useBaselineRef);
      }
    }
  }

  return true;
}

// ===============================================================================================================
// REPORTING
// ===============================================================================================================

void printPhaseLabel(PhaseId p) {
  switch (p) {
    case PHASE_BEFORE:  Serial.print(F("BEFORE ")); break;
    case PHASE_FORWARD: Serial.print(F("FWD 50 ")); break;
    case PHASE_STOP:    Serial.print(F("STOP   ")); break;
    case PHASE_REVERSE: Serial.print(F("REV 50 ")); break;
    case PHASE_AFTER:   Serial.print(F("AFTER  ")); break;
    default:            Serial.print(F("???    ")); break;
  }
}

void printReport(uint8_t motorMask) {
  Serial.println();
  Serial.println(F("======================================================================"));
  Serial.print(F("POT NOISE ANALYSIS REPORT  (Driven: "));
  bool first = true;
  for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
    if (motorMask & (1 << i)) {
      if (!first) Serial.print(F(", "));
      Serial.print(F("M"));
      Serial.print(i + 1);
      first = false;
    }
  }
  Serial.print(F(" @ PWM "));
  Serial.print(TEST_PWM);
  Serial.println(F(")"));
  Serial.println(F("======================================================================"));

  // Per-motor phase table
  for (uint8_t m = 0; m < NUM_MOTORS; ++m) {
    Serial.println();
    Serial.print(F("--- Pot A"));
    Serial.print(m);
    Serial.print(F(" (M"));
    Serial.print(m + 1);
    if (MOTORS[m].invertPot) Serial.print(F(", inverted"));
    Serial.println(F(") ---"));
    Serial.println(F("Phase   |   N  |   Mean  |  Min |  Max | Pk-Pk | StdDev | MaxStep | Spikes(>=4)"));
    Serial.println(F("--------+------+---------+------+------+-------+--------+---------+------------"));

    for (uint8_t p = 0; p < NUM_PHASES; ++p) {
      const PhaseStats &s = results[p][m];
      printPhaseLabel((PhaseId)p);
      Serial.print(F(" | "));
      if (s.count < 1000) Serial.print(' ');
      if (s.count < 100)  Serial.print(' ');
      Serial.print(s.count);
      Serial.print(F(" | "));
      Serial.print(s.mean, 2);
      Serial.print(F(" | "));
      Serial.print(s.minVal);
      Serial.print(F(" | "));
      Serial.print(s.maxVal);
      Serial.print(F(" | "));
      Serial.print(s.peakToPeak());
      Serial.print(F(" | "));
      Serial.print(s.stdDev(), 2);
      Serial.print(F(" | "));
      Serial.print(s.maxStep);
      Serial.print(F(" | "));
      if (p == PHASE_BEFORE) {
        Serial.println(F("baseline"));
      } else {
        Serial.println(s.spikes);
      }
    }
  }

  // Summary comparison: Before vs During Motion vs After
  Serial.println();
  Serial.println(F("======================================================================"));
  Serial.println(F("SUMMARY: BEFORE vs DURING MOTION vs AFTER"));
  Serial.println(F("======================================================================"));

  for (uint8_t m = 0; m < NUM_MOTORS; ++m) {
    const PhaseStats &before = results[PHASE_BEFORE][m];
    const PhaseStats &fwd    = results[PHASE_FORWARD][m];
    const PhaseStats &rev    = results[PHASE_REVERSE][m];
    const PhaseStats &after  = results[PHASE_AFTER][m];

    float beforeStd  = before.stdDev();
    float fwdStd     = fwd.stdDev();
    float revStd     = rev.stdDev();
    float afterStd   = after.stdDev();
    float worstStd   = (fwdStd > revStd) ? fwdStd : revStd;

    int16_t beforePk = before.peakToPeak();
    int16_t worstPk  = max(fwd.peakToPeak(), rev.peakToPeak());
    int16_t afterPk  = after.peakToPeak();

    float fwdShift   = fwd.mean - before.mean;
    float revShift   = rev.mean - before.mean;
    float worstShift = (fabsf(fwdShift) > fabsf(revShift)) ? fwdShift : revShift;
    float afterShift = after.mean - before.mean;

    float noiseRatio = (beforeStd > 0.05f) ? (worstStd / beforeStd) : worstStd;

    Serial.print(F("Pot A"));
    Serial.print(m);
    Serial.print(F(" (M"));
    Serial.print(m + 1);
    Serial.println(F("):"));

    Serial.print(F("  Before : Pk-Pk=")); Serial.print(beforePk);
    Serial.print(F("  StdDev="));       Serial.print(beforeStd, 2);
    Serial.print(F("  Mean="));         Serial.println(before.mean, 2);

    Serial.print(F("  Motion : Pk-Pk=")); Serial.print(worstPk);
    Serial.print(F("  StdDev="));       Serial.print(worstStd, 2);
    Serial.print(F("  Shift="));        Serial.print(worstShift, 2);
    Serial.print(F("  Ratio="));        Serial.print(noiseRatio, 2);
    Serial.println(F("x"));

    Serial.print(F("  After  : Pk-Pk=")); Serial.print(afterPk);
    Serial.print(F("  StdDev="));       Serial.print(afterStd, 2);
    Serial.print(F("  Shift="));        Serial.print(afterShift, 2);
    Serial.print(F("  DeltaStd="));     Serial.println(afterStd - beforeStd, 2);

    Serial.print(F("  Verdict: "));
    if (worstPk <= 4 && worstStd <= 1.2f && fabsf(worstShift) < 2.0f) {
      Serial.println(F("CLEAN (very low noise)"));
    } else if (worstPk <= 12 && worstStd <= 3.0f) {
      Serial.println(F("MODERATE NOISE (consider twisted/shielded pot wires or 0.1uF cap on wiper)"));
    } else {
      Serial.println(F("HIGH NOISE / EMI (check star ground, separate motor power from pot 5V/GND)"));
    }

    if (fabsf(worstShift) >= 3.0f) {
      Serial.println(F("  NOTE   : DC mean shifted during motor drive -> likely ground bounce / voltage drop!"));
    }
  }

  Serial.println(F("======================================================================"));
  Serial.println(F("Ready. Send 'analyze' (or 'analyze 1/2/3') to run again."));
  Serial.println();
}

void runAnalysis(uint8_t motorMask) {
  Serial.println();
  Serial.println(F("[Analyzer] Starting 5-second pot noise test (send 's' anytime to abort)..."));

  Serial.println(F("  1/5 BEFORE  (1.0s) : Motors OFF (recording baseline)..."));
  if (!recordPhase(PHASE_BEFORE, 0, 0, false)) {
    Serial.println(F("[Analyzer] ABORTED by user. All motors disabled."));
    return;
  }

  Serial.println(F("  2/5 FORWARD (1.0s) : Motors FWD @ PWM 50..."));
  if (!recordPhase(PHASE_FORWARD, motorMask, TEST_PWM, true)) {
    Serial.println(F("[Analyzer] ABORTED by user. All motors disabled."));
    return;
  }

  Serial.println(F("  3/5 STOP    (1.0s) : Motors STOPPED..."));
  if (!recordPhase(PHASE_STOP, 0, 0, true)) {
    Serial.println(F("[Analyzer] ABORTED by user. All motors disabled."));
    return;
  }

  Serial.println(F("  4/5 REVERSE (1.0s) : Motors REV @ PWM 50..."));
  if (!recordPhase(PHASE_REVERSE, motorMask, -(int16_t)TEST_PWM, true)) {
    Serial.println(F("[Analyzer] ABORTED by user. All motors disabled."));
    return;
  }

  Serial.println(F("  5/5 AFTER   (1.0s) : Motors OFF (recording post-move)..."));
  if (!recordPhase(PHASE_AFTER, 0, 0, true)) {
    Serial.println(F("[Analyzer] ABORTED by user. All motors disabled."));
    return;
  }

  stopAllMotors();
  printReport(motorMask);
}

// ===============================================================================================================
// SETUP & LOOP
// ===============================================================================================================

void setup() {
  Serial.begin(SERIAL_BAUD);

  for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
    pinMode(MOTORS[i].rpwmPin, OUTPUT);
    pinMode(MOTORS[i].lpwmPin, OUTPUT);
    pinMode(MOTORS[i].enPin,   OUTPUT);
    disableMotor(i);
  }

  // Match SMC3_IBT2 PWM frequencies & ADC prescaler (1 MHz ADC clock)
  initPWMTimer1(TIMER1_FREQ_KHZ);
  initPWMTimer2(TIMER2_FREQ_KHZ);
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  Serial.println();
  Serial.println(F("=== Pot Noise Analyzer Ready (SMC3_IBT2 Pinout, 115200 baud) ==="));
  Serial.println(F("Commands:"));
  Serial.println(F("  analyze    -> Test all 3 motors (PWM 50: 1s fwd, 1s stop, 1s rev)"));
  Serial.println(F("  analyze 1  -> Test Motor 1 only (still reads all 3 pots)"));
  Serial.println(F("  analyze 2  -> Test Motor 2 only (still reads all 3 pots)"));
  Serial.println(F("  analyze 3  -> Test Motor 3 only (still reads all 3 pots)"));
  Serial.println(F("  stop       -> Immediately disable all motors"));
  Serial.println(F("----------------------------------------------------------------"));
}

void loop() {
  if (Serial.available() == 0) {
    return;
  }

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toLowerCase();

  if (cmd.length() == 0) {
    return;
  }

  if (cmd == F("stop") || cmd == F("s")) {
    stopAllMotors();
    Serial.println(F(">>> All motors disabled."));
  } else if (cmd == F("analyze")) {
    runAnalysis(0b111);  // All 3 motors
  } else if (cmd == F("analyze 1")) {
    runAnalysis(0b001);
  } else if (cmd == F("analyze 2")) {
    runAnalysis(0b010);
  } else if (cmd == F("analyze 3")) {
    runAnalysis(0b100);
  } else {
    Serial.print(F("Unknown command: "));
    Serial.println(cmd);
    Serial.println(F("Use 'analyze', 'analyze 1', 'analyze 2', 'analyze 3', or 'stop'."));
  }
}
