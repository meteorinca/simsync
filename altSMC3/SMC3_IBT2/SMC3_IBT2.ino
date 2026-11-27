//****************************************************************************************************************
// SMC3_IBT2.ino — Clean IBT-2 (BTS7960) 3-Motor PID Motion Controller for Arduino Uno R3
//
// Full SMC3 v0.70 + SimSync protocol compatibility, stripped of all non-IBT-2 legacy code
// (no MODE1, no SoftwareSerial, no A5 pot scaling, no PORTD bit-banging).
//
// Uses true dual-PWM drive per IBT-2 (dedicated RPWM + LPWM + EN per motor):
//   Forward : RPWM = duty, LPWM = 0,    EN = HIGH
//   Reverse : RPWM = 0,    LPWM = duty, EN = HIGH
//   Brake   : RPWM = 0,    LPWM = 0,    EN = HIGH
//   Disabled: RPWM = 0,    LPWM = 0,    EN = LOW
//
// ===============================================================================================================
// PINOUT (Arduino Uno R3 -> 3x IBT-2 / BTS7960 + 3x 10k Potentiometers)
// ===============================================================================================================
//   Motor | RPWM (PWM) | LPWM (PWM) | EN (R_EN+L_EN) | Pot Wiper | Timer  | Notes
//   ------+------------+------------+----------------+-----------+--------+--------------------------------------
//   M1    | D5         | D6         | D4             | A0        | Timer0 | ~976 Hz (keeps millis/micros intact)
//   M2    | D9         | D10        | D8             | A1        | Timer1 | Configurable kHz (default 25 kHz);
//         |            |            |                |           |        | pot software-inverted (1023 - raw)
//   M3    | D3         | D11        | D7             | A2        | Timer2 | 31.25 kHz (>=15kHz) or 3.9 kHz
//
//   Serial: Hardware USB UART at 500,000 baud (D0/D1). Motors always start DISABLED on reset.
//
// ===============================================================================================================
// SERIAL PROTOCOL (5-byte framed packets: '[' + cmd + byte1 + byte2 + ']')
// ===============================================================================================================
//   [Axx],[Bxx],[Cxx]   Target position 0..1023 for M1/M2/M3
//   [Dxx],[Exx],[Fxx]   Kp * 100 for M1/M2/M3
//   [Gxx],[Hxx],[Ixx]   Ki * 100 for M1/M2/M3
//   [Jxx],[Kxx],[Lxx]   Kd * 100 for M1/M2/M3
//   [Mxx],[Nxx],[Oxx]   Ks (derivative sample divider) for M1/M2/M3
//   [Pxy],[Qxy],[Rxy]   x = PWMmin (offset), y = PWMmax for M1/M2/M3
//   [Sxy],[Txy],[Uxy]   x = CutoffLimitMin, y = InputClipMin for M1/M2/M3
//   [Vxy],[Wxy],[Xxy]   x = DeadZone, y = PWMrev for M1/M2/M3
//   [Zx0]               PID loop divider (1..10)
//   [~xy]               x = Timer1 freq kHz (M2), y = Timer2 freq kHz (M3)
//   [rdX]               Read parameter X (A-C, a-c, D-O, P-X, Y, Z, ~, !-#, $-&)
//   [mo1/2/3/0]         Auto-stream telemetry for M1/M2/M3 every 20 ms (mo0 = off)
//   [ena],[en1..3]      Enable all / individual motors (resets IBT-2 fault latch)
//   [stp]               Cancel active jogs and disable all motors immediately
//   [sav]               Save parameters to EEPROM (SMC3-compatible layout)
//   [ver]               Report firmware version (0.70)
//   [cap]               Report SimSync capabilities ([q, 0, 7]: jog + hold + filters)
//   [h,0,mask]          Sample exact ADC and hold-enable selected motors (bitmask 1..7)
//   [4/5/6,min]         Set RAM-only jog minimum bound for M1/M2/M3
//   [7/8/9,max]         Set RAM-only jog maximum bound for M1/M2/M3
//   [j/k/l,pwm,dur/10]  Open-loop timed jog pulse (signed PWM -100..100, 50..500 ms; pwm=0 stops)
//   [!/"/#,Hz*10]       Per-motor feedback LPF cutoff (0..200.0 Hz, RAM-only)
//   [$/%/&,Hz*10]       Per-motor derivative HPF cutoff (0..50.0 Hz, RAM-only)
//****************************************************************************************************************

#include <Arduino.h>
#include <EEPROM.h>

// ===============================================================================================================
// USER HARDWARE CONFIGURATION
// ===============================================================================================================

// Motor 1 (Timer0: D5/D6)
const uint8_t MOTOR1_RPWM_PIN   = 5;
const uint8_t MOTOR1_LPWM_PIN   = 6;
const uint8_t MOTOR1_EN_PIN     = 4;
const uint8_t MOTOR1_POT_PIN    = A0;
const bool    MOTOR1_INVERT_POT = false;
const bool    MOTOR1_INVERT_DIR = false;

// Motor 2 (Timer1: D9/D10) — pot inverted via software due to reverse mounting
const uint8_t MOTOR2_RPWM_PIN   = 9;
const uint8_t MOTOR2_LPWM_PIN   = 10;
const uint8_t MOTOR2_EN_PIN     = 8;
const uint8_t MOTOR2_POT_PIN    = A1;
const bool    MOTOR2_INVERT_POT = true;
const bool    MOTOR2_INVERT_DIR = false;

// Motor 3 (Timer2: D3/D11)
const uint8_t MOTOR3_RPWM_PIN   = 3;
const uint8_t MOTOR3_LPWM_PIN   = 11;
const uint8_t MOTOR3_EN_PIN     = 7;
const uint8_t MOTOR3_POT_PIN    = A2;
const bool    MOTOR3_INVERT_POT = false;
const bool    MOTOR3_INVERT_DIR = false;

// ===============================================================================================================
// CONSTANTS & REGISTER HELPERS
// ===============================================================================================================

#ifndef cbi
  #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
  #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

const uint8_t  NUM_MOTORS          = 3;
const uint32_t SERIAL_BAUD         = 500000;
const uint8_t  START_BYTE          = '[';
const uint8_t  END_BYTE            = ']';
const uint32_t PROCESS_PERIOD_US   = 250;     // ~4 kHz base tick
const uint16_t COMMS_TIMEOUT_TICKS = 60000;   // 15 seconds at 250 us/tick
const uint8_t  EEPROM_MAGIC        = 114;     // Standard SMC3 EEPROM signature
const uint16_t FIRMWARE_VERSION    = 70;      // Reported as v0.70 for SMC3Utils/SimSync
const uint16_t CAPABILITY_FLAGS    = 7;       // bit0: jog, bit1: hold, bit2: filters

// ===============================================================================================================
// HELPER TYPES: JOG PULSE & PID FILTERS
// ===============================================================================================================

struct JogPulse {
  int16_t  pwm      = 0;
  uint16_t minimum  = 512;
  uint16_t maximum  = 512;
  uint32_t deadline = 0;
  bool     running  = false;

  bool start(int16_t drive, uint16_t durationMs, int feedback, int ceiling, uint32_t nowMs) {
    if (running || drive == 0 || drive < -100 || drive > 100 ||
        drive > ceiling || -drive > ceiling || durationMs < 50 || durationMs > 500 ||
        minimum >= maximum || maximum > 1023 ||
        feedback <= (int)minimum + 4 || feedback >= (int)maximum - 4) {
      return false;
    }
    pwm      = drive;
    deadline = nowMs + durationMs;
    running  = true;
    return true;
  }

  bool update(int feedback, bool enabled, uint32_t nowMs) {
    if (running && (!enabled || feedback <= (int)minimum || feedback >= (int)maximum ||
                    (int32_t)(nowMs - deadline) >= 0)) {
      running = false;
    }
    return running;
  }
};

struct PIDFilters {
  uint16_t lpfTenths       = 0;
  uint16_t hpfTenths       = 0;
  float    position        = 0.0f;
  float    previousDelta   = 0.0f;
  float    highPassedDelta = 0.0f;
  float    lpAlpha         = 1.0f;
  float    hpAlpha         = 1.0f;
  float    lpDt            = -1.0f;
  float    hpDt            = -1.0f;
  bool     initialized     = false;

  void setLPF(uint16_t tenthsHz) {
    lpfTenths   = tenthsHz;
    lpDt        = -1.0f;
    initialized = false;
  }

  void setHPF(uint16_t tenthsHz) {
    hpfTenths       = tenthsHz;
    hpDt            = -1.0f;
    previousDelta   = 0.0f;
    highPassedDelta = 0.0f;
  }

  float feedback(float raw, float dt, bool disabled) {
    if (!initialized || disabled) {
      position        = raw;
      initialized     = true;
      previousDelta   = 0.0f;
      highPassedDelta = 0.0f;
      return raw;
    }
    if (lpfTenths == 0) {
      position = raw;
      return raw;
    }
    if (lpDt != dt) {
      float wdt = 0.62831853f * lpfTenths * dt;
      lpAlpha   = wdt / (1.0f + wdt);
      lpDt      = dt;
    }
    position += lpAlpha * (raw - position);
    return position;
  }

  float derivative(float delta, float dt) {
    if (hpfTenths == 0) {
      previousDelta = delta;
      return delta;
    }
    if (hpDt != dt) {
      hpAlpha = 1.0f / (1.0f + 0.62831853f * hpfTenths * dt);
      hpDt    = dt;
    }
    highPassedDelta = hpAlpha * (highPassedDelta + delta - previousDelta);
    previousDelta   = delta;
    return highPassedDelta;
  }
};

// ===============================================================================================================
// MOTOR CHANNEL STATE
// ===============================================================================================================

struct MotorChannel {
  // Hardware mapping
  uint8_t rpwmPin;
  uint8_t lpwmPin;
  uint8_t enPin;
  uint8_t potPin;
  bool    invertPot;
  bool    invertDir;

  // Live positions & output
  int  feedback;
  int  target;
  int  pwmOut;
  bool disabled;

  // PID tuning
  long kp_x100;
  long ki_x100;
  long kd_x100;
  int  ks;

  // Limits & PWM shaping
  int  pwmOffset;       // PWMmin
  int  pwmMax;
  int  pwmRev;
  int  deadZone;
  int  cutoffMin;
  int  cutoffMax;
  int  inputClipMin;
  int  inputClipMax;

  // Internal PID integrator / derivative state
  float cumError;
  float lastPosition;
  long  dTerm_x100;
  int   kdFilterCount;
  float derivativeDt;

  // Filters & open-loop jog
  PIDFilters filters;
  JogPulse   jog;
};

MotorChannel motors[NUM_MOTORS] = {
  // Motor 1
  {
    MOTOR1_RPWM_PIN, MOTOR1_LPWM_PIN, MOTOR1_EN_PIN, MOTOR1_POT_PIN,
    MOTOR1_INVERT_POT, MOTOR1_INVERT_DIR,
    512, 512, 0, true,
    100, 0, 40, 1,
    50, 100, 50, 0,
    23, 1000, 100, 923,
    0.0f, 512.0f, 0, 0, 0.0f,
    PIDFilters(), JogPulse()
  },
  // Motor 2
  {
    MOTOR2_RPWM_PIN, MOTOR2_LPWM_PIN, MOTOR2_EN_PIN, MOTOR2_POT_PIN,
    MOTOR2_INVERT_POT, MOTOR2_INVERT_DIR,
    512, 512, 0, true,
    420, 0, 40, 1,
    50, 100, 50, 0,
    23, 1000, 100, 923,
    0.0f, 512.0f, 0, 0, 0.0f,
    PIDFilters(), JogPulse()
  },
  // Motor 3
  {
    MOTOR3_RPWM_PIN, MOTOR3_LPWM_PIN, MOTOR3_EN_PIN, MOTOR3_POT_PIN,
    MOTOR3_INVERT_POT, MOTOR3_INVERT_DIR,
    512, 512, 0, true,
    420, 0, 40, 1,
    50, 100, 50, 0,
    23, 1000, 100, 923,
    0.0f, 512.0f, 0, 0, 0.0f,
    PIDFilters(), JogPulse()
  }
};

// ===============================================================================================================
// GLOBAL CONTROLLER STATE
// ===============================================================================================================

uint8_t  pidProcessDivider     = 1;
uint8_t  pidProcessCounter     = 0;
uint8_t  serialFeedbackCounter = 0;
uint8_t  serialFeedbackMotor   = 0;     // 0 = off, 1..3 = stream motor 1..3 every 20 ms
uint8_t  timer1FreqkHz         = 8;     // Motor 2 (D9/D10) PWM frequency in kHz (8 kHz gives reliable BTS7960 low-PWM drive)
uint8_t  timer2FreqkHz         = 31;    // Motor 3 (D3/D11) PWM frequency (>=15 -> 31.25 kHz, else 3.9 kHz)

uint32_t loopCount             = 0;
uint32_t lastLoopCount         = 0;
uint16_t commsTimeout          = 0;
uint8_t  powerScale            = 7;     // 7 = normal (/128), 9 = low power after 15 s comms timeout
float    pidFilterDt           = 0.00025f;

// Serial frame parser (5-byte packet: '[' cmd b1 b2 ']')
int8_t   rxIndex               = -1;
uint8_t  rxBuf[4]              = {0};

// ===============================================================================================================
// HARDWARE PWM TIMERS (Timer1 -> M2 D9/D10, Timer2 -> M3 D3/D11; Timer0 -> M1 D5/D6 stays default)
// ===============================================================================================================

void initPWMTimer1(uint8_t freqkHz) {
  uint8_t clamped = constrain(freqkHz, 1, 25);  // BTS7960 max rated PWM is 25 kHz
  // ATmega328P §16.11.7: ICR1 is read-only unless WGM13:0 already selects an ICR1-TOP mode.
  // Enter Mode 8 (PWM Phase & Frequency Correct, TOP = ICR1) with clock stopped first, then set ICR1.
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
  uint8_t prescale = (freqkHz >= 15) ? 0x01 : 0x02;  // 0x01 = 31.25 kHz, 0x02 = 3.9 kHz
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

// ===============================================================================================================
// IBT-2 MOTOR DRIVE & FEEDBACK HELPERS
// ===============================================================================================================

int readFeedback(uint8_t idx) {
  int raw = analogRead(motors[idx].potPin);
  return motors[idx].invertPot ? (1023 - raw) : raw;
}

void driveIBT2(uint8_t idx, int signedDuty) {
  MotorChannel &m = motors[idx];
  int drive = constrain(signedDuty, -255, 255);
  if (m.invertDir) {
    drive = -drive;
  }

  if (drive > 0) {
    writeMotorPinPWM(m.lpwmPin, 0);
    writeMotorPinPWM(m.rpwmPin, (uint8_t)drive);
  } else if (drive < 0) {
    writeMotorPinPWM(m.rpwmPin, 0);
    writeMotorPinPWM(m.lpwmPin, (uint8_t)(-drive));
  } else {
    writeMotorPinPWM(m.rpwmPin, 0);
    writeMotorPinPWM(m.lpwmPin, 0);
  }
}

void disableMotor(uint8_t idx) {
  MotorChannel &m = motors[idx];
  m.disabled = true;
  m.pwmOut   = 0;
  writeMotorPinPWM(m.rpwmPin, 0);
  writeMotorPinPWM(m.lpwmPin, 0);
  digitalWrite(m.enPin, LOW);
}

void disableAllMotors() {
  for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
    disableMotor(i);
  }
}

void enableMotor(uint8_t idx) {
  MotorChannel &m = motors[idx];
  writeMotorPinPWM(m.rpwmPin, 0);
  writeMotorPinPWM(m.lpwmPin, 0);
  // Toggle EN LOW -> HIGH to clear any latched BTS7960 overtemp/fault shutdown
  digitalWrite(m.enPin, LOW);
  digitalWrite(m.enPin, HIGH);
  m.disabled = false;
}

uint8_t disabledMask() {
  return (motors[0].disabled ? 1 : 0) |
         (motors[1].disabled ? 2 : 0) |
         (motors[2].disabled ? 4 : 0);
}

// ===============================================================================================================
// EEPROM STORAGE (Standard SMC3 layout — compatible with SMC3Utils & SimSync)
// ===============================================================================================================

void writeEEPROMWord(int addr, int val) {
  EEPROM.write(addr,     (uint8_t)((val >> 8) & 0xFF));
  EEPROM.write(addr + 1, (uint8_t)(val & 0xFF));
}

int readEEPROMWord(int addr) {
  int high = EEPROM.read(addr);
  int low  = EEPROM.read(addr + 1);
  return (high << 8) | low;
}

void writeEEPROM() {
  EEPROM.write(0, EEPROM_MAGIC);

  EEPROM.write(1,  motors[0].cutoffMin);
  EEPROM.write(2,  motors[0].inputClipMin);
  EEPROM.write(5,  motors[0].deadZone);

  EEPROM.write(6,  motors[1].cutoffMin);
  EEPROM.write(7,  motors[1].inputClipMin);
  EEPROM.write(10, motors[1].deadZone);

  writeEEPROMWord(11, (int)motors[0].kp_x100);
  writeEEPROMWord(13, (int)motors[0].ki_x100);
  writeEEPROMWord(15, (int)motors[0].kd_x100);
  writeEEPROMWord(17, (int)motors[1].kp_x100);
  writeEEPROMWord(19, (int)motors[1].ki_x100);
  writeEEPROMWord(21, (int)motors[1].kd_x100);

  EEPROM.write(23, motors[0].pwmOffset);
  EEPROM.write(24, motors[1].pwmOffset);
  EEPROM.write(25, motors[0].pwmMax);
  EEPROM.write(26, motors[1].pwmMax);
  EEPROM.write(27, motors[0].pwmRev);
  EEPROM.write(28, motors[1].pwmRev);
  EEPROM.write(29, motors[2].pwmRev);

  EEPROM.write(31, motors[2].cutoffMin);
  EEPROM.write(32, motors[2].inputClipMin);
  EEPROM.write(35, motors[2].deadZone);

  writeEEPROMWord(36, (int)motors[2].kp_x100);
  writeEEPROMWord(38, (int)motors[2].ki_x100);
  writeEEPROMWord(40, (int)motors[2].kd_x100);
  EEPROM.write(42, motors[2].pwmOffset);
  EEPROM.write(43, motors[2].pwmMax);

  writeEEPROMWord(44, motors[0].ks);
  writeEEPROMWord(46, motors[1].ks);
  writeEEPROMWord(48, motors[2].ks);

  EEPROM.write(50, constrain(pidProcessDivider, 1, 10));
  EEPROM.write(51, timer1FreqkHz);
  EEPROM.write(52, timer2FreqkHz);
}

void readEEPROM() {
  if (EEPROM.read(0) != EEPROM_MAGIC) {
    writeEEPROM();
    return;
  }

  motors[0].cutoffMin    = EEPROM.read(1);
  motors[0].inputClipMin = EEPROM.read(2);
  motors[0].cutoffMax    = 1023 - motors[0].cutoffMin;
  motors[0].inputClipMax = 1023 - motors[0].inputClipMin;
  motors[0].deadZone     = EEPROM.read(5);

  motors[1].cutoffMin    = EEPROM.read(6);
  motors[1].inputClipMin = EEPROM.read(7);
  motors[1].cutoffMax    = 1023 - motors[1].cutoffMin;
  motors[1].inputClipMax = 1023 - motors[1].inputClipMin;
  motors[1].deadZone     = EEPROM.read(10);

  motors[0].kp_x100      = readEEPROMWord(11);
  motors[0].ki_x100      = readEEPROMWord(13);
  motors[0].kd_x100      = readEEPROMWord(15);
  motors[1].kp_x100      = readEEPROMWord(17);
  motors[1].ki_x100      = readEEPROMWord(19);
  motors[1].kd_x100      = readEEPROMWord(21);

  motors[0].pwmOffset    = EEPROM.read(23);
  motors[1].pwmOffset    = EEPROM.read(24);
  motors[0].pwmMax       = EEPROM.read(25);
  motors[1].pwmMax       = EEPROM.read(26);
  motors[0].pwmRev       = EEPROM.read(27);
  motors[1].pwmRev       = EEPROM.read(28);
  motors[2].pwmRev       = EEPROM.read(29);

  motors[2].cutoffMin    = EEPROM.read(31);
  motors[2].inputClipMin = EEPROM.read(32);
  motors[2].cutoffMax    = 1023 - motors[2].cutoffMin;
  motors[2].inputClipMax = 1023 - motors[2].inputClipMin;
  motors[2].deadZone     = EEPROM.read(35);

  motors[2].kp_x100      = readEEPROMWord(36);
  motors[2].ki_x100      = readEEPROMWord(38);
  motors[2].kd_x100      = readEEPROMWord(40);
  motors[2].pwmOffset    = EEPROM.read(42);
  motors[2].pwmMax       = EEPROM.read(43);

  int ks0 = readEEPROMWord(44);
  int ks1 = readEEPROMWord(46);
  int ks2 = readEEPROMWord(48);
  motors[0].ks           = (ks0 < 1) ? 1 : ks0;
  motors[1].ks           = (ks1 < 1) ? 1 : ks1;
  motors[2].ks           = (ks2 < 1) ? 1 : ks2;

  uint8_t savedDivider   = EEPROM.read(50);
  pidProcessDivider      = constrain(savedDivider, 1, 10);
  uint8_t savedT1        = EEPROM.read(51);
  timer1FreqkHz          = (savedT1 >= 5 && savedT1 <= 35) ? savedT1 : 8;
  uint8_t savedT2        = EEPROM.read(52);
  timer2FreqkHz          = (savedT2 == 4 || savedT2 == 31) ? savedT2 : 31;
}

// ===============================================================================================================
// PID, HOLD, AND TIMED JOG LOGIC
// ===============================================================================================================

int calcMotorPID(uint8_t idx) {
  MotorChannel &m = motors[idx];
  float position  = m.filters.feedback((float)m.feedback, pidFilterDt, m.disabled);
  float error     = (float)m.target - position;

  if (m.disabled) {
    m.cumError      = 0.0f;
    m.lastPosition  = position;
    m.dTerm_x100    = 0;
    m.kdFilterCount = 0;
    m.derivativeDt  = 0.0f;
    return 0;
  }

  if (fabs(error) <= m.deadZone) {
    m.cumError = 0.0f;
  } else {
    m.cumError = constrain(m.cumError + error, -1024.0f, 1024.0f);
  }

  long pTerm_x100 = (long)(m.kp_x100 * error);
  long iTerm_x100 = (long)(m.ki_x100 * m.cumError);

  m.kdFilterCount++;
  m.derivativeDt += pidFilterDt;
  if (m.kdFilterCount >= m.ks) {
    float deltaPosition = position - m.lastPosition;
    m.lastPosition      = position;
    m.dTerm_x100        = (long)(m.kd_x100 * m.filters.derivative(deltaPosition, m.derivativeDt));
    m.kdFilterCount     = 0;
    m.derivativeDt      = 0.0f;
  }

  int rawOut = (int)((pTerm_x100 + iTerm_x100 - m.dTerm_x100) >> powerScale);
  return constrain(rawOut, -255, 255);
}

void applyMotorOutput(uint8_t idx, int pidOutput) {
  MotorChannel &m = motors[idx];
  int drive = 0;

  if (m.feedback > m.inputClipMax && m.pwmRev != 0) {
    drive = -m.pwmRev;
  } else if (m.feedback < m.inputClipMin && m.pwmRev != 0) {
    drive = m.pwmRev;
  } else if (abs(m.target - m.feedback) <= m.deadZone) {
    drive = 0;
  } else {
    int magnitude = min(abs(pidOutput) + m.pwmOffset, m.pwmMax);
    drive = (pidOutput >= 0) ? magnitude : -magnitude;
  }

  driveIBT2(idx, drive);
  m.pwmOut = abs(drive);
}

void endJog(uint8_t idx) {
  MotorChannel &m = motors[idx];
  m.jog.running = false;
  m.target      = m.feedback;
  disableMotor(idx);
}

void startJog(uint8_t idx, int drive, uint16_t durationMs) {
  if (drive == 0) {
    if (motors[idx].jog.running) endJog(idx);
    return;
  }
  // Only one open-loop jog may run, and all motors must currently be disabled
  for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
    if (motors[i].jog.running || !motors[i].disabled) return;
  }

  MotorChannel &m = motors[idx];
  int fb = readFeedback(idx);
  if (m.jog.minimum <= (uint16_t)m.cutoffMin || m.jog.maximum >= (uint16_t)(1023 - m.cutoffMin)) {
    return;
  }
  if (!m.jog.start(drive, durationMs, fb, m.pwmMax, millis())) {
    return;
  }
  enableMotor(idx);
}

bool serviceJog(uint8_t idx) {
  MotorChannel &m = motors[idx];
  if (!m.jog.running) return false;

  if (!m.jog.update(m.feedback, !m.disabled, millis())) {
    endJog(idx);
    return true;
  }
  driveIBT2(idx, m.jog.pwm);
  m.pwmOut = abs(m.jog.pwm);
  return true;
}

void holdMotors(uint8_t mask) {
  if (mask == 0 || (mask & ~0x07)) return;
  for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
    if (motors[i].jog.running) return;
  }

  int fb[NUM_MOTORS];
  for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
    fb[i] = readFeedback(i);
    if (mask & (1 << i)) {
      int clip   = motors[i].inputClipMin;
      int cutoff = motors[i].cutoffMin;
      if (fb[i] < clip + 4 || fb[i] > 1019 - clip || fb[i] <= cutoff || fb[i] >= 1023 - cutoff) {
        disableAllMotors();
        return;
      }
    }
  }

  for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
    if (mask & (1 << i)) {
      disableMotor(i);
      motors[i].feedback = fb[i];
      motors[i].target   = fb[i];
      calcMotorPID(i);
      motors[i].pwmOut   = 0;
    }
  }
  for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
    if (mask & (1 << i)) {
      enableMotor(i);
    }
  }
}

// ===============================================================================================================
// SERIAL PROTOCOL (5-BYTE FRAMED SMC3 + SIMSYNC COMMANDS)
// ===============================================================================================================

void sendTwoValues(uint8_t id, uint8_t v1, uint8_t v2) {
  uint8_t frame[5] = { START_BYTE, id, v1, v2, END_BYTE };
  Serial.write(frame, 5);
}

void sendWord(uint8_t id, uint16_t value) {
  sendTwoValues(id, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF));
}

uint16_t deltaLoopCount() {
  uint32_t delta = 0;
  if (lastLoopCount != 0 && (loopCount - lastLoopCount) <= 32000UL) {
    delta = loopCount - lastLoopCount;
  }
  lastLoopCount = loopCount;
  return (uint16_t)delta;
}

void handleReadCommand(uint8_t code) {
  uint8_t statusByte = (pidProcessDivider << 4) | disabledMask();

  switch (code) {
    case '!': case '"': case '#':
      sendWord(code, motors[code - '!'].filters.lpfTenths);
      break;
    case '$': case '%': case '&':
      sendWord(code, motors[code - '$'].filters.hpfTenths);
      break;
    case 'A': case 'B': case 'C': {
      uint8_t i = code - 'A';
      sendTwoValues(code, motors[i].feedback >> 2, motors[i].target >> 2);
      break;
    }
    case 'a': case 'b': case 'c': {
      uint8_t i = code - 'a';
      sendTwoValues(code, statusByte, constrain(motors[i].pwmOut, 0, 255));
      break;
    }
    case 'D': case 'E': case 'F':
      sendWord(code, (uint16_t)motors[code - 'D'].kp_x100);
      break;
    case 'G': case 'H': case 'I':
      sendWord(code, (uint16_t)motors[code - 'G'].ki_x100);
      break;
    case 'J': case 'K': case 'L':
      sendWord(code, (uint16_t)motors[code - 'J'].kd_x100);
      break;
    case 'M': case 'N': case 'O':
      sendWord(code, (uint16_t)motors[code - 'M'].ks);
      break;
    case 'P': case 'Q': case 'R': {
      uint8_t i = code - 'P';
      sendTwoValues(code, motors[i].pwmOffset, motors[i].pwmMax);
      break;
    }
    case 'S': case 'T': case 'U': {
      uint8_t i = code - 'S';
      sendTwoValues(code, motors[i].cutoffMin, motors[i].inputClipMin);
      break;
    }
    case 'V': case 'W': case 'X': {
      uint8_t i = code - 'V';
      sendTwoValues(code, motors[i].deadZone, motors[i].pwmRev);
      break;
    }
    case 'Y':
      sendTwoValues('Y', statusByte, 0);
      break;
    case 'Z':
      sendWord('Z', deltaLoopCount());
      break;
    case '~':
      sendTwoValues('~', timer1FreqkHz, timer2FreqkHz);
      break;
    default:
      break;
  }
}

void parseCommand() {
  commsTimeout = 0;

  uint8_t cmd = rxBuf[0];
  uint8_t b1  = rxBuf[1];
  uint8_t b2  = rxBuf[2];
  uint16_t wordVal = ((uint16_t)b1 << 8) | b2;

  switch (cmd) {
    case 'A': case 'B': case 'C': {
      uint8_t i = cmd - 'A';
      motors[i].target = constrain((int)wordVal, motors[i].inputClipMin, motors[i].inputClipMax);
      break;
    }
    case 'D': case 'E': case 'F':
      motors[cmd - 'D'].kp_x100 = wordVal;
      break;
    case 'G': case 'H': case 'I':
      motors[cmd - 'G'].ki_x100 = wordVal;
      break;
    case 'J': case 'K': case 'L':
      motors[cmd - 'J'].kd_x100 = wordVal;
      break;
    case 'M': case 'N': case 'O':
      motors[cmd - 'M'].ks = max(1, (int)wordVal);
      break;
    case 'P': case 'Q': case 'R': {
      uint8_t i = cmd - 'P';
      motors[i].pwmOffset = b1;
      motors[i].pwmMax    = b2;
      break;
    }
    case 'S': case 'T': case 'U': {
      uint8_t i = cmd - 'S';
      motors[i].cutoffMin    = b1;
      motors[i].cutoffMax    = 1023 - b1;
      motors[i].inputClipMin = b2;
      motors[i].inputClipMax = 1023 - b2;
      break;
    }
    case 'V': case 'W': case 'X': {
      uint8_t i = cmd - 'V';
      motors[i].deadZone = b1;
      motors[i].pwmRev   = b2;
      break;
    }
    case 'Z':
      pidProcessDivider = constrain(b1, 1, 10);
      break;
    case '~':
      timer1FreqkHz = b1;
      timer2FreqkHz = b2;
      initPWMTimer1(timer1FreqkHz);
      initPWMTimer2(timer2FreqkHz);
      break;
    case '!': case '"': case '#': {
      uint8_t i = cmd - '!';
      if (motors[i].disabled && !motors[i].jog.running && wordVal <= 2000) {
        motors[i].filters.setLPF(wordVal);
      }
      break;
    }
    case '$': case '%': case '&': {
      uint8_t i = cmd - '$';
      if (motors[i].disabled && !motors[i].jog.running && wordVal <= 500) {
        motors[i].filters.setHPF(wordVal);
      }
      break;
    }
    case '4': case '5': case '6': {
      uint8_t i = cmd - '4';
      if (!motors[i].jog.running) motors[i].jog.minimum = wordVal;
      break;
    }
    case '7': case '8': case '9': {
      uint8_t i = cmd - '7';
      if (!motors[i].jog.running) motors[i].jog.maximum = wordVal;
      break;
    }
    case 'j': case 'k': case 'l':
      startJog(cmd - 'j', (int8_t)b1, (uint16_t)b2 * 10U);
      break;
    case 'h':
      if (b1 == 0) holdMotors(b2);
      break;
    case 'c':
      if (b1 == 'a' && b2 == 'p') sendWord('q', CAPABILITY_FLAGS);
      break;
    case 'v':
      if (b1 == 'e' && b2 == 'r') sendWord('v', FIRMWARE_VERSION);
      break;
    case 'r':
      if (b1 == 'd') handleReadCommand(b2);
      break;
    case 'm':
      if (b1 == 'o' && b2 >= '0' && b2 <= '3') {
        serialFeedbackMotor = b2 - '0';
      }
      break;
    case 's':
      if (b1 == 't' && b2 == 'p') {
        for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
          if (motors[i].jog.running) endJog(i);
        }
        disableAllMotors();
      } else if (b1 == 'a' && b2 == 'v') {
        writeEEPROM();
      }
      break;
    case 'e':
      if (b1 == 'n') {
        if (b2 == 'a') {
          for (uint8_t i = 0; i < NUM_MOTORS; ++i) enableMotor(i);
        } else if (b2 >= '1' && b2 <= '3') {
          enableMotor(b2 - '1');
        }
      }
      break;
    default:
      break;
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();
    if (rxIndex < 0) {
      if (b == START_BYTE) {
        rxIndex = 0;
      }
    } else {
      rxBuf[rxIndex++] = b;
      if (rxIndex == 4) {
        if (rxBuf[3] == END_BYTE) {
          parseCommand();
        }
        rxIndex = -1;
      }
    }
  }
}

// ===============================================================================================================
// SETUP & MAIN CONTROL LOOP
// ===============================================================================================================

void setup() {
  Serial.begin(SERIAL_BAUD);

  for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
    pinMode(motors[i].rpwmPin, OUTPUT);
    pinMode(motors[i].lpwmPin, OUTPUT);
    pinMode(motors[i].enPin,   OUTPUT);
    disableMotor(i);
  }

  readEEPROM();

  // Configure high-frequency PWM on Timer1 (M2: D9/D10) and Timer2 (M3: D3/D11).
  // Timer0 (M1: D5/D6) stays at default ~976 Hz so millis() and micros() remain accurate.
  initPWMTimer1(timer1FreqkHz);
  initPWMTimer2(timer2FreqkHz);

  // Set ADC prescaler to 16 (1 MHz ADC clock -> ~13 us per analogRead)
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  // Initialize feedback to current pot positions, keeping motors disabled
  for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
    motors[i].feedback = readFeedback(i);
    motors[i].target   = motors[i].feedback;
  }
}

void loop() {
  static uint32_t nextTickUs        = micros();
  static uint32_t lastPIDFilterUs   = nextTickUs;

  // Wait for next 250 us tick (rollover-safe; resync after long ops like EEPROM write)
  while ((uint32_t)(micros() - nextTickUs) < PROCESS_PERIOD_US) {
    pollSerial();
  }
  uint32_t tickNowUs = micros();
  if ((uint32_t)(tickNowUs - nextTickUs) > 5000UL) {
    nextTickUs = tickNowUs;
  } else {
    nextTickUs += PROCESS_PERIOD_US;
  }

  pidProcessCounter++;
  if (pidProcessCounter >= pidProcessDivider) {
    pidProcessCounter = 0;

    uint32_t nowUs  = micros();
    pidFilterDt     = (float)(uint32_t)(nowUs - lastPIDFilterUs) * 0.000001f;
    lastPIDFilterUs = nowUs;

    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
      MotorChannel &m = motors[i];
      m.feedback = readFeedback(i);

      if (m.feedback > m.cutoffMax || m.feedback < m.cutoffMin) {
        if (m.jog.running) endJog(i);
        else disableMotor(i);
        continue;
      }

      if (!serviceJog(i)) {
        int pidOut = calcMotorPID(i);
        if (!m.disabled) {
          applyMotorOutput(i, pidOut);
        } else {
          m.pwmOut = 0;
        }
      }
    }

    loopCount++;
  }

  pollSerial();

  // Optional 20 ms auto-monitor stream ([mo1]/[mo2]/[mo3])
  serialFeedbackCounter++;
  if (serialFeedbackCounter >= 80) {
    serialFeedbackCounter = 0;
    if (serialFeedbackMotor >= 1 && serialFeedbackMotor <= NUM_MOTORS) {
      uint8_t i          = serialFeedbackMotor - 1;
      uint8_t statusByte = (pidProcessDivider << 4) | disabledMask();
      sendTwoValues('A' + i, motors[i].feedback >> 2, motors[i].target >> 2);
      sendTwoValues('a' + i, statusByte, constrain(motors[i].pwmOut, 0, 255));
    }
  }

  // Comms watchdog: drop PID power after 15 s of silence
  if (commsTimeout < COMMS_TIMEOUT_TICKS) {
    commsTimeout++;
    powerScale = 7;
  } else {
    powerScale = 9;
  }
}
