#  Complete Architecture & Component Guide: MotionSim (SMC3), SimSync & RaceSync

This document outlines the simplified, rock-solid hardware and software ecosystem for the 3DOF motion simulator rig, now standardized on the **Arduino UNO R3 running SMC3 (Mode 2)**.

---

## 1. Updated Architecture Overview

```mermaid
flowchart TD
    GAME[" DiRT Rally 2.0 / Racing Sim<br/>(Outputs UDP Telemetry on Port 20777)"]

    subgraph PC_LAYER [" PC Software Layer"]
        RACESYNC[" RaceSync (Web Dashboard)<br/>• Big-screen track progress & GPS map<br/>• Pseudo-multiplayer / spectator view<br/>• Live telemetry recording & replay"]
        SIMSYNC[" SimSync (Go Motion Engine) / SMC3Utils<br/>• Replaces SimTools (zero bloat, single binary)<br/>• 3DOF kinematics: G-forces ➔ Pitch/Roll/Heave<br/>• High-speed USB Serial Bridge (500k baud)<br/>• Formats setpoints [Axx][Bxx][Cxx] to Arduino"]
    end

    subgraph HW_CONTROLLER [" Hardware Controller (Arduino UNO R3)"]
        SMC3[" Arduino UNO R3 (SMC3 Firmware - Mode 2)<br/>• 4 kHz instantaneous PID loop<br/>• Clean ±1 count 10-bit AVR ADC sampling<br/>• Direct hardware PWM & direction control<br/>• Battle-tested reverse braking & clipping safety"]
    end

    subgraph RIG [" Physical Motion Rig Hardware"]
        DRIVERS[" 3x IBT-2 Motor Drivers (BTS7960 43A)"]
        MOTORS[" 3x 24V DC Actuator Motors (Roll, Pitch, Heave)"]
        POTS[" 3x 10k Feedback Potentiometers (0-5V)"]
    end

    %% Telemetry Stream
    GAME -->|UDP 20777 Broadcast| RACESYNC
    GAME -->|UDP 20777 Broadcast| SIMSYNC

    %% PC to Hardware
    SIMSYNC -->|USB Serial @ 500000 baud| SMC3

    %% Hardware Control Loop
    POTS -->|Analog 0-5V Feedback (A0, A1, A2)| SMC3
    SMC3 -->|PWM + DIR + EN (Pins 2-11)| DRIVERS
    DRIVERS -->|High Current 24V Drive| MOTORS
    MOTORS -.->|Physical Coupling| POTS
```

---

## 2. Why We Simplified to Arduino UNO (SMC3)

1. **Eliminated ADC Noise:** The ESP32-S3 internal SAR ADC had ~26 counts of peak-to-peak jitter due to RF/PWM switching noise. The Arduino UNO's AVR ADC delivers **±1 count precision**, completely eliminating motor buzzing and position chatter.
2. **Zero Inter-Chip Lag:** Both the ADC position reading and the PWM motor control happen inside the same ATmega328P chip at **4,000 Hz**. No UART bridges, level-shifter resistors, or inter-MCU framing latency.
3. **Proven Reliability:** SMC3 is the motion sim community's gold standard firmware for 3DOF rigs. It includes built-in reverse current braking, software travel limits, stall protection, and deadzone filtering out-of-the-box.
4. **Retired ESP32-S3 code:** `motionsimbot` and `motionsimbench` have been completely removed from the workspace in favor of [`SMC3.ino`](SMC3.ino).

---

## 3. Complete Pinout & Wiring: Arduino UNO to 3x IBT-2 Drivers

In [`SMC3.ino`](SMC3.ino), **`MODE2`** is enabled for the BTS7960 / IBT-2 43A H-Bridges.

### Wiring Table

See [pinguide.md](pinguide.md) for the checked Mode 2 wiring and upload checklist.

| Signal | Motor 1 (Front L) | Motor 2 (Front R) | Motor 3 (Rear) | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **RPWM (Direction / IN1)** | **Pin 2** | **Pin 4** | **Pin 6** | Mode 2 direction input |
| **LPWM (Speed / PWM / IN2)** | **Pin 9** | **Pin 10** | **Pin 11** | Mode 2 hardware PWM input |
| **R_EN & L_EN (Driver Enable)** | **Pin 3** | **Pin 5** | **Pin 7** | Tie IBT-2 `R_EN` and `L_EN` together |
| **Feedback Pot (Wiper)** | **A0** | **A1** | **A2** | Connect to center pin of 10k pot |
| **Motion Scaler (Optional)** | — | — | **A5** | Optional global scale potentiometer |

### Logic Power & Common Grounds
* **IBT-2 `VCC`:** Connect all 3 IBT-2 `VCC` pins to Arduino **5V**.
* **IBT-2 `GND`:** Connect all 3 IBT-2 `GND` pins to Arduino **GND**.
* **Potentiometer Rails:** Connect pot outer pins to Arduino **5V** and **GND**.
* **Motor Power Supply:** Ensure 24V Power Supply **Negative (- / 0V)** connects to Arduino **GND** for a clean common ground.

---

## 4. Component Roles in the Full Stack

| Component | Location | Role |
| :--- | :--- | :--- |
| **SMC3 Firmware** | [`SMC3.ino`](SMC3.ino) | Runs on Arduino UNO. Manages 4 kHz closed-loop PID, reads pots (A0-A2), drives 3x IBT-2s in Mode 2. Listens at 500,000 baud. |
| **SimSync** | `../simsync` | Go-native PC bridge. Reads game UDP 20777, filters and mixes pitch/roll/heave into three actuator targets, and streams `[Axx][Bxx][Cxx]` commands over USB serial. |
| **RaceSync** | `../racesync` | Dedicated spectator / side-monitor visualizer for DiRT Rally 2.0 (track map, stage progress, and live UDP session recording/playback). |

---

## 5. Next Steps

1. **Flash Arduino UNO:** Open [`SMC3.ino`](SMC3.ino) in Arduino IDE and upload to your Uno board (Board: *Arduino Uno*, Port: *COMx*).
2. **First Power-Up Check:** Use `SMC3Utils` on Windows or `simsync` in bench mode to verify pot tracking before engaging 24V motor power.

---

## 6. SimSync and firmware update (October 2, 2026)

### Wiring and upload reference

[pinguide.md](pinguide.md) is the quick wiring reference for this rig. It includes driver and pot connections, common grounding, motor power, upload steps and first bench checks.

The Mode 2 table above was corrected: RPWM connects to D2/D4/D6; LPWM connects to D9/D10/D11. Each driver's R_EN and L_EN are tied together to D3/D5/D7 respectively. This matches the supplied sketch and [SMC3 author's Mode 2 wiring clarification](https://www.xsimulator.net/community/threads/smc3-arduino-3dof-motor-driver-and-windows-utilities.4957/page-10).

### Firmware housekeeping

- MODE2 remains enabled by default for the Uno and IBT-2 drivers.
- A compile-time guard requires exactly one of MODE1 and MODE2.
- Motor 2 now checks its own CutoffLimitMin2 rather than Motor 1's lower cutoff.
- Motor 3 now checks its own CutoffLimitMin3 rather than Motor 1's lower cutoff.
- The sketch header documents the Mode 2 pin map, 500,000 baud and 0-1023 target range.
- PID defaults, PWM settings, EEPROM layout, serial protocol and startup enabling remain unchanged.

The sketch has been checked from source. Arduino IDE Verify, upload and powered rig testing remain outstanding. Uploading normally preserves EEPROM settings, so inspect actual PID, PWM and travel settings in SMC3Utils.

### Go backend and dashboard

SimSync now uses SMC3's native five-byte binary commands at 500,000 baud instead of the custom ESP32 protocol and HTTP fallback. It polls feedback for all three motors, sends direct targets and live PID gains, and saves controller settings to EEPROM. The dashboard and CSV use ADC counts, with nominal center 512 and four-count feedback resolution. PID display reads Motor 1; Send Live applies gains to all three motors.

The starting motion mix uses front-left, front-right and rear actuators. It is a filtered pitch/roll/heave mix, without washout or geometry calibration. Direct commands enter manual mode to prevent game telemetry from overwriting targets. Resume returns to game output. Use bench mode before game motion:

```powershell
.\simsync-updated.exe --no-udp
# Explicit port if automatic selection is ambiguous:
.\simsync-updated.exe --serial COM9 --no-udp
```

Pause and Latch Pause stop new host targets; SMC3 still controls the motors. Motors enable at firmware startup with initial targets of 512. Use the hardware motor-power stop for stopping motor power.

### Arduino port detection and executable refresh

Automatic selection uses USB serial metadata. It prefers official Uno VID/PID identities from [Arduino's AVR board definitions](https://github.com/arduino/ArduinoCore-avr/blob/master/boards.txt). If no official Uno is identified, one USB serial adapter can be selected as a clone candidate. Multiple matching Unos or multiple fallback USB candidates require --serial COMx. Non-USB legacy ports such as the old COM1 are excluded from automatic selection.

USB identity selects a candidate, not a verified controller. SimSync marks the controller ready only after the supplied SMC3 firmware answers the version 0.70 probe. It re-enumerates candidates when retrying instead of retaining a guessed COM port. Opening the Uno can reset it, so the connection waits for boot before probing.

The reported auto-detected ESP32 on COM1 messages came from the older simsync.exe, whose file timestamp was September 22, 2026. That executable was still running during this update. The refreshed build is supplied as simsync-updated.exe so it can be launched after closing the old process without replacing its running executable. New logs describe an Arduino USB serial candidate, then confirmed SMC3 firmware once verified.
