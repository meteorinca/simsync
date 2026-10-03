# Uno + SMC3 Mode 2 wiring guide

**Controller:** Arduino Uno R3 (ATmega328P, 5V logic)  
**Drivers:** three IBT-2 / BTS7960 modules  
**Firmware:** [SMC3.ino](SMC3.ino), MODE2 enabled  
**USB serial:** 500,000 baud  
**Feedback:** three 10k potentiometers, one per motor

## Quick wiring table

Read the printed labels on each IBT-2. Header orientation varies between modules.

| IBT-2 terminal | Motor 1: front left | Motor 2: front right | Motor 3: rear |
|---|---|---|---|
| **RPWM / IN1** | Uno **D2** | Uno **D4** | Uno **D6** |
| **LPWM / IN2** | Uno **D9** | Uno **D10** | Uno **D11** |
| **R_EN + L_EN** | Tie together to **D3** | Tie together to **D5** | Tie together to **D7** |
| VCC (logic) | Uno 5V | Uno 5V | Uno 5V |
| GND (logic) | Common ground | Common ground | Common ground |
| R_IS / L_IS | Leave unconnected | Leave unconnected | Leave unconnected |
| B+ (motor supply) | Fused +24V branch | Fused +24V branch | Fused +24V branch |
| B- (motor supply) | Supply negative | Supply negative | Supply negative |
| M+ / M- | Motor 1 leads | Motor 2 leads | Motor 3 leads |
| Pot wiper | Uno **A0** | Uno **A1** | Uno **A2** |

**Mode 2 puts hardware PWM on LPWM.** Keep each driver's R_EN and L_EN paired on its assigned Uno pin. Follow this table rather than the old RPWM/LPWM table in the original context document.

The signal mapping agrees with the supplied sketch and [SMC3 author RufusDufus's Mode 2 clarification, post #188](https://www.xsimulator.net/community/threads/smc3-arduino-3dof-motor-driver-and-windows-utilities.4957/page-10).

## One motor, at a glance

```text
Uno D2  ------------------ IBT-2 RPWM
Uno D9  ------------------ IBT-2 LPWM
Uno D3  --------+--------- IBT-2 R_EN
                +--------- IBT-2 L_EN
Uno 5V  ------------------ IBT-2 VCC
Uno GND ------------------ IBT-2 GND

Fused +24V --------------- IBT-2 B+
Supply negative ---------- IBT-2 B-
Motor leads -------------- IBT-2 M+ and M-

Uno 5V  -------- pot outer terminal
Uno A0  -------- pot wiper (usually center terminal)
Uno GND -------- pot other outer terminal
```

Repeat using the Motor 2 and Motor 3 columns above. Confirm the pot wiper with a meter if its terminals are unfamiliar.

## Power and feedback

- Power the Uno from its USB connection during setup. Supply 5V logic to the drivers and pots from the Uno. **24V goes only to driver B+; never to Uno VIN, 5V or a signal pin.**
- Join supply negative, driver B-/GND and Uno GND at a common ground point. Keep high-current motor return paths on heavy wiring directly to the supply, away from the Uno and its signal wiring.
- Each pot has one outer terminal at 5V, the other at GND, and its wiper at A0, A1 or A2. Mechanically couple it to its own actuator so it measures actual motion. Keep wiring clear of motor leads and avoid pot end stops over the full actuator travel.
- Aim for roughly **512 counts at mechanical center**. ADC range is 0-1023; SimSync feedback is quantized in four-count steps. Configure usable travel and cutoff limits inside the physical and pot end stops.
- Use fused motor supply branches, wiring/connectors sized for motor current, and an accessible hardware motor-power stop. Support the rig when motor power is off.

## Pins to leave alone

| Uno pin | Use in this sketch |
|---|---|
| D0 / D1 | Hardware UART used by USB serial; leave free |
| D8 | PID timing diagnostic; leave free |
| D12 / D13 | Optional second serial, disabled by default; leave free |
| A3 / A4 | Unused |
| A5 | Optional motion scaler, disabled by default; leave unconnected |
| AREF / 3.3V | Not needed for this wiring |

## Upload checklist

1. Keep **24V motor power off** while wiring, uploading and checking feedback.
2. Open [SMC3.ino](SMC3.ino) in Arduino IDE. Accept its request to move the sketch into a folder named **SMC3**, if prompted. Arduino expects **SMC3/SMC3.ino**.
3. Select **Arduino Uno** from the Arduino AVR Boards package and the Uno's COM port. The standard EEPROM and SoftwareSerial libraries come with the AVR core.
4. Confirm the top of the sketch has **MODE2 enabled** and MODE1 commented out. Leave SECOND_SERIAL, both pot-scaling options and REVERSE_MOTOR1 commented out for the initial test.
5. Close SimSync, SMC3Utils and Serial Monitor before Verify/Upload. These applications share the same serial port.
6. Verify and upload. The IDE handles the upload speed; **500,000 baud is the running firmware's serial speed**, not an upload setting.

Uploading does **not normally erase EEPROM**. Existing SMC3 PID, PWM and travel settings are loaded at startup. A fresh EEPROM gets the sketch defaults. Check actual settings in SMC3Utils instead of assuming the values beside the variable declarations are active.

## First bench check

1. With motor power still off, connect using SMC3Utils or SimSync bench mode:

   ```powershell
   .\simsync.exe --serial COM9 --no-udp
   ```

   Substitute the Uno's COM port. Use one application at a time.
2. Move each feedback pot gently and confirm the corresponding motor's feedback moves smoothly. Establish center and travel limits in SMC3Utils before powered tests.
3. Test one unloaded motor at a time with conservative PWM and gains. With power off between changes, correct motor/feedback direction so a small target change makes actual position approach the target. If it moves away, cut motor power immediately; reverse that motor's leads or the pot's outer wires, then recheck.
4. Save verified settings in SMC3Utils. Switch to SimSync and use small manual target changes before trying game motion. SimSync's Send Live applies the displayed PID gains to all three motors.

**The sketch enables all motors at startup and targets start at 512.** Applying 24V can cause movement even while SimSync says PAUSED. Opening USB serial can reset the Uno. SimSync PAUSE/LATCH PAUSE stops new host targets; it does not turn off the driver enables or motor power.

## Firmware housekeeping done

- MODE2 remains the default; a compile-time check rejects selecting both modes or neither.
- Corrected Motor 2's lower feedback cutoff to use CutoffLimitMin2.
- Corrected Motor 3's lower feedback cutoff to use CutoffLimitMin3.
- Added the Mode 2 pin map beside the mode selection and corrected the documented target range to 0-1023.

PID behavior, default gains, PWM settings, EEPROM format, serial protocol and startup motor enabling are preserved. The wiring and code were checked from source; actual Arduino compilation and powered hardware testing still need the IDE and your Uno.
