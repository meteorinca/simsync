# SimSync

Go motion engine and local browser dashboard for an Arduino Uno running the supplied SMC3.ino (Mode 2, firmware version 0.70).

## Run

### Linux / Omarchy

```sh
go build -o simsync ./cmd/simsync
./simsync --no-udp
```

The native Linux executable is `simsync`, without `.exe`. This repository pins Go through mise for Omarchy; if mise asks, review and trust `.mise.toml` with `mise trust`. USB auto-detection supports Linux too. To select a board explicitly, use `./simsync --serial /dev/ttyACM0 --no-udp` (some clones appear as `/dev/ttyUSB0`; a `/dev/serial/by-id/` path is preferable when available).

Keep motor power off during the first USB connection: opening the port can reset the Uno, and this firmware enables motors at startup even when the dashboard is paused. Leave USB power connected to inspect feedback. With only one motor installed, use bench mode and that motor's manual target; game motion expects three motors.

If no board is detected, check `arduino-cli board list` and `/dev/serial/by-id/`, and try a known data-capable USB cable. A missing USB device is not fixed by serial permissions. If the port exists but access is denied, inspect its ownership with `ls -l` and configure access for the actual device group; do not run the dashboard as root.

### Windows

```powershell
go build -o simsync.exe ./cmd/simsync
.\simsync.exe --serial COM9 --no-udp
```

The dashboard opens at http://localhost:7070. Close SMC3Utils before connecting because only one application can own the serial port. Flash the supplied sketch using Arduino IDE with Arduino Uno selected.

Use bench mode first to check feedback and direct targets. Resume permits commands. A direct target switches to manual control, so game telemetry cannot overwrite it. Resume again to return to game motion. Clear Pause clears the host latch but leaves output paused.

For game telemetry:

```powershell
.\simsync.exe --serial COM9
```

## Controller and dashboard

- Native SMC3 serial at 500,000 baud; five-byte position commands with big-endian 10-bit values.
- Read-only connection probe for firmware 0.70, followed by position polling at 50 Hz and settings polling at 2 Hz. Motor 1 receives the bench profile below on connection.
- Positions and errors use ADC counts (0-1023), with nominal center 512. Firmware feedback has four-count resolution. PWM feedback is unsigned magnitude (0-255), without direction information.
- Live target, actual, error and PWM scope; session CSV export.
- Gains and controller settings have live readback for the selected motor. Apply Gains affects only that motor. Ki = 0 edits the form for PD tuning; Apply Gains sends it. Save EEPROM saves all controller channels, including PWM and firmware margins.
- Motor Limits exposes PWM minimum, maximum, reverse PWM, deadzone, firmware clip/cutoff margins, and session-local host minimum, maximum and center. Pause before configuring. Include in output selects the motor for commands; it does not disable excluded hardware.
- Firmware margins are symmetric: `margin` through `1023-margin`. The existing protocol limits margins to 255. Require `1 <= cutoff < clip <= 255`; host bounds must sit at least four counts inside the clipped range. Host bounds constrain commands but do not disable the Arduino or prevent physical overshoot. Narrower hardware-enforced ranges require a firmware change.
- Enable Motor sets a target at current feedback and enables that motor. Resume enables included motors at their current feedback positions. Both require configured bounds and fresh feedback inside them, and are blocked while jog mode is enabled.
- Manual targets, center and jog use one synchronized motor selection. Commands outside configured bounds, on excluded or disabled motors, or with stale feedback are rejected. Game output uses only included motors and pauses if a target fails validation.
- Bounded step tests run only in bench mode, apply gains to one motor, and accept signed steps up to 40 counts. Preflight checks bounds and current tracking before any gain write. Pause or Cancel / Hold aborts without a return movement. Successful tests return to baseline. The grid sweep is unavailable.
- Step metrics use a minimum four-count settling tolerance and require at least 200 ms in tolerance through the end of the recording. Missing rise/settling measurements are shown as unavailable, not zero. These are host-observed measurements at the serial feedback resolution.

Host bounds and included motors reset on reconnect or restart. EEPROM saves controller settings only. Readback is shown separately from editable fields; Read reloads the selected motor's form. With motor power off, establish conservative host bounds from known safe feedback positions, then configure limits, enable the selected motor, and resume when ready. Do not use powered travel to discover potentiometer stops.

The HTTP dashboard binds only to loopback. Mutating API requests require POST with `X-SimSync: 1`. Startup applies Motor 1's bench profile in RAM after controller discovery; firmware uploads and EEPROM saves are separate operations.

The updated firmware starts with motors disabled. Pause, Latch Pause and Stop Jog send `[stp]`, which disables every motor with this firmware; older firmware may only stop a jog. A hardware stop remains necessary for removing motor power.

## Jogging and angle display

Open **Open-loop Jog** and enable jog mode. This pauses PID/game output and permits bounded timed pulses with UDP enabled. Disable jog mode before Resume. Automatic polarity checking has been removed; use verified, labelled motor wiring.

The plot is the primary view; secondary settings and tests are collapsible. **Angle calibration** maps counts to degrees per motor: supply the count at zero degrees and the potentiometer's physical angle across ADC 0–1023. Calibration is saved in this browser. Until calibrated, positions remain honestly labelled ADC counts. Position commands and steps convert degrees back to counts; serial commands, advanced ADC limits, gains and CSV remain in original units.

Motor 1's connection profile is Ki 0, host bounds 50–650, center 512, PWM min/max/reverse 0/100/50, cutoff 23 and clip 40. No EEPROM save is automatic.

## Motion mapping

UDP port 20777 accepts the existing racing telemetry format. Longitudinal force maps to pitch, lateral force to roll and mean suspension travel to heave, with an adjustable low-pass filter and per-axis limits. This is a simple starting mix, without washout or geometry calibration.

Motor 1 is front left, Motor 2 front right, Motor 3 rear. The mix is:

- Front left: neutral + pitch + roll + heave
- Front right: neutral + pitch - roll + heave
- Rear: neutral - pitch + heave

Verify motor ordering and direction on the actual rig. Gain and limit values now mean counts, not degrees. Legacy configuration field names PitchNeutral, RollNeutral and HeaveNeutral refer to the three motor centers. Motor commands are bounded to 0-1023; firmware input clipping may further restrict travel.

Motion starts paused. Stale game telemetry stops new game targets; serial feedback loss pauses host output. Resume after reconnection.

## Flags

| Flag | Default | Purpose |
|---|---|---|
| --serial | auto | Prefer Uno USB identity; a single USB adapter is allowed for clones; ambiguous candidates require an explicit port |
| --baud | 500000 | Must match the sketch |
| --no-udp | false | Bench mode |
| --no-serial | false | Dashboard only |
| --udp-port | 20777 | Game telemetry port |
| --port | 7070 | Dashboard port |
| --session-sec | 300 | Session buffer duration |
| --open-browser | true | Open browser on startup |

## Verification

```powershell
go test ./...
go vet ./...
```

Protocol and motion tests run without hardware. Actual serial transport, motor movement and rig commissioning require a connected Uno and bench verification.
