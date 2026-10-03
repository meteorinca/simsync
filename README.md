# SimSync

Go motion engine and local browser dashboard for an Arduino Uno running the supplied SMC3.ino (Mode 2, firmware version 0.70).

## Run

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
- Read-only connection probe for firmware 0.70, followed by polling of all three motors.
- Positions and errors use ADC counts (0-1023), with nominal center 512. Firmware feedback has four-count resolution. PWM feedback is unsigned magnitude (0-255), without direction information.
- Live target, actual, error and PWM scope; session CSV export.
- PID display reads Motor 1. Send Live applies Kp/Ki/Kd to all three motors. Gains use the sketch's value-times-100 encoding. Save to EEPROM sends [sav], saving all controller settings.
- Host step tests and grid sweeps use ADC counts and apply gains to all motors. Start these in bench mode. Host pause prevents further targets; feedback loss aborts a step.
- Use SMC3Utils for per-motor tuning, PWM, travel limits, potentiometer alignment and initial commissioning. The supplied firmware has no on-device calibration or autotune command.

Pause and Latch Pause stop Go from sending new targets. They do not disable motor power or the Uno's PID loop. SMC3 holds its existing target and enables motors during firmware startup. A hardware stop remains necessary for stopping motor power. Connection and reconnection do not write targets, enable commands or PID defaults.

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
