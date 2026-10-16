# SimSync

Go motion engine and local browser dashboard for an Arduino Uno running the supplied SMC3.ino (Mode 2, firmware version 0.70).

## Run

### Linux / Omarchy

```sh
./build.sh
./simsync --no-udp
```

`./build.sh` builds the executable and embeds the latest web UI. It works from any directory, keeps the previous executable if compilation fails, and does not restart an existing instance. After a successful build, stop the running instance and start it again. Go must be available in PATH.

The native Linux executable is `simsync`, without `.exe`. This repository pins Go through mise for Omarchy; if mise asks, review and trust `.mise.toml` with `mise trust`. USB auto-detection supports Linux too. To select a board explicitly, use `./simsync --serial /dev/ttyACM0 --no-udp` (some clones appear as `/dev/ttyUSB0`; a `/dev/serial/by-id/` path is preferable when available).

Keep motor power off during the first USB connection: opening the port can reset the Uno. The supplied firmware starts with motors disabled. Leave USB power connected to inspect feedback. With only one motor installed, use bench mode and that motor's manual target; game motion expects three motors.

If no board is detected, check `arduino-cli board list` and `/dev/serial/by-id/`, and try a known data-capable USB cable. A missing USB device is not fixed by serial permissions. If the port exists but access is denied, inspect its ownership with `ls -l` and configure access for the actual device group; do not run the dashboard as root.

### Windows

```powershell
.\build.ps1
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
- Read-only connection probe for firmware 0.70, followed by position polling at 50 Hz and settings polling at 2 Hz. Saved settings are restored after discovery and on reconnect; host output starts paused.
- Positions and errors use ADC counts (0-1023), with default center 338. Firmware feedback has four-count resolution. PWM feedback is unsigned magnitude (0-255), without direction information.
- Live target, actual, error and PWM scope; session CSV export.
- Gains and controller settings have live readback for the selected motor. Apply Gains affects only that motor. Ki = 0 edits the form for PD tuning; Apply Gains sends it. Save EEPROM saves all controller channels, including PWM and firmware margins.
- Motor Limits exposes PWM minimum, maximum, reverse PWM, deadzone, firmware clip/cutoff margins, and persistent host minimum, maximum and center. Pause before configuring. Include in output selects the motor for commands; it does not disable excluded hardware.
- Firmware margins are symmetric: `margin` through `1023-margin`. The existing protocol limits margins to 255. Require `1 <= cutoff < clip <= 255`; host bounds must sit at least four counts inside the clipped range. Host bounds constrain commands but do not disable the Arduino or prevent physical overshoot. Narrower hardware-enforced ranges require a firmware change.
- Enable Motor sets a target at current feedback and enables that motor. Resume enables included motors at their current feedback positions. Both require configured bounds and fresh feedback inside them, and are blocked while jog mode is enabled.
- Manual targets, center and jog use one synchronized motor selection. Commands outside configured bounds, on excluded or disabled motors, or with stale feedback are rejected. Game output uses only included motors and pauses if a target fails validation.
- Bounded step tests run only in bench mode, apply gains to one motor, and accept signed steps up to 40 counts. Preflight checks bounds and current tracking before any gain write. Pause or Cancel / Hold aborts without a return movement. Successful tests return to baseline. The grid sweep is unavailable.
- Step metrics use a minimum four-count settling tolerance and require at least 200 ms in tolerance through the end of the recording. Missing rise/settling measurements are shown as unavailable, not zero. These are host-observed measurements at the serial feedback resolution.

Host bounds, centers and included motors are restored from `simsync-settings.md` on reconnect and restart. UI Apply saves tuning there automatically. EEPROM saves controller settings only. Readback is shown separately from editable fields; Read reloads the selected motor's form. With motor power off, establish conservative host bounds from known safe feedback positions, then configure limits, enable the selected motor, and resume when ready. Do not use powered travel to discover potentiometer stops.

The HTTP dashboard binds only to loopback. Mutating API requests require POST with `X-SimSync: 1`. Startup restores the Markdown settings in RAM after controller discovery; firmware uploads and EEPROM saves are separate operations.

The updated firmware starts with motors disabled. Pause, Latch Pause and Stop Jog send `[stp]`, which disables every motor with this firmware; older firmware may only stop a jog. A hardware stop remains necessary for removing motor power.

## Persistent settings

SimSync creates **`simsync-settings.md` beside its executable** if missing and reads it at startup. Its short tables list per-motor PID, Ks, PWM, clipping, deadzone, host bounds, center and inclusion, plus shared timing and motion mapping. UI Apply saves changes automatically, including gains applied by a step test; restarting or reconnecting restores those settings. Save EEPROM remains a separate Arduino operation.

Edit the Markdown values while SimSync is stopped, then restart; no rebuild is needed for settings changes. An invalid file produces a specific startup error and is kept intact. Writes replace the file only after a complete save; failed saves are reported in the UI. Use `--settings /path/to/file.md` to choose another location. Startup restores configuration with output paused and sends no center/target/enable command.

## Motor Tuning tab

The dashboard keeps the live plot, motor status, game telemetry and host output controls. Detailed position controls, PID, limits, jog, timing, mapping, display calibration and test results appear only on **Motor Tuning**, alongside the same live plot. Selecting a motor never sends a hardware command. **Run only M1/M2/M3** stops all motors, holds/enables the selected motor and enables manual/test output for it. Commands to other motors are blocked. Solo mode is session-local and does not change saved inclusion or gains. **Pause / Stop all motors** ends solo mode; **Resume** enables all included motors and permits game output again. Step tests/signals still require `--no-udp` bench mode.

Each motor has separate **feedback LPF** (0/off or 0.1–200 Hz) and **derivative HPF** (0/off or 0.1–50 Hz), adjustable in 0.1 Hz increments. When both are on, HPF must be below LPF. Filters run inside the Arduino PID loop at its measured update interval, with derivative timing accounting for Ks. LPF smooths feedback for P/I/D; HPF is an optional washout in the D branch only. DC position feedback remains intact, and raw ADC feedback still drives clip/cutoff checks and telemetry. Both filters default off, including when loading older profiles. Pause to apply; controller readback is shown separately from edits. Apply Filters persists each motor's values to the host Markdown profile and restores them on reconnect. They are RAM settings in the Arduino, not saved by Save EEPROM. Upload the updated firmware to use filters; unsupported firmware cannot silently ignore a saved nonzero filter configuration.

The firmware also avoids blanking the bridge on every PID tick when direction is unchanged. Direction changes retain the bridge-disable switching sequence. This removes a possible source of audible switching modulation; physical holding noise still needs validation on the rig.

To approach higher PWM/gains gradually, use the stability ladder in Motor Tuning:

1. Test one unloaded motor with verified feedback direction. Restore its last stable PWM cap and gains, keep Ki = 0, and repeat ±4–8 count steps.
2. For holding chatter, inspect feedback/wiring and try a 1–2 raw-count deadzone. Try feedback LPF around 80 Hz as an initial experiment, keeping HPF off. Lower cutoff adds delay and can worsen stability; keep filter values fixed during each gain comparison.
3. Increase PWM max by 5–10 counts, then repeat the same steps in both directions. If ringing grows, restore the previous cap, increase Kd slightly and retest before trying more PWM. Avoid a 100 → 200 jump.
4. Once the new cap is stable, try about 5% more Kp, repeat the tests, and roll back if holding or settling worsens. Change one setting at a time. Increasing the cap only changes a response that was hitting the old cap; it does not add damping. With Kp 20 and an 8-count initial error, this firmware's P term alone asks for approximately 125 PWM counts, which a cap of 100 clips but 200 permits.
5. Validate several positions and the actual operating load while monitoring current/temperature. The goal is repeatable tracking with acceptable overshoot, not maximum numerical PWM or Kp. Integral action comes only after stable PD behavior. Aggressive gains, mechanical resonance, noisy pots or wrong feedback direction are not cured by filters.

Resume validates all included motors before writing an enable command, and attempts a stop if the write fails. Updated firmware supports an atomic hold/enable mask: the Uno samples exact feedback for each selected motor, resets PID history and enables the mask together. Upload the updated `SMC3/SMC3.ino` with motor power off to use this feature. Earlier firmware retains per-motor target/enable commands.

With timed-stop-capable firmware, the host monitors enabled motors for sustained divergence from a stationary target: error growing more than 16 counts beyond its closest observed value for 100 ms sends `[stp]` to disable all motors and latches the motor-specific reason. A new target resets the observation. This detects some runaway and severe ringing cases; it does not prove correct wiring, protect against every moving-target failure, or replace firmware travel cutoffs and a hardware power stop. Clear Pause explicitly clears the fault for another setup attempt. Verify each unloaded motor's feedback direction using short jog pulses before enabling PID; positive drive must increase that motor's feedback counts.

- **P20 / D0.4** stages Kp 20, Ki 0, Kd 0.4 in the gain form. Apply Gains sends the values; Apply also saves them for restart.
- Show firmware cutoff, input clipping and configured host bounds on the scope; select automatic or full-travel scaling. Lines use controller readback, not unapplied edits. Freeze Display pauses only the plotted view. Save Image downloads a PNG; Export CSV saves the session.
- RMS and peak error plus peak PWM summarize the last five seconds. PWM is unsigned magnitude, not direction. Feedback resolution remains four counts.
- **Save profile / Load profile** store selected-motor gain, limit and filter edits in this browser. Loading stages fields only. Apply Gains / Limits / Filters send changes; Save EEPROM persists the original firmware settings for all channels, excluding these new filters. Profiles exclude host inclusion and shared timing settings.
- **Ks** controls derivative filtering (1–100 samples). Fpid divider (1–10) affects all motors. M1/M2 share their PWM timer (5–35 kHz); M3 selects approximately 3.906 or 31.25 kHz. Pause and turn jog mode off before Apply Timing. Timing readback is shown separately from edits.
- Sine, triangle and square signals require bench mode (`--no-udp`), resumed output, and a configured, enabled motor with fresh feedback tracking its target. Amplitude is ±1–20 ADC counts around starting feedback. Both endpoints must fit host bounds. Each run completes whole cycles in at most three seconds and returns to baseline on success. Stop / Pause aborts with no return movement. Signals and step tests exclude each other and do not run continuously in the browser.

## Jogging and angle display

Open **Open-loop Jog** and enable jog mode. This pauses PID/game output and permits bounded timed pulses with UDP enabled. Disable jog mode before Resume. Automatic polarity checking has been removed; use verified, labelled motor wiring.

The tuning scope's **Display** selector switches between counts (default), approximate degrees, and range percent. Default pot endpoints are **8 counts = 0°** and **668 counts = 270°**, giving `(count − 8) × 270 / 660` degrees. Range percent uses `(count − 8) × 100 / 660`; errors and differences use the same scale without the offset. The midpoint, count 338, displays 135° or 50%.

**Display range** allows per-motor count/angle endpoints. Mappings and the display preference are saved in this browser; existing saved angle calibrations retain their mapping. Outside the calibrated range, values are extrapolated rather than clipped and may be unreliable. Approximate angles are for readability, without a claim of physical accuracy. **Manual position inputs, position buttons, step tests, signal amplitudes, host bounds, serial commands and CSV always use ADC counts**, regardless of the display unit. Display endpoints do not change motor safety limits.

New settings files start with P 20 / I 0 / D 0.4, center **338 counts** (the midpoint of 8–668), host bounds 50–650, PWM min/max/reverse 0/100/50, cutoff 23 and clip 40. M1 and M2 are included initially; M3 is optional. No EEPROM save or motion enable is automatic.

Motor Tuning has individual motor status cards, a selected-motor enable action, positive/negative step tests, and response guidance from recorded measurements. The comparison uses the same motor, signed step size and direction; staging the best successful measured gains fills the fields without sending commands. Apply Gains sends them. Validate both directions under operating load. Existing settings files preserve their output selection: if a motor is excluded, select it, check Include in output and Apply Limits while paused.

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
| --settings | beside executable | Read/write Markdown tuning file |
| --session-sec | 300 | Session buffer duration |
| --open-browser | true | Open browser on startup |

## Verification

```powershell
go test ./...
go vet ./...
```

Protocol and motion tests run without hardware. Actual serial transport, motor movement and rig commissioning require a connected Uno and bench verification.
