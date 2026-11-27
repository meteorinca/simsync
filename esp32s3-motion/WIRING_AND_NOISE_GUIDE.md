# ESP32-S3 Noise-Free 2DOF / 3DOF Wiring & Hardware Guide

> **CRITICAL VOLTAGE WARNING — READ BEFORE WIRING**
> The ESP32-S3 uses **3.3V logic** and its ADC1 pins are **NOT 5V tolerant** (absolute max 3.6V).
> - **Power all 3 potentiometers from ESP32-S3 `3V3`**, NEVER from 5V!
> - **IBT-2 `VCC` (logic power)** can be powered from `3V3` or `5V` (BTS7960 74HC244 buffer accepts 3.3V HIGH inputs cleanly at both 3.3V and 5V VCC; powering IBT-2 `VCC` from `5V` while driving RPWM/LPWM/EN at 3.3V works reliably on genuine 74HC244/74HCT244 IBT-2 boards — if your clone has strict CMOS thresholds at 5V, power IBT-2 `VCC` from **3.3V**, which is well within the 74HC244 2–6V spec!).
> - **NEVER connect 24V motor power to any ESP32-S3 pin.**

---

## 1. Why the Old ESP32-S3 Setup Had ~26 Counts of ADC Noise — And How This Build Eliminates It

Previously (see `FullStory.md`), the ESP32-S3 had ~26 counts of peak-to-peak jitter because:
1. **WiFi HTTP server was running** -> WiFi TX bursts pull 250–350 mA spikes on the 3.3V rail and couple RF straight into the SAR ADC.
2. **Single-shot `analogRead()`** captured raw switching spikes directly with no calibration or oversampling.

This ESP-IDF v5.5.4 firmware attacks noise at **every layer simultaneously**:
- **Zero RF**: WiFi & Bluetooth are completely disabled (`CONFIG_ESP_WIFI_ENABLED=n`, `CONFIG_BT_ENABLED=n`).
- **Continuous GDMA ADC1 @ 72 kSPS (24 kSPS per motor)**: Hardware DMA continuously scans GPIO4, GPIO5, GPIO6 with zero CPU overhead.
- **12x Oversampling + Outlier-Trimmed Mean per 2 kHz PID tick**: Discards the highest and lowest raw sample in every 500 µs batch (killing any single-sample PWM switching spike) and averages the remaining 10+ samples (+1.8 bits ENOB).
- **eFuse Curve-Fitting Calibration**: Uses factory-burned ESP32-S3 eFuse calibration curves (`adc_cali_create_scheme_curve_fitting`) to remove SAR ADC nonlinearity.
- **2nd-Order Butterworth Biquad LPF (80 Hz default)** on Core 1 FPU: 40 dB/decade roll-off with minimal phase lag.
- **Center-Aligned 20 kHz Ultrasonic MCPWM**: Synchronized symmetric up-down PWM cuts motor current ripple & EMI in half and eliminates audible motor whine.
- **Physical Pin Separation**: ADC1 pots live on GPIO4/5/6 (next to `3V3`), physically separated from PWM outputs and avoiding all strapping/USB/Octal-PSRAM pins.

---

## 2. Exact Pinout (ESP32-S3 -> 3x IBT-2 / BTS7960 + 3x 10k Pots)

All pins are configurable via `idf.py menuconfig` -> `SimSync ESP32-S3 Motion Controller`.

| Signal | Motor 1 (Front-Left) | Motor 2 (Front-Right) | Motor 3 (Rear / 3DOF) | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **RPWM** (MCPWM) | **GPIO 10** | **GPIO 13** | **GPIO 39** | 20 kHz center-aligned |
| **LPWM** (MCPWM) | **GPIO 11** | **GPIO 14** | **GPIO 40** | 20 kHz center-aligned |
| **R_EN + L_EN** (tied) | **GPIO 12** | **GPIO 21** | **GPIO 41** | Starts LOW on boot (safe) |
| **Pot Wiper (ADC1)** | **GPIO 4** (`ADC1_CH3`) | **GPIO 5** (`ADC1_CH4`) | **GPIO 6** (`ADC1_CH5`) | **0–3.3V ONLY!** |
| **Pot Outer Pins** | `3V3` & `GND` | `3V3` & `GND` | `3V3` & `GND` | Add 100 nF cap wiper->GND |
| **IBT-2 Logic VCC/GND** | `3V3` (or `5V`) & `GND` | `3V3` (or `5V`) & `GND` | `3V3` (or `5V`) & `GND` | Common logic ground |
| **IBT-2 B+ / B-** | Fused `+24V` / `0V` | Fused `+24V` / `0V` | Fused `+24V` / `0V` | Heavy gauge, star ground |
| **Optional HW E-Stop** | **GPIO 42** (or disabled `-1`) | — | — | NC switch to GND -> nanosecond MCPWM hardware fault cut |

> **USB Cable**: Plug into the ESP32-S3's **native USB port** (labeled `USB` / `OTG`, connected to GPIO19/20 — *not* the `UART` CH340/CP2102 port!). Native USB Serial/JTAG gives sub-millisecond 12 Mbps binary throughput AND supports `idf.py flash` over the exact same cable.

---

## 3. Recommended 5-Minute Hardware Noise-Proofing Checklist

Even with our software DSP, three tiny hardware habits turn a good rig into a rock-solid **< 1–2 count RMS (12-bit)** rig:

1. **100 nF (0.1 µF) ceramic capacitor** soldered/wired directly between each pot wiper (`GPIO4`, `GPIO5`, `GPIO6`) and `GND` as close to the ESP32-S3 header as possible. Together with a 10k pot (max ~2.5 kΩ source impedance at mid-travel), this forms a clean passive ~630 Hz hardware anti-aliasing filter right before the ADC sample-and-hold!
2. **Twisted or shielded pot wires** routed physically away from the high-current 24V `M+`/`M-` motor cables.
3. **Star ground**: Run heavy 24V return wires (`B-`) directly to the 24V PSU negative terminal, and run a single logic ground reference from the PSU negative / IBT-2 GND to the ESP32-S3 `GND` so 30A motor return currents never flow across the potentiometer ground wire.

---

## 4. Safe First Power-Up & Verification

1. **Keep 24V motor power OFF.**
2. Flash the ESP32-S3 via native USB (`.\flash.ps1`).
3. Start the Go Live Tuner in bench mode:
   ```powershell
   .\simsync-s3.exe --bench
   ```
4. Open `http://127.0.0.1:7080` in your browser.
5. Check the **ADC Noise Floor & Live State** panel:
   - Verify `12x DMA` (or similar) samples/tick and `Core1: ~20–40 µs` loop time.
   - Gently rotate M1 and M2 pots by hand and verify smooth 0–4095 movement.
   - Check stationary `Noise RMS` and `p2p` counts (expect ~0.3–1.5 RMS counts at 12-bit = **< 0.4 counts in 10-bit SMC3 terms!**).
6. Confirm pot travel bounds sit safely inside `cutoff` (`92..4003`) and `clip` (`160..3935`), then turn on 24V motor power with an accessible hardware E-Stop, click **Hold & Enable**, and test small `Nudge ±80` steps before switching to Dirt Rally 2.0 UDP mode!
