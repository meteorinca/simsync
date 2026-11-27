# SimSync ESP32-S3 — Full-Capability 2DOF / 3DOF Motion Stack

A self-contained, ultra-low-latency, noise-free motion simulator stack built specifically around the **ESP32-S3** (native ESP-IDF v5.5.4) and a zero-allocation **Go Classical Washout Engine & Live Web Tuner**.

## Folder Layout

```text
esp32s3-motion/
├── firmware/                   # Native ESP-IDF v5.5.4 C firmware for ESP32-S3
│   ├── CMakeLists.txt
│   ├── sdkconfig.defaults      # 240 MHz, WiFi/BT OFF, IRAM/cache-safe RT ISRs
│   └── main/
│       ├── Kconfig.projbuild   # Menuconfig pinout, 2DOF/3DOF, PID/ADC/PWM rates
│       ├── motion_config.h     # Shared 12-bit structs & Turbo v2 binary spec
│       ├── adc_engine.c/.h     # ADC1 Continuous GDMA 72 kSPS + trimmed mean + eFuse curve-fit
│       ├── mcpwm_drive.c/.h    # Center-aligned 20 kHz MCPWM + hardware GPIO E-Stop fault
│       ├── pid_control.c/.h    # Core 1 FPU PID, 2nd-order Butterworth Biquad, Kff, anti-windup
│       ├── protocol.c/.h       # Core 0 Native USB Serial/JTAG (SMC3 v0.70 + Turbo v2 12-bit)
│       ├── nvs_store.c/.h      # Off-core NVS flash profile storage
│       └── main.c              # Dual-core FreeRTOS + hardware gptimer 2 kHz RT scheduler
├── host/                       # Zero-allocation Go Motion Cueing Engine & Web UI
│   ├── cmd/simsync-s3/main.go  # 500 Hz interpolated output loop + 500 ms smooth center watchdog
│   └── internal/
│       ├── telemetry/dirt.go   # Zero-alloc Dirt Rally 2.0 (extradata=3, 264B/256B) UDP listener
│       ├── washout/            # 2nd-order Biquad classical washout + tanh knee + crank asin
│       ├── transport/          # Auto-detect ESP32-S3 USB + Turbo v2 12-bit & SMC3 fallback
│       └── web/                # Embedded single-file HTML5 Canvas 60fps scope & live tuner
├── flash.ps1                   # One-command ESP-IDF v5.5.4 build & flash script
├── build_host.ps1              # Runs Go tests, go vet, and builds simsync-s3.exe
└── WIRING_AND_NOISE_GUIDE.md   # Pinout, 3.3V ADC safety, and noise-elimination guide
```

---

## Hardware Capabilities Used on ESP32-S3

| Subsystem | ESP32-S3 Hardware Feature Used | Benefit |
| :--- | :--- | :--- |
| **Power / RF** | WiFi & Bluetooth fully disabled in `sdkconfig` | Eliminates 250 mA TX rail spikes that caused 26-count ADC jitter |
| **Position Sensing** | **ADC1 Continuous GDMA** @ 72 kSPS (24 kSPS/motor) + **eFuse Curve Fitting** + outlier-trimmed mean | 12x hardware oversampling per 2 kHz tick, kills PWM switching spikes, true linearized 12-bit (0–4095) |
| **Motor Drive** | **MCPWM Group 0** center-aligned (UP-DOWN) 20 kHz symmetric dual-PWM (1000-step duty) | Dead-silent ultrasonic drive on all 3 IBT-2s, lower EMI, zero timer compromises |
| **Hardware Safety** | Optional **MCPWM GPIO Fault** (`CONFIG_SIMSYNC_ESTOP_GPIO`) | Physical E-Stop cuts all 6 PWM outputs LOW in silicon nanoseconds |
| **Real-Time Loop** | **Core 1** pinned FreeRTOS task woken by **1 MHz `gptimer`** ISR | Deterministic 2 kHz (configurable to 4 kHz) float32 FPU PID with < 2 µs jitter |
| **USB Link** | **Core 0 Native USB Serial/JTAG** (no CH340/FTDI chip) | Sub-ms 12 Mbps binary throughput + simultaneous **SMC3 v0.70** & **Turbo v2 12-bit** protocol |
| **Persistence** | **NVS Flash** saved asynchronously on Core 0 | Saving settings never stalls Core 1 PID or motor PWM |

---

## Quick Start

### 1. Build & Flash ESP32-S3 Firmware

Plug the ESP32-S3 in via its **native USB port** (labeled `USB`), then run:

```powershell
cd esp32s3-motion
.\flash.ps1                  # Auto-detect port, build & flash
# Or specify COM port explicitly:
.\flash.ps1 -Port COM12
# Or build without flashing:
.\flash.ps1 -BuildOnly
```

To customize pins, 2DOF vs 3DOF default, 2 kHz vs 4 kHz loop rate, or optional silicon ADC IIR / E-Stop GPIO:

```powershell
. C:\Espressif\tools\Microsoft.v5.5.4.PowerShell_profile.ps1
idf.py -C esp32s3-motion\firmware menuconfig
```

### 2. Build & Run the Go Washout Engine + Live Web Tuner

```powershell
cd esp32s3-motion
.\build_host.ps1

# First bench check (motors start DISABLED, 24V power OFF):
.\simsync-s3.exe --bench

# Driving mode (Dirt Rally 2.0 UDP on port 20777, 500 Hz interpolated output):
.\simsync-s3.exe
```

Open **`http://127.0.0.1:7080`** for the live 12-bit oscilloscope, real-time **ADC noise floor meter** (RMS & p2p counts), Core 1 execution/jitter telemetry, live PID & Biquad LPF sliders, and 2DOF/3DOF classical washout tuning.

### 3. Dirt Rally 2.0 UDP Config

In `Documents\My Games\DiRT Rally 2.0\hardwaresettings\hardware_settings_config.xml`:

```xml
<motion_platform>
  <udp enabled="true" extradata="3" ip="127.0.0.1" port="20777" delay="1" />
</motion_platform>
```

### 4. Drop-In Legacy Compatibility

Because the ESP32-S3 firmware auto-multiplexes **SMC3 v0.70 + SimSync** (`[Axx]..`, `[cap]`, `[h,0,mask]`, `[j/k/l]`, `[!-#]`, `[$-&]`) alongside **Turbo v2 12-bit**, you can also point the existing `simsync.exe` (or `SMC3Utils`) straight at the ESP32-S3 COM port anytime!

### 5. Upgrading from 2DOF to 3DOF Later

All 3 ADC1 DMA channels, all 3 MCPWM H-bridge outputs (GPIO39/40/41 + GPIO6), and NVS configs are already wired into the firmware and Go engine. When you bolt on the 3rd motor:
1. Wire M3 IBT-2 & pot per `WIRING_AND_NOISE_GUIDE.md`.
2. Switch **DOF Mode** from `2DOF (M1 + M2)` to `3DOF (M1 + M2 + M3)` in the web UI (or `profile.json`, where `motor3_role` supports `"heave"` for a 3-point platform or `"traction"` for rear traction loss).
3. Click **Apply & Save Washout** and **Save NVS**.
