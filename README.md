# ARC Reactor MK-I Firmware

ESP32-C3 Super Mini firmware for an educational wearable prototype. The sketch contains code paths for an AD8232 ECG waveform dashboard, an MPU6050 orientation and step counter, touch-controlled sleep, a status LED, and a Wi-Fi web interface.

> **Status:** The prototype is currently running live. The supplied handover marks selected bring-up, lead-off, body-position, dashboard-stability, and touch/sleep checks as done. ECG/BPM reference accuracy, step-count accuracy, sleep current, and power-bank tests remain pending. The exact successful build environment and build log are not recorded. See Docs/TEST_STATUS.md.

## Repository layout

```text
Firmware/
  ARC_Reactor_Sensor___Dashboard/
    ARC_Reactor_Sensor___Dashboard.ino
    secrets.h                 # Local Wi-Fi settings; ignored by Git
Tests/
  run_host_simulation.ps1     # Host-side algorithm and dashboard syntax checks
  TEST_REPORT.md              # Results and test limitations
Docs/                         # Project documentation
Hardware/                     # Hardware records and diagrams
```

## Firmware and hardware references

The sketch header lists these connections for the current code revision:

| Function | ESP32-C3 pin |
|---|---:|
| AD8232 output | GPIO3 |
| AD8232 lead-off + / - | GPIO6 / GPIO7 |
| MPU6050 SDA / SCL | GPIO4 / GPIO5 |
| Touch input | GPIO1 |
| WS2812 / NeoPixel data | GPIO10 |

These pin assignments come from the firmware source. Confirm them against the exact board and modules before wiring; ESP32-C3 Super Mini variants can differ. Check each module’s voltage and logic limits. Do not assume a 5 V signal is safe for an ESP32-C3 input.

The dashboard code exposes ECG samples and telemetry over HTTP. The source includes Wi-Fi station mode and a fallback access point. The firmware also includes sensor filtering, posture classification, step counting, orientation calibration, and deep-sleep behavior. The handover records selected prototype checks as complete; see Docs/TEST_STATUS.md for remaining verification.

## Build and upload

1. Install Arduino IDE and Espressif ESP32 board support using the official installation instructions.
2. Open `Firmware/ARC_Reactor_Sensor___Dashboard/ARC_Reactor_Sensor___Dashboard.ino`.
3. Select **ESP32C3 Dev Module** and enable **USB CDC On Boot** if those options are available for the board package and board.
4. Optional: create a local `secrets.h` in the sketch folder with the Wi-Fi settings expected by the sketch (`WIFI_SSID`, `WIFI_PASS`, `AP_SSID`, `AP_PASS`). Keep credentials private; `secrets.h` is Git-ignored. Without it, the sketch uses its fallback access-point settings. Change the fallback password in the firmware before relying on that access point.
5. Compile first. Only upload after confirming the selected board and port. Open Serial Monitor at 115200 baud for startup messages.

The repository does not yet record the ESP32 board package version or a successful Arduino build. The build steps above are setup guidance, not a build result.

## Run host checks

From the repository root in PowerShell:

```powershell
.\Tests\run_host_simulation.ps1
```

The host runner checks selected filtering and posture algorithms with synthetic inputs and validates the embedded dashboard JavaScript syntax. See [`Tests/TEST_REPORT.md`](Tests/TEST_REPORT.md) for the full chart, source fingerprint, and limitations.

These checks do **not** compile the complete Arduino sketch or simulate the ESP32 peripherals, network, or sensors.

## Safety and limitations

- This is an educational prototype, not a medical device. Do not use its readings for diagnosis, treatment, emergency response, or health decisions.
- The AD8232 module is not medical isolation. Do not connect electrodes to a person while the prototype is connected to a computer, charger, mains-powered equipment, or other unisolated equipment. Human testing requires a properly isolated, battery-only setup and qualified guidance.
- Verify power, wiring, module revisions, and signal limits before connecting hardware. Stop if anything heats unexpectedly or a battery is damaged, swollen, or hot.
- No hardware validation, clinical validation, or safety certification is claimed.

## Project status

Record board and sensor revisions, wiring, firmware version, build environment, bench setup, and results as they are verified. Keep simulated results separate from build, bench, and human-use results.
## Project documentation

See [Docs/README.md](Docs/README.md) for the overview, hardware notes, firmware details, build steps, and test status.
