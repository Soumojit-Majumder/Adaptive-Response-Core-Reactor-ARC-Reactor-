# ARC Reactor MK-I: Project Overview

## Project summary
ARC Reactor MK-I is an ESP32-C3 chest-worn prototype combining a single-lead ECG front end, motion sensing, touch control, and a live browser dashboard. The first prototype is currently running live, so its successful build and upload are confirmed; exact toolchain versions and a build log are not recorded in the repository.

This is an educational engineering prototype, not a medical device, diagnostic instrument, or emergency response system.

## Current prototype
| Feature | Status |
|---|---|
| Live ECG waveform and electrode lead-off indication | Working prototype |
| Heart-rate estimate | Implemented; reference accuracy validation pending |
| Step counter and walking indication | Working prototype |
| Body orientation, pitch/roll, and axis display | Working prototype |
| One-time orientation calibration saved to flash | Working prototype |
| Browser dashboard served over Wi-Fi | Working prototype |
| Touch control, blue status LED, and deep sleep | Working prototype |

The status follows the project handover. Test details and pending criteria are in TEST_STATUS.md.

## System architecture
Chest electrodes -> AD8232 -> ESP32-C3 ADC -> firmware -> Wi-Fi HTTP server -> browser dashboard
MPU6050 accelerometer -> I2C -> ESP32-C3
TTP223 touch pad -> ESP32-C3 GPIO
ESP32-C3 GPIO -> WS2812 blue status pixel

The firmware is a single Arduino sketch with ECG sampling, motion processing, an HTTP server, and the main touch/Wi-Fi loop.

## Repository guide
- Firmware: ../Firmware/ARC_Reactor_Sensor___Dashboard/ARC_Reactor_Sensor___Dashboard.ino
- Hardware and wiring: HARDWARE_AND_WIRING.md
- Firmware and dashboard: FIRMWARE_AND_DASHBOARD.md
- Build and upload: BUILD_AND_UPLOAD.md
- Test matrix and results: TEST_STATUS.md
- Host test runner and report: ../Tests/run_host_simulation.ps1 and ../Tests/TEST_REPORT.md

## Limits
The device shows a single-lead waveform and an estimated heart rate. It does not provide clinical interpretation. Heart-rate reference accuracy remains pending. The prototype is not an emergency monitoring system. See HARDWARE_AND_WIRING.md for safety limits.