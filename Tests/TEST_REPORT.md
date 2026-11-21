# Firmware Host Simulation Report

**Run date:** 2026-11-21  
**Target:** `Firmware/ARC_Reactor_Sensor___Dashboard/ARC_Reactor_Sensor___Dashboard.ino`  
**Firmware SHA-256:** `32634D06F7922C83672A44AC10B5F4419AF2068BB9AF81BBA9F779095243910B`

## Result

**PASS for the host checks listed below.** This is a partial software simulation, not a full ESP32-C3 firmware build or hardware validation.

| Check | Result | Evidence |
|---|---|---|
| Host C++ harness compile | PASS | `g++ -std=c++17 -Wall -Wextra -Werror`; no warnings or errors |
| Median of five, high/low impulse | PASS | Both synthetic outlier assertions passed |
| RR interval median | PASS | Seven values with one high outlier returned expected median |
| 50 Hz notch | PASS | Synthetic 50 Hz input fell below RMS threshold 0.05 after settling |
| 10 Hz notch passband | PASS | Synthetic 10 Hz RMS remained above threshold 0.65 |
| Pose classification | PASS | Upright, upside-down, back, stomach, side, and tilt vectors passed |
| Degenerate calibration fallback | PASS | Parallel calibration vectors produced orthogonal basis vectors |
| Dashboard JavaScript syntax | PASS | Embedded script accepted by `node --check` |

**Assertions:** 12 C++ simulation assertions passed, plus JavaScript syntax check.

## How to rerun

From the repository root in PowerShell:

```powershell
.\Tests\run_host_simulation.ps1
```

The runner extracts the tested algorithms and embedded dashboard script from the firmware sketch at run time. It creates temporary build files under the system temporary directory and does not alter the firmware source.

## Not tested

- Full Arduino/ESP32-C3 compile, link, memory use, or upload was not independently run as part of this host-test session. The prototype is currently running live, confirming successful build and upload; exact build logs and tool versions are not recorded here.
- FreeRTOS scheduling, real ADC sampling rate, ECG heart-rate detection, MPU6050 I2C behavior, step counting, deep sleep/wake, flash preferences, Wi-Fi fallback/reconnect, HTTP responses, or browser rendering.
- Wiring, voltage levels, signal quality, LED behavior, battery safety, or any human-worn behavior.

Do not treat these host results as hardware, medical, or safety validation. Run an ESP32-C3 build and bench tests with a signal source and the intended modules before claiming those paths pass.
## Detailed test chart

| Test name | Input / pass parameter | Result |
|---|---|---|
| Host C++ harness compile | C++17 with `-Wall -Wextra -Werror`; zero warnings and errors | PASS |
| Median filter: high impulse | Input `1, 2, 100, 3, 4`; expected result `3` | PASS |
| Median filter: low impulse | Input `1, -100, 2, 3, 4`; expected result `2` | PASS |
| RR interval median | Seven values `100, 200, 300, 400, 500, 600, 9000`; expected median `400` | PASS |
| 50 Hz notch filter | Synthetic 50 Hz signal; RMS after settling must be `< 0.05` | PASS |
| 10 Hz notch passband | Synthetic 10 Hz signal; RMS after settling must be `> 0.65` | PASS |
| Upright pose | Vector `(0, 1, 0)` must classify as upright | PASS |
| Upside-down pose | Vector `(0, -1, 0)` must classify as upside-down | PASS |
| Back pose | Vector `(0, 0, 1)` must classify as back | PASS |
| Stomach pose | Vector `(0, 0, -1)` must classify as stomach | PASS |
| Side pose | Vector `(1, 0, 0)` must classify as side | PASS |
| Tilted pose | Vector `(0.4, 0.4, 0.4)` must classify as tilted | PASS |
| Calibration fallback | Parallel calibration vectors; basis dot product must be `< 0.001` | PASS |
| Dashboard JavaScript syntax | Embedded script must pass `node --check` | PASS |

**Summary:** 12 C++ simulation assertions and the dashboard JavaScript syntax check passed. The ESP32-C3 full build and hardware checks were not run; see “Not tested” above.