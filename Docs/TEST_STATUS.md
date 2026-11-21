# Test Status

## Status handling
The prototype is currently running live; its successful build and upload are confirmed. Exact build-tool versions and logs are not recorded. Prototype hardware statuses marked Done in the supplied handover are listed as PASS. Pending items remain PENDING. Handover computer simulations and the repository host test suite are shown separately.

## Prototype test plan
| # | Test | Method / pass criterion | Status |
|---:|---|---|---|
| 1 | Power and sensors | Power up; read Serial Monitor and MPU6050 ID; dashboard reports IMU OK | PASS |
| 2 | ECG with known signal | ECG simulator/fixed-rate source; displayed rate within 3 BPM | PENDING |
| 3 | BPM against reference | Pulse oximeter/smartwatch and manual 60-second count; within 5 BPM | PENDING |
| 4 | Noise and interference | Compare dashboard open/closed, fresh/reused pads, battery and mains power; stable BPM and small baseline noise | PENDING |
| 5 | Lead-off | Remove each electrode; LEADS OFF within 1 s and BPM displays -- | PASS |
| 6 | Steps | Walk 20, 50, 100 steps, three trials each; within 5%; fidget/wave/type for 60 s without false steps | PENDING |
| 7 | Body position | Calibrate, hold each position for 10 s; correct label at least 95% of the time | PASS |
| 8 | Dashboard stability | Run 60 minutes; check errors/rate/delay, restart router, open on two devices | PASS |
| 9 | Touch and sleep | 20 on/off cycles, single and accidental taps; reliable operation without false triggers | PASS |
| 10 | Sleep current | Measure current during deep sleep; target to be confirmed | PENDING |
| 11 | Power-bank mode | Test 0.5 A, 1 A, 2 A; 5 V within 5%; record sag, temperature, protection cutoff | PENDING |

## Computer simulations recorded in the handover
These are synthetic-input simulations, not human measurements.

| Simulation | Recorded result | Status |
|---|---|---|
| Heart-rate detector, clean synthetic ECG at 55-140 BPM | Correct in tested cases; one case reported at 94%; no wrong readings reported | PASS (simulation) |
| Heart-rate detector with 1-2 sample Wi-Fi-style spikes every 0.155 s | 94-100% correct; 0% wrong readings reported | PASS (simulation) |
| Heart-rate detector with 3-sample spikes | Shows -- about 88% of the time; about 12% wrong readings reported | LIMITATION |
| Heart-rate timing at 235 Hz | Handover reports BPM remains correct because elapsed microseconds are used | PASS (simulation) |
| Step counter, walking at 1.4-2.5 steps/s | Correct in tested cases; standing/fidgeting produced no false steps | PASS (simulation) |
| Body position after calibration | 1400/1400 simulated orientations classified correctly with randomized mounting angles | PASS (simulation) |

## Repository host simulation
The host suite in ../Tests/TEST_REPORT.md compiled extracted source algorithms using C++17, -Wall, -Wextra, and -Werror. All 12 C++ assertions passed; the embedded dashboard JavaScript passed node --check. Checks covered median filters, notch behavior, six canonical pose vectors, and parallel-calibration fallback.

The host suite does not exercise the full heart-rate or step-counting loops, Wi-Fi, HTTP, FreeRTOS timing, or physical sensors. Do not treat the handover’s earlier simulations and the latest host suite as one test run. See ../Tests/TEST_REPORT.md for exact parameters and rerun instructions.