# Firmware and Dashboard

## Runtime structure
The firmware is a single Arduino sketch targeting ESP32-C3. It uses ESP32 Arduino and ESP-IDF APIs for Wi-Fi, I2C, preferences, deep sleep, timing, and HTTP serving.

| Task or loop | Rate / trigger | Responsibility |
|---|---|---|
| ECG sampler | 250 Hz target; 4 ms interval | Reads AD8232, filters samples, updates ECG ring buffer and BPM estimate |
| Motion task | 50 Hz target; 20 ms interval | Reads MPU6050, estimates gravity direction, posture and steps |
| HTTP server | Browser request | Serves the dashboard and telemetry; handles calibration/reset commands |
| Main loop | Continuous | Watches touch input and retries home Wi-Fi when applicable |

The firmware measures ECG sample rate using elapsed microseconds and sends it to the dashboard.

## ECG processing
The source implements four-read ADC averaging, a five-sample median filter, a 50 Hz notch filter, slow baseline tracking, and an ECG ring buffer. The heart-rate detector uses slope energy, a refractory interval, median RR intervals, and consistency checks before showing BPM. Reference accuracy testing remains pending. The handover records a known limitation with three-sample synthetic spike bursts; see TEST_STATUS.md.

## Motion processing
The MPU6050 is configured for plus/minus 4 g and sampled at a target of 50 Hz. A slow gravity estimate supports orientation; acceleration change supports step detection. Calibration records upright and front directions in ESP32 preferences. Posture uses a roughly 0.70 axis threshold and a stable candidate for about 0.8 seconds. Step logic uses 0.06 g threshold, 0.025 g re-arm, a 250 ms minimum interval, and begins counting after a five-step sequence. These are implementation parameters, not claims of measurement accuracy.

## Dashboard and network
| Route | Purpose |
|---|---|
| / | Dashboard page |
| /data?seq=... | JSON telemetry and ECG samples |
| /cmd?c=... | Calibration and step-reset commands |

Commands include up, front, resetcal, and resetsteps. Wi-Fi settings can come from a local secrets.h; otherwise the sketch uses fallback access-point settings. Keep credentials private and change the fallback access-point password before use.

The dashboard displays ECG, BPM, lead status, steps, posture, pitch/roll, accelerometer axes, Wi-Fi information, sample rate, and display delay.

## Touch and sleep
The source uses two taps within 700 ms to enter deep sleep. GPIO1 is configured as a wake source; after waking, a second tap is required within 1.5 seconds. The handover marks touch/sleep behavior done. Sleep-current measurement remains pending.