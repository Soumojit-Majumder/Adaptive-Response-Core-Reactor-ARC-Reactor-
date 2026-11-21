# Build and Upload

## Environment
The firmware targets an ESP32-C3 Super Mini using the Arduino framework. The handover specifies Arduino IDE, ESP32C3 Dev Module, and USB CDC On Boot enabled. The prototype is currently running live, confirming a successful build and upload. The exact Arduino IDE version, ESP32 board-package version, build output, and upload log are not recorded here. The steps below document the project setup for repeat builds.

## Arduino IDE steps
1. Install Arduino IDE and Espressif ESP32 board support.
2. Open Firmware/ARC_Reactor_Sensor___Dashboard/ARC_Reactor_Sensor___Dashboard.ino.
3. Select ESP32C3 Dev Module and enable USB CDC On Boot where available.
4. Optionally add a local secrets.h beside the sketch with WIFI_SSID, WIFI_PASS, AP_SSID, and AP_PASS. Do not commit credentials.
5. Compile and record the toolchain and result before uploading.
6. Select the board’s serial port and upload. Open Serial Monitor at 115200 baud.

Without secrets.h, the sketch uses fallback access-point settings. Change the fallback password in the source before relying on that access point.

## Host simulation
From the repository root, run:

    .\Tests\run_host_simulation.ps1

This compiles selected algorithms with a host C++ compiler and checks dashboard JavaScript syntax. It does not compile or upload the ESP32 firmware.

## Build record
No complete ESP32-C3 build, upload, or firmware memory report is recorded in the repository. Record the date, firmware revision, Arduino IDE and ESP32 package versions, board options, compile output, upload result, and warnings when a build is completed.