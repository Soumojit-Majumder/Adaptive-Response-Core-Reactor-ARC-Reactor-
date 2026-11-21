# Hardware and Wiring

## Modules and connections
| Module | Role | Connection recorded in the firmware/handover |
|---|---|---|
| ESP32-C3 Super Mini | Controller, Wi-Fi, ADC and GPIO | Board-specific pin exposure must be checked |
| AD8232 ECG module | Analog ECG front end | OUTPUT GPIO3; LO+ GPIO6; LO- GPIO7; SDN to 3V3; power 3V3 and GND |
| MPU6050 / GY-521 | Accelerometer for motion and orientation | SDA GPIO4; SCL GPIO5; AD0 GND (address 0x68); power 3V3 and GND |
| TTP223 touch module | Touch input | I/O GPIO1; power 3V3 and GND |
| WS2812 / NeoPixel blue pixel | Active status indicator | DIN through 330 ohm resistor to GPIO10; 5 V and GND |
| Three Ag/AgCl electrode pads and cable | ECG signal pickup | Connect to the AD8232 electrode cable |

These assignments come from the sketch and handover. Confirm exact board and breakout revisions before wiring; variants can differ.

## Power
The handover describes powering the prototype from a 5 V battery bank or battery supply into the board. It does not identify a verified cell, protection circuit, converter, measured current, or runtime. The power-bank output test remains pending.

Check the exact WS2812 input-level and current requirements. Do not assume a 3.3 V GPIO meets the data-input threshold when the LED is powered at 5 V.

## Bring-up status
The handover marks power/sensor bring-up as done. The detailed test matrix is in TEST_STATUS.md. Record board and module revisions, wiring, supply, firmware revision, and observations for reproducibility.

## Safety
- Educational prototype only; not for diagnosis, treatment, emergency response, or health decisions.
- The AD8232 breakout is not medical isolation. Do not attach electrodes to a person while the prototype is connected to a computer, charger, mains-powered equipment, or other unisolated equipment.
- Human testing requires a properly isolated, battery-only setup and qualified guidance.
- Verify supply voltage, GPIO limits, and module current requirements.
- Li-ion/LiPo cells can fail or burn if shorted, damaged, overcharged, or charged incorrectly. Use compatible protection and charging. Stop using damaged, swollen, hot, or leaking cells.
- Do not wear the prototype if conductors are exposed, the enclosure is uncomfortable, or any part heats unexpectedly.