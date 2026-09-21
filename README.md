# laban4

Tilt-compensated wearable compass firmware for **ESP32 + GY-85** (ADXL345 + ITG-3205/ITG-3200 + HMC5883L).

This repository is intentionally built from scratch. It does not depend on earlier compass repositories.

The body frame used by the project is:

- **+X**: toward the wearer's head
- **+Y**: toward the wearer's left side
- **+Z**: into the wearer's body
- **-Z**: forward / travel direction used for compass heading

Calibration is performed entirely on-device through Arduino Serial Monitor; no Python or desktop calibration utility is required.

> Firmware and full calibration instructions are added in the following commits.
