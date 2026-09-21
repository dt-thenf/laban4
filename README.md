# laban4

Self-calibrating, tilt-compensated wearable compass for **ESP32 + GY-85**.

GY-85 sensor set used by this firmware:

- ADXL345 accelerometer — I2C 0x53
- ITG-3205 / ITG-3200 gyroscope — I2C 0x68
- HMC5883L magnetometer — I2C 0x1E

The repository is built from scratch and does not depend on earlier compass repositories or PC-side calibration scripts.

## Body coordinate system

The whole project uses one explicit body frame:

~~~text
                     +X
              toward the head
                      ^
                      |
        +Y <----------+
     wearer's left    |
                      |
              +Z goes INTO the body

        -Z = FORWARD / travel direction
~~~

For heading, the direction vector is therefore fixed in code as:

~~~text
forward_body = [0, 0, -1]
~~~

The heading is the azimuth of **-Z**, not the azimuth of an arbitrary HMC5883L X/Y pair.

## Why this remains correct when tilted

A level-only compass often uses atan2(My, Mx). That fails when the module tilts because the magnetometer's vertical field component leaks into the two axes used for heading.

laban4 instead uses a vector formulation:

1. Estimate the local **up/gravity vector** from ADXL345, stabilized by ITG-3205 gyro propagation.
2. Correct HMC5883L data for hard-iron and soft-iron distortion.
3. Project the corrected magnetic vector onto the plane perpendicular to gravity to obtain horizontal magnetic north.
4. Project body forward vector **-Z** onto the same horizontal plane.
5. Compute the signed angle from north to projected forward.
6. Apply circular filtering so the 359° -> 0° boundary does not create a jump.
7. Add configurable magnetic declination if true-north heading is required.

This avoids Euler-angle singularities in the core compass calculation.

There is one unavoidable physical singularity: if **-Z itself points almost vertically up/down**, its horizontal projection approaches zero, so the azimuth of that direction is not well-defined. The firmware detects this and reports HDG=HOLD instead of producing a random angle.

## On-device calibration only

No Python, MATLAB, desktop fitting tool, CSV export, or external calibration program is required.

All calibration is done through **Arduino Serial Monitor** and saved in ESP32 NVS.

### Magnetometer

The firmware gathers 3D HMC5883L samples and performs an embedded ellipsoid least-squares fit:

~~~text
(x-c)^T Q (x-c) = 1
~~~

From that fit it obtains:

- hard-iron center offset c
- full symmetric 3x3 soft-iron / cross-axis correction matrix
- fit RMS error
- correction condition number

The fitted matrix maps the measured ellipsoid back toward a sphere.

### Accelerometer

ADXL345 is calibrated with the same 3D ellipsoid approach while the module is rotated slowly through all orientations. Obvious high-dynamic-acceleration samples are rejected before fitting.

### Gyroscope

ITG-3205 zero-rate bias is averaged while the module is stationary. The routine also measures standard deviation and rejects calibration if the module moved too much.

## Wiring: ESP32

Recommended direct 3.3 V wiring:

| GY-85 | ESP32 |
|---|---|
| 3.3V | 3V3 |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

Default pins can be changed at the top of laban4.ino:

~~~cpp
#define LABAN4_SDA 21
#define LABAN4_SCL 22
~~~

The firmware uses 400 kHz I2C.

## Arduino IDE

1. Install the ESP32 Arduino core.
2. Open laban4.ino.
3. Select the correct ESP32 board and COM port.
4. Upload.
5. Open Serial Monitor at **115200 baud** with newline enabled.

No third-party Arduino sensor library is required. The sketch only uses ESP32/Arduino core components: Arduino.h, Wire.h and Preferences.h.

## First startup

Run:

~~~text
scan
~~~

Expected devices:

~~~text
0x1E HMC5883L
0x53 ADXL345
0x68 ITG3205
~~~

Then run the full calibration:

~~~text
cal all
~~~

The sequence is:

1. Keep the module completely still for the gyro calibration.
2. After the prompt changes to 3D calibration, slowly rotate the module through as many orientations as possible for about 40 seconds.
3. Use broad figure-eight motions.
4. Make every positive and negative module axis point upward/downward at some point.
5. Keep the sensor away from steel tables, magnets, loudspeakers, motors, power transformers, large batteries and high-current wiring.

The accepted calibration is automatically written to NVS.

Check it with:

~~~text
status
~~~

## Serial commands

| Command | Function |
|---|---|
| help | command list |
| status | calibration flags, matrices, maps, settings |
| scan | I2C scan |
| raw | one raw and calibrated sensor sample |
| heading | print one current heading |
| stream on / stream off | continuous heading output |
| rate 10 | output rate from 1 to 50 Hz |
| decl 0.0 | east-positive magnetic declination in degrees |
| cal gyro | stationary gyro-bias calibration |
| cal accel | 30 s accelerometer ellipsoid calibration |
| cal mag | 40 s magnetometer ellipsoid calibration |
| cal all | complete gyro + accel + magnetometer calibration |
| map show | show signed sensor-to-body mappings |
| map accel +x +y +z | set accel signed axis permutation |
| map gyro +x +y +z | set gyro signed axis permutation |
| map mag +x +y +z | set magnetometer signed axis permutation |
| factory reset | restore defaults |

## Sensor-to-body axis mapping

The default assumes sensor axes follow the GY-85 module axes:

~~~text
body X = +sensor X
body Y = +sensor Y
body Z = +sensor Z
~~~

This is stored as:

~~~text
+x +y +z
~~~

Some GY-85 clones or board revisions may mount a sensor with a different signed permutation. You do **not** need to edit the source. Change it in Serial Monitor, for example:

~~~text
map mag +y -x +z
~~~

That means:

~~~text
body X = +mag Y
body Y = -mag X
body Z = +mag Z
~~~

Each of X/Y/Z must be used exactly once. After changing a map, run the corresponding calibration again.

## Setting magnetic declination

The compass naturally returns magnetic-north heading. To obtain true-north heading:

~~~text
decl <degrees>
~~~

Convention:

- east declination: positive
- west declination: negative

Example only:

~~~text
decl 1.25
~~~

Use the current declination for the actual operating location; do not copy the example value.

## Output example

~~~text
HDG=123.42 deg ESE  raw=124.01  A=1.004  M=0.996  forwardH=0.931  GAM
~~~

Where:

- HDG: filtered true/magnetic heading depending on decl
- raw: unfiltered heading before circular smoothing
- A: calibrated accelerometer magnitude, ideally near 1 g when quasi-static
- M: calibrated magnetometer magnitude, ideally near 1 after accepted mag fit
- forwardH: length of horizontal projection of body -Z
- GAM: gyro, accel and magnetometer calibrations are present

forwardH approaching zero means the requested forward axis is approaching vertical, where azimuth becomes ill-conditioned.

## Practical verification

After cal all:

1. Keep the module away from metal.
2. Point body **-Z** toward a known north reference.
3. Note heading.
4. Tilt/roll the module substantially while keeping the **horizontal projection of -Z** aimed in the same direction.
5. The heading should remain close to the original value.
6. Repeat facing east, south and west.
7. If heading changes strongly with tilt, first inspect A, M, axis maps and magnetic surroundings before changing filtering constants.

## Important limits

- HMC5883L is an older magnetometer and many low-cost modules contain clones or substituted parts.
- This firmware expects an HMC5883L-compatible device at 0x1E. QMC5883L devices commonly found at 0x0D are a different chip/register map and are not silently treated as HMC5883L.
- Nearby ferromagnetic material can invalidate any compass, even after a good calibration.
- Calibration should be performed with the sensor installed in its final wearable assembly, because screws, batteries and wiring can change the magnetic distortion.
- Strong linear acceleration temporarily corrupts gravity inferred from the accelerometer. The gyro propagation reduces that effect, but an old GY-85 cannot match the dynamic performance of a modern factory-calibrated phone IMU.
- Heading of the chosen body direction is mathematically undefined when that direction is vertical. The code explicitly detects this geometry.

## Technical references

The implementation follows the physical model used in established eCompass literature:

- NXP/Freescale AN4248, Implementing a Tilt-Compensated eCompass using Accelerometer and Magnetometer Sensors:
  https://www.nxp.com/docs/en/application-note/AN4248.pdf
- NXP/Freescale AN4246, Calibrating an eCompass in the Presence of Hard- and Soft-Iron Interference:
  https://www.nxp.com/docs/en/application-note/AN4246.pdf
- Analog Devices ADXL345 data sheet:
  https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf
- TDK/InvenSense ITG-3200 product specification:
  https://invensense.tdk.com/wp-content/uploads/2015/02/ITG-3200-Datasheet.pdf
- Honeywell HMC5883L data sheet mirror:
  https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf

See docs/ALGORITHM.md for the equations implemented in the sketch.
