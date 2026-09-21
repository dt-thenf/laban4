# Algorithm notes

This document describes the implementation choices in laban4. The runtime heading engine is intentionally vector-based instead of depending on roll/pitch/yaw Euler angles.

## 1. Coordinate frame

Body axes are fixed by the project:

- +X: toward the wearer's head
- +Y: toward the wearer's left
- +Z: into the wearer's body
- -Z: forward direction whose azimuth is requested

The body frame is right-handed for the intended mounting.

The forward vector is:

~~~text
f = [0, 0, -1]^T
~~~

Sensor readings are first corrected in each sensor's native coordinates and then mapped into this body frame with a signed permutation.

## 2. Accelerometer and magnetometer calibration

A sphere measured by a distorted 3-axis sensor becomes an offset ellipsoid.

The firmware fits the algebraic form:

~~~text
x^T A x + b^T x = 1
~~~

where A is symmetric.

The unknown vector contains nine coefficients:

~~~text
[Axx, Ayy, Azz, Axy, Axz, Ayz, bx, by, bz]
~~~

For each sample, the feature row is:

~~~text
[x^2, y^2, z^2, 2xy, 2xz, 2yz, x, y, z]
~~~

The firmware accumulates the normal equations directly and solves the 9x9 system using pivoted Gauss-Jordan elimination.

The ellipsoid center is:

~~~text
c = -0.5 A^-1 b
~~~

After translating x = y + c:

~~~text
y^T Q y = 1

Q = A / (1 + c^T A c)
~~~

Q must be positive definite for a physically meaningful ellipsoid.

The 3x3 symmetric eigendecomposition is:

~~~text
Q = V D V^T
~~~

The correction matrix is:

~~~text
S = V sqrt(D) V^T
~~~

and the corrected sample is:

~~~text
x_corrected = S (x_raw - c)
~~~

For magnetometer data, this simultaneously removes the fitted hard-iron offset and symmetric soft-iron/cross-axis distortion.

The fit is rejected if:

- there are too few samples
- the sample cloud does not span all three axes sufficiently
- the quadratic is singular
- Q is not positive definite
- corrected-sphere RMS error is too large
- the correction condition number is excessive

## 3. Gyroscope calibration

ITG-3205 is configured for its +/-2000 degree/s full scale.

The nominal sensitivity used by the firmware is:

~~~text
14.375 LSB / (degree/s)
~~~

The module remains stationary while 500 samples are collected over approximately 5 seconds.

For each axis:

~~~text
bias = mean(raw)
noise = standard_deviation(raw) / 14.375
~~~

If measured noise exceeds the stationary threshold, calibration is rejected rather than saving a contaminated bias.

## 4. Gravity / up-vector estimation

The accelerometer gives an absolute gravity reference when quasi-static, but motion adds linear acceleration.

The gyroscope gives short-term rotational dynamics without relying on gravity, but its integrated estimate drifts.

laban4 therefore uses a complementary vector filter.

Let u be the world-up vector expressed in the body frame and omega the calibrated body angular rate.

A fixed world vector observed from a rotating body evolves as:

~~~text
du/dt = -omega x u
      =  u x omega
~~~

The gyro propagation step is:

~~~text
u <- normalize(u + (u x omega) dt)
~~~

The normalized accelerometer direction is then blended back toward u.

The accelerometer correction weight is reduced when the calibrated acceleration magnitude departs from 1 g, because that is evidence of dynamic linear acceleration.

## 5. Tilt-compensated north vector

Let m be the corrected magnetometer vector and u the unit up vector.

Remove the vertical magnetic component:

~~~text
n_h = m - u (m dot u)
n   = normalize(n_h)
~~~

This is the horizontal magnetic-north direction in body coordinates.

The same operation is applied to the required body forward vector f:

~~~text
f_h = f - u (f dot u)
f_p = normalize(f_h)
~~~

If the magnitude of f_h is nearly zero, the requested body direction is nearly vertical. Its azimuth is then mathematically ill-conditioned, so the firmware holds the last valid compass result.

## 6. Heading angle

With an ENU-style local horizontal basis:

~~~text
east = normalize(north x up)
~~~

The magnetic azimuth of body forward is:

~~~text
heading_mag = atan2(f_p dot east,
                    f_p dot north)
~~~

The result is wrapped to 0..360 degrees.

True heading is:

~~~text
heading_true = heading_mag + declination
~~~

The firmware uses the convention that east magnetic declination is positive.

## 7. Circular smoothing

A normal scalar low-pass filter fails around north. For example, averaging 359 degrees and 1 degree numerically gives 180 degrees.

laban4 filters the angle on the unit circle:

~~~text
cx = cos(heading)
sy = sin(heading)
~~~

The two components are low-pass filtered and renormalized. Heading is recovered with atan2.

The filter response becomes faster while measured angular speed is high.

## 8. Magnetic disturbance check

After successful ellipsoid calibration, magnetometer samples should have a corrected magnitude near one.

The runtime rejects grossly abnormal field magnitudes before updating heading. This does not guarantee freedom from magnetic interference: a local field can rotate the vector without changing its magnitude much. Mechanical placement and environmental testing remain essential.

## 9. Why there is no PC calibration utility

The stated design goal is that field calibration can be completed with only:

- ESP32
- connected GY-85
- Arduino Serial Monitor

Therefore all sample collection, ellipsoid fitting, quality checks, matrix generation and NVS persistence execute on the ESP32 itself.

## References

NXP/Freescale AN4248:
https://www.nxp.com/docs/en/application-note/AN4248.pdf

NXP/Freescale AN4246:
https://www.nxp.com/docs/en/application-note/AN4246.pdf

Analog Devices ADXL345 data sheet:
https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf

TDK/InvenSense ITG-3200 product specification:
https://invensense.tdk.com/wp-content/uploads/2015/02/ITG-3200-Datasheet.pdf

Honeywell HMC5883L data sheet mirror:
https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf
