/*
  laban4 - ESP32 + GY-85 wearable tilt-compensated compass

  GY-85 sensors:
    ADXL345  : 3-axis accelerometer, I2C 0x53
    ITG-3205 : 3-axis gyroscope,     I2C 0x68
    HMC5883L : 3-axis magnetometer,  I2C 0x1E

  Project body frame:
    +X = toward wearer's head
    +Y = toward wearer's left
    +Z = into wearer's body
    -Z = forward / travel direction

  Heading is NOT computed from atan2(mx,my) on a level board. Instead:
    1) gravity/up is estimated from accelerometer + gyro complementary fusion,
    2) magnetic field is projected onto the gravity-horizontal plane,
    3) body forward (-Z) is projected onto the same plane,
    4) heading is the signed angle from magnetic north to projected forward.

  Calibration is fully on-device through Serial Monitor:
    cal gyro
    cal accel
    cal mag
    cal all

  No external Python/tool is required.

  Target: ESP32 Arduino core.
*/

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>
#include <float.h>
#include <stddef.h>

#ifndef LABAN4_SDA
#define LABAN4_SDA 21
#endif

#ifndef LABAN4_SCL
#define LABAN4_SCL 22
#endif

static constexpr uint32_t SERIAL_BAUD = 115200;
static constexpr uint8_t ADXL345_ADDR = 0x53;
static constexpr uint8_t ITG3205_ADDR = 0x68;
static constexpr uint8_t HMC5883L_ADDR = 0x1E;

static constexpr float DEG2RAD_F = 0.01745329251994329577f;
static constexpr float RAD2DEG_F = 57.295779513082320876f;
static constexpr float ITG3205_LSB_PER_DPS = 14.375f;

// 25 Hz * 40 s = 1000 samples. Capacity leaves margin.
static constexpr int CAL_MAX_SAMPLES = 1200;

struct Raw3 {
  int16_t x;
  int16_t y;
  int16_t z;
};

struct Vec3 {
  float x;
  float y;
  float z;
};

static inline Vec3 v3(float x, float y, float z) { return {x, y, z}; }
static inline Vec3 add3(const Vec3 &a, const Vec3 &b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
static inline Vec3 sub3(const Vec3 &a, const Vec3 &b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
static inline Vec3 mul3(const Vec3 &a, float s) { return {a.x*s, a.y*s, a.z*s}; }
static inline float dot3(const Vec3 &a, const Vec3 &b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline Vec3 cross3(const Vec3 &a, const Vec3 &b) {
  return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}
static inline float norm3(const Vec3 &a) { return sqrtf(dot3(a, a)); }
static inline Vec3 normalize3(const Vec3 &a) {
  const float n = norm3(a);
  return (n > 1.0e-9f) ? mul3(a, 1.0f/n) : v3(0,0,0);
}
static inline float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : ((x > hi) ? hi : x);
}
static inline float wrap360(float d) {
  while (d < 0.0f) d += 360.0f;
  while (d >= 360.0f) d -= 360.0f;
  return d;
}

enum CalFlags : uint16_t {
  CAL_GYRO  = 1u << 0,
  CAL_ACCEL = 1u << 1,
  CAL_MAG   = 1u << 2
};

struct CalibrationData {
  uint32_t magic;
  uint16_t version;
  uint16_t flags;

  // raw gyro counts at zero angular rate
  float gyroBias[3];

  // Corrected vector = matrix * (raw - center).
  // For accel and mag, a successful ellipsoid calibration maps samples to
  // approximately a unit sphere.
  float accelCenter[3];
  float accelMatrix[9];
  float magCenter[3];
  float magMatrix[9];

  // Signed-permutation mapping from each sensor's corrected axes into body axes.
  // Example {+1,+2,+3}: body X=+sensor X, body Y=+sensor Y, body Z=+sensor Z.
  // Values are +/-1, +/-2, +/-3 and absolute values must be unique.
  int8_t accelMap[3];
  int8_t gyroMap[3];
  int8_t magMap[3];
  int8_t reservedMap;

  // East-positive declination. true_heading = magnetic_heading + declination.
  float declinationDeg;

  uint16_t outputHz;
  uint16_t reserved;

  uint32_t crc;
};

static constexpr uint32_t CAL_MAGIC = 0x4C423434UL; // "LB44"
static constexpr uint16_t CAL_VERSION = 4;

CalibrationData cal;
Preferences prefs;

Raw3 accelCalSamples[CAL_MAX_SAMPLES];
Raw3 magCalSamples[CAL_MAX_SAMPLES];

bool sensorsReady = false;
bool streamEnabled = true;
bool upInitialized = false;
bool magFilterInitialized = false;
bool headingFilterInitialized = false;

Vec3 upEstimate = {1,0,0};
Vec3 magFiltered = {0,0,0};
float smoothCos = 1.0f;
float smoothSin = 0.0f;
float lastHeadingDeg = NAN;
float lastRawHeadingDeg = NAN;
float lastAccelNorm = NAN;
float lastMagNorm = NAN;
float lastForwardHorizontal = NAN;
bool lastHeadingValid = false;
uint32_t lastUpdateUs = 0;
uint32_t lastPrintMs = 0;
String serialLine;

// ---------- persistent calibration ----------

uint32_t crc32Bytes(const uint8_t *data, size_t len) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; ++b) {
      crc = (crc >> 1) ^ (0xEDB88320UL & (-(int32_t)(crc & 1)));
    }
  }
  return ~crc;
}

void setIdentityMap(int8_t m[3]) {
  m[0] = +1;
  m[1] = +2;
  m[2] = +3;
}

void setDiag(float M[9], float d0, float d1, float d2) {
  for (int i = 0; i < 9; ++i) M[i] = 0.0f;
  M[0] = d0;
  M[4] = d1;
  M[8] = d2;
}

void setCalibrationDefaults() {
  memset(&cal, 0, sizeof(cal));
  cal.magic = CAL_MAGIC;
  cal.version = CAL_VERSION;
  cal.flags = 0;

  // ADXL345 full-resolution scale is nominally about 256 counts/g.
  setDiag(cal.accelMatrix, 1.0f/256.0f, 1.0f/256.0f, 1.0f/256.0f);

  // HMC5883L default gain is nominally 1090 LSB/Gauss. This uncalibrated
  // fallback gives a physically scaled vector; ellipsoid calibration replaces it.
  setDiag(cal.magMatrix, 1.0f/1090.0f, 1.0f/1090.0f, 1.0f/1090.0f);

  setIdentityMap(cal.accelMap);
  setIdentityMap(cal.gyroMap);
  setIdentityMap(cal.magMap);

  cal.declinationDeg = 0.0f;
  cal.outputHz = 10;
}

void refreshCalibrationCRC() {
  cal.crc = crc32Bytes(reinterpret_cast<const uint8_t*>(&cal),
                       offsetof(CalibrationData, crc));
}

bool saveCalibration() {
  cal.magic = CAL_MAGIC;
  cal.version = CAL_VERSION;
  refreshCalibrationCRC();

  if (!prefs.begin("laban4", false)) return false;
  const size_t written = prefs.putBytes("cal", &cal, sizeof(cal));
  prefs.end();
  return written == sizeof(cal);
}

bool loadCalibration() {
  CalibrationData tmp;
  if (!prefs.begin("laban4", true)) {
    setCalibrationDefaults();
    return false;
  }

  const size_t n = prefs.getBytesLength("cal");
  if (n != sizeof(tmp)) {
    prefs.end();
    setCalibrationDefaults();
    return false;
  }

  prefs.getBytes("cal", &tmp, sizeof(tmp));
  prefs.end();

  const uint32_t expected = crc32Bytes(reinterpret_cast<const uint8_t*>(&tmp),
                                       offsetof(CalibrationData, crc));
  if (tmp.magic != CAL_MAGIC || tmp.version != CAL_VERSION || tmp.crc != expected) {
    setCalibrationDefaults();
    return false;
  }

  cal = tmp;
  if (cal.outputHz < 1 || cal.outputHz > 50) cal.outputHz = 10;
  return true;
}

// ---------- I2C / sensor drivers ----------

bool i2cPing(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  const size_t got = Wire.requestFrom((int)addr, (int)n, (int)true);
  if (got != n) {
    while (Wire.available()) (void)Wire.read();
    return false;
  }
  for (size_t i = 0; i < n; ++i) buf[i] = (uint8_t)Wire.read();
  return true;
}

bool initADXL345() {
  uint8_t id = 0;
  if (!readRegs(ADXL345_ADDR, 0x00, &id, 1)) return false;
  if (id != 0xE5) {
    Serial.printf("[WARN] ADXL345 DEVID=0x%02X, expected 0xE5\n", id);
  }

  if (!writeReg(ADXL345_ADDR, 0x2D, 0x00)) return false; // standby
  if (!writeReg(ADXL345_ADDR, 0x31, 0x09)) return false; // full-res, +/-4 g
  if (!writeReg(ADXL345_ADDR, 0x2C, 0x0A)) return false; // 100 Hz
  if (!writeReg(ADXL345_ADDR, 0x2D, 0x08)) return false; // measure
  delay(10);
  return true;
}

bool initITG3205() {
  if (!i2cPing(ITG3205_ADDR)) return false;

  writeReg(ITG3205_ADDR, 0x3E, 0x80); // device reset
  delay(60);
  if (!writeReg(ITG3205_ADDR, 0x3E, 0x01)) return false; // PLL with X gyro ref
  if (!writeReg(ITG3205_ADDR, 0x15, 9)) return false;    // 1 kHz/(9+1)=100 Hz
  if (!writeReg(ITG3205_ADDR, 0x16, 0x1B)) return false; // FS=2000 dps, DLPF=42 Hz
  delay(10);
  return true;
}

bool initHMC5883L() {
  if (!i2cPing(HMC5883L_ADDR)) return false;

  uint8_t id[3] = {0,0,0};
  if (readRegs(HMC5883L_ADDR, 0x0A, id, 3)) {
    if (!(id[0] == 'H' && id[1] == '4' && id[2] == '3')) {
      Serial.printf("[WARN] HMC5883L ID='%c%c%c'. Clone/compatible device may behave differently.\n",
                    id[0], id[1], id[2]);
    }
  }

  if (!writeReg(HMC5883L_ADDR, 0x00, 0x78)) return false; // 8-average, 75 Hz, normal
  if (!writeReg(HMC5883L_ADDR, 0x01, 0x20)) return false; // +/-1.3 G, 1090 LSB/G
  if (!writeReg(HMC5883L_ADDR, 0x02, 0x00)) return false; // continuous mode
  delay(20);
  return true;
}

bool initSensors() {
  const bool a = initADXL345();
  const bool g = initITG3205();
  const bool m = initHMC5883L();

  Serial.printf("ADXL345 : %s\n", a ? "OK" : "FAIL");
  Serial.printf("ITG3205 : %s\n", g ? "OK" : "FAIL");
  Serial.printf("HMC5883L: %s\n", m ? "OK" : "FAIL");

  sensorsReady = a && g && m;
  return sensorsReady;
}

bool readAccelRaw(Raw3 &r) {
  uint8_t b[6];
  if (!readRegs(ADXL345_ADDR, 0x32, b, 6)) return false;
  r.x = (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
  r.y = (int16_t)((uint16_t)b[2] | ((uint16_t)b[3] << 8));
  r.z = (int16_t)((uint16_t)b[4] | ((uint16_t)b[5] << 8));
  return true;
}

bool readGyroRaw(Raw3 &r) {
  uint8_t b[6];
  if (!readRegs(ITG3205_ADDR, 0x1D, b, 6)) return false;
  r.x = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
  r.y = (int16_t)(((uint16_t)b[2] << 8) | b[3]);
  r.z = (int16_t)(((uint16_t)b[4] << 8) | b[5]);
  return true;
}

bool readMagRaw(Raw3 &r) {
  uint8_t b[6];
  if (!readRegs(HMC5883L_ADDR, 0x03, b, 6)) return false;

  // HMC5883L register order is X, Z, Y (big endian).
  r.x = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
  r.z = (int16_t)(((uint16_t)b[2] << 8) | b[3]);
  r.y = (int16_t)(((uint16_t)b[4] << 8) | b[5]);

  // HMC5883L reports -4096 on overflow/saturation.
  if (r.x == -4096 || r.y == -4096 || r.z == -4096) return false;
  return true;
}

// ---------- calibration application / axis mapping ----------

Vec3 applyMatrixCalibration(const Raw3 &r, const float center[3], const float M[9]) {
  const float d0 = (float)r.x - center[0];
  const float d1 = (float)r.y - center[1];
  const float d2 = (float)r.z - center[2];
  return {
    M[0]*d0 + M[1]*d1 + M[2]*d2,
    M[3]*d0 + M[4]*d1 + M[5]*d2,
    M[6]*d0 + M[7]*d1 + M[8]*d2
  };
}

Vec3 applyAxisMap(const Vec3 &s, const int8_t map[3]) {
  const float v[3] = {s.x, s.y, s.z};
  float out[3] = {0,0,0};
  for (int i = 0; i < 3; ++i) {
    const int8_t code = map[i];
    const int idx = abs((int)code) - 1;
    out[i] = ((code >= 0) ? 1.0f : -1.0f) * v[idx];
  }
  return {out[0], out[1], out[2]};
}

Vec3 calibratedAccelBody(const Raw3 &r) {
  return applyAxisMap(applyMatrixCalibration(r, cal.accelCenter, cal.accelMatrix),
                      cal.accelMap);
}

Vec3 calibratedMagBody(const Raw3 &r) {
  return applyAxisMap(applyMatrixCalibration(r, cal.magCenter, cal.magMatrix),
                      cal.magMap);
}

Vec3 calibratedGyroBody(const Raw3 &r) {
  Vec3 s = {
    ((float)r.x - cal.gyroBias[0]) / ITG3205_LSB_PER_DPS,
    ((float)r.y - cal.gyroBias[1]) / ITG3205_LSB_PER_DPS,
    ((float)r.z - cal.gyroBias[2]) / ITG3205_LSB_PER_DPS
  };
  return applyAxisMap(s, cal.gyroMap);
}

// ---------- full 3D ellipsoid fit ----------
// Algebraic fit:
//   x^T A x + b^T x = 1
// Then center c = -0.5 A^-1 b.
// With y=x-c: y^T Q y = 1, Q=A/(1+c^T A c).
// The correction sqrt(Q) maps the ellipsoid to a sphere.
//
// This captures hard-iron offset plus a symmetric 3x3 soft-iron/cross-axis
// correction without requiring a PC-side fitter.

bool solve9(double aug[9][10], double out[9]) {
  for (int col = 0; col < 9; ++col) {
    int pivot = col;
    double best = fabs(aug[pivot][col]);
    for (int r = col + 1; r < 9; ++r) {
      const double v = fabs(aug[r][col]);
      if (v > best) { best = v; pivot = r; }
    }
    if (best < 1.0e-14) return false;

    if (pivot != col) {
      for (int c = col; c < 10; ++c) {
        const double tmp = aug[col][c];
        aug[col][c] = aug[pivot][c];
        aug[pivot][c] = tmp;
      }
    }

    const double d = aug[col][col];
    for (int c = col; c < 10; ++c) aug[col][c] /= d;

    for (int r = 0; r < 9; ++r) {
      if (r == col) continue;
      const double f = aug[r][col];
      if (fabs(f) < 1.0e-20) continue;
      for (int c = col; c < 10; ++c) aug[r][c] -= f * aug[col][c];
    }
  }

  for (int i = 0; i < 9; ++i) out[i] = aug[i][9];
  return true;
}

bool invert3(const double A[3][3], double inv[3][3]) {
  const double det =
      A[0][0]*(A[1][1]*A[2][2] - A[1][2]*A[2][1])
    - A[0][1]*(A[1][0]*A[2][2] - A[1][2]*A[2][0])
    + A[0][2]*(A[1][0]*A[2][1] - A[1][1]*A[2][0]);

  if (fabs(det) < 1.0e-14) return false;
  const double id = 1.0 / det;

  inv[0][0] =  (A[1][1]*A[2][2] - A[1][2]*A[2][1]) * id;
  inv[0][1] = -(A[0][1]*A[2][2] - A[0][2]*A[2][1]) * id;
  inv[0][2] =  (A[0][1]*A[1][2] - A[0][2]*A[1][1]) * id;
  inv[1][0] = -(A[1][0]*A[2][2] - A[1][2]*A[2][0]) * id;
  inv[1][1] =  (A[0][0]*A[2][2] - A[0][2]*A[2][0]) * id;
  inv[1][2] = -(A[0][0]*A[1][2] - A[0][2]*A[1][0]) * id;
  inv[2][0] =  (A[1][0]*A[2][1] - A[1][1]*A[2][0]) * id;
  inv[2][1] = -(A[0][0]*A[2][1] - A[0][1]*A[2][0]) * id;
  inv[2][2] =  (A[0][0]*A[1][1] - A[0][1]*A[1][0]) * id;
  return true;
}

void jacobiEigenSym3(const double src[3][3], double V[3][3], double eval[3]) {
  double A[3][3];
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      A[r][c] = src[r][c];
      V[r][c] = (r == c) ? 1.0 : 0.0;
    }
  }

  for (int iter = 0; iter < 32; ++iter) {
    int p = 0, q = 1;
    double mx = fabs(A[0][1]);
    if (fabs(A[0][2]) > mx) { p = 0; q = 2; mx = fabs(A[0][2]); }
    if (fabs(A[1][2]) > mx) { p = 1; q = 2; mx = fabs(A[1][2]); }
    if (mx < 1.0e-13) break;

    const double app = A[p][p];
    const double aqq = A[q][q];
    const double apq = A[p][q];

    const double theta = 0.5 * atan2(2.0*apq, aqq-app);
    const double c = cos(theta);
    const double s = sin(theta);

    for (int k = 0; k < 3; ++k) {
      if (k == p || k == q) continue;
      const double akp = A[k][p];
      const double akq = A[k][q];
      const double nkp = c*akp - s*akq;
      const double nkq = s*akp + c*akq;
      A[k][p] = A[p][k] = nkp;
      A[k][q] = A[q][k] = nkq;
    }

    A[p][p] = c*c*app - 2.0*s*c*apq + s*s*aqq;
    A[q][q] = s*s*app + 2.0*s*c*apq + c*c*aqq;
    A[p][q] = A[q][p] = 0.0;

    for (int k = 0; k < 3; ++k) {
      const double vkp = V[k][p];
      const double vkq = V[k][q];
      V[k][p] = c*vkp - s*vkq;
      V[k][q] = s*vkp + c*vkq;
    }
  }

  eval[0] = A[0][0];
  eval[1] = A[1][1];
  eval[2] = A[2][2];
}

bool fitEllipsoid(const Raw3 *samples, int n, float normalization,
                  float centerOut[3], float matrixOut[9],
                  float &rmsOut, float &conditionOut) {
  if (n < 120) return false;

  double normal[9][9] = {};
  double rhs[9] = {};

  for (int i = 0; i < n; ++i) {
    const double x = (double)samples[i].x / normalization;
    const double y = (double)samples[i].y / normalization;
    const double z = (double)samples[i].z / normalization;
    const double f[9] = {
      x*x, y*y, z*z,
      2.0*x*y, 2.0*x*z, 2.0*y*z,
      x, y, z
    };

    for (int r = 0; r < 9; ++r) {
      rhs[r] += f[r];
      for (int c = 0; c < 9; ++c) normal[r][c] += f[r]*f[c];
    }
  }

  double aug[9][10];
  for (int r = 0; r < 9; ++r) {
    for (int c = 0; c < 9; ++c) aug[r][c] = normal[r][c];
    // Tiny regularization against nearly singular sample sets.
    aug[r][r] += 1.0e-10;
    aug[r][9] = rhs[r];
  }

  double p[9];
  if (!solve9(aug, p)) return false;

  double A[3][3] = {
    {p[0], p[3], p[4]},
    {p[3], p[1], p[5]},
    {p[4], p[5], p[2]}
  };
  const double b[3] = {p[6], p[7], p[8]};

  double invA[3][3];
  if (!invert3(A, invA)) return false;

  double c0[3] = {0,0,0};
  for (int r = 0; r < 3; ++r) {
    c0[r] = -0.5 * (invA[r][0]*b[0] + invA[r][1]*b[1] + invA[r][2]*b[2]);
  }

  double Ac[3] = {
    A[0][0]*c0[0] + A[0][1]*c0[1] + A[0][2]*c0[2],
    A[1][0]*c0[0] + A[1][1]*c0[1] + A[1][2]*c0[2],
    A[2][0]*c0[0] + A[2][1]*c0[1] + A[2][2]*c0[2]
  };
  const double cAc = c0[0]*Ac[0] + c0[1]*Ac[1] + c0[2]*Ac[2];
  const double k = 1.0 + cAc;
  if (!isfinite(k) || fabs(k) < 1.0e-12) return false;

  double Q[3][3];
  for (int r = 0; r < 3; ++r)
    for (int cc = 0; cc < 3; ++cc)
      Q[r][cc] = A[r][cc] / k;

  double V[3][3];
  double eval[3];
  jacobiEigenSym3(Q, V, eval);

  double emin = DBL_MAX;
  double emax = 0.0;
  for (int i = 0; i < 3; ++i) {
    if (!isfinite(eval[i]) || eval[i] <= 1.0e-10) return false;
    if (eval[i] < emin) emin = eval[i];
    if (eval[i] > emax) emax = eval[i];
  }

  // S = V * sqrt(D) * V^T.
  double S[3][3] = {};
  for (int r = 0; r < 3; ++r) {
    for (int cc = 0; cc < 3; ++cc) {
      double s = 0.0;
      for (int j = 0; j < 3; ++j) {
        s += V[r][j] * sqrt(eval[j]) * V[cc][j];
      }
      S[r][cc] = s;
    }
  }

  for (int i = 0; i < 3; ++i) centerOut[i] = (float)(c0[i] * normalization);
  for (int r = 0; r < 3; ++r) {
    for (int cc = 0; cc < 3; ++cc) {
      matrixOut[3*r+cc] = (float)(S[r][cc] / normalization);
    }
  }

  double err2 = 0.0;
  for (int i = 0; i < n; ++i) {
    const double dx = samples[i].x - centerOut[0];
    const double dy = samples[i].y - centerOut[1];
    const double dz = samples[i].z - centerOut[2];
    const double vx = matrixOut[0]*dx + matrixOut[1]*dy + matrixOut[2]*dz;
    const double vy = matrixOut[3]*dx + matrixOut[4]*dy + matrixOut[5]*dz;
    const double vz = matrixOut[6]*dx + matrixOut[7]*dy + matrixOut[8]*dz;
    const double nr = sqrt(vx*vx + vy*vy + vz*vz);
    const double e = nr - 1.0;
    err2 += e*e;
  }

  rmsOut = (float)sqrt(err2 / n);
  conditionOut = (float)sqrt(emax / emin);
  return isfinite(rmsOut) && isfinite(conditionOut);
}

void sampleSpans(const Raw3 *s, int n, int32_t span[3]) {
  int16_t mn[3] = {INT16_MAX, INT16_MAX, INT16_MAX};
  int16_t mx[3] = {INT16_MIN, INT16_MIN, INT16_MIN};
  for (int i = 0; i < n; ++i) {
    const int16_t v[3] = {s[i].x, s[i].y, s[i].z};
    for (int a = 0; a < 3; ++a) {
      if (v[a] < mn[a]) mn[a] = v[a];
      if (v[a] > mx[a]) mx[a] = v[a];
    }
  }
  for (int a = 0; a < 3; ++a) span[a] = (int32_t)mx[a] - (int32_t)mn[a];
}

// ---------- calibration routines ----------

bool calibrateGyro() {
  Serial.println();
  Serial.println("=== GYRO CALIBRATION ===");
  Serial.println("Keep the module completely STILL for 5 seconds.");
  delay(1000);

  const int N = 500;
  double sum[3] = {0,0,0};
  double sum2[3] = {0,0,0};
  int good = 0;

  for (int i = 0; i < N; ++i) {
    Raw3 r;
    if (readGyroRaw(r)) {
      const double v[3] = {(double)r.x, (double)r.y, (double)r.z};
      for (int a = 0; a < 3; ++a) {
        sum[a] += v[a];
        sum2[a] += v[a]*v[a];
      }
      ++good;
    }
    if ((i % 100) == 0) Serial.printf("  %d%%\n", (100*i)/N);
    delay(10);
  }

  if (good < N*9/10) {
    Serial.println("[FAIL] Too many gyro I2C read errors.");
    return false;
  }

  float mean[3], sdDps[3];
  bool stable = true;
  for (int a = 0; a < 3; ++a) {
    mean[a] = (float)(sum[a] / good);
    double var = sum2[a]/good - (sum[a]/good)*(sum[a]/good);
    if (var < 0) var = 0;
    sdDps[a] = (float)(sqrt(var) / ITG3205_LSB_PER_DPS);
    if (sdDps[a] > 2.5f) stable = false;
  }

  Serial.printf("bias raw: X=%.2f Y=%.2f Z=%.2f\n", mean[0], mean[1], mean[2]);
  Serial.printf("noise sd: X=%.2f Y=%.2f Z=%.2f deg/s\n", sdDps[0], sdDps[1], sdDps[2]);

  if (!stable) {
    Serial.println("[FAIL] Module moved during calibration. Retry on a stable surface.");
    return false;
  }

  memcpy(cal.gyroBias, mean, sizeof(mean));
  cal.flags |= CAL_GYRO;
  if (!saveCalibration()) {
    Serial.println("[WARN] Calibration computed but NVS save failed.");
  }
  Serial.println("[OK] Gyro bias saved.");
  return true;
}

bool calibrateVectors(bool doAccel, bool doMag, uint32_t durationMs) {
  Serial.println();
  Serial.println("=== 3D VECTOR CALIBRATION ===");
  if (doAccel && doMag) {
    Serial.println("Accelerometer + magnetometer will be calibrated together.");
  } else if (doAccel) {
    Serial.println("Accelerometer calibration.");
  } else {
    Serial.println("Magnetometer calibration.");
  }
  Serial.println("Rotate the module SLOWLY through every orientation.");
  Serial.println("Use large 3D figure-eight motions and make every +/- axis point up/down.");
  Serial.println("Keep away from steel, magnets, speakers, motors and high-current wiring.");
  Serial.println("Starting in 3 seconds...");
  delay(3000);

  int na = 0, nm = 0;
  const uint32_t t0 = millis();
  uint32_t lastProgress = 0;

  while ((millis() - t0) < durationMs && (na < CAL_MAX_SAMPLES || nm < CAL_MAX_SAMPLES)) {
    if (doAccel && na < CAL_MAX_SAMPLES) {
      Raw3 a;
      if (readAccelRaw(a)) {
        // Nominal static magnitude is around 256 counts. Discard obvious motion shocks.
        const float nn = sqrtf((float)a.x*a.x + (float)a.y*a.y + (float)a.z*a.z);
        if (nn > 150.0f && nn < 370.0f) accelCalSamples[na++] = a;
      }
    }

    if (doMag && nm < CAL_MAX_SAMPLES) {
      Raw3 m;
      if (readMagRaw(m)) magCalSamples[nm++] = m;
    }

    const uint32_t elapsed = millis() - t0;
    if (elapsed - lastProgress >= 1000) {
      lastProgress = elapsed;
      const int pct = (int)((100UL * elapsed) / durationMs);
      Serial.printf("  %d%%  accel=%d  mag=%d\n", pct, na, nm);
    }

    delay(40); // ~25 Hz
  }

  bool anySaved = false;

  if (doAccel) {
    int32_t span[3] = {0,0,0};
    sampleSpans(accelCalSamples, na, span);
    Serial.printf("Accel span raw: X=%ld Y=%ld Z=%ld\n",
                  (long)span[0], (long)span[1], (long)span[2]);

    float center[3], M[9], rms=0, cond=0;
    const bool coverage = span[0] > 330 && span[1] > 330 && span[2] > 330;
    const bool fit = coverage && fitEllipsoid(accelCalSamples, na, 256.0f,
                                               center, M, rms, cond);
    Serial.printf("Accel fit: samples=%d rms=%.4f condition=%.2f\n", na, rms, cond);

    if (fit && rms < 0.14f && cond < 3.0f) {
      memcpy(cal.accelCenter, center, sizeof(center));
      memcpy(cal.accelMatrix, M, sizeof(M));
      cal.flags |= CAL_ACCEL;
      anySaved = true;
      Serial.println("[OK] Accelerometer ellipsoid calibration accepted.");
    } else {
      Serial.println("[FAIL] Accelerometer coverage/fit is poor. Rotate more slowly through ALL axes and retry.");
    }
  }

  if (doMag) {
    int32_t span[3] = {0,0,0};
    sampleSpans(magCalSamples, nm, span);
    Serial.printf("Mag span raw: X=%ld Y=%ld Z=%ld\n",
                  (long)span[0], (long)span[1], (long)span[2]);

    float center[3], M[9], rms=0, cond=0;
    const bool coverage = span[0] > 260 && span[1] > 260 && span[2] > 260;
    const bool fit = coverage && fitEllipsoid(magCalSamples, nm, 500.0f,
                                               center, M, rms, cond);
    Serial.printf("Mag fit: samples=%d rms=%.4f condition=%.2f\n", nm, rms, cond);

    if (fit && rms < 0.18f && cond < 8.0f) {
      memcpy(cal.magCenter, center, sizeof(center));
      memcpy(cal.magMatrix, M, sizeof(M));
      cal.flags |= CAL_MAG;
      anySaved = true;
      Serial.println("[OK] Magnetometer hard/soft-iron ellipsoid calibration accepted.");
    } else {
      Serial.println("[FAIL] Magnetometer coverage/fit is poor. Move away from magnetic interference and retry.");
    }
  }

  if (anySaved) {
    if (saveCalibration()) Serial.println("[OK] Calibration saved to ESP32 NVS.");
    else Serial.println("[WARN] Calibration computed but NVS save failed.");
  }
  return anySaved;
}

void calibrateAll() {
  Serial.println();
  Serial.println("========== LABAN4 FULL CALIBRATION ==========");
  Serial.println("Step 1/2: gyro zero-rate bias.");
  if (!calibrateGyro()) {
    Serial.println("[STOP] Fix gyro calibration before continuing.");
    return;
  }
  Serial.println();
  Serial.println("Step 2/2: accel + magnetometer 3D ellipsoid calibration.");
  calibrateVectors(true, true, 40000);
  Serial.println("========== CALIBRATION FINISHED ==========");
}

// ---------- heading engine ----------

const char* cardinal(float h) {
  static const char* c[16] = {
    "N","NNE","NE","ENE","E","ESE","SE","SSE",
    "S","SSW","SW","WSW","W","WNW","NW","NNW"
  };
  int idx = (int)floorf((wrap360(h) + 11.25f) / 22.5f) & 15;
  return c[idx];
}

bool computeHeading(float dt, const Raw3 &ar, const Raw3 &gr, const Raw3 &mr) {
  const Vec3 a = calibratedAccelBody(ar);
  const Vec3 gDps = calibratedGyroBody(gr);
  const Vec3 m = calibratedMagBody(mr);

  const float aNorm = norm3(a);
  const float mNorm = norm3(m);
  lastAccelNorm = aNorm;
  lastMagNorm = mNorm;

  if (!upInitialized && aNorm > 0.2f) {
    upEstimate = normalize3(a);
    upInitialized = true;
  }

  if (!upInitialized) return false;

  // Gyro propagates the gravity vector in the body frame:
  // d(up_body)/dt = -omega x up = up x omega.
  if ((cal.flags & CAL_GYRO) && dt > 0.0f && dt < 0.1f) {
    const Vec3 w = mul3(gDps, DEG2RAD_F);
    upEstimate = add3(upEstimate, mul3(cross3(upEstimate, w), dt));
    upEstimate = normalize3(upEstimate);
  }

  // Accelerometer correction. Trust it less during strong linear acceleration.
  if (aNorm > 0.45f && aNorm < 1.65f) {
    const Vec3 aUnit = mul3(a, 1.0f/aNorm);
    const float trust = clampf(1.0f - fabsf(aNorm - 1.0f)/0.40f, 0.0f, 1.0f);
    const float base = (cal.flags & CAL_GYRO) ? 0.035f : 0.16f;
    const float beta = base * trust;
    upEstimate = normalize3(add3(mul3(upEstimate, 1.0f-beta), mul3(aUnit, beta)));
  }

  // Reject gross magnetic disturbance only after a unit-sphere mag calibration exists.
  bool magneticOK = (mNorm > 1.0e-6f);
  if (cal.flags & CAL_MAG) {
    magneticOK = magneticOK && (mNorm > 0.50f) && (mNorm < 1.80f);
  }

  if (magneticOK) {
    const float alpha = 0.18f;
    if (!magFilterInitialized) {
      magFiltered = m;
      magFilterInitialized = true;
    } else {
      magFiltered = add3(mul3(magFiltered, 1.0f-alpha), mul3(m, alpha));
    }
  }

  if (!magFilterInitialized) return false;

  // User-defined forward direction is body -Z.
  const Vec3 forwardBody = {0.0f, 0.0f, -1.0f};

  // Project forward and magnetic field into the plane perpendicular to gravity.
  Vec3 fHoriz = sub3(forwardBody, mul3(upEstimate, dot3(forwardBody, upEstimate)));
  const float fNorm = norm3(fHoriz);
  lastForwardHorizontal = fNorm;

  Vec3 northHoriz = sub3(magFiltered, mul3(upEstimate, dot3(magFiltered, upEstimate)));
  const float nNorm = norm3(northHoriz);

  // If forward is almost vertical, azimuth of that axis is physically ill-conditioned.
  if (!magneticOK || fNorm < 0.12f || nNorm < 1.0e-5f) {
    lastHeadingValid = false;
    return false;
  }

  fHoriz = mul3(fHoriz, 1.0f/fNorm);
  northHoriz = mul3(northHoriz, 1.0f/nNorm);

  // ENU convention: east = north x up.
  Vec3 east = normalize3(cross3(northHoriz, upEstimate));
  if (norm3(east) < 0.5f) {
    lastHeadingValid = false;
    return false;
  }

  const float xNorth = dot3(fHoriz, northHoriz);
  const float yEast  = dot3(fHoriz, east);
  float magneticHeading = atan2f(yEast, xNorth) * RAD2DEG_F;
  magneticHeading = wrap360(magneticHeading);

  float trueHeading = wrap360(magneticHeading + cal.declinationDeg);
  lastRawHeadingDeg = trueHeading;

  // Circular low-pass avoids the 359/0 degree discontinuity.
  const float gyroSpeed = norm3(gDps);
  const float alphaH = clampf(0.10f + 0.004f*gyroSpeed, 0.10f, 0.45f);
  const float hr = trueHeading * DEG2RAD_F;
  const float cx = cosf(hr), sy = sinf(hr);

  if (!headingFilterInitialized) {
    smoothCos = cx;
    smoothSin = sy;
    headingFilterInitialized = true;
  } else {
    smoothCos = (1.0f-alphaH)*smoothCos + alphaH*cx;
    smoothSin = (1.0f-alphaH)*smoothSin + alphaH*sy;
    const float n = sqrtf(smoothCos*smoothCos + smoothSin*smoothSin);
    if (n > 1.0e-6f) {
      smoothCos /= n;
      smoothSin /= n;
    }
  }

  lastHeadingDeg = wrap360(atan2f(smoothSin, smoothCos) * RAD2DEG_F);
  lastHeadingValid = true;
  return true;
}

void updateCompass() {
  if (!sensorsReady) return;

  Raw3 ar, gr, mr;
  if (!readAccelRaw(ar) || !readGyroRaw(gr) || !readMagRaw(mr)) return;

  const uint32_t now = micros();
  float dt = 0.02f;
  if (lastUpdateUs != 0) {
    dt = (float)(now - lastUpdateUs) * 1.0e-6f;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.02f;
  }
  lastUpdateUs = now;

  computeHeading(dt, ar, gr, mr);
}

void printHeadingLine() {
  if (lastHeadingValid && isfinite(lastHeadingDeg)) {
    Serial.printf("HDG=%6.2f deg %-3s  raw=%6.2f  A=%.3f  M=%.3f  forwardH=%.3f  %s%s%s\n",
                  lastHeadingDeg, cardinal(lastHeadingDeg), lastRawHeadingDeg,
                  lastAccelNorm, lastMagNorm, lastForwardHorizontal,
                  (cal.flags & CAL_GYRO)  ? "G" : "-",
                  (cal.flags & CAL_ACCEL) ? "A" : "-",
                  (cal.flags & CAL_MAG)   ? "M" : "-");
  } else {
    Serial.printf("HDG=HOLD  A=%.3f  M=%.3f  forwardH=%.3f  reason=%s\n",
                  lastAccelNorm, lastMagNorm, lastForwardHorizontal,
                  (lastForwardHorizontal < 0.12f) ? "forward(-Z) near vertical" : "mag/geometry invalid");
  }
}

// ---------- serial console ----------

void printMap(const char *name, const int8_t m[3]) {
  auto axisName = [](int8_t code) -> String {
    String s = (code >= 0) ? "+" : "-";
    int a = abs((int)code);
    s += (a == 1 ? "X" : (a == 2 ? "Y" : "Z"));
    return s;
  };
  Serial.printf("%s: bodyX=%s bodyY=%s bodyZ=%s\n",
                name,
                axisName(m[0]).c_str(),
                axisName(m[1]).c_str(),
                axisName(m[2]).c_str());
}

void printMatrix(const char *name, const float c[3], const float M[9]) {
  Serial.printf("%s center: [%.6f %.6f %.6f]\n", name, c[0], c[1], c[2]);
  Serial.printf("%s M0: [%.8f %.8f %.8f]\n", name, M[0], M[1], M[2]);
  Serial.printf("%s M1: [%.8f %.8f %.8f]\n", name, M[3], M[4], M[5]);
  Serial.printf("%s M2: [%.8f %.8f %.8f]\n", name, M[6], M[7], M[8]);
}

void printStatus() {
  Serial.println();
  Serial.println("=== LABAN4 STATUS ===");
  Serial.printf("Body frame: +X=head, +Y=left, +Z=inward, FORWARD=-Z\n");
  Serial.printf("Sensors: %s\n", sensorsReady ? "READY" : "NOT READY");
  Serial.printf("Calibration flags: gyro=%s accel=%s mag=%s\n",
                (cal.flags & CAL_GYRO) ? "YES" : "NO",
                (cal.flags & CAL_ACCEL) ? "YES" : "NO",
                (cal.flags & CAL_MAG) ? "YES" : "NO");
  Serial.printf("Declination: %.3f deg (east positive)\n", cal.declinationDeg);
  Serial.printf("Output: %u Hz, stream=%s\n", cal.outputHz, streamEnabled ? "ON" : "OFF");
  Serial.printf("Gyro bias raw: [%.3f %.3f %.3f]\n",
                cal.gyroBias[0], cal.gyroBias[1], cal.gyroBias[2]);
  printMatrix("ACC", cal.accelCenter, cal.accelMatrix);
  printMatrix("MAG", cal.magCenter, cal.magMatrix);
  printMap("accel map", cal.accelMap);
  printMap("gyro map ", cal.gyroMap);
  printMap("mag map  ", cal.magMap);
  if (isfinite(lastHeadingDeg)) Serial.printf("Last heading: %.2f deg %s\n", lastHeadingDeg, cardinal(lastHeadingDeg));
}

void printRaw() {
  Raw3 a,g,m;
  if (!readAccelRaw(a) || !readGyroRaw(g) || !readMagRaw(m)) {
    Serial.println("[FAIL] Sensor read.");
    return;
  }
  const Vec3 ac = calibratedAccelBody(a);
  const Vec3 gc = calibratedGyroBody(g);
  const Vec3 mc = calibratedMagBody(m);

  Serial.printf("RAW accel: %d %d %d\n", a.x,a.y,a.z);
  Serial.printf("RAW gyro : %d %d %d\n", g.x,g.y,g.z);
  Serial.printf("RAW mag  : %d %d %d\n", m.x,m.y,m.z);
  Serial.printf("BODY accel: %.4f %.4f %.4f |norm|=%.4f\n", ac.x,ac.y,ac.z,norm3(ac));
  Serial.printf("BODY gyro : %.3f %.3f %.3f deg/s\n", gc.x,gc.y,gc.z);
  Serial.printf("BODY mag  : %.5f %.5f %.5f |norm|=%.5f\n", mc.x,mc.y,mc.z,norm3(mc));
}

void scanI2C() {
  Serial.println("I2C scan:");
  int found = 0;
  for (uint8_t a = 1; a < 127; ++a) {
    if (i2cPing(a)) {
      Serial.printf("  0x%02X", a);
      if (a == ADXL345_ADDR) Serial.print(" ADXL345");
      else if (a == ITG3205_ADDR) Serial.print(" ITG3205");
      else if (a == HMC5883L_ADDR) Serial.print(" HMC5883L");
      Serial.println();
      ++found;
    }
  }
  if (!found) Serial.println("  none");
}

void printHelp() {
  Serial.println();
  Serial.println("=== LABAN4 SERIAL COMMANDS ===");
  Serial.println("help                  : show this menu");
  Serial.println("status                : calibration, maps, matrices, heading");
  Serial.println("scan                  : I2C scan");
  Serial.println("raw                   : one raw + calibrated sensor sample");
  Serial.println("heading               : print current heading once");
  Serial.println("stream on|off         : continuous heading output");
  Serial.println("rate <1..50>          : output lines per second");
  Serial.println("decl <degrees>        : magnetic declination, east positive");
  Serial.println("cal gyro              : 5 s stationary gyro bias");
  Serial.println("cal accel             : 30 s slow all-orientation accel ellipsoid fit");
  Serial.println("cal mag               : 40 s 3D magnetometer hard/soft-iron fit");
  Serial.println("cal all               : gyro + accel + mag, fully on-device");
  Serial.println("map show              : show sensor->body signed axis maps");
  Serial.println("map accel <X> <Y> <Z> : e.g. map accel +x -y +z");
  Serial.println("map gyro  <X> <Y> <Z> : each token is +/-x/y/z; axes unique");
  Serial.println("map mag   <X> <Y> <Z> : mapping outputs BODY X,Y,Z respectively");
  Serial.println("factory reset         : erase calibration/settings and restore defaults");
  Serial.println();
  Serial.println("Required body frame: +X=head, +Y=left, +Z=inward, forward=-Z.");
}

int8_t parseAxisToken(String s) {
  s.trim();
  s.toLowerCase();
  if (s.length() < 1 || s.length() > 2) return 0;

  int sign = +1;
  int pos = 0;
  if (s[0] == '+') { sign = +1; pos = 1; }
  else if (s[0] == '-') { sign = -1; pos = 1; }

  if (pos >= (int)s.length()) return 0;
  char c = s[pos];
  int axis = (c == 'x') ? 1 : ((c == 'y') ? 2 : ((c == 'z') ? 3 : 0));
  if (!axis) return 0;
  return (int8_t)(sign * axis);
}

bool validMap3(const int8_t m[3]) {
  const int a = abs((int)m[0]), b = abs((int)m[1]), c = abs((int)m[2]);
  return a >= 1 && a <= 3 && b >= 1 && b <= 3 && c >= 1 && c <= 3
      && a != b && a != c && b != c;
}

void handleMapCommand(const String &line) {
  if (line == "map show") {
    printMap("accel map", cal.accelMap);
    printMap("gyro map ", cal.gyroMap);
    printMap("mag map  ", cal.magMap);
    return;
  }

  char buf[96];
  line.toCharArray(buf, sizeof(buf));
  char *t0 = strtok(buf, " ");
  char *sensor = strtok(nullptr, " ");
  char *a0 = strtok(nullptr, " ");
  char *a1 = strtok(nullptr, " ");
  char *a2 = strtok(nullptr, " ");
  if (!t0 || !sensor || !a0 || !a1 || !a2) {
    Serial.println("Usage: map accel|gyro|mag +x +y +z");
    return;
  }

  int8_t m[3] = {
    parseAxisToken(String(a0)),
    parseAxisToken(String(a1)),
    parseAxisToken(String(a2))
  };
  if (!validMap3(m)) {
    Serial.println("[FAIL] Map must use each of X/Y/Z exactly once, with optional +/- sign.");
    return;
  }

  if (strcmp(sensor, "accel") == 0) memcpy(cal.accelMap, m, 3);
  else if (strcmp(sensor, "gyro") == 0) memcpy(cal.gyroMap, m, 3);
  else if (strcmp(sensor, "mag") == 0) memcpy(cal.magMap, m, 3);
  else {
    Serial.println("[FAIL] Sensor must be accel, gyro or mag.");
    return;
  }

  saveCalibration();
  upInitialized = false;
  magFilterInitialized = false;
  headingFilterInitialized = false;
  Serial.println("[OK] Axis map saved.");
  printMap(sensor, m);
}

void factoryReset() {
  setCalibrationDefaults();
  saveCalibration();
  upInitialized = false;
  magFilterInitialized = false;
  headingFilterInitialized = false;
  Serial.println("[OK] Factory defaults restored.");
}

void handleCommand(String line) {
  line.trim();
  line.toLowerCase();
  if (line.length() == 0) return;

  if (line == "help" || line == "?") printHelp();
  else if (line == "status") printStatus();
  else if (line == "scan") scanI2C();
  else if (line == "raw") printRaw();
  else if (line == "heading") printHeadingLine();
  else if (line == "stream on") { streamEnabled = true; Serial.println("[OK] stream ON"); }
  else if (line == "stream off") { streamEnabled = false; Serial.println("[OK] stream OFF"); }
  else if (line.startsWith("rate ")) {
    int hz = line.substring(5).toInt();
    if (hz < 1 || hz > 50) Serial.println("[FAIL] rate must be 1..50");
    else {
      cal.outputHz = (uint16_t)hz;
      saveCalibration();
      Serial.printf("[OK] rate=%d Hz\n", hz);
    }
  }
  else if (line.startsWith("decl ")) {
    float d = line.substring(5).toFloat();
    if (!isfinite(d) || d < -180.0f || d > 180.0f) Serial.println("[FAIL] decl range is -180..+180 deg");
    else {
      cal.declinationDeg = d;
      saveCalibration();
      headingFilterInitialized = false;
      Serial.printf("[OK] declination=%.3f deg (east positive)\n", d);
    }
  }
  else if (line == "cal gyro") calibrateGyro();
  else if (line == "cal accel") calibrateVectors(true, false, 30000);
  else if (line == "cal mag") calibrateVectors(false, true, 40000);
  else if (line == "cal all") calibrateAll();
  else if (line.startsWith("map ")) handleMapCommand(line);
  else if (line == "factory reset") factoryReset();
  else Serial.println("Unknown command. Type: help");
}

// ---------- Arduino entry points ----------

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(700);

  Serial.println();
  Serial.println("==============================================");
  Serial.println(" LABAN4 - GY-85 WEARABLE TILT COMPASS");
  Serial.println(" +X=head  +Y=left  +Z=inward  FORWARD=-Z");
  Serial.println("==============================================");

  const bool hadCal = loadCalibration();
  Serial.printf("NVS calibration: %s\n", hadCal ? "loaded" : "defaults");

  Wire.begin(LABAN4_SDA, LABAN4_SCL);
  Wire.setClock(400000);

  initSensors();
  printHelp();

  if (!(cal.flags & CAL_MAG)) {
    Serial.println("[NOTE] Magnetometer is not calibrated. Run: cal all");
  }
}

void loop() {
  // Keep sensor engine near 50 Hz.
  static uint32_t lastSensorMs = 0;
  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastSensorMs) >= 20) {
    lastSensorMs = nowMs;
    updateCompass();
  }

  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      handleCommand(serialLine);
      serialLine = "";
    } else if (serialLine.length() < 120) {
      serialLine += ch;
    }
  }

  const uint16_t hz = (cal.outputHz >= 1 && cal.outputHz <= 50) ? cal.outputHz : 10;
  const uint32_t period = 1000UL / hz;
  if (streamEnabled && (uint32_t)(nowMs - lastPrintMs) >= period) {
    lastPrintMs = nowMs;
    printHeadingLine();
  }
}
