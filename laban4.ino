/*
  laban4 - La bàn đeo người ESP32 + GY-85 có bù nghiêng

  Các cảm biến trên GY-85:
    ADXL345  : gia tốc kế 3 trục,          I2C 0x53
    ITG-3205 : con quay hồi chuyển 3 trục, I2C 0x68
    HMC5883L : từ kế 3 trục,               I2C 0x1E

  Hệ trục cơ thể của dự án:
    +X = hướng lên đầu người đeo
    +Y = hướng sang tay trái người đeo
    +Z = hướng vào trong cơ thể
    -Z = hướng tiến / hướng di chuyển cần xác định

  Heading KHÔNG được tính trực tiếp bằng atan2(mx,my) khi bo nằm ngang.
  Thay vào đó:
    1) ước lượng vector trọng lực/hướng lên bằng gia tốc kế + gyro,
    2) chiếu vector từ trường lên mặt phẳng ngang vuông góc trọng lực,
    3) chiếu hướng tiến của cơ thể (-Z) lên cùng mặt phẳng,
    4) heading là góc có dấu từ Bắc từ đến hướng tiến đã chiếu.

  Toàn bộ hiệu chuẩn chạy trực tiếp trên module qua Serial Monitor:
    cal gyro
    cal accel
    cal mag
    cal all

  Không cần Python hay công cụ hiệu chuẩn bên ngoài.

  Nền tảng đích: ESP32 Arduino core.
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

// 25 Hz * 40 s = 1000 mẫu. Dung lượng 1200 mẫu để chừa biên.
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

  // Giá trị thô của gyro khi vận tốc góc bằng 0.
  float gyroBias[3];

  // Vector đã hiệu chỉnh = ma trận * (dữ liệu thô - tâm).
  // Với gia tốc kế và từ kế, hiệu chuẩn ellipsoid thành công sẽ đưa các mẫu
  // về gần một hình cầu đơn vị.
  float accelCenter[3];
  float accelMatrix[9];
  float magCenter[3];
  float magMatrix[9];

  // Ánh xạ hoán vị có dấu từ trục đã hiệu chỉnh của cảm biến sang trục cơ thể.
  // Ví dụ {+1,+2,+3}: X cơ thể=+X cảm biến, Y cơ thể=+Y cảm biến, Z cơ thể=+Z cảm biến.
  // Giá trị là +/-1, +/-2, +/-3 và trị tuyệt đối không được trùng nhau.
  int8_t accelMap[3];
  int8_t gyroMap[3];
  int8_t magMap[3];
  int8_t reservedMap;

  // Độ từ thiên phía Đông là số dương. heading_thật = heading_từ + độ_từ_thiên.
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

// ---------- lưu dữ liệu hiệu chuẩn lâu dài ----------

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

  // Ở chế độ full-resolution, ADXL345 có tỷ lệ danh định khoảng 256 count/g.
  setDiag(cal.accelMatrix, 1.0f/256.0f, 1.0f/256.0f, 1.0f/256.0f);

  // Gain mặc định của HMC5883L danh định là 1090 LSB/Gauss. Khi chưa hiệu chuẩn,
  // hệ số này chỉ tạo scale ban đầu; hiệu chuẩn ellipsoid sẽ thay thế nó.
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

// ---------- I2C / trình điều khiển cảm biến ----------

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
    Serial.printf("[CẢNH BÁO] ADXL345 DEVID=0x%02X, giá trị mong đợi 0xE5\n", id);
  }

  if (!writeReg(ADXL345_ADDR, 0x2D, 0x00)) return false; // chế độ chờ
  if (!writeReg(ADXL345_ADDR, 0x31, 0x09)) return false; // full-resolution, +/-4 g
  if (!writeReg(ADXL345_ADDR, 0x2C, 0x0A)) return false; // 100 Hz
  if (!writeReg(ADXL345_ADDR, 0x2D, 0x08)) return false; // chế độ đo
  delay(10);
  return true;
}

bool initITG3205() {
  if (!i2cPing(ITG3205_ADDR)) return false;

  writeReg(ITG3205_ADDR, 0x3E, 0x80); // đặt lại thiết bị
  delay(60);
  if (!writeReg(ITG3205_ADDR, 0x3E, 0x01)) return false; // PLL dùng gyro X làm tham chiếu
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
      Serial.printf("[CẢNH BÁO] HMC5883L ID='%c%c%c'. Chip clone/tương thích có thể hoạt động khác.\n",
                    id[0], id[1], id[2]);
    }
  }

  if (!writeReg(HMC5883L_ADDR, 0x00, 0x78)) return false; // trung bình 8 mẫu, 75 Hz, chế độ bình thường
  if (!writeReg(HMC5883L_ADDR, 0x01, 0x20)) return false; // +/-1.3 G, 1090 LSB/G
  if (!writeReg(HMC5883L_ADDR, 0x02, 0x00)) return false; // chế độ đo liên tục
  delay(20);
  return true;
}

bool initSensors() {
  const bool a = initADXL345();
  const bool g = initITG3205();
  const bool m = initHMC5883L();

  Serial.printf("ADXL345 : %s\n", a ? "OK" : "LỖI");
  Serial.printf("ITG3205 : %s\n", g ? "OK" : "LỖI");
  Serial.printf("HMC5883L: %s\n", m ? "OK" : "LỖI");

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

  // Thứ tự thanh ghi HMC5883L là X, Z, Y (big endian).
  r.x = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
  r.z = (int16_t)(((uint16_t)b[2] << 8) | b[3]);
  r.y = (int16_t)(((uint16_t)b[4] << 8) | b[5]);

  // HMC5883L trả về -4096 khi tràn/bão hòa.
  if (r.x == -4096 || r.y == -4096 || r.z == -4096) return false;
  return true;
}

// ---------- áp dụng hiệu chuẩn / ánh xạ trục ----------

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

// ---------- fit ellipsoid 3D đầy đủ ----------
// Dạng fit đại số:
//   x^T A x + b^T x = 1
// Khi đó tâm c = -0.5 A^-1 b.
// Với y=x-c: y^T Q y = 1, Q=A/(1+c^T A c).
// Ma trận hiệu chỉnh sqrt(Q) biến ellipsoid về hình cầu.
//
// Cách này bù offset hard-iron cùng ma trận đối xứng 3x3 cho soft-iron/tương tác chéo
// mà không cần chương trình fit trên máy tính.

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
    // Regularization rất nhỏ để hạn chế tập mẫu gần suy biến.
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

// ---------- các hàm hiệu chuẩn ----------

bool calibrateGyro() {
  Serial.println();
  Serial.println("=== HIỆU CHUẨN GYRO ===");
  Serial.println("Giữ module HOÀN TOÀN ĐỨNG YÊN trong 5 giây.");
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
    Serial.println("[LỖI] Có quá nhiều lần đọc gyro qua I2C thất bại.");
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

  Serial.printf("bias thô: X=%.2f Y=%.2f Z=%.2f\n", mean[0], mean[1], mean[2]);
  Serial.printf("độ lệch chuẩn nhiễu: X=%.2f Y=%.2f Z=%.2f độ/s\n", sdDps[0], sdDps[1], sdDps[2]);

  if (!stable) {
    Serial.println("[LỖI] Module đã bị di chuyển khi hiệu chuẩn. Hãy đặt trên bề mặt ổn định và thử lại.");
    return false;
  }

  memcpy(cal.gyroBias, mean, sizeof(mean));
  cal.flags |= CAL_GYRO;
  if (!saveCalibration()) {
    Serial.println("[CẢNH BÁO] Đã tính xong hiệu chuẩn nhưng lưu NVS thất bại.");
  }
  Serial.println("[OK] Đã lưu bias gyro.");
  return true;
}

bool calibrateVectors(bool doAccel, bool doMag, uint32_t durationMs) {
  Serial.println();
  Serial.println("=== HIỆU CHUẨN VECTOR 3D ===");
  if (doAccel && doMag) {
    Serial.println("Gia tốc kế và từ kế sẽ được hiệu chuẩn cùng lúc.");
  } else if (doAccel) {
    Serial.println("Đang hiệu chuẩn gia tốc kế.");
  } else {
    Serial.println("Đang hiệu chuẩn từ kế.");
  }
  Serial.println("Xoay module CHẬM qua tất cả các tư thế có thể.");
  Serial.println("Thực hiện chuyển động số 8 rộng trong không gian 3D và lần lượt đưa mọi trục +/- lên trên/xuống dưới.");
  Serial.println("Giữ xa thép, nam châm, loa, động cơ và dây dẫn dòng điện lớn.");
  Serial.println("Bắt đầu sau 3 giây...");
  delay(3000);

  int na = 0, nm = 0;
  const uint32_t t0 = millis();
  uint32_t lastProgress = 0;

  while ((millis() - t0) < durationMs && (na < CAL_MAX_SAMPLES || nm < CAL_MAX_SAMPLES)) {
    if (doAccel && na < CAL_MAX_SAMPLES) {
      Raw3 a;
      if (readAccelRaw(a)) {
        // Khi tĩnh, độ lớn danh định khoảng 256 count. Loại các mẫu bị xung gia tốc rõ rệt.
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
    Serial.printf("Biên dữ liệu thô gia tốc kế: X=%ld Y=%ld Z=%ld\n",
                  (long)span[0], (long)span[1], (long)span[2]);

    float center[3], M[9], rms=0, cond=0;
    const bool coverage = span[0] > 330 && span[1] > 330 && span[2] > 330;
    const bool fit = coverage && fitEllipsoid(accelCalSamples, na, 256.0f,
                                               center, M, rms, cond);
    Serial.printf("Fit gia tốc kế: mẫu=%d rms=%.4f condition=%.2f\n", na, rms, cond);

    if (fit && rms < 0.14f && cond < 3.0f) {
      memcpy(cal.accelCenter, center, sizeof(center));
      memcpy(cal.accelMatrix, M, sizeof(M));
      cal.flags |= CAL_ACCEL;
      anySaved = true;
      Serial.println("[OK] Hiệu chuẩn ellipsoid gia tốc kế đạt yêu cầu.");
    } else {
      Serial.println("[LỖI] Dữ liệu/fit gia tốc kế chưa đạt. Hãy xoay chậm hơn qua TẤT CẢ các trục rồi thử lại.");
    }
  }

  if (doMag) {
    int32_t span[3] = {0,0,0};
    sampleSpans(magCalSamples, nm, span);
    Serial.printf("Biên dữ liệu thô từ kế: X=%ld Y=%ld Z=%ld\n",
                  (long)span[0], (long)span[1], (long)span[2]);

    float center[3], M[9], rms=0, cond=0;
    const bool coverage = span[0] > 260 && span[1] > 260 && span[2] > 260;
    const bool fit = coverage && fitEllipsoid(magCalSamples, nm, 500.0f,
                                               center, M, rms, cond);
    Serial.printf("Fit từ kế: mẫu=%d rms=%.4f condition=%.2f\n", nm, rms, cond);

    if (fit && rms < 0.18f && cond < 8.0f) {
      memcpy(cal.magCenter, center, sizeof(center));
      memcpy(cal.magMatrix, M, sizeof(M));
      cal.flags |= CAL_MAG;
      anySaved = true;
      Serial.println("[OK] Hiệu chuẩn ellipsoid hard/soft-iron của từ kế đạt yêu cầu.");
    } else {
      Serial.println("[LỖI] Dữ liệu/fit từ kế chưa đạt. Hãy tránh nguồn nhiễu từ và thử lại.");
    }
  }

  if (anySaved) {
    if (saveCalibration()) Serial.println("[OK] Đã lưu dữ liệu hiệu chuẩn vào NVS của ESP32.");
    else Serial.println("[CẢNH BÁO] Đã tính xong hiệu chuẩn nhưng lưu NVS thất bại.");
  }
  return anySaved;
}

void calibrateAll() {
  Serial.println();
  Serial.println("========== HIỆU CHUẨN TOÀN BỘ LABAN4 ==========");
  Serial.println("Bước 1/2: hiệu chuẩn bias gyro khi đứng yên.");
  if (!calibrateGyro()) {
    Serial.println("[DỪNG] Cần hiệu chuẩn gyro thành công trước khi tiếp tục.");
    return;
  }
  Serial.println();
  Serial.println("Bước 2/2: hiệu chuẩn ellipsoid 3D cho gia tốc kế + từ kế.");
  calibrateVectors(true, true, 40000);
  Serial.println("========== ĐÃ KẾT THÚC HIỆU CHUẨN ==========");
}

// ---------- bộ tính heading ----------

const char* cardinal(float h) {
  static const char* c[16] = {
    "B","BĐB","ĐB","ĐĐB","Đ","ĐĐN","ĐN","NĐN",
    "N","NTN","TN","TTN","T","TTB","TB","BTB"
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

  // Gyro lan truyền vector trọng lực trong hệ trục cơ thể:
  // d(up_body)/dt = -omega x up = up x omega.
  if ((cal.flags & CAL_GYRO) && dt > 0.0f && dt < 0.1f) {
    const Vec3 w = mul3(gDps, DEG2RAD_F);
    upEstimate = add3(upEstimate, mul3(cross3(upEstimate, w), dt));
    upEstimate = normalize3(upEstimate);
  }

  // Hiệu chỉnh bằng gia tốc kế. Giảm độ tin cậy khi có gia tốc tuyến tính mạnh.
  if (aNorm > 0.45f && aNorm < 1.65f) {
    const Vec3 aUnit = mul3(a, 1.0f/aNorm);
    const float trust = clampf(1.0f - fabsf(aNorm - 1.0f)/0.40f, 0.0f, 1.0f);
    const float base = (cal.flags & CAL_GYRO) ? 0.035f : 0.16f;
    const float beta = base * trust;
    upEstimate = normalize3(add3(mul3(upEstimate, 1.0f-beta), mul3(aUnit, beta)));
  }

  // Chỉ loại nhiễu từ lớn theo độ lớn sau khi đã có hiệu chuẩn từ kế về cầu đơn vị.
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

  // Hướng tiến theo yêu cầu của dự án là trục -Z của cơ thể.
  const Vec3 forwardBody = {0.0f, 0.0f, -1.0f};

  // Chiếu hướng tiến và từ trường lên mặt phẳng vuông góc với trọng lực.
  Vec3 fHoriz = sub3(forwardBody, mul3(upEstimate, dot3(forwardBody, upEstimate)));
  const float fNorm = norm3(fHoriz);
  lastForwardHorizontal = fNorm;

  Vec3 northHoriz = sub3(magFiltered, mul3(upEstimate, dot3(magFiltered, upEstimate)));
  const float nNorm = norm3(northHoriz);

  // Nếu hướng tiến gần thẳng đứng, azimuth của trục đó trở nên kém xác định về mặt vật lý.
  if (!magneticOK || fNorm < 0.12f || nNorm < 1.0e-5f) {
    lastHeadingValid = false;
    return false;
  }

  fHoriz = mul3(fHoriz, 1.0f/fNorm);
  northHoriz = mul3(northHoriz, 1.0f/nNorm);

  // Quy ước ENU: Đông = Bắc x Hướng lên.
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

  // Lọc thấp theo đường tròn tránh gián đoạn tại biên 359/0 độ.
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
    Serial.printf("HƯỚNG=%6.2f độ %-3s  thô=%6.2f  A=%.3f  M=%.3f  ngangTiến=%.3f  %s%s%s\n",
                  lastHeadingDeg, cardinal(lastHeadingDeg), lastRawHeadingDeg,
                  lastAccelNorm, lastMagNorm, lastForwardHorizontal,
                  (cal.flags & CAL_GYRO)  ? "G" : "-",
                  (cal.flags & CAL_ACCEL) ? "A" : "-",
                  (cal.flags & CAL_MAG)   ? "M" : "-");
  } else {
    Serial.printf("HƯỚNG=GIỮ  A=%.3f  M=%.3f  ngangTiến=%.3f  lý_do=%s\n",
                  lastAccelNorm, lastMagNorm, lastForwardHorizontal,
                  (lastForwardHorizontal < 0.12f) ? "hướng tiến (-Z) gần thẳng đứng" : "từ trường/hình học không hợp lệ");
  }
}

// ---------- giao diện Serial Monitor ----------

void printMap(const char *name, const int8_t m[3]) {
  auto axisName = [](int8_t code) -> String {
    String s = (code >= 0) ? "+" : "-";
    int a = abs((int)code);
    s += (a == 1 ? "X" : (a == 2 ? "Y" : "Z"));
    return s;
  };
  Serial.printf("%s: cơ_thể_X=%s cơ_thể_Y=%s cơ_thể_Z=%s\n",
                name,
                axisName(m[0]).c_str(),
                axisName(m[1]).c_str(),
                axisName(m[2]).c_str());
}

void printMatrix(const char *name, const float c[3], const float M[9]) {
  Serial.printf("%s tâm: [%.6f %.6f %.6f]\n", name, c[0], c[1], c[2]);
  Serial.printf("%s M0: [%.8f %.8f %.8f]\n", name, M[0], M[1], M[2]);
  Serial.printf("%s M1: [%.8f %.8f %.8f]\n", name, M[3], M[4], M[5]);
  Serial.printf("%s M2: [%.8f %.8f %.8f]\n", name, M[6], M[7], M[8]);
}

void printStatus() {
  Serial.println();
  Serial.println("=== TRẠNG THÁI LABAN4 ===");
  Serial.printf("Hệ trục: +X=đầu, +Y=trái, +Z=vào cơ thể, HƯỚNG TIẾN=-Z\n");
  Serial.printf("Cảm biến: %s\n", sensorsReady ? "SẴN SÀNG" : "CHƯA SẴN SÀNG");
  Serial.printf("Trạng thái hiệu chuẩn: gyro=%s accel=%s mag=%s\n",
                (cal.flags & CAL_GYRO) ? "CÓ" : "KHÔNG",
                (cal.flags & CAL_ACCEL) ? "CÓ" : "KHÔNG",
                (cal.flags & CAL_MAG) ? "CÓ" : "KHÔNG");
  Serial.printf("Độ từ thiên: %.3f độ (phía Đông là dương)\n", cal.declinationDeg);
  Serial.printf("Xuất dữ liệu: %u Hz, stream=%s\n", cal.outputHz, streamEnabled ? "ON" : "OFF");
  Serial.printf("Bias gyro thô: [%.3f %.3f %.3f]\n",
                cal.gyroBias[0], cal.gyroBias[1], cal.gyroBias[2]);
  printMatrix("ACC", cal.accelCenter, cal.accelMatrix);
  printMatrix("MAG", cal.magCenter, cal.magMatrix);
  printMap("accel map", cal.accelMap);
  printMap("gyro map ", cal.gyroMap);
  printMap("mag map  ", cal.magMap);
  if (isfinite(lastHeadingDeg)) Serial.printf("Heading gần nhất: %.2f độ %s\n", lastHeadingDeg, cardinal(lastHeadingDeg));
}

void printRaw() {
  Raw3 a,g,m;
  if (!readAccelRaw(a) || !readGyroRaw(g) || !readMagRaw(m)) {
    Serial.println("[LỖI] Không đọc được cảm biến.");
    return;
  }
  const Vec3 ac = calibratedAccelBody(a);
  const Vec3 gc = calibratedGyroBody(g);
  const Vec3 mc = calibratedMagBody(m);

  Serial.printf("THÔ accel: %d %d %d\n", a.x,a.y,a.z);
  Serial.printf("THÔ gyro : %d %d %d\n", g.x,g.y,g.z);
  Serial.printf("THÔ mag  : %d %d %d\n", m.x,m.y,m.z);
  Serial.printf("CƠ_THỂ accel: %.4f %.4f %.4f |norm|=%.4f\n", ac.x,ac.y,ac.z,norm3(ac));
  Serial.printf("CƠ_THỂ gyro : %.3f %.3f %.3f độ/s\n", gc.x,gc.y,gc.z);
  Serial.printf("CƠ_THỂ mag  : %.5f %.5f %.5f |norm|=%.5f\n", mc.x,mc.y,mc.z,norm3(mc));
}

void scanI2C() {
  Serial.println("Quét I2C:");
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
  if (!found) Serial.println("  không tìm thấy thiết bị");
}

void printHelp() {
  Serial.println();
  Serial.println("=== CÁC LỆNH SERIAL CỦA LABAN4 ===");
  Serial.println("help                  : hiện danh sách lệnh");
  Serial.println("status                : xem hiệu chuẩn, mapping, ma trận và heading");
  Serial.println("scan                  : quét thiết bị I2C");
  Serial.println("raw                   : in một mẫu thô và mẫu đã hiệu chỉnh");
  Serial.println("heading               : in heading hiện tại một lần");
  Serial.println("stream on|off         : bật/tắt xuất heading liên tục");
  Serial.println("rate <1..50>          : số dòng dữ liệu xuất mỗi giây");
  Serial.println("decl <degrees>        : độ từ thiên, phía Đông là số dương");
  Serial.println("cal gyro              : hiệu chuẩn bias gyro, đứng yên 5 giây");
  Serial.println("cal accel             : fit ellipsoid gia tốc kế 30 giây, xoay chậm đủ hướng");
  Serial.println("cal mag               : fit hard/soft-iron từ kế 3D trong 40 giây");
  Serial.println("cal all               : hiệu chuẩn gyro + accel + mag hoàn toàn trên module");
  Serial.println("map show              : xem ánh xạ có dấu từ cảm biến sang cơ thể");
  Serial.println("map accel <X> <Y> <Z> : ví dụ map accel +x -y +z");
  Serial.println("map gyro  <X> <Y> <Z> : mỗi mục là +/-x/y/z; không được lặp trục");
  Serial.println("map mag   <X> <Y> <Z> : ánh xạ lần lượt ra X,Y,Z của cơ thể");
  Serial.println("factory reset         : xóa hiệu chuẩn/cài đặt và khôi phục mặc định");
  Serial.println();
  Serial.println("Hệ trục yêu cầu: +X=hướng đầu, +Y=trái, +Z=vào cơ thể, hướng tiến=-Z.");
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
    Serial.println("Cách dùng: map accel|gyro|mag +x +y +z");
    return;
  }

  int8_t m[3] = {
    parseAxisToken(String(a0)),
    parseAxisToken(String(a1)),
    parseAxisToken(String(a2))
  };
  if (!validMap3(m)) {
    Serial.println("[LỖI] Mapping phải dùng mỗi trục X/Y/Z đúng một lần, có thể thêm dấu +/-.");
    return;
  }

  if (strcmp(sensor, "accel") == 0) memcpy(cal.accelMap, m, 3);
  else if (strcmp(sensor, "gyro") == 0) memcpy(cal.gyroMap, m, 3);
  else if (strcmp(sensor, "mag") == 0) memcpy(cal.magMap, m, 3);
  else {
    Serial.println("[LỖI] Cảm biến phải là accel, gyro hoặc mag.");
    return;
  }

  saveCalibration();
  upInitialized = false;
  magFilterInitialized = false;
  headingFilterInitialized = false;
  Serial.println("[OK] Đã lưu ánh xạ trục.");
  printMap(sensor, m);
}

void factoryReset() {
  setCalibrationDefaults();
  saveCalibration();
  upInitialized = false;
  magFilterInitialized = false;
  headingFilterInitialized = false;
  Serial.println("[OK] Đã khôi phục cài đặt mặc định.");
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
  else if (line == "stream on") { streamEnabled = true; Serial.println("[OK] Đã bật stream"); }
  else if (line == "stream off") { streamEnabled = false; Serial.println("[OK] Đã tắt stream"); }
  else if (line.startsWith("rate ")) {
    int hz = line.substring(5).toInt();
    if (hz < 1 || hz > 50) Serial.println("[LỖI] rate phải nằm trong khoảng 1..50");
    else {
      cal.outputHz = (uint16_t)hz;
      saveCalibration();
      Serial.printf("[OK] tốc độ xuất=%d Hz\n", hz);
    }
  }
  else if (line.startsWith("decl ")) {
    float d = line.substring(5).toFloat();
    if (!isfinite(d) || d < -180.0f || d > 180.0f) Serial.println("[LỖI] decl phải nằm trong khoảng -180..+180 độ");
    else {
      cal.declinationDeg = d;
      saveCalibration();
      headingFilterInitialized = false;
      Serial.printf("[OK] độ từ thiên=%.3f độ (phía Đông là dương)\n", d);
    }
  }
  else if (line == "cal gyro") calibrateGyro();
  else if (line == "cal accel") calibrateVectors(true, false, 30000);
  else if (line == "cal mag") calibrateVectors(false, true, 40000);
  else if (line == "cal all") calibrateAll();
  else if (line.startsWith("map ")) handleMapCommand(line);
  else if (line == "factory reset") factoryReset();
  else Serial.println("Lệnh không hợp lệ. Gõ: help");
}

// ---------- điểm vào Arduino ----------

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(700);

  Serial.println();
  Serial.println("==============================================");
  Serial.println(" LABAN4 - LA BÀN ĐEO NGƯỜI GY-85 CÓ BÙ NGHIÊNG");
  Serial.println(" +X=đầu  +Y=trái  +Z=vào cơ thể  HƯỚNG TIẾN=-Z");
  Serial.println("==============================================");

  const bool hadCal = loadCalibration();
  Serial.printf("Dữ liệu hiệu chuẩn NVS: %s\n", hadCal ? "đã tải" : "mặc định");

  Wire.begin(LABAN4_SDA, LABAN4_SCL);
  Wire.setClock(400000);

  initSensors();
  printHelp();

  if (!(cal.flags & CAL_MAG)) {
    Serial.println("[LƯU Ý] Từ kế chưa được hiệu chuẩn. Hãy chạy: cal all");
  }
}

void loop() {
  // Duy trì vòng cập nhật cảm biến gần 50 Hz.
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