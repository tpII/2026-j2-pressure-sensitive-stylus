#include "MPU6050Raw.h"

namespace {
constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
constexpr uint8_t REG_CONFIG       = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t REG_WHO_AM_I     = 0x75;
}  // namespace

bool MPU6050Raw::escribir(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(_dir);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

uint8_t MPU6050Raw::whoAmI() {
  Wire.beginTransmission(_dir);
  Wire.write(REG_WHO_AM_I);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  if (Wire.requestFrom(_dir, (uint8_t)1) != 1) return 0xFF;
  return Wire.read();
}

bool MPU6050Raw::begin(int sda, int scl, uint8_t dir, uint32_t i2cHz) {
  _dir = dir;
  Wire.begin(sda, scl, i2cHz);
  delay(50);
  if (!escribir(REG_PWR_MGMT_1, 0x80)) return false;  // reset
  delay(100);
  bool ok = true;
  ok &= escribir(REG_PWR_MGMT_1, 0x01);    // despierta, reloj = PLL giro X
  delay(10);
  ok &= escribir(REG_CONFIG, 0x03);        // DLPF ~44 Hz -> base 1 kHz
  ok &= escribir(REG_SMPLRT_DIV, 0x04);    // 1 kHz / (1+4) = 200 Hz interno
  ok &= escribir(REG_GYRO_CONFIG, 0x08);   // +-500 dps
  ok &= escribir(REG_ACCEL_CONFIG, 0x08);  // +-4 g
  return ok;
}

bool MPU6050Raw::leer(ImuMuestra &m) {
  Wire.beginTransmission(_dir);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(_dir, (uint8_t)14) != 14) return false;

  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = Wire.read();
  auto s16 = [&](int i) { return (int16_t)((b[i] << 8) | b[i + 1]); };

  m.ax = s16(0) / ACC_LSB_G - _oax;
  m.ay = s16(2) / ACC_LSB_G - _oay;
  m.az = s16(4) / ACC_LSB_G - _oaz;
  m.tempC = s16(6) / 340.0f + 36.53f;
  m.gx = s16(8)  / GYRO_LSB_DPS - _bgx;
  m.gy = s16(10) / GYRO_LSB_DPS - _bgy;
  m.gz = s16(12) / GYRO_LSB_DPS - _bgz;
  return true;
}

bool MPU6050Raw::calibrarGiro(uint16_t n) {
  float bx = _bgx, by = _bgy, bz = _bgz;
  setGyroBias(0, 0, 0);
  double sx = 0, sy = 0, sz = 0;
  float minx = 1e9, maxx = -1e9;  // para detectar movimiento
  ImuMuestra m;
  uint16_t ok = 0;
  for (uint16_t i = 0; i < n; i++) {
    if (leer(m)) {
      sx += m.gx; sy += m.gy; sz += m.gz; ok++;
      minx = min(minx, m.gx + m.gy + m.gz);
      maxx = max(maxx, m.gx + m.gy + m.gz);
    }
    delay(2);
  }
  if (ok < n / 2 || (maxx - minx) > 10.0f) {  // se movio o fallo el bus
    setGyroBias(bx, by, bz);
    return false;
  }
  setGyroBias(sx / ok, sy / ok, sz / ok);
  return true;
}
