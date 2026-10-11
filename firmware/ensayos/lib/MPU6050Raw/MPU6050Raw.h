// Driver minimo del MPU6050 (GY-521) por registros, sin librerias externas.
// Rangos elegidos: acelerometro +-4 g, giroscopo +-500 dps, DLPF 44 Hz.
#pragma once
#include <Arduino.h>
#include <Wire.h>

struct ImuMuestra {
  float ax, ay, az;   // aceleracion [g]
  float gx, gy, gz;   // velocidad angular [deg/s]
  float tempC;        // temperatura interna [C]
};

class MPU6050Raw {
 public:
  static constexpr uint8_t DIR_DEFECTO = 0x68;  // AD0 a GND

  // Inicializa I2C y configura el sensor. Devuelve false si no responde.
  bool begin(int sda = 21, int scl = 22, uint8_t dir = DIR_DEFECTO,
             uint32_t i2cHz = 400000);

  // Lee los 14 bytes de datos (accel, temp, giro) en una sola transaccion.
  bool leer(ImuMuestra &m);

  // Lee el registro WHO_AM_I (0x75). Original: 0x68. Clones: 0x70, 0x72, 0x98...
  uint8_t whoAmI();

  // Bias del giroscopo [deg/s] que se resta en cada lectura.
  void setGyroBias(float bx, float by, float bz) { _bgx = bx; _bgy = by; _bgz = bz; }

  // Offset del acelerometro [g] que se resta en cada lectura (ensayo 3).
  void setAccelOffset(float ox, float oy, float oz) { _oax = ox; _oay = oy; _oaz = oz; }

  // Promedia n muestras con el sensor QUIETO y fija el bias del giroscopo.
  // Devuelve false si detecta movimiento durante la medicion.
  bool calibrarGiro(uint16_t n = 500);

  static constexpr float ACC_LSB_G   = 8192.0f;  // +-4 g
  static constexpr float GYRO_LSB_DPS = 65.5f;   // +-500 dps

 private:
  bool escribir(uint8_t reg, uint8_t val);
  uint8_t _dir = DIR_DEFECTO;
  float _bgx = 0, _bgy = 0, _bgz = 0;
  float _oax = 0, _oay = 0, _oaz = 0;
};
