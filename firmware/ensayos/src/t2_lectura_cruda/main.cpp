// TEST 2 - Lectura cruda a tasa fija
// Objetivo: ver accel [g] y giro [deg/s] en vivo y medir la tasa REAL
// de muestreo y el jitter del periodo (requisito 50-100 Hz).
//
// Salida en formato Teleplot (extension de VS Code): ">nombre:valor"
#include <Arduino.h>
#include "MPU6050Raw.h"

constexpr uint32_t FREC_HZ = 100;
constexpr uint32_t PERIODO_US = 1000000UL / FREC_HZ;

MPU6050Raw imu;
uint32_t proximo;

// Estadisticas de temporizacion (ventana de 1 s)
uint32_t nMuestras = 0, ultimoT = 0, dtMin = UINT32_MAX, dtMax = 0, tVentana = 0;
uint32_t lecturaMax = 0, fallos = 0;

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1500);
  Serial.println("\n=== TEST 2: Lectura cruda ===");
  while (!imu.begin()) {
    Serial.println("MPU6050 no responde. Correr el TEST 1.");
    delay(1000);
  }
  proximo = micros();
  ultimoT = tVentana = proximo;
}

void loop() {
  // Planificador simple: espera hasta el proximo instante (sin delay()).
  if ((int32_t)(micros() - proximo) < 0) return;
  proximo += PERIODO_US;

  uint32_t t0 = micros();
  ImuMuestra m;
  bool ok = imu.leer(m);
  uint32_t tLectura = micros() - t0;
  if (!ok) { fallos++; return; }

  // Temporizacion
  uint32_t dt = t0 - ultimoT;
  ultimoT = t0;
  if (nMuestras > 0) { dtMin = min(dtMin, dt); dtMax = max(dtMax, dt); }
  lecturaMax = max(lecturaMax, tLectura);
  nMuestras++;

  Serial.printf(">ax:%.3f\n>ay:%.3f\n>az:%.3f\n", m.ax, m.ay, m.az);
  Serial.printf(">gx:%.2f\n>gy:%.2f\n>gz:%.2f\n", m.gx, m.gy, m.gz);

  // Cada 1 s: tasa medida, jitter y tiempo de lectura I2C
  if (t0 - tVentana >= 1000000UL) {
    float seg = (t0 - tVentana) / 1e6f;
    Serial.printf(">tasa_hz:%.1f\n", nMuestras / seg);
    Serial.printf(">dt_min_us:%u\n>dt_max_us:%u\n", dtMin, dtMax);
    Serial.printf(">lectura_i2c_max_us:%u\n>fallos_i2c:%u\n", lecturaMax, fallos);
    Serial.printf(">temp_c:%.1f\n", m.tempC);
    nMuestras = 0; dtMin = UINT32_MAX; dtMax = 0; lecturaMax = 0;
    tVentana = t0;
  }
}
