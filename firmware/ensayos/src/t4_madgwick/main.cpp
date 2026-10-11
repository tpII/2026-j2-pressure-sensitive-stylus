// TEST 4 - Filtro de Madgwick embebido
// Objetivo: orientacion (roll/pitch/yaw) a 100 Hz, ajuste de beta en vivo,
// tiempo de computo del filtro y log CSV para analisis offline.
//
// Comandos por el monitor serie (escribir y Enter):
//   b0.05   -> cambia beta a 0.05
//   c       -> recalibra bias del giroscopo (dejar quieto ~2 s)
//   l       -> alterna modo Teleplot <-> modo CSV (para tools/logger.py)
//   z       -> reinicia la orientacion desde el acelerometro
//   a       -> activa/desactiva la correccion de offset del acelerometro
#include <Arduino.h>
#include "MPU6050Raw.h"
#include "MadgwickIMU.h"

constexpr uint32_t FREC_HZ = 100;
constexpr uint32_t PERIODO_US = 1000000UL / FREC_HZ;
constexpr float DEG2RAD = 0.01745329252f;

// Offsets del acelerometro [g] medidos en el ensayo 3 (2026-10-01), con cada eje
// horizontal: ay = -0.0198 g, az = -0.0855 g. El de X no se midio (estaba vertical).
constexpr float ACC_OFF_X = 0.0f, ACC_OFF_Y = -0.0198f, ACC_OFF_Z = -0.0855f;
bool corrAcel = true;



MPU6050Raw imu;
MadgwickIMU filtro(0.1f);

bool modoCSV = false;

void aplicarOffsetAcel() {
  if (corrAcel) imu.setAccelOffset(ACC_OFF_X, ACC_OFF_Y, ACC_OFF_Z);
  else imu.setAccelOffset(0, 0, 0);
}
uint32_t proximo, ultimoT;
uint32_t filtroMaxUs = 0, cicloMaxUs = 0, tVentana = 0, nMuestras = 0;

void calibrar() {
  Serial.println("# Calibrando giroscopo, NO mover...");
  while (!imu.calibrarGiro(500)) {
    Serial.println("# Se movio durante la calibracion, reintentando...");
  }
  Serial.println("# Calibracion OK");
}

void reiniciarOrientacion() {
  ImuMuestra m;
  if (imu.leer(m)) filtro.initDesdeAcel(m.ax, m.ay, m.az);
}

void procesarComandos() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;
  switch (cmd[0]) {
    case 'b': {
      float b = cmd.substring(1).toFloat();
      if (b > 0 && b < 5) { filtro.beta = b; }
      Serial.printf("# beta = %.4f\n", filtro.beta);
      break;
    }
    case 'c': calibrar(); reiniciarOrientacion(); break;
    case 'a':
      corrAcel = !corrAcel; aplicarOffsetAcel(); reiniciarOrientacion();
      Serial.printf("# correccion offset acelerometro: %s\n", corrAcel ? "ON" : "OFF");
      break;
    case 'z': reiniciarOrientacion(); Serial.println("# orientacion reiniciada"); break;
    case 'l':
      modoCSV = !modoCSV;
      if (modoCSV)
        Serial.println("t_us,ax,ay,az,gx,gy,gz,q0,q1,q2,q3,roll,pitch,yaw,beta");
      else
        Serial.println("# modo Teleplot");
      break;
  }
  proximo = micros();  // no "recuperar" los ciclos perdidos
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(20);
  delay(1500);
  Serial.println("\n# === TEST 4: Madgwick ===");
  while (!imu.begin()) { Serial.println("# MPU6050 no responde."); delay(1000); }
  aplicarOffsetAcel();
  calibrar();
  reiniciarOrientacion();
  proximo = ultimoT = tVentana = micros();
}

void loop() {
  procesarComandos();
  if ((int32_t)(micros() - proximo) < 0) return;
  proximo += PERIODO_US;

  uint32_t t0 = micros();
  ImuMuestra m;
  if (!imu.leer(m)) return;

  // dt real medido (no el nominal) para integrar correctamente
  float dt = (t0 - ultimoT) / 1e6f;
  ultimoT = t0;
  if (dt <= 0 || dt > 0.1f) dt = 1.0f / FREC_HZ;

  uint32_t tf = micros();
  filtro.update(m.gx * DEG2RAD, m.gy * DEG2RAD, m.gz * DEG2RAD, m.ax, m.ay, m.az, dt);
  uint32_t filtroUs = micros() - tf;
  filtroMaxUs = max(filtroMaxUs, filtroUs);

  float r = filtro.roll(), p = filtro.pitch(), y = filtro.yaw();

  if (modoCSV) {
    Serial.printf("%u,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.5f,%.5f,%.5f,%.5f,%.2f,%.2f,%.2f,%.3f\n",
                  t0, m.ax, m.ay, m.az, m.gx, m.gy, m.gz,
                  filtro.q0, filtro.q1, filtro.q2, filtro.q3, r, p, y, filtro.beta);
  } else {
    Serial.printf(">roll:%.2f\n>pitch:%.2f\n>yaw:%.2f\n", r, p, y);
  }

  uint32_t cicloUs = micros() - t0;
  cicloMaxUs = max(cicloMaxUs, cicloUs);
  nMuestras++;
  if (!modoCSV && t0 - tVentana >= 1000000UL) {
    Serial.printf(">tasa_hz:%.1f\n", nMuestras * 1e6f / (t0 - tVentana));
    Serial.printf(">filtro_max_us:%u\n>ciclo_max_us:%u\n", filtroMaxUs, cicloMaxUs);
    Serial.printf(">beta:%.3f\n", filtro.beta);
    Serial.printf(">corr_acel:%d\n", corrAcel ? 1 : 0);
    nMuestras = 0; filtroMaxUs = 0; cicloMaxUs = 0; tVentana = t0;
  }
}
