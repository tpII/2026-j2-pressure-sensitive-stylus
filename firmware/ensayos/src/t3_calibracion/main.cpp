// TEST 3 - Calibracion estatica, ruido y deriva del giroscopo
// Dejar el sensor QUIETO y APOYADO PLANO (chip hacia arriba) durante todo el test.
//
// Fase 1 (~10 s): media y desvio estandar de cada eje (offset y ruido).
// Fase 2 (60 s):  integra el giroscopo SIN y CON correccion de bias y muestra
//                 cuantos grados "se inventa" -> dato para el analisis de deriva.
#include <Arduino.h>
#include "MPU6050Raw.h"

constexpr uint32_t FREC_HZ = 100;
constexpr uint32_t PERIODO_US = 1000000UL / FREC_HZ;
constexpr int N_CAL = 1000;          // 10 s a 100 Hz
constexpr uint32_t DERIVA_S = 60;

MPU6050Raw imu;

// Espera activa hasta el proximo periodo
void esperarPeriodo(uint32_t &proximo) {
  while ((int32_t)(micros() - proximo) < 0) {}
  proximo += PERIODO_US;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1500);
  Serial.println("\n=== TEST 3: Calibracion y deriva ===");
  while (!imu.begin()) { Serial.println("MPU6050 no responde."); delay(1000); }
  Serial.println("Dejar el sensor quieto y plano. Arranca en 3 s...");
  delay(3000);

  // ---------------- Fase 1: offsets y ruido ----------------
  double s[6] = {0}, s2[6] = {0};
  ImuMuestra m;
  uint32_t proximo = micros();
  for (int i = 0; i < N_CAL; i++) {
    esperarPeriodo(proximo);
    imu.leer(m);
    float v[6] = {m.ax, m.ay, m.az, m.gx, m.gy, m.gz};
    for (int k = 0; k < 6; k++) { s[k] += v[k]; s2[k] += (double)v[k] * v[k]; }
  }
  const char *nom[6] = {"ax [g]", "ay [g]", "az [g]", "gx [dps]", "gy [dps]", "gz [dps]"};
  float media[6], desvio[6];
  Serial.println("\n--- Fase 1: offsets y ruido (sensor quieto) ---");
  Serial.println("eje        media       desvio(ruido)");
  for (int k = 0; k < 6; k++) {
    media[k] = s[k] / N_CAL;
    desvio[k] = sqrt(s2[k] / N_CAL - (double)media[k] * media[k]);
    Serial.printf("%-9s %+9.4f   %9.4f\n", nom[k], media[k], desvio[k]);
  }
  float normaG = sqrt(media[0]*media[0] + media[1]*media[1] + media[2]*media[2]);
  Serial.printf("|g| medido = %.4f g (ideal 1.000)\n", normaG);
  Serial.printf("Bias giro sugerido: setGyroBias(%.4f, %.4f, %.4f);\n",
                media[3], media[4], media[5]);

  // ---------------- Fase 2: deriva integrando el giro ----------------
  Serial.printf("\n--- Fase 2: deriva integrando el giroscopo %u s ---\n", DERIVA_S);
  Serial.println("t[s]  sinCorr_x  sinCorr_y  sinCorr_z | conCorr_x  conCorr_y  conCorr_z  [deg]");
  float angSin[3] = {0}, angCon[3] = {0};
  const float dt = 1.0f / FREC_HZ;
  proximo = micros();
  for (uint32_t i = 1; i <= DERIVA_S * FREC_HZ; i++) {
    esperarPeriodo(proximo);
    imu.leer(m);
    float g[3] = {m.gx, m.gy, m.gz};
    for (int k = 0; k < 3; k++) {
      angSin[k] += g[k] * dt;
      angCon[k] += (g[k] - media[3 + k]) * dt;
    }
    if (i % (5 * FREC_HZ) == 0) {
      Serial.printf("%4u  %+9.2f  %+9.2f  %+9.2f | %+9.2f  %+9.2f  %+9.2f\n",
                    i / FREC_HZ, angSin[0], angSin[1], angSin[2],
                    angCon[0], angCon[1], angCon[2]);
    }
  }
  Serial.println("\nFIN. Anotar estos valores para el informe (ruido, bias y deriva).");
  Serial.println("Reset (boton EN) para repetir.");
}

void loop() {}
