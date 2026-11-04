#include <Arduino.h>

/*
 * main.cpp (PlatformIO) — Streaming CRUDO del sensor Hall SS49E (Arduino Uno)
 *
 * Conexión:
 *   SS49E VCC -> 3.3V
 *   SS49E GND -> GND
 *   SS49E OUT -> A0
 *
 * - Lectura directa con analogRead(A0): SIN filtros, SIN promedios,
 *   SIN delay de estabilización, SIN autotara.
 * - Envío por Serial a 115200 baudios, una línea JSON por muestra:
 *     {"raw":342,"voltage":1.672}
 * - Tasa: 50 Hz (período 20 ms) temporizado con millis(), no bloqueante.
 */

const uint8_t  SENSOR_PIN   = A0;
const uint32_t BAUD         = 115200;
const uint32_t PERIOD_MS    = 20;      // 50 Hz
const float    VREF         = 5.0;     // Referencia del ADC (DEFAULT = AVcc = 5V)
const float    ADC_MAX      = 1023.0;  // ADC de 10 bits

uint32_t lastSample = 0;

void setup() {
  Serial.begin(BAUD);
  analogReference(DEFAULT);            // 5V
  lastSample = millis();
}

void loop() {
  uint32_t now = millis();
  if (now - lastSample >= PERIOD_MS) {
    lastSample += PERIOD_MS;           // mantiene la cadencia sin deriva acumulada

    int   raw     = analogRead(SENSOR_PIN);   // valor crudo 0..1023
    float voltage = raw * VREF / ADC_MAX;

    // ~30 bytes/línea * 50 Hz = ~1.5 kB/s (muy por debajo de 11.5 kB/s @115200)
    Serial.print(F("{\"raw\":"));
    Serial.print(raw);
    Serial.print(F(",\"voltage\":"));
    Serial.print(voltage, 3);
    Serial.println(F("}"));
  }
}

