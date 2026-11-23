#include <Arduino.h>

/*
 * main.cpp (PlatformIO) — Streaming FILTRADO del sensor Hall SS49E (Arduino Uno)
 *
 * Conexión:
 *   SS49E VCC -> 3.3V
 *   SS49E GND -> GND
 *   SS49E OUT -> A0
 *
 * - Lectura con analogRead(A0) + filtrado digital:
 *     1) Spike rejection: si la muestra difiere de la última válida en más de
 *        SPIKE_THRESHOLD se descarta (falso contacto). Si el salto persiste
 *        SPIKE_CONFIRM_CYCLES ciclos seguidos, se acepta como movimiento real.
 *     2) Filtro exponencial IIR (EMA) con coeficiente ALPHA sobre la muestra validada.
 *   La primera lectura inicializa el filtro para que no arranque desde cero.
 * - Envío por Serial a 115200 baudios, una línea JSON por muestra:
 *     {"raw":342,"voltage":1.672}
 *   "raw" es el valor filtrado redondeado y "voltage" se calcula con el valor filtrado.
 * - Tasa: 50 Hz (período 20 ms) temporizado con millis(), no bloqueante.
 */

#define SPIKE_THRESHOLD       80     // cuentas ADC
#define SPIKE_CONFIRM_CYCLES  2      // ciclos seguidos para aceptar un salto grande
#define ALPHA                 0.20f  // coeficiente del EMA

const uint8_t  SENSOR_PIN   = A0;
const uint32_t BAUD         = 115200;
const uint32_t PERIOD_MS    = 20;      // 50 Hz
const float    VREF         = 5.0;     // Referencia del ADC (DEFAULT = AVcc = 5V)
const float    ADC_MAX      = 1023.0;  // ADC de 10 bits

uint32_t lastSample = 0;

// Estado del filtro
bool    filterInitialized = false;
int     last_valid_raw    = 0;
uint8_t spike_count       = 0;
float   filtered_adc      = 0.0f;

void setup() {
  Serial.begin(BAUD);
  analogReference(DEFAULT);            // 5V
  lastSample = millis();
}

void loop() {
  uint32_t now = millis();
  if (now - lastSample >= PERIOD_MS) {
    lastSample += PERIOD_MS;           // mantiene la cadencia sin deriva acumulada

    int raw_adc = analogRead(SENSOR_PIN);   // valor crudo 0..1023

    // --- Inicialización con la primera lectura ---
    if (!filterInitialized) {
      last_valid_raw    = raw_adc;
      filtered_adc      = (float)raw_adc;
      spike_count       = 0;
      filterInitialized = true;
    }

    // --- 1) Spike rejection ---
    int diff = raw_adc - last_valid_raw;
    if (abs(diff) > SPIKE_THRESHOLD) {
      spike_count++;
      if (spike_count >= SPIKE_CONFIRM_CYCLES) {
        // El salto se sostuvo: es movimiento real
        last_valid_raw = raw_adc;
        spike_count    = 0;
      }
      // Si no, se descarta la muestra y se mantiene last_valid_raw
    } else {
      last_valid_raw = raw_adc;
      spike_count    = 0;
    }
    int valid_adc = last_valid_raw;

    // --- 2) Filtro exponencial IIR (EMA) ---
    filtered_adc = (ALPHA * valid_adc) + ((1.0f - ALPHA) * filtered_adc);

    int   out_raw = (int)(filtered_adc + 0.5f);       // redondeo (valor siempre >= 0)
    float voltage = filtered_adc * VREF / ADC_MAX;

    // ~30 bytes/línea * 50 Hz = ~1.5 kB/s (muy por debajo de 11.5 kB/s @115200)
    Serial.print(F("{\"raw\":"));
    Serial.print(out_raw);
    Serial.print(F(",\"voltage\":"));
    Serial.print(voltage, 3);
    Serial.println(F("}"));
  }
}
