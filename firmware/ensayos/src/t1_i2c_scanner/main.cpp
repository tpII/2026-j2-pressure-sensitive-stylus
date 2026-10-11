// TEST 1 - Escaner I2C + WHO_AM_I
// Objetivo: confirmar cableado y soldadura. Debe aparecer un dispositivo en 0x68.
// Cableado: VCC->3V3, GND->GND, SDA->GPIO21, SCL->GPIO22, AD0->GND
#include <Arduino.h>
#include <Wire.h>

constexpr int PIN_SDA = 21;
constexpr int PIN_SCL = 22;

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1500);
  Wire.begin(PIN_SDA, PIN_SCL, 100000);  // 100 kHz para el escaneo
  Serial.println("\n=== TEST 1: Escaner I2C ===");
}

void loop() {
  int encontrados = 0;
  Serial.println("Escaneando 0x01..0x7F ...");
  for (uint8_t dir = 1; dir < 127; dir++) {
    Wire.beginTransmission(dir);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  -> dispositivo en 0x%02X", dir);
      if (dir == 0x68 || dir == 0x69) {
        // Leer WHO_AM_I (0x75)
        Wire.beginTransmission(dir);
        Wire.write(0x75);
        Wire.endTransmission(false);
        Wire.requestFrom(dir, (uint8_t)1);
        uint8_t who = Wire.available() ? Wire.read() : 0xFF;
        Serial.printf("  (MPU6050? WHO_AM_I = 0x%02X%s)", who,
                      who == 0x68 ? " OK original" : " -> clon/compatible, revisar");
      }
      Serial.println();
      encontrados++;
    }
  }
  if (encontrados == 0) {
    Serial.println("  Nada encontrado. Revisar: SDA/SCL invertidos, GND comun,");
    Serial.println("  VCC a 3V3, pines del GY-521 soldados, protoboard bien insertado.");
  }
  Serial.printf("Total: %d\n\n", encontrados);
  delay(3000);
}
