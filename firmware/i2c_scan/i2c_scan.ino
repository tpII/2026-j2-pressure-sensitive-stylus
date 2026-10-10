#include <Wire.h>

void setup() {
    Serial.begin(115200);
    Wire.begin(41, 42);
    Wire.setClock(100000);
    delay(500);
    Serial.println("Escaneando I2C...");

    int encontrados = 0;
    for (int addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        int error = Wire.endTransmission();
        if (error == 0) {
            Serial.print("Dispositivo en 0x");
            if (addr < 16) Serial.print("0");
            Serial.println(addr, HEX);
            encontrados++;
        }
    }

    if (encontrados == 0) {
        Serial.println("Ningun dispositivo encontrado.");
        Serial.println("Revisar conexiones y pines.");
    } else {
        Serial.print(encontrados);
        Serial.println(" dispositivo(s) encontrado(s).");
    }
}

void loop() {
    delay(3000);
    setup();
}
