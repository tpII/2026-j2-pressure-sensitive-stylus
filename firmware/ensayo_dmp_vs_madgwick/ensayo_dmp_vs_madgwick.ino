// =============================================================================
// ENSAYO DMP vs MADGWICK - J2 Pressure Sensitive Stylus | Taller de Proyecto II - UNLP
//
// Pregunta: ¿el DMP del propio MPU6050 (fusion de InvenSense dentro de la IMU)
// estima la orientacion mejor que el Madgwick que se desarrollo por software?
//
// Para que la comparacion sea justa, en cada muestra se guardan JUNTOS:
//   - el cuaternion que entrega el DMP (paquete del FIFO, 100 Hz), y
//   - la lectura cruda del acelerometro y del giroscopo (registros 0x3B..0x48),
//     leida apenas llega el paquete.
// El Madgwick (beta fijo, adaptativo y beta = 0 en movimiento) se calcula en la
// PC sobre esa misma lectura cruda, con tools/comparar_dmp_madgwick.py. Asi todas
// las variantes ven exactamente los mismos datos.
//
// Libreria: "MPU6050" de Electronic Cats (Gestor de librerias de Arduino), que
// trae MPU6050_6Axis_MotionApps20.h (el firmware del DMP de i2cdevlib).
// Placa: ESP32 Dev Module. Conexion igual que siempre: SDA 21, SCL 22. El pin INT
// no hace falta (se consulta el FIFO).
//
// IMPORTANTE:
//  - Al arrancar, el sensor tiene que estar QUIETO y PLANO (cara del chip hacia
//    arriba): CalibrateAccel/CalibrateGyro ajustan los offsets suponiendo +1 g en z.
//  - El DMP fija las escalas en +-2 g y +-2000 grados/s (el firmware propio usa
//    +-4 g y +-500 grados/s). Se leen de los registros y se informan en #cfg.
//    Con +-2 g, una sacudida fuerte puede saturar el acelerometro (columna sat).
//  - El DMP tarda unos 15-20 s en estabilizarse despues de arrancar: esperar
//    quieto antes de grabar.
//
// Salida (921600 baudios), una linea por paquete del DMP:
//   M,t_ms,dt_us,dw,dx,dy,dz,ax,ay,az,gx,gy,gz,sat,perdidos
//   dw..dz: cuaternion del DMP (sensor -> mundo, misma convencion que el Madgwick)
//   ax..az [g], gx..gz [grados/s]: lectura cruda en la misma muestra
//   perdidos: paquetes salteados hasta ahora, estimados por el dt (si el loop se atrasa)
// Mensajes: lineas que empiezan con '#'.
// =============================================================================
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define I2C_SDA     21
#define I2C_SCL     22
#define SERIAL_BAUD 921600

MPU6050 mpu;
uint8_t fifoBuffer[64];
uint16_t packetSize = 0;
float accSens = 16384.0f, gyroSens = 16.4f;   // se recalculan con los registros
unsigned long t0 = 0, tPrev = 0;
uint32_t perdidos = 0;

void leerEscalas() {
    uint8_t ac = 0, gc = 0;
    I2Cdev::readByte(0x68, 0x1C, &ac);   // ACCEL_CONFIG
    I2Cdev::readByte(0x68, 0x1B, &gc);   // GYRO_CONFIG
    int afs = (ac >> 3) & 3, gfs = (gc >> 3) & 3;
    accSens = 16384.0f / (1 << afs);     // 16384, 8192, 4096, 2048 LSB/g
    gyroSens = 131.0f / (1 << gfs);      // 131, 65.5, 32.8, 16.4 LSB/(grados/s)
    Serial.printf("#cfg escala_acc_g=%d escala_giro_dps=%d\n", 2 << afs, 250 << gfs);
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(500);
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    mpu.initialize();
    while (!mpu.testConnection()) {
        Serial.println("# Esperando MPU6050... (revisar SDA=21, SCL=22, VCC, GND)");
        delay(500);
        mpu.initialize();
    }
    Serial.println("# Cargando el firmware del DMP...");
    uint8_t st = mpu.dmpInitialize();
    if (st != 0) {
        Serial.printf("# ERROR: dmpInitialize devolvio %u (1 = carga de memoria, 2 = configuracion)\n", st);
        while (true) delay(1000);
    }
    Serial.println("# Calibrando offsets - sensor QUIETO y PLANO (chip hacia arriba)...");
    mpu.CalibrateAccel(6);
    // La libreria lleva z a 32768 >> FS (2 g, satura) en vez de 16384 >> FS (1 g).
    // Se corrige midiendo: el registro de offset esta en escala +-16 g (2048 LSB/g),
    // 8 veces mas grueso que la lectura en +-2 g. Se conserva el bit 0 del registro.
    mpu.setZAccelOffset(mpu.getZAccelOffset() - 2048);
    for (int it = 0; it < 4; it++) {
        delay(50);
        long suma = 0;
        for (int k = 0; k < 200; k++) { suma += mpu.getAccelerationZ(); delay(2); }
        float err = (16384 >> mpu.getFullScaleAccelRange()) - suma / 200.0f;
        int16_t off = mpu.getZAccelOffset();
        mpu.setZAccelOffset(((off + (int16_t)lroundf(err * (1 << mpu.getFullScaleAccelRange()) / 8)) & 0xFFFE) | (off & 1));
    }
    mpu.CalibrateGyro(6);
    Serial.printf("#cfg offsets_acc=%d,%d,%d offsets_giro=%d,%d,%d\n",
                  mpu.getXAccelOffset(), mpu.getYAccelOffset(), mpu.getZAccelOffset(),
                  mpu.getXGyroOffset(), mpu.getYGyroOffset(), mpu.getZGyroOffset());
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    leerEscalas();
    Serial.println("# Calibracion OK - esperar ~20 s quieto a que el DMP se estabilice");
    Serial.println("#cols M,t_ms,dt_us,dw,dx,dy,dz,ax,ay,az,gx,gy,gz,sat,perdidos");
    t0 = millis();
    tPrev = micros();
}

void loop() {
    // dmpGetCurrentFIFOPacket devuelve el paquete MAS NUEVO (y limpia el FIFO si
    // se lleno). Si el loop se atrasa se saltean paquetes: se cuentan por el dt.
    if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return;
    unsigned long ahora = micros();
    if (tPrev && ahora - tPrev > 15000) perdidos += (ahora - tPrev + 5000) / 10000 - 1;

    Quaternion q;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);   // lectura cruda, misma muestra

    bool sat = abs(ax) > 32000 || abs(ay) > 32000 || abs(az) > 32000 ||
               abs(gx) > 32000 || abs(gy) > 32000 || abs(gz) > 32000;
    Serial.printf("M,%lu,%lu,%.6f,%.6f,%.6f,%.6f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%d,%lu\n",
                  millis() - t0, ahora - tPrev, q.w, q.x, q.y, q.z,
                  ax / accSens, ay / accSens, az / accSens,
                  gx / gyroSens, gy / gyroSens, gz / gyroSens, sat ? 1 : 0, (unsigned long)perdidos);
    tPrev = ahora;
}
