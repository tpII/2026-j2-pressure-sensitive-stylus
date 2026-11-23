// =============================================================================
// LAPIZ WIFI - firmware de la version final con transmision Wi-Fi
// J2 - Pressure Sensitive Stylus | Taller de Proyecto II - UNLP
//
// Es firmware/lapiz_orientacion (mismo Madgwick embebido, beta adaptativo 0,033 en
// movimiento / 0,1 en reposo, misma calibracion y mismo Hall) pero en vez de mandar
// todo por el puerto serie manda por Wi-Fi solo lo necesario para dibujar el trazo
// en la SPA (webapp/index.html), igual que el visor de Processing.
//
// Red: el ESP32 levanta su propia red (SoftAP) "J2-Stylus", clave "stylus2026",
// y un servidor WebSocket en ws://192.168.4.1:81/ (como firmware/wifi_stream).
//
// Reparto entre nucleos (como firmware/wifi_stream):
//   core 1  sensado: un timer de hardware lo dispara cada 10 ms (100 Hz). Lee el
//           MPU6050 y el Hall, corre el Madgwick y deja el ultimo estado.
//   core 0  red: atiende el WebSocket y cada 20 ms (50 Hz) manda el ultimo estado.
// Si la red se traba, el sensado no se entera: sigue a 100 Hz.
//
// Trama: una linea de texto por mensaje, 5 campos separados por coma
//   q0,q1,q2,q3,c
//   q0..q3: cuaternion de orientacion (sensor -> mundo, z del mundo hacia arriba)
//   c:      1 = la punta toca (Hall), 0 = en el aire o Hall no conectado
//   ejemplo: 0.7759,0.0484,0.6278,-0.0392,1
//
// Conexiones: MPU6050 SDA -> 21, SCL -> 22. Hall SS49E: 1 VCC -> 3V3, 2 GND -> GND,
// 3 OUT -> GPIO 34. OJO: no GPIO 4 como en lapiz_orientacion, porque el GPIO 4 es
// del ADC2 y el ADC2 no funciona con el Wi-Fi prendido. Sin el Hall anda igual
// (el lapiz se baja con la barra espaciadora en la SPA).
//
// Libreria: WebSockets de Markus Sattler (Gestor de librerias). Placa: ESP32 Dev Module.
// Por el puerto serie (115200) solo informa el estado una vez por segundo.
// =============================================================================
#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <WebSocketsServer.h>

// ---------------------------------------------------------------- red --------
static const char    *AP_SSID    = "J2-Stylus";
static const char    *AP_PASS    = "stylus2026";
static const uint8_t  AP_CANAL   = 6;
static const uint16_t WS_PUERTO  = 81;
static const uint32_t ENVIO_MS   = 20;        // 50 tramas por segundo

// ------------------------------------------------------------- sensores ------
#define MPU_ADDR      0x68
#define PWR_MGMT_1    0x6B
#define GYRO_CONFIG   0x1B
#define ACCEL_CONFIG  0x1C
#define ACCEL_XOUT_H  0x3B
#define MPU_CONFIG    0x1A

#define ACCEL_SENS    8192.0f   // +-4 g
#define GYRO_SENS     65.5f     // +-500 grados/s
#define DEG_TO_RAD_F  (M_PI / 180.0f)
#define RAD_TO_DEG_F  (180.0f / M_PI)

#define I2C_SDA       21
#define I2C_SCL       22

#define SAMPLE_FREQ   100.0f
#define PERIOD_US     ((uint32_t)(1000000UL / SAMPLE_FREQ))
#define BETA          0.033f    // en movimiento
#define BETA_REPOSO   0.1f      // en reposo
#define CALIB_SAMPLES 200

// Deteccion de reposo (igual que lapiz_orientacion)
#define ZUPT_ACCEL    0.03f
#define ZUPT_GYRO     0.05f     // rad/s
#define ZUPT_ALPHA    0.1f
#define ZUPT_SAMPLES  10

// Hall SS49E (igual que lapiz_orientacion, salvo el pin)
#define HALL_PIN           34     // ADC1: funciona con el Wi-Fi prendido
#define HALL_ALFA          0.30f
#define HALL_PICO_MV       300.0f
#define HALL_PICO_CICLOS   2
#define HALL_ON_MV         150.0f
#define HALL_OFF_MV        100.0f
#define HALL_BASE_MUESTRAS 100

static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
static float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
static float accBiasX = 0, accBiasY = 0, accBiasZ = 0;
static bool quieto = false;
static uint16_t quietCount = 0;
static float linMagF = 0, gyroMagF = 0;
static uint8_t i2cFails = 0;
static float hallFilt = 0, hallValido = 0, hallBase = 0;
static uint8_t hallPicos = 0;
static bool hallInit = false, contacto = false, hayHall = false;

// Ultimo estado: lo escribe el sensado (core 1) y lo lee la red (core 0)
struct Estado { float q[4]; bool contacto; };
static Estado ultimo = {{1, 0, 0, 0}, false};
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t muestrasSeg = 0;    // solo para el informe por serie

WebSocketsServer ws(WS_PUERTO);
TaskHandle_t hSensado = NULL;

float invSqrt(float x) { return 1.0f / sqrtf(x); }

// ----------------------------------------------------------------- hardware --
void hwInit() {
    Wire.beginTransmission(MPU_ADDR); Wire.write(PWR_MGMT_1);   Wire.write(0x00); Wire.endTransmission(true);
    Wire.beginTransmission(MPU_ADDR); Wire.write(MPU_CONFIG);   Wire.write(0x03); Wire.endTransmission(true);
    Wire.beginTransmission(MPU_ADDR); Wire.write(GYRO_CONFIG);  Wire.write(0x08); Wire.endTransmission(true);
    Wire.beginTransmission(MPU_ADDR); Wire.write(ACCEL_CONFIG); Wire.write(0x08); Wire.endTransmission(true);
}

bool hwRead(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(MPU_ADDR, 14, true) < 14) return false;
    int16_t rawAx = (Wire.read() << 8) | Wire.read();
    int16_t rawAy = (Wire.read() << 8) | Wire.read();
    int16_t rawAz = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();
    int16_t rawGx = (Wire.read() << 8) | Wire.read();
    int16_t rawGy = (Wire.read() << 8) | Wire.read();
    int16_t rawGz = (Wire.read() << 8) | Wire.read();
    ax = rawAx / ACCEL_SENS; ay = rawAy / ACCEL_SENS; az = rawAz / ACCEL_SENS;
    gx = (rawGx / GYRO_SENS) * DEG_TO_RAD_F;
    gy = (rawGy / GYRO_SENS) * DEG_TO_RAD_F;
    gz = (rawGz / GYRO_SENS) * DEG_TO_RAD_F;
    return true;
}

void i2cRecover() {
    Wire.end(); delayMicroseconds(500);
    Wire.begin(I2C_SDA, I2C_SCL); Wire.setClock(100000);
    hwInit(); delay(10);
}

// ------------------------------------------------------------------- filtro --
void madgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float beta) {
    float recipNorm, s0, s1, s2, s3, qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
    qDot1 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    qDot2 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    qDot3 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    qDot4 = 0.5f * ( q0*gz + q1*gy - q2*gx);
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;
        _2q0 = 2.0f*q0; _2q1 = 2.0f*q1; _2q2 = 2.0f*q2; _2q3 = 2.0f*q3;
        _4q0 = 4.0f*q0; _4q1 = 4.0f*q1; _4q2 = 4.0f*q2;
        _8q1 = 8.0f*q1; _8q2 = 8.0f*q2;
        q0q0 = q0*q0; q1q1 = q1*q1; q2q2 = q2*q2; q3q3 = q3*q3;
        s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
        s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
        s2 = 4.0f*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
        s3 = 4.0f*q1q1*q3 - _2q1*ax + 4.0f*q2q2*q3 - _2q2*ay;
        recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;
        qDot1 -= beta * s0; qDot2 -= beta * s1; qDot3 -= beta * s2; qDot4 -= beta * s3;
    }
    q0 += qDot1 / SAMPLE_FREQ; q1 += qDot2 / SAMPLE_FREQ;
    q2 += qDot3 / SAMPLE_FREQ; q3 += qDot4 / SAMPLE_FREQ;
    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

void detectarReposo(float ax, float ay, float az, float gx, float gy, float gz) {
    float linMag  = fabsf(sqrtf(ax*ax + ay*ay + az*az) - 1.0f);
    float gyroMag = sqrtf(gx*gx + gy*gy + gz*gz);
    linMagF  += ZUPT_ALPHA * (linMag  - linMagF);
    gyroMagF += ZUPT_ALPHA * (gyroMag - gyroMagF);
    if (linMagF < ZUPT_ACCEL && gyroMagF < ZUPT_GYRO) { if (quietCount < 0xFFFF) quietCount++; }
    else quietCount = 0;
    quieto = (quietCount >= ZUPT_SAMPLES);
}

// ------------------------------------------------------------- calibracion ---
void calibrateSensors() {
    Serial.println("Calibrando - dejar el lapiz QUIETO (en cualquier posicion)...");
    double sGx = 0, sGy = 0, sGz = 0, sAx = 0, sAy = 0, sAz = 0;
    int count = 0;
    while (count < CALIB_SAMPLES) {
        float ax, ay, az, gx, gy, gz;
        if (hwRead(ax, ay, az, gx, gy, gz)) {
            sGx += gx; sGy += gy; sGz += gz; sAx += ax; sAy += ay; sAz += az; count++;
        }
        delay(10);
    }
    gyroBiasX = sGx / CALIB_SAMPLES; gyroBiasY = sGy / CALIB_SAMPLES; gyroBiasZ = sGz / CALIB_SAMPLES;
    float mAx = sAx / CALIB_SAMPLES, mAy = sAy / CALIB_SAMPLES, mAz = sAz / CALIB_SAMPLES;
    float mNorm = sqrtf(mAx*mAx + mAy*mAy + mAz*mAz);
    if (mNorm < 0.5f || mNorm > 1.5f) {
        Serial.println("Aviso: |a| fuera de rango durante la calibracion, bias de acelerometro = 0");
        accBiasX = accBiasY = accBiasZ = 0.0f;
    } else {
        accBiasX = mAx - mAx / mNorm; accBiasY = mAy - mAy / mNorm; accBiasZ = mAz - mAz / mNorm;
    }
}

// Orientacion inicial desde el acelerometro (sensor quieto): yaw = 0
void initOrientacion() {
    float ax, ay, az, gx, gy, gz;
    double sx = 0, sy = 0, sz = 0; int n = 0;
    while (n < 20) {
        if (hwRead(ax, ay, az, gx, gy, gz)) { sx += ax - accBiasX; sy += ay - accBiasY; sz += az - accBiasZ; n++; }
        delay(5);
    }
    ax = sx / n; ay = sy / n; az = sz / n;
    float r = atan2f(ay, az);
    float p = atan2f(-ax, sqrtf(ay*ay + az*az));
    float cr = cosf(r / 2), sr = sinf(r / 2), cp = cosf(p / 2), sp = sinf(p / 2);
    q0 = cr*cp; q1 = sr*cp; q2 = cr*sp; q3 = -sr*sp;
}

// --------------------------------------------------------------------- hall --
float hallLeer() {
    float mv = (float)analogReadMilliVolts(HALL_PIN);
    if (!hallInit) { hallValido = hallFilt = mv; hallPicos = 0; hallInit = true; }
    if (fabsf(mv - hallValido) > HALL_PICO_MV) {
        if (++hallPicos >= HALL_PICO_CICLOS) { hallValido = mv; hallPicos = 0; }
    } else { hallValido = mv; hallPicos = 0; }
    hallFilt = HALL_ALFA * hallValido + (1.0f - HALL_ALFA) * hallFilt;
    return hallFilt;
}

void hallMedirBase() {
    double s = 0;
    for (int k = 0; k < HALL_BASE_MUESTRAS; k++) { s += analogReadMilliVolts(HALL_PIN); delay(10); }
    hallBase = s / HALL_BASE_MUESTRAS;
    hallInit = false; contacto = false;
    hayHall = hallBase > 300 && hallBase < 3000;
    Serial.printf("Hall: base %.0f mV%s\n", hallBase,
                  hayHall ? "" : " -> fuera de rango, Hall no conectado (el contacto queda en 0)");
}

// ------------------------------------------------------------ sensado (core 1)
void IRAM_ATTR onTimer() {
    BaseType_t debeCeder = pdFALSE;
    vTaskNotifyGiveFromISR(hSensado, &debeCeder);
    portYIELD_FROM_ISR(debeCeder);
}

void tareaSensado(void *arg) {
    uint32_t cuenta = 0, tVentana = millis();
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        float ax, ay, az, gx, gy, gz;
        if (!hwRead(ax, ay, az, gx, gy, gz)) {
            if (++i2cFails >= 5) { i2cRecover(); i2cFails = 0; }
            continue;
        }
        i2cFails = 0;
        gx -= gyroBiasX; gy -= gyroBiasY; gz -= gyroBiasZ;
        ax -= accBiasX;  ay -= accBiasY;  az -= accBiasZ;

        detectarReposo(ax, ay, az, gx, gy, gz);
        madgwickUpdate(gx, gy, gz, ax, ay, az, quieto ? BETA_REPOSO : BETA);

        if (hayHall) {
            float dHall = fabsf(hallLeer() - hallBase);
            if (!contacto && dHall > HALL_ON_MV) contacto = true;
            else if (contacto && dHall < HALL_OFF_MV) contacto = false;
        }

        portENTER_CRITICAL(&mux);
        ultimo.q[0] = q0; ultimo.q[1] = q1; ultimo.q[2] = q2; ultimo.q[3] = q3;
        ultimo.contacto = contacto;
        portEXIT_CRITICAL(&mux);

        cuenta++;
        if (millis() - tVentana >= 1000) { muestrasSeg = cuenta; cuenta = 0; tVentana += 1000; }
    }
}

// ---------------------------------------------------------------- red (core 0)
void tareaRed(void *arg) {
    uint32_t tEnvio = millis();
    char msg[64];
    for (;;) {
        ws.loop();
        if (millis() - tEnvio >= ENVIO_MS) {
            tEnvio += ENVIO_MS;
            Estado e;
            portENTER_CRITICAL(&mux);
            e = ultimo;
            portEXIT_CRITICAL(&mux);
            int n = snprintf(msg, sizeof(msg), "%.4f,%.4f,%.4f,%.4f,%d",
                             e.q[0], e.q[1], e.q[2], e.q[3], e.contacto ? 1 : 0);
            ws.broadcastTXT(msg, n);
        }
        vTaskDelay(1);
    }
}

// ----------------------------------------------------------------- setup ------
void setup() {
    Serial.begin(115200);
    delay(300);

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);
    while (true) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x75);
        if (Wire.endTransmission(false) == 0) {
            Wire.requestFrom(MPU_ADDR, 1, true);
            if (Wire.available() && Wire.read() == 0x68) break;
        }
        Serial.println("Esperando MPU6050... (revisar SDA=21, SCL=22, VCC, GND)");
        delay(500);
    }
    hwInit();
    delay(100);
    calibrateSensors();
    analogReadResolution(12);
    analogSetPinAttenuation(HALL_PIN, ADC_11db);
    hallMedirBase();
    initOrientacion();
    Serial.println("Calibracion OK");

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS, AP_CANAL);
    ws.begin();
    Serial.printf("Red: %s  clave: %s  ->  ws://%s:%u/\n",
                  AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str(), WS_PUERTO);

    xTaskCreatePinnedToCore(tareaSensado, "sensado", 4096, NULL, 10, &hSensado, 1);
    xTaskCreatePinnedToCore(tareaRed,     "red",     8192, NULL,  5, NULL,      0);

    // timer de hardware a 1 MHz (1 tick = 1 us) que dispara el sensado cada 10 ms
    hw_timer_t *timer = timerBegin(1000000);
    timerAttachInterrupt(timer, &onTimer);
    timerAlarm(timer, PERIOD_US, true, 0);
}

// loop() solo informa por serie una vez por segundo
void loop() {
    Estado e;
    portENTER_CRITICAL(&mux);
    e = ultimo;
    portEXIT_CRITICAL(&mux);
    Serial.printf("%lu muestras/s  clientes=%u  %.4f,%.4f,%.4f,%.4f,%d\n",
                  (unsigned long)muestrasSeg, ws.connectedClients(),
                  e.q[0], e.q[1], e.q[2], e.q[3], e.contacto ? 1 : 0);
    delay(1000);
}
