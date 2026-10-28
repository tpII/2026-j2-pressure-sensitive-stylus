// =============================================================================
// firmware/madgwick_filter/madgwick_filter.ino
// Cambios 11/10/2026: (1) calibracion del acelerometro en cualquier orientacion
//                     (2) atenuacion de velocidad VEL_DECAY 0.98 -> 0.995
// Cambio 22/10/2026:  (3) beta en reposo BETA_REPOSO 0.5 -> 0.1
// Buscar "CORRECCION" para ver cada cambio. El resto queda igual al original.
// =============================================================================
#include <Wire.h>
#include <math.h>

#define MPU_ADDR      0x68
#define PWR_MGMT_1    0x6B
#define GYRO_CONFIG   0x1B
#define ACCEL_CONFIG  0x1C
#define ACCEL_XOUT_H  0x3B
#define MPU_CONFIG    0x1A   // DLPF

#define ACCEL_SENS    8192.0f   // +-4 g
#define GYRO_SENS     65.5f     // +-500 dps
#define DEG_TO_RAD    (M_PI / 180.0f)

// Pines I2C: ESP32 DevKit (ESP-WROOM-32) de la catedra = 21/22.
// (En la placa ESP32-S3 de pruebas se usaban 41/42.)
#define I2C_SDA       21
#define I2C_SCL       22
#define SERIAL_BAUD   921600

#define SAMPLE_FREQ   100.0f
#define PERIOD_US     ((unsigned long)(1000000UL / SAMPLE_FREQ))
// CORRECCION (22/10): BETA_REPOSO 0.5 -> 0.1. Ensayo ensayo_beta (dos filtros en
// paralelo sobre las mismas muestras): con 0.5 el ruido de orientacion en reposo era
// 5 veces mayor (0,57 vs 0,11 grados/muestra) sin converger mas rapido que 0.1.
// Con 0.033 / 0.1 el reposo queda igual que beta fijo 0.1 y durante el movimiento
// el error es aproximadamente la mitad.
#define BETA          0.033f   // en movimiento (valor sugerido por Madgwick sin magnetometro)
#define BETA_REPOSO   0.1f     // en reposo (mejor valor del ensayo del 01/10)
#define CALIB_SAMPLES 200

#define G_MS2         9.81f
#define ZUPT_ACCEL    0.03f
#define ZUPT_GYRO     0.05f
// Deteccion de reposo (ZUPT) sobre valores PROMEDIADOS (media movil exponencial,
// ~0,1 s) en lugar de muestras sueltas: el ruido ya no corta la cuenta en reposo,
// y un desplazamiento suave no se confunde con reposo en una sola muestra.
// El reposo se confirma tras ZUPT_SAMPLES muestras seguidas (0,1 s a 100 Hz).
#define ZUPT_ALPHA    0.1f
// Mitigaciones de la doble integracion (ver informe: deriva ~ t^2):
#define ACC_DEADBAND  0.02f    // [g] aceleracion lineal por debajo de esto se toma como 0 (ruido/residuo de gravedad)
// CORRECCION (11/10): con 0.98 la velocidad caia a la mitad en ~0,35 s y un trazo de
// 20 cm se dibujaba como unos pocos cm. El ZUPT ya pone la velocidad en cero en reposo,
// asi que la atenuacion solo tiene que acotar la deriva DURANTE el movimiento.
// 0.995 -> constante de tiempo ~2 s (mitad en ~1,4 s): un trazo de 1-2 s conserva
// la mayor parte de su longitud. Subir a 0.998 si los trazos siguen saliendo cortos;
// bajar a 0.99 si el punto deriva demasiado entre reposos.
#define VEL_DECAY     0.995f
#define ZUPT_SAMPLES  10

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
static float accBiasX = 0.0f, accBiasY = 0.0f, accBiasZ = 0.0f;
static float velX = 0.0f, velY = 0.0f, velZ = 0.0f;
static float posX = 0.0f, posY = 0.0f, posZ = 0.0f;
static bool quieto = false;
static uint16_t quietCount = 0;
static float linMagF = 0.0f, gyroMagF = 0.0f;   // magnitudes filtradas para el ZUPT
static uint8_t i2cFails = 0;
static unsigned long lastMicros = 0;

// 1/sqrt(x) con la FPU del ESP32: igual de rapido y exacto que el "fast inverse
// square root" del paper, y sin el type-punning que el compilador puede romper.
float invSqrt(float x) {
    return 1.0f / sqrtf(x);
}

void hwInit() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(PWR_MGMT_1);
    Wire.write(0x00);
    Wire.endTransmission(true);

    // DLPF ~44 Hz (accel 44 Hz / giro 42 Hz): reduce ruido y aliasing al muestrear a 100 Hz
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_CONFIG);
    Wire.write(0x03);
    Wire.endTransmission(true);

    // Giroscopo +-500 dps (evita saturar en trazos rapidos)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(GYRO_CONFIG);
    Wire.write(0x08);
    Wire.endTransmission(true);

    // Acelerometro +-4 g
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACCEL_CONFIG);
    Wire.write(0x08);
    Wire.endTransmission(true);
}

bool hwRead(float &ax, float &ay, float &az,
            float &gx, float &gy, float &gz)
{
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

    ax = rawAx / ACCEL_SENS;
    ay = rawAy / ACCEL_SENS;
    az = rawAz / ACCEL_SENS;

    gx = (rawGx / GYRO_SENS) * DEG_TO_RAD;
    gy = (rawGy / GYRO_SENS) * DEG_TO_RAD;
    gz = (rawGz / GYRO_SENS) * DEG_TO_RAD;
    return true;
}

void madgwickUpdate(float gx, float gy, float gz,
                    float ax, float ay, float az, float beta)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    qDot1 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    qDot2 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    qDot3 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    qDot4 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        _2q0 = 2.0f*q0;  _2q1 = 2.0f*q1;
        _2q2 = 2.0f*q2;  _2q3 = 2.0f*q3;
        _4q0 = 4.0f*q0;  _4q1 = 4.0f*q1;  _4q2 = 4.0f*q2;
        _8q1 = 8.0f*q1;  _8q2 = 8.0f*q2;
        q0q0 = q0*q0; q1q1 = q1*q1; q2q2 = q2*q2; q3q3 = q3*q3;

        s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
        s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
        s2 = 4.0f*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
        s3 = 4.0f*q1q1*q3 - _2q1*ax + 4.0f*q2q2*q3 - _2q2*ay;

        recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    q0 += qDot1 * (1.0f / SAMPLE_FREQ);
    q1 += qDot2 * (1.0f / SAMPLE_FREQ);
    q2 += qDot3 * (1.0f / SAMPLE_FREQ);
    q3 += qDot4 * (1.0f / SAMPLE_FREQ);

    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm;
    q2 *= recipNorm; q3 *= recipNorm;
}

void updateDisplacement(float ax, float ay, float az,
                        float gx, float gy, float gz)
{
    const float DT = 1.0f / SAMPLE_FREQ;

    float ax_w = (1.0f - 2.0f*(q2*q2 + q3*q3))*ax
               + 2.0f*(q1*q2 - q0*q3)*ay
               + 2.0f*(q1*q3 + q0*q2)*az;

    float ay_w = 2.0f*(q1*q2 + q0*q3)*ax
               + (1.0f - 2.0f*(q1*q1 + q3*q3))*ay
               + 2.0f*(q2*q3 - q0*q1)*az;

    float az_w = 2.0f*(q1*q3 - q0*q2)*ax
               + 2.0f*(q2*q3 + q0*q1)*ay
               + (1.0f - 2.0f*(q1*q1 + q2*q2))*az;

    az_w -= 1.0f;

    // Reposo detectado con el MODULO del acelerometro (|a| ~ 1 g) y el giro, que no
    // dependen de la orientacion estimada: si la orientacion tiene error, la resta de
    // la gravedad deja un residuo grande y antes el ZUPT nunca se activaba.
    float linMag  = fabsf(sqrtf(ax*ax + ay*ay + az*az) - 1.0f);
    float gyroMag = sqrtf(gx*gx + gy*gy + gz*gz);

    linMagF  += ZUPT_ALPHA * (linMag  - linMagF);
    gyroMagF += ZUPT_ALPHA * (gyroMag - gyroMagF);
    bool quietoAhora = (linMagF < ZUPT_ACCEL && gyroMagF < ZUPT_GYRO);
    if (quietoAhora) { if (quietCount < 0xFFFF) quietCount++; }
    else quietCount = 0;
    quieto = (quietCount >= ZUPT_SAMPLES);

    // Zona muerta: el residuo de gravedad por error de orientacion (~0,01-0,03 g)
    // integrado dos veces es la principal fuente de deriva.
    if (fabsf(ax_w) < ACC_DEADBAND) ax_w = 0.0f;
    if (fabsf(ay_w) < ACC_DEADBAND) ay_w = 0.0f;
    if (fabsf(az_w) < ACC_DEADBAND) az_w = 0.0f;

    if (quieto) {
        velX = 0.0f; velY = 0.0f; velZ = 0.0f;
    } else {
        velX = VEL_DECAY * velX + ax_w * G_MS2 * DT;
        velY = VEL_DECAY * velY + ay_w * G_MS2 * DT;
        velZ = VEL_DECAY * velZ + az_w * G_MS2 * DT;
    }

    posX += velX * DT;
    posY += velY * DT;
    posZ += velZ * DT;
}

float getRoll()  {
    return atan2f(2.0f*(q2*q3 - q0*q1),
                  2.0f*q0*q0 + 2.0f*q3*q3 - 1.0f) * (180.0f / M_PI);
}
float getPitch() {
    return -asinf(2.0f*(q1*q3 + q0*q2)) * (180.0f / M_PI);
}
float getYaw()   {
    return atan2f(2.0f*(q1*q2 - q0*q3),
                  2.0f*q0*q0 + 2.0f*q1*q1 - 1.0f) * (180.0f / M_PI);
}

// Inicializa el cuaternion a partir del acelerometro (sensor quieto): roll y pitch
// correctos desde el arranque, yaw = 0.
void initOrientacion() {
    float ax, ay, az, gx, gy, gz;
    double sx = 0, sy = 0, sz = 0; int n = 0;
    while (n < 20) {
        if (hwRead(ax, ay, az, gx, gy, gz)) {
            sx += ax - accBiasX; sy += ay - accBiasY; sz += az - accBiasZ; n++;
        }
        delay(5);
    }
    ax = sx / n; ay = sy / n; az = sz / n;
    float r = atan2f(ay, az);
    float p = atan2f(-ax, sqrtf(ay*ay + az*az));
    float cr = cosf(r / 2), sr = sinf(r / 2), cp = cosf(p / 2), sp = sinf(p / 2);
    q0 = cr*cp; q1 = sr*cp; q2 = cr*sp; q3 = -sr*sp;
}

void calibrateSensors() {
    // CORRECCION (11/10): antes se suponia el sensor PLANO (gravedad = +1 g en Z) y
    // todo lo que no fuera (0,0,1) se tomaba como bias. En la protoboard el sensor
    // esta de costado (gravedad sobre X): se "restaba" 1 g de X y se sumaba 1 g a Z,
    // el filtro creia que estaba plano y al girarlo la gravedad mal restada se
    // integraba como desplazamiento. Ahora la gravedad se toma en la direccion
    // medida, sea cual sea la posicion del sensor durante la calibracion.
    Serial.println("Calibrando - dejar el sensor QUIETO (en cualquier posicion)...");
    double sumGx = 0, sumGy = 0, sumGz = 0;
    double sumAx = 0, sumAy = 0, sumAz = 0;
    int count = 0;

    while (count < CALIB_SAMPLES) {
        float ax, ay, az, gx, gy, gz;
        if (hwRead(ax, ay, az, gx, gy, gz)) {
            sumGx += gx; sumGy += gy; sumGz += gz;
            sumAx += ax; sumAy += ay; sumAz += az;
            count++;
        }
        delay(10);
    }

    gyroBiasX = (float)(sumGx / CALIB_SAMPLES);
    gyroBiasY = (float)(sumGy / CALIB_SAMPLES);
    gyroBiasZ = (float)(sumGz / CALIB_SAMPLES);

    // Promedio del acelerometro en reposo = gravedad (1 g, direccion desconocida) + bias.
    // Se toma como gravedad el vector unitario en la direccion medida y el resto como
    // bias. Asi se corrige el error de escala en la direccion de la gravedad y el
    // filtro arranca con la orientacion real (de costado, inclinado o plano).
    float mAx = (float)(sumAx / CALIB_SAMPLES);
    float mAy = (float)(sumAy / CALIB_SAMPLES);
    float mAz = (float)(sumAz / CALIB_SAMPLES);
    float mNorm = sqrtf(mAx*mAx + mAy*mAy + mAz*mAz);
    if (mNorm < 0.5f || mNorm > 1.5f) {
        // Lectura incoherente (se movio o fallo el sensor): no se aplica bias de acelerometro
        Serial.println("Aviso: |a| fuera de rango durante la calibracion, bias de acelerometro = 0");
        accBiasX = accBiasY = accBiasZ = 0.0f;
    } else {
        accBiasX = mAx - mAx / mNorm;
        accBiasY = mAy - mAy / mNorm;
        accBiasZ = mAz - mAz / mNorm;
    }

    Serial.print("Bias giro (rad/s): ");
    Serial.print(gyroBiasX, 5); Serial.print(", ");
    Serial.print(gyroBiasY, 5); Serial.print(", ");
    Serial.println(gyroBiasZ, 5);

    Serial.print("Bias accel (g): ");
    Serial.print(accBiasX, 5); Serial.print(", ");
    Serial.print(accBiasY, 5); Serial.print(", ");
    Serial.println(accBiasZ, 5);
    Serial.println("Calibracion lista.");
}

void i2cRecover() {
    Wire.end();
    delayMicroseconds(500);
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);
    hwInit();
    delay(10);
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(500);
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);

    while (true) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x75);
        if (Wire.endTransmission(false) == 0) {
            Wire.requestFrom(MPU_ADDR, 1, true);
            if (Wire.available() && Wire.read() == 0x68) break;
        }
        Serial.println("Esperando MPU6050...");
        delay(200);
    }

    hwInit();
    delay(100);
    calibrateSensors();
    initOrientacion();
    Serial.println("q0,q1,q2,q3,roll,pitch,yaw,px,py,pz,quieto,linMagF");
    lastMicros = micros();
}

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'z' || cmd == 'Z') initOrientacion();   // reinicia la orientacion (sensor quieto)
        if (cmd == 'c' || cmd == 'C') {
            posX = 0.0f; posY = 0.0f; posZ = 0.0f;
            velX = 0.0f; velY = 0.0f; velZ = 0.0f;
        }
    }

    // Periodo fijo: se avanza el instante objetivo en PERIOD_US (no se acumula atraso).
    unsigned long now = micros();
    if ((long)(now - lastMicros) < (long)PERIOD_US) return;
    lastMicros += PERIOD_US;
    if ((long)(now - lastMicros) > (long)(5 * PERIOD_US)) lastMicros = now;  // tras un corte largo, resincroniza

    float ax, ay, az, gx, gy, gz;
    if (!hwRead(ax, ay, az, gx, gy, gz)) {
        if (++i2cFails >= 5) { i2cRecover(); i2cFails = 0; }
        return;
    }
    i2cFails = 0;

    gx -= gyroBiasX;
    gy -= gyroBiasY;
    gz -= gyroBiasZ;

    ax -= accBiasX;
    ay -= accBiasY;
    az -= accBiasZ;

    madgwickUpdate(gx, gy, gz, ax, ay, az, quieto ? BETA_REPOSO : BETA);
    updateDisplacement(ax, ay, az, gx, gy, gz);

    Serial.print(q0, 4); Serial.print(",");
    Serial.print(q1, 4); Serial.print(",");
    Serial.print(q2, 4); Serial.print(",");
    Serial.print(q3, 4); Serial.print(",");
    Serial.print(getRoll(),  2); Serial.print(",");
    Serial.print(getPitch(), 2); Serial.print(",");
    Serial.print(getYaw(),   2); Serial.print(",");
    Serial.print(posX, 4); Serial.print(",");
    Serial.print(posY, 4); Serial.print(",");
    Serial.print(posZ, 4); Serial.print(",");
    Serial.print(quieto ? 1 : 0);   Serial.print(",");
    Serial.println(linMagF, 4);      // diagnostico: | |a| - 1 g | filtrado [g] (umbral ZUPT_ACCEL)
}
