// =============================================================================
// ENSAYO DE DESPLAZAMIENTO - integracion doble SIN correcciones
// J2 - Pressure Sensitive Stylus | Taller de Proyecto II - UNLP
//
// ¿se puede calcular el desplazamiento del lapiz con esta
// IMU (MPU6050) integrando dos veces la aceleracion?
//
// Sobre las MISMAS muestras se integran dos posiciones, sin ninguna correccion
// (sin ZUPT, sin zona muerta, sin atenuacion de velocidad, sin detector de
// reposo). La integracion no se frena nunca: solo se reinicia con 'c'.
//   A) CON MADGWICK: la aceleracion se rota al mundo con el cuaternion del
//      filtro (beta fijo) y se resta la gravedad medida en la referencia.
//   B) SIN MADGWICK: se resta la gravedad medida en la referencia directamente
//      en ejes del sensor. Solo es valido si el sensor NO se inclina (se puede
//      deslizar y girar alrededor de la vertical).
// Como la gravedad de la referencia se mide con el mismo sensor, el offset del
// acelerometro se cancela en B; lo que queda es ruido + deriva del offset.
//
// Salida (921600 baudios), una linea por muestra (100 Hz):
//   D,t_ms,Au,Av,An,Bu,Bv,Bn,incl_deg,sat,ciclo_us,ax,ay,az,gx,gy,gz
//   Au, Av, Bu, Bv: posicion en el plano de la mesa (perpendicular a la gravedad) [cm]
//   An, Bn: posicion vertical [cm]
//   incl_deg: inclinacion del sensor desde la referencia, segun el Madgwick
//   ax..gz: lectura CRUDA del sensor [g] y [grados/s], para reprocesar en la PC
// Mensajes del firmware: lineas que empiezan con '#'; '#cfg' lleva la calibracion.
//
// Comandos por serie: 'c' = nueva referencia (sensor QUIETO, 1 s): mide la
// gravedad y el bias del giroscopo, reinicia orientacion, velocidad y posicion.
// =============================================================================
#include <Wire.h>
#include <math.h>

#define MPU_ADDR      0x68
#define PWR_MGMT_1    0x6B
#define GYRO_CONFIG   0x1B
#define ACCEL_CONFIG  0x1C
#define ACCEL_XOUT_H  0x3B
#define MPU_CONFIG    0x1A

#define ACCEL_SENS    8192.0f   // +-4 g
#define GYRO_SENS     65.5f     // +-500 dps
#define DEG_TO_RAD_F  (M_PI / 180.0f)
#define RAD_TO_DEG_F  (180.0f / M_PI)

#define I2C_SDA       21
#define I2C_SCL       22
#define SERIAL_BAUD   921600

#define SAMPLE_FREQ   100.0f
#define DT            (1.0f / SAMPLE_FREQ)
#define PERIOD_US     ((unsigned long)(1000000UL / SAMPLE_FREQ))
#define CALIB_SAMPLES 200
#define REF_SAMPLES   100       // muestras promediadas al tomar la referencia (1 s)

// Beta FIJO: el beta adaptativo de madgwick_filter depende del detector de reposo,
// que aca no se usa. 0,033 es el valor en movimiento de madgwick_filter: menos
// contaminacion de la aceleracion lineal durante el trazo. La convergencia lenta
// no importa porque la orientacion inicial se toma de la gravedad medida.
#define BETA          0.033f

#define G_MS2         9.81f
#define SAT_ACC_G     3.9f      // cerca del fondo de escala +-4 g
#define SAT_GYRO_DPS  490.0f    // cerca del fondo de escala +-500 dps

static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
static float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
static float accBiasX = 0, accBiasY = 0, accBiasZ = 0;
static uint8_t i2cFails = 0;
static unsigned long lastMicros = 0, t0Millis = 0;

// Referencia (se toma con 'c')
static float g0[3] = {0, 0, 1};      // gravedad medida en reposo, ejes del sensor [g]
static float gMag = 1;               // su modulo [g]
static float e1[3], e2[3], e3[3];    // base: e1, e2 en el plano de la mesa, e3 vertical
static float R0[3][3];               // sensor -> mundo en el instante de la referencia

// Integradores
static float velA[3], posA[3];   // A: marco del mundo
static float velB[3], posB[3];   // B: ejes del sensor

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
    q0 += qDot1 * DT; q1 += qDot2 * DT; q2 += qDot3 * DT; q3 += qDot4 * DT;
    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

// Matriz sensor -> mundo del cuaternion actual
void matrizRot(float R[3][3]) {
    R[0][0] = 1 - 2*(q2*q2 + q3*q3); R[0][1] = 2*(q1*q2 - q0*q3);     R[0][2] = 2*(q1*q3 + q0*q2);
    R[1][0] = 2*(q1*q2 + q0*q3);     R[1][1] = 1 - 2*(q1*q1 + q3*q3); R[1][2] = 2*(q2*q3 - q0*q1);
    R[2][0] = 2*(q1*q3 - q0*q2);     R[2][1] = 2*(q2*q3 + q0*q1);     R[2][2] = 1 - 2*(q1*q1 + q2*q2);
}

// Integracion doble pura (Euler), sin ninguna correccion
void integrar(const float lin[3], float vel[3], float pos[3]) {
    for (int k = 0; k < 3; k++) {
        vel[k] += lin[k] * G_MS2 * DT;
        pos[k] += vel[k] * DT;
    }
}

// --------------------------------------------------------------- referencia --
void calibrateSensors() {
    Serial.println("# Calibrando - dejar el sensor QUIETO...");
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
        Serial.println("# Aviso: |a| fuera de rango durante la calibracion, bias de acelerometro = 0");
        accBiasX = accBiasY = accBiasZ = 0.0f;
    } else {
        // Calibracion en cualquier orientacion (igual que madgwick_filter)
        accBiasX = mAx - mAx / mNorm; accBiasY = mAy - mAy / mNorm; accBiasZ = mAz - mAz / mNorm;
    }
}

// Nueva referencia con el sensor quieto: gravedad (para A y B), bias del
// giroscopo actualizado, orientacion inicial, base del plano de la mesa y todo en cero.
void tomarReferencia() {
    float ax, ay, az, gx, gy, gz;
    double sx = 0, sy = 0, sz = 0, sgx = 0, sgy = 0, sgz = 0; int k = 0;
    while (k < REF_SAMPLES) {
        if (hwRead(ax, ay, az, gx, gy, gz)) {
            sx += ax - accBiasX; sy += ay - accBiasY; sz += az - accBiasZ;
            sgx += gx; sgy += gy; sgz += gz; k++;
        }
        delay(10);
    }
    g0[0] = sx / k; g0[1] = sy / k; g0[2] = sz / k;
    gMag = sqrtf(g0[0]*g0[0] + g0[1]*g0[1] + g0[2]*g0[2]);
    gyroBiasX = sgx / k; gyroBiasY = sgy / k; gyroBiasZ = sgz / k;

    // Orientacion inicial desde la gravedad medida (yaw = 0)
    float r = atan2f(g0[1], g0[2]);
    float p = atan2f(-g0[0], sqrtf(g0[1]*g0[1] + g0[2]*g0[2]));
    float cr = cosf(r / 2), sr = sinf(r / 2), cp = cosf(p / 2), sp = sinf(p / 2);
    q0 = cr*cp; q1 = sr*cp; q2 = cr*sp; q3 = -sr*sp;
    matrizRot(R0);

    // Base: e3 vertical (sentido de la gravedad medida), e1 y e2 en el plano de la mesa
    for (int i = 0; i < 3; i++) e3[i] = g0[i] / gMag;
    float aux[3] = {1, 0, 0};
    if (fabsf(e3[0]) > 0.9f) { aux[0] = 0; aux[1] = 1; }
    e1[0] = aux[1]*e3[2] - aux[2]*e3[1];
    e1[1] = aux[2]*e3[0] - aux[0]*e3[2];
    e1[2] = aux[0]*e3[1] - aux[1]*e3[0];
    float n1 = invSqrt(e1[0]*e1[0] + e1[1]*e1[1] + e1[2]*e1[2]);
    e1[0] *= n1; e1[1] *= n1; e1[2] *= n1;
    e2[0] = e3[1]*e1[2] - e3[2]*e1[1];
    e2[1] = e3[2]*e1[0] - e3[0]*e1[2];
    e2[2] = e3[0]*e1[1] - e3[1]*e1[0];

    for (int i = 0; i < 3; i++) velA[i] = posA[i] = velB[i] = posB[i] = 0;
    Serial.println("# Referencia tomada");
    Serial.printf("#cfg gravedad_g=%.5f,%.5f,%.5f\n", g0[0], g0[1], g0[2]);
    Serial.printf("#cfg bias_acc_g=%.5f,%.5f,%.5f\n", accBiasX, accBiasY, accBiasZ);
    Serial.printf("#cfg bias_gyro_dps=%.4f,%.4f,%.4f\n",
                  gyroBiasX * RAD_TO_DEG_F, gyroBiasY * RAD_TO_DEG_F, gyroBiasZ * RAD_TO_DEG_F);
    Serial.printf("#cfg beta=%.3f\n", BETA);
    Serial.println("#cols D,t_ms,Au,Av,An,Bu,Bv,Bn,incl_deg,sat,ciclo_us,ax,ay,az,gx,gy,gz");
    t0Millis = millis();
    lastMicros = micros();
}

// --------------------------------------------------------------- setup/loop --
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
        Serial.println("# Esperando MPU6050... (revisar SDA=21, SCL=22, VCC, GND)");
        delay(200);
    }
    hwInit();
    delay(100);
    calibrateSensors();
    Serial.println("# Calibracion OK");
    tomarReferencia();
}

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'c' || cmd == 'C') tomarReferencia();
    }

    unsigned long now = micros();
    if ((long)(now - lastMicros) < (long)PERIOD_US) return;
    lastMicros += PERIOD_US;
    if ((long)(now - lastMicros) > (long)(5 * PERIOD_US)) lastMicros = now;
    unsigned long tIni = micros();

    float rax, ray, raz, rgx, rgy, rgz;
    if (!hwRead(rax, ray, raz, rgx, rgy, rgz)) {
        if (++i2cFails >= 5) { i2cRecover(); i2cFails = 0; }
        return;
    }
    i2cFails = 0;

    // Saturacion (sobre la lectura cruda, antes de restar bias)
    bool sat = fabsf(rax) > SAT_ACC_G || fabsf(ray) > SAT_ACC_G || fabsf(raz) > SAT_ACC_G ||
               fabsf(rgx) * RAD_TO_DEG_F > SAT_GYRO_DPS || fabsf(rgy) * RAD_TO_DEG_F > SAT_GYRO_DPS ||
               fabsf(rgz) * RAD_TO_DEG_F > SAT_GYRO_DPS;

    float gx = rgx - gyroBiasX, gy = rgy - gyroBiasY, gz = rgz - gyroBiasZ;
    float ax = rax - accBiasX,  ay = ray - accBiasY,  az = raz - accBiasZ;

    madgwickUpdate(gx, gy, gz, ax, ay, az, BETA);

    // Inclinacion desde la referencia: angulo entre la gravedad que estima el
    // filtro (en ejes del sensor) y la gravedad medida en la referencia
    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    float cosI = vx*e3[0] + vy*e3[1] + vz*e3[2];
    float incl = acosf(fminf(1.0f, fmaxf(-1.0f, cosI))) * RAD_TO_DEG_F;

    // A) con Madgwick: aceleracion al mundo y se resta (0, 0, |g0|)
    float R[3][3];
    matrizRot(R);
    float linA[3];
    for (int i = 0; i < 3; i++) linA[i] = R[i][0]*ax + R[i][1]*ay + R[i][2]*az;
    linA[2] -= gMag;
    integrar(linA, velA, posA);

    // B) sin Madgwick: se resta la gravedad de la referencia en ejes del sensor
    float linB[3] = {ax - g0[0], ay - g0[1], az - g0[2]};
    integrar(linB, velB, posB);

    // A se pasa a ejes del sensor en la referencia (p = R0^T * posA) para comparar con B
    float pA[3];
    for (int i = 0; i < 3; i++) pA[i] = R0[0][i]*posA[0] + R0[1][i]*posA[1] + R0[2][i]*posA[2];
    // Proyeccion sobre el plano de la mesa (u, v) y la vertical (n), en cm
    float Au = 100.0f * (pA[0]*e1[0] + pA[1]*e1[1] + pA[2]*e1[2]);
    float Av = 100.0f * (pA[0]*e2[0] + pA[1]*e2[1] + pA[2]*e2[2]);
    float An = 100.0f * (pA[0]*e3[0] + pA[1]*e3[1] + pA[2]*e3[2]);
    float Bu = 100.0f * (posB[0]*e1[0] + posB[1]*e1[1] + posB[2]*e1[2]);
    float Bv = 100.0f * (posB[0]*e2[0] + posB[1]*e2[1] + posB[2]*e2[2]);
    float Bn = 100.0f * (posB[0]*e3[0] + posB[1]*e3[1] + posB[2]*e3[2]);

    unsigned long ciclo = micros() - tIni;   // lectura + filtro + integracion (sin el envio)
    Serial.printf("D,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%d,%lu,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f\n",
                  millis() - t0Millis, Au, Av, An, Bu, Bv, Bn, incl, sat ? 1 : 0, ciclo,
                  rax, ray, raz, rgx * RAD_TO_DEG_F, rgy * RAD_TO_DEG_F, rgz * RAD_TO_DEG_F);
}
