// =============================================================================
// ENSAYO DE DESPLAZAMIENTO - con orientacion (A) vs sin rotacion (B)
// Basado en firmware/madgwick_filter/madgwick_filter.ino (misma lectura,
// calibracion, filtro con beta 0,033 / 0,1, detector de reposo e integracion).
//
// Pregunta del ensayo: el error del desplazamiento, ¿viene de la doble
// integracion en si, o de combinarla con la orientacion?
//
// Sobre las MISMAS muestras se integran dos desplazamientos:
//   A) CON ORIENTACION (como madgwick_filter): la aceleracion se rota al mundo con
//      el cuaternion del Madgwick y se resta la gravedad (0,0,1 g).
//   B) SIN ROTACION: se resta una gravedad FIJA, medida en reposo al tomar la
//      referencia (tecla C), directamente en ejes del sensor. El filtro no
//      interviene. Solo es valido si el sensor se DESLIZA SIN GIRAR.
//   C) SIN ROTACION y SIN CORRECCIONES: igual que B pero sin zona muerta ni
//      atenuacion de velocidad (solo ZUPT). Muestra cuanto acortan el trazo
//      esas dos correcciones.
//   A y B usan la misma zona muerta, ZUPT y atenuacion de velocidad.
//   OJO: el detector de reposo de madgwick_filter no ve un deslizamiento sin giro
//   (ver DETECTOR_ORIGINAL). Este ensayo usa por defecto un detector corregido.
//
// Como leer el resultado (deslizando el sensor sin girarlo, contra una regla):
//   - B bien y A mal  -> el problema es la orientacion (gravedad mal restada).
//   - A y B igual mal -> el problema es la doble integracion / ruido del sensor.
// Ademas se informa el tiempo de cada ciclo y si el sensor llego al fondo de
// escala, para descartar que el ESP32 o el MPU6050 se saturen.
//
// Salida (921600 baudios), una linea por muestra (100 Hz):
//   D,t_ms,Au,Av,An,Bu,Bv,Bn,Cu,Cv,Cn,quieto,giro_deg,sat,ciclo_us
//   u, v: ejes en el plano de la mesa (perpendicular a la gravedad) [cm]
//   n: eje vertical [cm]; giro_deg: cuanto se INCLINO el sensor desde la referencia
//   (el giro alrededor de la vertical no cuenta: no cambia la gravedad y no afecta a B)
// Mensajes del firmware: lineas que empiezan con '#'.
//
// Comandos por serie: 'c' = nueva referencia (sensor QUIETO): mide la gravedad,
// reinicia orientacion, velocidad y posicion.
// 'u' = cambia el umbral del detector de reposo (0,030 -> 0,020 -> 0,015 g).
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
#define REF_SAMPLES   50        // muestras promediadas al tomar la referencia (0,5 s)

#define BETA          0.033f    // en movimiento
#define BETA_REPOSO   0.1f      // en reposo

// Igual que madgwick_filter.ino
#define G_MS2         9.81f
#define ZUPT_ACCEL    0.03f     // valor inicial; se cambia con 'u'
#define ZUPT_GYRO     0.05f
#define ZUPT_ALPHA    0.1f
#define ZUPT_SAMPLES  10
#define ACC_DEADBAND  0.02f
#define VEL_DECAY     0.995f

// Detector de reposo:
//   0 = |a - gravedad estimada| (aceleracion lineal): detecta tambien un
//       deslizamiento horizontal sin giro.
//   1 = ORIGINAL de madgwick_filter: | |a| - 1 g |. Una aceleracion horizontal
//       casi no cambia el modulo (0,12 g de costado -> |a| = 1,007 g), asi que
//       un deslizamiento sin giro se toma como reposo y el ZUPT anula la velocidad.
#define DETECTOR_ORIGINAL 0

#define SAT_ACC_G     3.9f      // cerca del fondo de escala +-4 g
#define SAT_GYRO_DPS  490.0f    // cerca del fondo de escala +-500 dps

static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
static float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
static float accBiasX = 0, accBiasY = 0, accBiasZ = 0;
static bool quieto = false;
static uint16_t quietCount = 0;
static float linMagF = 0, gyroMagF = 0;
static uint8_t i2cFails = 0;
static unsigned long lastMicros = 0, t0Millis = 0;

// Referencia (se toma con 'c')
static float g0[3] = {0, 0, 1};              // gravedad medida en reposo, ejes del sensor (unitaria)
static float e1[3], e2[3], e3[3];            // base: e1, e2 en el plano de la mesa, e3 vertical
static float R0[3][3];                       // sensor -> mundo en el instante de la referencia
static float gP[3] = {0, 0, 1};              // gravedad de la referencia rotada solo con el giroscopo

// Integradores
static float velA[3], posA[3];   // A: marco del mundo
static float velB[3], posB[3];   // B: ejes del sensor
static float velC[3], posC[3];   // C: ejes del sensor, sin zona muerta ni atenuacion
static float zuptAccel = ZUPT_ACCEL;
const float UMBRALES[] = {0.03f, 0.02f, 0.015f};
static int umbralIdx = 0;

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

// Matriz sensor -> mundo del cuaternion actual (la misma que usa madgwick_filter
// para rotar la aceleracion)
void matrizRot(float R[3][3]) {
    R[0][0] = 1 - 2*(q2*q2 + q3*q3); R[0][1] = 2*(q1*q2 - q0*q3);     R[0][2] = 2*(q1*q3 + q0*q2);
    R[1][0] = 2*(q1*q2 + q0*q3);     R[1][1] = 1 - 2*(q1*q1 + q3*q3); R[1][2] = 2*(q2*q3 - q0*q1);
    R[2][0] = 2*(q1*q3 - q0*q2);     R[2][1] = 2*(q2*q3 + q0*q1);     R[2][2] = 1 - 2*(q1*q1 + q2*q2);
}

void actualizarReposo(float ax, float ay, float az, float gx, float gy, float gz) {
#if DETECTOR_ORIGINAL
    float linMag  = fabsf(sqrtf(ax*ax + ay*ay + az*az) - 1.0f);
#else
    // Gravedad que estima el filtro, en ejes del sensor
    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    float lx = ax - vx, ly = ay - vy, lz = az - vz;
    float linMag  = sqrtf(lx*lx + ly*ly + lz*lz);
#endif
    float gyroMag = sqrtf(gx*gx + gy*gy + gz*gz);
    linMagF  += ZUPT_ALPHA * (linMag  - linMagF);
    gyroMagF += ZUPT_ALPHA * (gyroMag - gyroMagF);
    bool quietoAhora = (linMagF < zuptAccel && gyroMagF < ZUPT_GYRO);
    if (quietoAhora) { if (quietCount < 0xFFFF) quietCount++; } else quietCount = 0;
    quieto = (quietCount >= ZUPT_SAMPLES);
}

// Zona muerta + ZUPT + atenuacion + integracion: identico para A y B
void integrar(float lin[3], float vel[3], float pos[3]) {
    for (int k = 0; k < 3; k++) {
        if (fabsf(lin[k]) < ACC_DEADBAND) lin[k] = 0.0f;
        if (quieto) vel[k] = 0.0f;
        else        vel[k] = VEL_DECAY * vel[k] + lin[k] * G_MS2 * DT;
        pos[k] += vel[k] * DT;
    }
}

// Solo ZUPT, sin zona muerta ni atenuacion (variante C)
void integrarSinCorrecciones(const float lin[3], float vel[3], float pos[3]) {
    for (int k = 0; k < 3; k++) {
        if (quieto) vel[k] = 0.0f;
        else        vel[k] += lin[k] * G_MS2 * DT;
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
        accBiasX = mAx - mAx / mNorm; accBiasY = mAy - mAy / mNorm; accBiasZ = mAz - mAz / mNorm;
    }
}

// Nueva referencia con el sensor quieto: gravedad fija para B, orientacion inicial
// para A, base del plano de la mesa y todo en cero.
void tomarReferencia() {
    float ax, ay, az, gx, gy, gz;
    double sx = 0, sy = 0, sz = 0; int k = 0;
    while (k < REF_SAMPLES) {
        if (hwRead(ax, ay, az, gx, gy, gz)) {
            sx += ax - accBiasX; sy += ay - accBiasY; sz += az - accBiasZ; k++;
        }
        delay(10);
    }
    float mx = sx / k, my = sy / k, mz = sz / k;
    float mn = sqrtf(mx*mx + my*my + mz*mz);
    g0[0] = mx / mn; g0[1] = my / mn; g0[2] = mz / mn;

    // Orientacion inicial desde la gravedad medida (yaw = 0)
    float r = atan2f(g0[1], g0[2]);
    float p = atan2f(-g0[0], sqrtf(g0[1]*g0[1] + g0[2]*g0[2]));
    float cr = cosf(r / 2), sr = sinf(r / 2), cp = cosf(p / 2), sp = sinf(p / 2);
    q0 = cr*cp; q1 = sr*cp; q2 = cr*sp; q3 = -sr*sp;
    matrizRot(R0);

    // Base: e3 vertical (sentido de la gravedad medida), e1 y e2 en el plano de la mesa
    for (int i = 0; i < 3; i++) e3[i] = g0[i];
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

    for (int i = 0; i < 3; i++) { velA[i] = posA[i] = velB[i] = posB[i] = velC[i] = posC[i] = 0; gP[i] = g0[i]; }
    quietCount = ZUPT_SAMPLES; quieto = true; linMagF = 0; gyroMagF = 0;
    Serial.printf("# Referencia tomada. Gravedad en ejes del sensor: %.3f, %.3f, %.3f\n", g0[0], g0[1], g0[2]);
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
    tomarReferencia();
    Serial.println("# Calibracion OK");
    Serial.printf("#cfg umbral_reposo=%.3f g\n", zuptAccel);
    Serial.println("#cols D,t_ms,Au,Av,An,Bu,Bv,Bn,Cu,Cv,Cn,quieto,giro_deg,sat,ciclo_us");
}

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'c' || cmd == 'C') tomarReferencia();
        if (cmd == 'u' || cmd == 'U') {
            umbralIdx = (umbralIdx + 1) % 3;
            zuptAccel = UMBRALES[umbralIdx];
            Serial.printf("#cfg umbral_reposo=%.3f g\n", zuptAccel);
        }
    }

    unsigned long now = micros();
    if ((long)(now - lastMicros) < (long)PERIOD_US) return;
    lastMicros += PERIOD_US;
    if ((long)(now - lastMicros) > (long)(5 * PERIOD_US)) lastMicros = now;
    unsigned long tIni = micros();

    float ax, ay, az, gx, gy, gz;
    if (!hwRead(ax, ay, az, gx, gy, gz)) {
        if (++i2cFails >= 5) { i2cRecover(); i2cFails = 0; }
        return;
    }
    i2cFails = 0;

    // Saturacion (sobre la lectura cruda, antes de restar bias)
    bool sat = fabsf(ax) > SAT_ACC_G || fabsf(ay) > SAT_ACC_G || fabsf(az) > SAT_ACC_G ||
               fabsf(gx) * RAD_TO_DEG_F > SAT_GYRO_DPS || fabsf(gy) * RAD_TO_DEG_F > SAT_GYRO_DPS ||
               fabsf(gz) * RAD_TO_DEG_F > SAT_GYRO_DPS;

    gx -= gyroBiasX; gy -= gyroBiasY; gz -= gyroBiasZ;
    ax -= accBiasX;  ay -= accBiasY;  az -= accBiasZ;

    actualizarReposo(ax, ay, az, gx, gy, gz);
    madgwickUpdate(gx, gy, gz, ax, ay, az, quieto ? BETA_REPOSO : BETA);

    // Cuanto giro el sensor desde la referencia (integral del giro; vale para giros chicos)
    // Inclinacion desde la referencia: la gravedad de la referencia se rota solo con el
    // giroscopo (vector fijo en el mundo visto desde el sensor: dv/dt = v x w, rotacion
    // exacta) y se mide el angulo con g0. El giro alrededor de la vertical no la cambia.
    float wMag = sqrtf(gx*gx + gy*gy + gz*gz);
    float th = wMag * DT;
    if (th > 1e-9f) {
        float kx = gx / wMag, ky = gy / wMag, kz = gz / wMag;
        float c = cosf(th), s = sinf(th);
        float kv0 = ky*gP[2] - kz*gP[1], kv1 = kz*gP[0] - kx*gP[2], kv2 = kx*gP[1] - ky*gP[0];
        float kd = (kx*gP[0] + ky*gP[1] + kz*gP[2]) * (1.0f - c);
        float n0 = gP[0]*c - kv0*s + kx*kd, n1 = gP[1]*c - kv1*s + ky*kd, n2 = gP[2]*c - kv2*s + kz*kd;
        float rn = invSqrt(n0*n0 + n1*n1 + n2*n2);
        gP[0] = n0*rn; gP[1] = n1*rn; gP[2] = n2*rn;
    }
    float cx = gP[1]*g0[2] - gP[2]*g0[1], cy = gP[2]*g0[0] - gP[0]*g0[2], cz = gP[0]*g0[1] - gP[1]*g0[0];
    float giroDeg = atan2f(sqrtf(cx*cx + cy*cy + cz*cz), gP[0]*g0[0] + gP[1]*g0[1] + gP[2]*g0[2]) * RAD_TO_DEG_F;

    // A) con orientacion: aceleracion al mundo y se resta (0,0,1 g)
    float R[3][3];
    matrizRot(R);
    float linA[3];
    for (int i = 0; i < 3; i++) linA[i] = R[i][0]*ax + R[i][1]*ay + R[i][2]*az;
    linA[2] -= 1.0f;
    integrar(linA, velA, posA);

    // B) sin rotacion: se resta la gravedad fija en ejes del sensor
    float linB[3] = {ax - g0[0], ay - g0[1], az - g0[2]};
    integrarSinCorrecciones(linB, velC, posC);   // C: antes de que integrar() aplique la zona muerta
    integrar(linB, velB, posB);

    // A se pasa a ejes del sensor en la referencia (p = R0^T * posA) para comparar con B
    float pA[3], pB[3];
    for (int i = 0; i < 3; i++) {
        pA[i] = R0[0][i]*posA[0] + R0[1][i]*posA[1] + R0[2][i]*posA[2];
        pB[i] = posB[i];
    }
    // Proyeccion sobre el plano de la mesa (u, v) y la vertical (n), en cm
    float Au = 100.0f * (pA[0]*e1[0] + pA[1]*e1[1] + pA[2]*e1[2]);
    float Av = 100.0f * (pA[0]*e2[0] + pA[1]*e2[1] + pA[2]*e2[2]);
    float An = 100.0f * (pA[0]*e3[0] + pA[1]*e3[1] + pA[2]*e3[2]);
    float Bu = 100.0f * (pB[0]*e1[0] + pB[1]*e1[1] + pB[2]*e1[2]);
    float Bv = 100.0f * (pB[0]*e2[0] + pB[1]*e2[1] + pB[2]*e2[2]);
    float Bn = 100.0f * (pB[0]*e3[0] + pB[1]*e3[1] + pB[2]*e3[2]);
    float Cu = 100.0f * (posC[0]*e1[0] + posC[1]*e1[1] + posC[2]*e1[2]);
    float Cv = 100.0f * (posC[0]*e2[0] + posC[1]*e2[1] + posC[2]*e2[2]);
    float Cn = 100.0f * (posC[0]*e3[0] + posC[1]*e3[1] + posC[2]*e3[2]);

    unsigned long ciclo = micros() - tIni;   // lectura + filtro + integracion (sin el envio)
    Serial.printf("D,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.1f,%d,%lu\n",
                  millis() - t0Millis, Au, Av, An, Bu, Bv, Bn, Cu, Cv, Cn, quieto ? 1 : 0, giroDeg, sat ? 1 : 0, ciclo);
}
