// =============================================================================
// ENSAYO DE BETA - beta adaptativo (0,033 / 0,5) vs beta fijo 0,1
// Basado en firmware/madgwick_filter/madgwick_filter.ino (misma lectura del
// MPU6050, misma calibracion, mismo detector de reposo y mismo filtro).
//
// Los DOS filtros corren EN PARALELO sobre las MISMAS muestras: una sola corrida
// alcanza para compararlos, y los dos ven exactamente el mismo movimiento (en dos
// corridas seguidas el movimiento manual nunca sale igual).
//
//   Filtro A: beta adaptativo (BETA_MOV en movimiento, BETA_REP en reposo)
//   Filtro B: beta fijo (BETA_FIJO)
//
// Uso (Arduino IDE, placa "ESP32 Dev Module", monitor serie a 921600):
//   1. Dejar el sensor quieto al arrancar (calibracion, ~3 s).
//   2. Seguir las indicaciones del monitor serie (40 s en total):
//        0-10 s  QUIETO
//       10-25 s  INCLINAR de golpe unos 30-45 grados y SOSTENER quieto
//       25-30 s  SACUDIR (moderado, sin golpes)
//       30-40 s  QUIETO (en cualquier posicion)
//   3. Al final se imprime la tabla comparativa y el resultado.
//   Enviar 'r' por el monitor serie para repetir el ensayo.
//
// Metricas (sobre la DIRECCION DE LA GRAVEDAD estimada, como en el ensayo del
// 01/10; asi no influye el yaw ni el gimbal lock):
//   - Ruido en reposo: variacion RMS entre muestras consecutivas (2-10 s).
//   - Error 2 s y 5 s despues de inclinar (promedio de 0,2 s alrededor de ese
//     instante): contra la gravedad medida por el
//     acelerometro en los ultimos 3 s del sostén (22-25 s).
//   - Error durante la sacudida (RMS / max): contra una referencia que arranca en
//     la gravedad del sostén y se rota solo con el giroscopo. En 15 s el
//     giroscopo (con bias compensado) deriva menos de 0,3 grados, y el
//     acelerometro, que es lo que contamina la sacudida, no interviene.
//   - Error 1 s despues de la sacudida: recuperacion al volver al reposo.
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
#define DEG_TO_RAD_F  (M_PI / 180.0f)
#define RAD_TO_DEG_F  (180.0f / M_PI)

#define I2C_SDA       21
#define I2C_SCL       22
#define SERIAL_BAUD   921600

#define SAMPLE_FREQ   100.0f
#define PERIOD_US     ((unsigned long)(1000000UL / SAMPLE_FREQ))
#define CALIB_SAMPLES 200

// ---- Los dos betas que se comparan ----
#define BETA_MOV      0.033f   // filtro A en movimiento (valor del paper de Madgwick)
#define BETA_REP      0.1f     // antes 0.5f
#define BETA_FIJO     0.1f     // filtro B (mejor valor del ensayo del 01/10)

// Detector de reposo: identico al de madgwick_filter.ino
#define ZUPT_ACCEL    0.03f
#define ZUPT_GYRO     0.05f
#define ZUPT_ALPHA    0.1f
#define ZUPT_SAMPLES  10

// ---- Rutina (en muestras a 100 Hz) ----
#define N_REPOSO      1000   //  0-10 s
#define N_SOSTEN      1500   // 10-25 s
#define N_SACUDIDA     500   // 25-30 s
#define N_FINAL       1000   // 30-40 s
#define I_SOSTEN      (N_REPOSO)
#define I_SACUDIDA    (N_REPOSO + N_SOSTEN)
#define I_FINAL       (I_SACUDIDA + N_SACUDIDA)
#define N_TOTAL       (I_FINAL + N_FINAL)
#define N_REF_SOSTEN   300   // ultimos 3 s del sostén -> gravedad de referencia
#define UMBRAL_MOV    0.35f  // [rad/s] ~20 grados/s: por encima se considera movimiento
#define SAT_DPS       480.0f // cerca del fondo de escala de +-500 dps

// 1 = imprime ademas cada muestra en CSV (para guardar con un logger)
#define IMPRIMIR_CSV  0

struct Filtro { float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; };

static Filtro fA, fB;
static float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
static float accBiasX = 0, accBiasY = 0, accBiasZ = 0;
static bool quieto = false;
static uint16_t quietCount = 0;
static float linMagF = 0, gyroMagF = 0;
static uint8_t i2cFails = 0;
static unsigned long lastMicros = 0;

// ---- Estado del ensayo ----
static int n = 0;                 // muestra actual de la rutina (0..N_TOTAL)
static bool corriendo = false;
static float prevA[3], prevB[3];
static double ruidoA = 0, ruidoB = 0; static int nRuido = 0;
static double accIni[3], accSost[3], accFin[3];
static float ref[3];              // referencia rotada con el giroscopo (desde 25 s)
static bool saturo = false;
// Buffers: sostén (la referencia se conoce recien al final) y sacudida + final
static float gA1[N_SOSTEN][3], gB1[N_SOSTEN][3], giro1[N_SOSTEN];
static float errA2[N_SACUDIDA + N_FINAL], errB2[N_SACUDIDA + N_FINAL], giro2[N_SACUDIDA + N_FINAL];

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
// Mismo codigo que madgwick_filter.ino, aplicado a un Filtro concreto.
void madgwickUpdate(Filtro &f, float gx, float gy, float gz,
                    float ax, float ay, float az, float beta) {
    float q0 = f.q0, q1 = f.q1, q2 = f.q2, q3 = f.q3;
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
    f.q0 = q0 * recipNorm; f.q1 = q1 * recipNorm; f.q2 = q2 * recipNorm; f.q3 = q3 * recipNorm;
}

// Direccion de la gravedad que estima el filtro, en ejes del sensor
// (es lo que deberia medir el acelerometro en reposo).
void gravedad(const Filtro &f, float v[3]) {
    v[0] = 2.0f * (f.q1*f.q3 - f.q0*f.q2);
    v[1] = 2.0f * (f.q0*f.q1 + f.q2*f.q3);
    v[2] = f.q0*f.q0 - f.q1*f.q1 - f.q2*f.q2 + f.q3*f.q3;
}

// Angulo entre dos vectores [grados]; atan2 es preciso tambien en angulos chicos.
float angulo(const float a[3], const float b[3]) {
    float cx = a[1]*b[2] - a[2]*b[1];
    float cy = a[2]*b[0] - a[0]*b[2];
    float cz = a[0]*b[1] - a[1]*b[0];
    float dot = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
    return atan2f(sqrtf(cx*cx + cy*cy + cz*cz), dot) * RAD_TO_DEG_F;
}

void normalizar(float v[3]) {
    float r = invSqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    v[0] *= r; v[1] *= r; v[2] *= r;
}

// Detector de reposo (igual que updateDisplacement de madgwick_filter.ino)
void actualizarReposo(float ax, float ay, float az, float gx, float gy, float gz) {
    float linMag  = fabsf(sqrtf(ax*ax + ay*ay + az*az) - 1.0f);
    float gyroMag = sqrtf(gx*gx + gy*gy + gz*gz);
    linMagF  += ZUPT_ALPHA * (linMag  - linMagF);
    gyroMagF += ZUPT_ALPHA * (gyroMag - gyroMagF);
    bool quietoAhora = (linMagF < ZUPT_ACCEL && gyroMagF < ZUPT_GYRO);
    if (quietoAhora) { if (quietCount < 0xFFFF) quietCount++; } else quietCount = 0;
    quieto = (quietCount >= ZUPT_SAMPLES);
}

// Orientacion inicial desde el acelerometro (igual para los dos filtros)
void initOrientacion() {
    float ax, ay, az, gx, gy, gz;
    double sx = 0, sy = 0, sz = 0; int k = 0;
    while (k < 20) {
        if (hwRead(ax, ay, az, gx, gy, gz)) {
            sx += ax - accBiasX; sy += ay - accBiasY; sz += az - accBiasZ; k++;
        }
        delay(5);
    }
    ax = sx / k; ay = sy / k; az = sz / k;
    float r = atan2f(ay, az);
    float p = atan2f(-ax, sqrtf(ay*ay + az*az));
    float cr = cosf(r / 2), sr = sinf(r / 2), cp = cosf(p / 2), sp = sinf(p / 2);
    fA.q0 = cr*cp; fA.q1 = sr*cp; fA.q2 = cr*sp; fA.q3 = -sr*sp;
    fB = fA;
}

void calibrateSensors() {
    Serial.println("Calibrando - dejar el sensor QUIETO (en cualquier posicion)...");
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
    Serial.println("Calibracion lista.");
}

// ------------------------------------------------------------------- ensayo --
void iniciarEnsayo() {
    initOrientacion();
    n = 0; nRuido = 0; ruidoA = ruidoB = 0; saturo = false;
    for (int k = 0; k < 3; k++) { accIni[k] = accSost[k] = accFin[k] = 0; }
    quietCount = 0; linMagF = 0; gyroMagF = 0; quieto = false;
    Serial.println();
    Serial.println("===== ENSAYO DE BETA: A = adaptativo, B = fijo =====");
    Serial.printf("A: beta %.3f en movimiento / %.3f en reposo    B: beta %.3f\n", BETA_MOV, BETA_REP, BETA_FIJO);
    Serial.println("[ 0 s] QUIETO, no tocar el sensor");
    corriendo = true;
    lastMicros = micros();
}

void avisos() {
    if (n % 100 != 0) return;
    int s = n / 100;
    if (n == I_SOSTEN)        Serial.println("[10 s] >>> INCLINAR AHORA de golpe (30-45 grados) y SOSTENER QUIETO");
    else if (n == I_SACUDIDA) Serial.println("[25 s] >>> SACUDIR (moderado, sin golpes)");
    else if (n == I_FINAL)    Serial.println("[30 s] >>> DEJAR QUIETO (en cualquier posicion)");
    else if (s == 22)         Serial.println("[22 s] seguir sosteniendo quieto...");
    else if (s % 5 == 0 && s > 0) Serial.printf("[%2d s]\n", s);
}

void procesarMuestra(float ax, float ay, float az, float gx, float gy, float gz) {
    actualizarReposo(ax, ay, az, gx, gy, gz);
    madgwickUpdate(fA, gx, gy, gz, ax, ay, az, quieto ? BETA_REP : BETA_MOV);
    madgwickUpdate(fB, gx, gy, gz, ax, ay, az, BETA_FIJO);

    float vA[3], vB[3];
    gravedad(fA, vA); gravedad(fB, vB);
    float gyroMag = sqrtf(gx*gx + gy*gy + gz*gz);
    float maxG = fmaxf(fabsf(gx), fmaxf(fabsf(gy), fabsf(gz))) * RAD_TO_DEG_F;
    if (maxG > SAT_DPS) saturo = true;

    if (n < I_SOSTEN) {
        if (n >= 200) {   // ruido en reposo, 2-10 s
            float dA = angulo(vA, prevA), dB = angulo(vB, prevB);
            ruidoA += dA * dA; ruidoB += dB * dB; nRuido++;
        }
        if (n >= 500) { accIni[0] += ax; accIni[1] += ay; accIni[2] += az; }
    } else if (n < I_SACUDIDA) {
        int i = n - I_SOSTEN;
        for (int k = 0; k < 3; k++) { gA1[i][k] = vA[k]; gB1[i][k] = vB[k]; }
        giro1[i] = gyroMag;
        if (i >= N_SOSTEN - N_REF_SOSTEN) { accSost[0] += ax; accSost[1] += ay; accSost[2] += az; }
    } else {
        if (n == I_SACUDIDA) {   // arranca la referencia en la gravedad medida en el sostén
            ref[0] = accSost[0]; ref[1] = accSost[1]; ref[2] = accSost[2];
            normalizar(ref);
        }
        // Un vector fijo en el mundo, visto desde el sensor que gira con w, gira
        // -|w|*dt alrededor de w (dv/dt = v x w). Rotacion exacta (Rodrigues) para
        // no acumular error de discretizacion en giros rapidos.
        float th = gyroMag / SAMPLE_FREQ;
        if (th > 1e-9f) {
            float kx = gx / gyroMag, ky = gy / gyroMag, kz = gz / gyroMag;
            float c = cosf(th), s = sinf(th);
            float kxv0 = ky*ref[2] - kz*ref[1];
            float kxv1 = kz*ref[0] - kx*ref[2];
            float kxv2 = kx*ref[1] - ky*ref[0];
            float kdv  = (kx*ref[0] + ky*ref[1] + kz*ref[2]) * (1.0f - c);
            float rx = ref[0]*c - kxv0*s + kx*kdv;
            float ry = ref[1]*c - kxv1*s + ky*kdv;
            float rz = ref[2]*c - kxv2*s + kz*kdv;
            ref[0] = rx; ref[1] = ry; ref[2] = rz;
            normalizar(ref);
        }
        int j = n - I_SACUDIDA;
        errA2[j] = angulo(vA, ref); errB2[j] = angulo(vB, ref); giro2[j] = gyroMag;
        if (n >= N_TOTAL - 200) { accFin[0] += ax; accFin[1] += ay; accFin[2] += az; }
    }

    if (IMPRIMIR_CSV) {
        Serial.printf("%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n", n, quieto ? 1 : 0,
                      ax, ay, az, gyroMag, vA[0], vA[1], vA[2], vB[0], vB[1], vB[2]);
    }
    for (int k = 0; k < 3; k++) { prevA[k] = vA[k]; prevB[k] = vB[k]; }
}

// Ultima muestra en [desde, hasta) con giro por encima del umbral (-1 si no hay)
int ultimoMovimiento(const float *giro, int desde, int hasta) {
    int ult = -1;
    for (int i = desde; i < hasta; i++) if (giro[i] > UMBRAL_MOV) ult = i;
    return ult;
}

// Error medio en una ventana de 0,2 s que arranca en i (evita que una sola
// muestra ruidosa decida la metrica)
float errVentana(float g[][3], int i, const float r[3]) {
    float s = 0; int k = 0;
    for (; k < 20 && i + k < N_SOSTEN; k++) s += angulo(g[i + k], r);
    return s / k;
}
float medVentana(const float *e, int j) {
    float s = 0; int k = 0;
    for (; k < 20 && j + k < N_SACUDIDA + N_FINAL; k++) s += e[j + k];
    return s / k;
}

void fila(const char *nombre, float a, float b, int &vA, int &vB) {
    const char *gana = (a < b) ? "A" : (b < a) ? "B" : "=";
    if (a < b) vA++; else if (b < a) vB++;
    Serial.printf("%-34s %8.2f %8.2f    %s\n", nombre, a, b, gana);
}

void analizar() {
    corriendo = false;
    bool valido = true;
    Serial.println();
    Serial.println("===== RESULTADOS =====");

    float dIni[3] = {(float)accIni[0], (float)accIni[1], (float)accIni[2]};
    float refSost[3] = {(float)accSost[0], (float)accSost[1], (float)accSost[2]};
    normalizar(dIni); normalizar(refSost);
    float inclinacion = angulo(dIni, refSost);

    // Fin de la inclinacion: ultimo movimiento en los primeros 10 s del sostén
    int iFin = ultimoMovimiento(giro1, 0, 1000);
    int movSosten = ultimoMovimiento(giro1, N_SOSTEN - N_REF_SOSTEN, N_SOSTEN);
    int jFin = ultimoMovimiento(giro2, 0, N_SACUDIDA + N_FINAL);

    Serial.printf("Inclinacion realizada: %.1f grados\n", inclinacion);
    if (iFin < 0)       { Serial.println("ERROR: no se detecto la inclinacion a los 10 s."); valido = false; }
    if (inclinacion < 20.0f) { Serial.println("AVISO: inclinacion menor a 20 grados, la comparacion pierde sentido."); valido = false; }
    if (movSosten >= 0) { Serial.println("AVISO: el sensor se movio en los ultimos 3 s del sosten (22-25 s)."); valido = false; }
    if (jFin < 100)     { Serial.println("AVISO: no se detecto la sacudida."); valido = false; }
    if (saturo)         { Serial.println("AVISO: el giroscopo llego al fondo de escala (sacudida demasiado fuerte)."); valido = false; }

    float dFin[3] = {(float)accFin[0], (float)accFin[1], (float)accFin[2]};
    normalizar(dFin);
    float errRef = angulo(ref, dFin);
    Serial.printf("Control de la referencia del giroscopo al final: %.2f grados (deberia ser < 2)\n", errRef);
    if (errRef > 2.0f || jFin > N_SACUDIDA + N_FINAL - 300) {
        Serial.println("AVISO: referencia poco confiable o el sensor no quedo quieto al final.");
        valido = false;
    }
    if (!valido) {
        Serial.println("\n>>> ENSAYO NO VALIDO. Enviar 'r' para repetir.");
        return;
    }

    float ruidoAf = sqrt(ruidoA / nRuido), ruidoBf = sqrt(ruidoB / nRuido);
    int i2 = min(iFin + 190, N_SOSTEN - 20), i5 = min(iFin + 490, N_SOSTEN - 20);
    float e2A = errVentana(gA1, i2, refSost), e2B = errVentana(gB1, i2, refSost);
    float e5A = errVentana(gA1, i5, refSost), e5B = errVentana(gB1, i5, refSost);

    int jSac = ultimoMovimiento(giro2, 0, N_SACUDIDA + 200);   // fin de la sacudida
    double sA = 0, sB = 0; float mA = 0, mB = 0;
    for (int j = 0; j <= jSac; j++) {
        sA += errA2[j] * errA2[j]; sB += errB2[j] * errB2[j];
        mA = fmaxf(mA, errA2[j]); mB = fmaxf(mB, errB2[j]);
    }
    float rmsA = sqrt(sA / (jSac + 1)), rmsB = sqrt(sB / (jSac + 1));
    int j1 = min(jSac + 90, N_SACUDIDA + N_FINAL - 20);
    float recA = medVentana(errA2, j1), recB = medVentana(errB2, j1);

    int vA = 0, vB = 0;
    Serial.println();
    Serial.printf("%-34s %8s %8s    %s\n", "Metrica [grados]", "A adapt", "B 0,1", "gana");
    fila("Ruido en reposo (por muestra, RMS)", ruidoAf, ruidoBf, vA, vB);
    fila("Error 2 s despues de inclinar", e2A, e2B, vA, vB);
    fila("Error 5 s despues de inclinar", e5A, e5B, vA, vB);
    fila("Error durante la sacudida (RMS)", rmsA, rmsB, vA, vB);
    fila("Error durante la sacudida (max)", mA, mB, vA, vB);
    fila("Error 1 s despues de la sacudida", recA, recB, vA, vB);

    Serial.println();
    if (vA > vB)      Serial.printf(">>> RESULTADO: gana A, beta ADAPTATIVO (%d de 6 metricas)\n", vA);
    else if (vB > vA) Serial.printf(">>> RESULTADO: gana B, beta FIJO %.2f (%d de 6 metricas)\n", BETA_FIJO, vB);
    else              Serial.println(">>> RESULTADO: empate");
    Serial.println("Enviar 'r' para repetir el ensayo.");
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
        Serial.println("Esperando MPU6050...");
        delay(200);
    }
    hwInit();
    delay(100);
    calibrateSensors();
    iniciarEnsayo();
}

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();
        if ((cmd == 'r' || cmd == 'R') && !corriendo) iniciarEnsayo();
    }
    if (!corriendo) return;

    unsigned long now = micros();
    if ((long)(now - lastMicros) < (long)PERIOD_US) return;
    lastMicros += PERIOD_US;
    if ((long)(now - lastMicros) > (long)(5 * PERIOD_US)) lastMicros = now;

    float ax, ay, az, gx, gy, gz;
    if (!hwRead(ax, ay, az, gx, gy, gz)) {
        if (++i2cFails >= 5) { i2cRecover(); i2cFails = 0; }
        return;
    }
    i2cFails = 0;
    gx -= gyroBiasX; gy -= gyroBiasY; gz -= gyroBiasZ;
    ax -= accBiasX;  ay -= accBiasY;  az -= accBiasZ;

    avisos();
    procesarMuestra(ax, ay, az, gx, gy, gz);
    n++;
    if (n >= N_TOTAL) analizar();
}
