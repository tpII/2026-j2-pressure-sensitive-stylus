#include <Wire.h>
#include <math.h>

#define MPU_ADDR      0x68
#define PWR_MGMT_1    0x6B
#define GYRO_CONFIG   0x1B
#define ACCEL_CONFIG  0x1C
#define ACCEL_XOUT_H  0x3B

#define ACCEL_SENS    16384.0f
#define GYRO_SENS     131.0f
#define DEG_TO_RAD    (M_PI / 180.0f)

#define SAMPLE_FREQ   100.0f
#define BETA          0.033f
#define CALIB_SAMPLES 200

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
static uint8_t i2cFails = 0;
static unsigned long lastMicros = 0;

float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void hwInit() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(PWR_MGMT_1);
    Wire.write(0x00);
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(GYRO_CONFIG);
    Wire.write(0x00);
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACCEL_CONFIG);
    Wire.write(0x00);
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
                    float ax, float ay, float az)
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

        qDot1 -= BETA * s0;
        qDot2 -= BETA * s1;
        qDot3 -= BETA * s2;
        qDot4 -= BETA * s3;
    }

    q0 += qDot1 * (1.0f / SAMPLE_FREQ);
    q1 += qDot2 * (1.0f / SAMPLE_FREQ);
    q2 += qDot3 * (1.0f / SAMPLE_FREQ);
    q3 += qDot4 * (1.0f / SAMPLE_FREQ);

    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm;
    q2 *= recipNorm; q3 *= recipNorm;
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

void calibrateGyro() {
    Serial.println("Calibrando giroscopio - mantener sensor quieto...");
    double sumX = 0, sumY = 0, sumZ = 0;
    int count = 0;

    while (count < CALIB_SAMPLES) {
        float ax, ay, az, gx, gy, gz;
        if (hwRead(ax, ay, az, gx, gy, gz)) {
            sumX += gx; sumY += gy; sumZ += gz;
            count++;
        }
        delay(10);
    }

    gyroBiasX = (float)(sumX / CALIB_SAMPLES);
    gyroBiasY = (float)(sumY / CALIB_SAMPLES);
    gyroBiasZ = (float)(sumZ / CALIB_SAMPLES);

    Serial.print("Bias (rad/s): ");
    Serial.print(gyroBiasX, 5); Serial.print(", ");
    Serial.print(gyroBiasY, 5); Serial.print(", ");
    Serial.println(gyroBiasZ, 5);
    Serial.println("Calibracion lista.");
}

void i2cRecover() {
    Wire.end();
    delayMicroseconds(500);
    Wire.begin(41, 42);
    Wire.setClock(100000);
    hwInit();
    delay(10);
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Wire.begin(41, 42);
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
    calibrateGyro();
    Serial.println("q0,q1,q2,q3,roll,pitch,yaw");
}

void loop() {
    unsigned long now = micros();
    if ((now - lastMicros) < (unsigned long)(1000000UL / SAMPLE_FREQ)) return;
    lastMicros = now;

    float ax, ay, az, gx, gy, gz;
    if (!hwRead(ax, ay, az, gx, gy, gz)) {
        if (++i2cFails >= 5) { i2cRecover(); i2cFails = 0; }
        return;
    }
    i2cFails = 0;

    gx -= gyroBiasX;
    gy -= gyroBiasY;
    gz -= gyroBiasZ;

    madgwickUpdate(gx, gy, gz, ax, ay, az);

    Serial.print(q0, 4); Serial.print(",");
    Serial.print(q1, 4); Serial.print(",");
    Serial.print(q2, 4); Serial.print(",");
    Serial.print(q3, 4); Serial.print(",");
    Serial.print(getRoll(),  2); Serial.print(",");
    Serial.print(getPitch(), 2); Serial.print(",");
    Serial.println(getYaw(), 2);
}
