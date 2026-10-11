#include "MadgwickIMU.h"

static constexpr float RAD2DEG = 57.29577951f;

void MadgwickIMU::update(float gx, float gy, float gz,
                         float ax, float ay, float az, float dt) {
  // Derivada del cuaternion por el giroscopo: qDot = 0.5 * q (x) (0, w)
  float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  float qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  float qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

  // Correccion por gradiente descendente solo si el acelerometro es valido
  float norma = sqrtf(ax * ax + ay * ay + az * az);
  if (norma > 1e-6f) {
    ax /= norma; ay /= norma; az /= norma;

    // Funcion objetivo f: diferencia entre gravedad estimada y medida
    float f0 = 2.0f * (q1 * q3 - q0 * q2) - ax;
    float f1 = 2.0f * (q0 * q1 + q2 * q3) - ay;
    float f2 = 2.0f * (0.5f - q1 * q1 - q2 * q2) - az;

    // Gradiente = J^T * f
    float s0 = -2.0f * q2 * f0 + 2.0f * q1 * f1;
    float s1 =  2.0f * q3 * f0 + 2.0f * q0 * f1 - 4.0f * q1 * f2;
    float s2 = -2.0f * q0 * f0 + 2.0f * q3 * f1 - 4.0f * q2 * f2;
    float s3 =  2.0f * q1 * f0 + 2.0f * q2 * f1;

    float ns = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (ns > 1e-9f) {
      s0 /= ns; s1 /= ns; s2 /= ns; s3 /= ns;
      qDot0 -= beta * s0;
      qDot1 -= beta * s1;
      qDot2 -= beta * s2;
      qDot3 -= beta * s3;
    }
  }

  // Integracion y normalizacion
  q0 += qDot0 * dt; q1 += qDot1 * dt; q2 += qDot2 * dt; q3 += qDot3 * dt;
  float nq = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 /= nq; q1 /= nq; q2 /= nq; q3 /= nq;
}

void MadgwickIMU::initDesdeAcel(float ax, float ay, float az) {
  float r = atan2f(ay, az);
  float p = atan2f(-ax, sqrtf(ay * ay + az * az));
  float cr = cosf(r / 2), sr = sinf(r / 2);
  float cp = cosf(p / 2), sp = sinf(p / 2);
  // yaw = 0
  q0 = cr * cp;
  q1 = sr * cp;
  q2 = cr * sp;
  q3 = -sr * sp;
}

float MadgwickIMU::roll() const {
  return atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD2DEG;
}

float MadgwickIMU::pitch() const {
  float s = 2.0f * (q0 * q2 - q3 * q1);
  if (s > 1.0f) s = 1.0f;
  if (s < -1.0f) s = -1.0f;
  return asinf(s) * RAD2DEG;
}

float MadgwickIMU::yaw() const {
  return atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD2DEG;
}
