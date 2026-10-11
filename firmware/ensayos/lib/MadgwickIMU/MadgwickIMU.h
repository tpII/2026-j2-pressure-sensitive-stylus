// Filtro de orientacion de Madgwick, version IMU (giroscopo + acelerometro).
// Implementacion propia basada en: S. Madgwick, "An efficient orientation
// filter for inertial and inertial/magnetic sensor arrays" (2010).
//
// Sin magnetometro el yaw NO tiene referencia absoluta: va a derivar.
// Roll y pitch quedan corregidos por la gravedad.
#pragma once
#include <math.h>

class MadgwickIMU {
 public:
  explicit MadgwickIMU(float beta = 0.1f) : beta(beta) {}

  // gx,gy,gz en rad/s; ax,ay,az en cualquier unidad (se normaliza); dt en s.
  void update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

  // Inicializa el cuaternion a partir del acelerometro (sensor quieto),
  // asi no hay que esperar a que el filtro converja desde (1,0,0,0).
  void initDesdeAcel(float ax, float ay, float az);

  // Angulos de Euler en grados (convencion ZYX: yaw-pitch-roll).
  float roll() const;
  float pitch() const;
  float yaw() const;

  float beta;                          // ganancia del filtro (ajustable)
  float q0 = 1, q1 = 0, q2 = 0, q3 = 0; // cuaternion de orientacion
};
