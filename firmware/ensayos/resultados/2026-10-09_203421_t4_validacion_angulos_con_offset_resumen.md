# Validacion de roll/pitch - t4_madgwick (2026-10-09 20:37)

Por angulo: 6 s para alinear y 5 s de registro con el sensor quieto.
Signo de roll/pitch invertido (convencion del inclinometro).
Referencia: valor leido en el inclinometro.

| Eje | Variable | Objetivo [deg] | Referencia [deg] | Media filtro [deg] | Desvio [deg] | Error [deg] | Muestras | Otro eje (media) [deg] |
|---|---|---|---|---|---|---|---|---|
| X | roll | +0 | +0.0 | +6.22 | 0.17 | +6.22 | 502 | pitch -0.2 |
| X | roll | -45 | -45.0 | +43.24 | 0.19 | +88.24 | 502 | pitch +0.8 |
| X | roll | -90 | -90.0 | +87.46 | 0.18 | +177.46 | 509 | pitch +3.9 |
| X | roll | +45 | +44.0 | -44.80 | 0.20 | -88.80 | 500 | pitch -0.4 |
| X | roll | +90 | +90.0 | -86.77 | 0.15 | -176.77 | 500 | pitch -0.4 |
| Y | pitch | +0 | +0.0 | +5.73 | 0.19 | +5.73 | 501 | roll +1.8 |
| Y | pitch | -45 | -45.0 | -45.25 | 0.16 | -0.25 | 503 | roll -0.1 |
| Y | pitch | -90 | -90.0 | -87.46 | 0.20 | +2.54 | 506 | roll +12.4 |
| Y | pitch | +45 | +45.0 | +42.46 | 0.26 | -2.54 | 500 | roll +6.3 |
| Y | pitch | +90 | +90.0 | +79.12 | 0.12 | -10.88 | 502 | roll +17.6 |

Eje X: error absoluto medio (sin +-90) 61.09 deg, maximo 88.80 deg.

Eje Y: error absoluto medio (sin +-90) 2.84 deg, maximo 5.73 deg.

Nota: en pitch = +-90 deg los angulos de Euler son singulares (gimbal lock); esos valores se informan aparte.
