# Validacion de roll/pitch - t4_madgwick (2026-10-09 20:17)

Por angulo: 6 s para alinear y 5 s de registro con el sensor quieto.
Referencia: valor leido en el inclinometro.

| Eje | Variable | Objetivo [deg] | Referencia [deg] | Media filtro [deg] | Desvio [deg] | Error [deg] | Muestras | Otro eje (media) [deg] |
|---|---|---|---|---|---|---|---|---|
| X | roll | +0 | -3.0 | -0.95 | 0.22 | +2.05 | 501 | pitch +1.8 |
| X | roll | -45 | -45.0 | +48.03 | 0.16 | +93.03 | 502 | pitch -4.6 |
| X | roll | -90 | +0.0 | +96.13 | 0.15 | +96.13 | 500 | pitch -3.7 |
| X | roll | +45 | +45.0 | -48.29 | 0.17 | -93.29 | 500 | pitch -4.5 |
| X | roll | +90 | +90.0 | -93.31 | 0.16 | -183.31 | 500 | pitch -2.2 |
| Y | pitch | +0 | +0.0 | -0.92 | 0.40 | -0.92 | 500 | roll -1.4 |
| Y | pitch | -45 | -45.0 | +48.33 | 0.32 | +93.33 | 505 | roll +0.5 |
| Y | pitch | -90 | -90.0 | +81.69 | 4.25 | +171.69 | 501 | roll -46.3 |
| Y | pitch | +45 | +46.0 | -48.56 | 0.26 | -94.56 | 501 | roll -8.6 |
| Y | pitch | +90 | +90.0 | -83.58 | 0.12 | -173.58 | 502 | roll -110.3 |

Eje X: error absoluto medio (sin +-90) 62.79 deg, maximo 93.29 deg.

Eje Y: error absoluto medio (sin +-90) 62.94 deg, maximo 94.56 deg.

Nota: en pitch = +-90 deg los angulos de Euler son singulares (gimbal lock); esos valores se informan aparte.
