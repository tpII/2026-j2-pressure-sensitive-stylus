# Analisis - validacion de roll/pitch (2026-10-09, corrida 201509)

## Ejes y signos
- Los ejes estan bien asignados: al inclinar en X se mueve roll y el pitch queda a menos de 5 grados; al inclinar en Y se mueve pitch.
- El signo esta invertido respecto de la convencion usada (eje X del dibujo del GY-521): +45 da -48, -45 da +48. Es solo convencion; se corrige invirtiendo el signo en el analisis o en el firmware.
- La referencia de -90 en X quedo cargada como 0 (error de tipeo); se toma -90.

## Error con el signo corregido
| Eje | Referencia [deg] | Filtro (signo corregido) [deg] | Error [deg] |
|---|---|---|---|
| X roll | -3 / 0 | +0.95 | ~+1 a +4 (referencia del celular dudosa en 0) |
| X roll | -45 | -48.03 | -3.0 |
| X roll | -90 | -96.13 | -6.1 |
| X roll | +45 | +48.29 | +3.3 |
| X roll | +90 | +93.31 | +3.3 |
| Y pitch | 0 | +0.92 | +0.9 |
| Y pitch | -45 | -48.33 | -3.3 |
| Y pitch | -90 | -81.69 | singular (gimbal lock, desvio 4.3, roll -46) |
| Y pitch | +46 | +48.56 | +2.6 |
| Y pitch | +90 | +83.58 | singular (gimbal lock, roll -110) |

## Causa del error sistematico
El filtro sobreestima el angulo en ~3 grados a 45 y ~3-6 grados a 90, con el mismo signo en ambos sentidos.
Coincide con el offset del eje Z del acelerometro medido en el ensayo 3 (az = -0.0855 g con ese eje horizontal):
atan2(sin45, cos45 - 0.0855) = 48.7 grados y atan2(1, -0.0855) = 94.9 grados, practicamente lo medido (48.0-48.6 y 93.3-96.1).
El filtro no compensa offsets del acelerometro, solo el bias del giroscopo.

## Proximo paso
Restar los offsets del acelerometro medidos en el ensayo 3 (ay = -0.0198 g, az = -0.0855 g) y repetir la validacion.
Se espera bajar el error a ~1 grado. Los valores de pitch = +-90 se informan aparte (singularidad de Euler).
