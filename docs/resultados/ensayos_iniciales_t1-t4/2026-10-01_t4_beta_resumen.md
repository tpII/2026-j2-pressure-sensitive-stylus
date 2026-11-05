# Test 4 - Calibracion de beta del filtro de Madgwick (2026-10-01)

Archivos: `*_t4_beta_0.03.csv`, `*_t4_beta_0.1.csv`, `*_t4_beta_0.3.csv`, `*_t4_beta_0.5.csv` (datos crudos),
`2026-10-01_t4_beta_comparacion.csv` (metricas), `2026-10-01_t4_beta_comparacion.png` (figura).

## Procedimiento
Cuatro corridas de 40 s a 100 Hz, una por beta, con la misma rutina manual:
0-10 s reposo; ~10 s inclinacion brusca de ~30-45 grados; 10-25 s sostener; 25-30 s sacudir; 30-40 s reposo.

Las metricas se calculan sobre la **direccion de la gravedad estimada por el filtro** (vector derivado de roll y pitch),
no sobre los angulos de Euler por separado: en la posicion de reposo del montaje el pitch quedaba en -80 a -88 grados,
cerca del gimbal lock, donde roll y yaw individuales no son confiables.

## Resultados

| beta | Ruido en reposo [grados/muestra RMS] | Error 2 s despues de inclinar [grados] | Error a 5 s [grados] | Error durante sacudida RMS / max [grados] |
|---|---|---|---|---|
| 0.03 | 0.032 | 6.0 | 2.3 | 6.0 / 11.8 |
| 0.1  | 0.094 | 1.8 | 0.4 | 5.6 / 8.7 |
| 0.3  | 0.286 | 2.7 | 1.9 | 15.3 / 27.1 |
| 0.5  | 0.476 | 2.7 | 0.8 | 7.6 / 12.3 |

## Interpretacion
- El ruido en reposo crece de forma practicamente proporcional a beta (x15 entre 0.03 y 0.5): con beta alto el filtro
  sigue al acelerometro, que es ruidoso.
- Con beta = 0.03 la convergencia es lenta: 2 s despues de inclinar todavia hay 6 grados de error y tarda ~6 s en bajar de 2 grados.
- beta = 0.1 tuvo la mejor convergencia de las cuatro corridas (1.8 grados a 2 s, 0.4 grados a 5 s) con ruido todavia bajo (<0.1 grados/muestra).
- Durante la sacudida todos los valores se desvian 6-15 grados RMS: las aceleraciones del movimiento contaminan la
  referencia de gravedad. La intensidad de la sacudida fue manual y no es igual entre corridas, por lo que esta columna
  es solo orientativa.

**Valor adoptado: beta = 0.1** (compromiso entre ruido en reposo y velocidad de convergencia). Coincide con el valor por defecto del firmware.

## Limitaciones del ensayo
- Movimientos manuales: el angulo de inclinacion (30-47 grados) y la intensidad de la sacudida variaron entre corridas.
- Una sola corrida por beta. Para el informe final convendria repetir con un soporte fijo (plano inclinado a angulo conocido) y 3 corridas por valor.
- El montaje en protoboard deja el eje X del sensor vertical en reposo (pitch ~ -85 grados): conviene definir la orientacion
  de montaje en el lapiz para que la postura tipica de escritura quede lejos de pitch = +-90 grados, o trabajar con el cuaternion.
