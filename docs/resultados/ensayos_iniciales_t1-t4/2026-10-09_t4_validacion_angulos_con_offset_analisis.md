# Analisis - validacion con correccion de offset del acelerometro (2026-10-09, corrida 203421)

Correccion aplicada en firmware: ay = -0.0198 g, az = -0.0855 g (offsets del ensayo 3).

## Errores (filtro - inclinometro), con el signo de cada eje ajustado a la convencion del inclinometro
Nota: en esta corrida el roll ya salio con el signo del inclinometro (la opcion --invertir-signo lo dio vuelta);
el pitch si necesito la inversion. Se toma el signo correcto de cada eje.

| Angulo | Roll sin correccion (09/10 201509) | Roll con correccion | Pitch sin correccion | Pitch con correccion |
|---|---|---|---|---|
| -45 | -3.0 | +1.8 | -3.3 | -0.3 |
| +45 | +3.3 | +0.8 | +2.6 | -2.5 |
| -90 | -6.1 | +2.5 | singular | +2.5 |
| +90 | +3.3 | -3.2 | singular | singular (79.1, roll 17.6) |

- A +-45 grados el error absoluto medio baja de 3.1 a 1.4 grados.
- En roll +-90 el error pasa de sobreestimar (+3 a +6) a subestimar levemente (-3 a +2.5): el offset del eje Z explicaba
  el sesgo; el residuo puede venir del offset/escala del eje X (no medido) o del alineado con el celular.
- Las lecturas a 0 grados dieron 6.2 (roll) y 5.7 (pitch), cuando la simetria de +-45 indica un cero real de ~1 grado:
  probablemente el protoboard no estaba apoyado plano en esos dos registros. Conviene repetir solo el punto de 0.
- Pitch +90 sigue singular (gimbal lock), como se esperaba.

## Repeticion del punto 0 (protoboard apoyado plano)
roll = -1.04 deg (desvio 0.11 deg, 501 muestras), pitch = +1.68 deg, contra 0.0 deg del inclinometro.
Confirma que las lecturas de 6 deg de la corrida 203421 fueron por mal apoyo. El cero residual de 1-1.7 deg
es consistente con la asimetria observada en +-45 y puede deberse a la inclinacion del modulo sobre sus pines
o al alineado con el celular. Detalle: `2026-10-09_t4_punto_cero.txt`.

## Resumen final de la validacion (con correccion de offset)
| Angulo | Error roll [deg] | Error pitch [deg] |
|---|---|---|
| 0 | -1.0 | +1.7 |
| -45 | +1.8 | -0.3 |
| +45 | +0.8 | -2.5 |
| -90 | +2.5 | +2.5 |
| +90 | -3.2 | singular (gimbal lock) |

Error absoluto medio entre 0 y +-45 grados: 1.4 deg. Hasta +-90 grados (sin el punto singular): 1.8 deg; maximo 3.2 deg.
