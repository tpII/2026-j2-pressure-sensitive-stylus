# Temporizacion - t4_madgwick (grabado con nombre por defecto t2) (2026-10-01 20:48)

Duracion: 120 s - ventanas de 1 s (se descarta la primera).

| Metrica | Minimo | Promedio | Maximo | Ventanas |
|---|---|---|---|---|
| tasa_hz | 100.0 | 100.0 | 100.0 | 118 |
| filtro_max_us | 25.0 | 25.0 | 25.0 | 118 |
| ciclo_max_us | 894.0 | 904.7 | 913.0 | 118 |

## Interpretacion
- Tasa sostenida de 100.0 Hz en las 118 ventanas (requisito 50-100 Hz: cumple).
- Calculo del filtro de Madgwick: 25 us por muestra (0.25 % del periodo de 10 ms).
- Ciclo completo (lectura I2C + filtro + envio serie): ~905 us maximo (~9 % del periodo). El costo dominante es la lectura I2C y el envio por puerto serie, no el filtro.
- Captura: `2026-10-01_t4_madgwick_teleplot.png`. Con pitch cercano a -90 grados (sensor casi vertical) roll y yaw quedan mal definidos (gimbal lock de los angulos de Euler); el cuaternion interno no tiene ese problema. Los saltos de yaw entre +180 y -180 son el pasaje por el limite del rango, no un error.
