# Temporizacion - t2_lectura_cruda (2026-10-01 21:17)

Duracion: 120 s - ventanas de 1 s (se descarta la primera).

| Metrica | Minimo | Promedio | Maximo | Ventanas |
|---|---|---|---|---|
| tasa_hz | 100.0 | 100.0 | 100.0 | 119 |
| dt_min_us | 9999.0 | 9999.0 | 9999.0 | 119 |
| dt_max_us | 10001.0 | 10001.0 | 10001.0 | 119 |
| lectura_i2c_max_us | 535.0 | 539.3 | 559.0 | 119 |
| fallos_i2c | 0.0 | 0.0 | 0.0 | 119 |
| temp_c | 20.5 | 20.8 | 21.1 | 119 |

Jitter maximo respecto del periodo nominal de 10000 us: 1 us (0.01 %).

## Interpretacion
- Tasa de muestreo sostenida de 100.0 Hz durante 120 s (requisito 50-100 Hz: cumple).
- Periodo entre muestras entre 9999 y 10001 us: jitter de +-1 us (0.01 %), es decir, temporizacion practicamente determinista con el planificador por micros().
- Lectura I2C de los 14 bytes del MPU6050 a 400 kHz: 535-559 us (~5.5 % del periodo). Es el costo dominante del ciclo.
- 0 fallos de lectura I2C en ~12 000 muestras.
- Temperatura interna del sensor 20.5-21.1 C durante la prueba (dato de contexto para la deriva del bias).
